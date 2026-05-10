/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/planning/open_space/parking/parking_pose_selector.h"

#include <algorithm>
#include "gtest/gtest.h"

#include "modules/common/math/polygon2d.h"
#include "modules/planning/open_space/coarse_trajectory_generator/node3d.h"

namespace apollo {
namespace planning {
namespace parking {

using apollo::common::math::Vec2d;

namespace {

apollo::common::VehicleParam MakeVehicleParam() {
  apollo::common::VehicleParam vehicle_param;
  vehicle_param.set_length(4.933);
  vehicle_param.set_width(2.11);
  vehicle_param.set_back_edge_to_center(1.043);
  vehicle_param.set_front_edge_to_center(3.89);
  vehicle_param.set_wheel_base(2.8448);
  vehicle_param.set_max_steer_angle(8.20304748437);
  vehicle_param.set_steer_ratio(16.0);
  return vehicle_param;
}

apollo::common::VehicleParam MakeSmallerVehicleParam() {
  apollo::common::VehicleParam vehicle_param;
  vehicle_param.set_length(4.5);
  vehicle_param.set_width(1.92);
  vehicle_param.set_back_edge_to_center(0.95);
  vehicle_param.set_front_edge_to_center(3.55);
  vehicle_param.set_wheel_base(2.8448);
  vehicle_param.set_max_steer_angle(8.20304748437);
  vehicle_param.set_steer_ratio(16.0);
  return vehicle_param;
}

apollo::common::VehicleParam Make4p6VehicleParam() {
  constexpr double kReferenceLength = 4.933;
  constexpr double kReferenceFront = 3.89;
  constexpr double kReferenceBack = 1.043;
  constexpr double kTargetLength = 4.6;

  apollo::common::VehicleParam vehicle_param;
  vehicle_param.set_length(kTargetLength);
  vehicle_param.set_width(2.11);
  vehicle_param.set_back_edge_to_center(kTargetLength * kReferenceBack /
                                        kReferenceLength);
  vehicle_param.set_front_edge_to_center(kTargetLength * kReferenceFront /
                                         kReferenceLength);
  vehicle_param.set_wheel_base(2.8448);
  vehicle_param.set_max_steer_angle(8.20304748437);
  vehicle_param.set_steer_ratio(16.0);
  return vehicle_param;
}

apollo::common::VehicleParam Make4p0VehicleParam() {
  apollo::common::VehicleParam vehicle_param;
  vehicle_param.set_length(4.0);
  vehicle_param.set_width(2.11);
  vehicle_param.set_back_edge_to_center(0.8451246705858504);
  vehicle_param.set_front_edge_to_center(3.1548753294141495);
  vehicle_param.set_wheel_base(2.8448);
  vehicle_param.set_max_steer_angle(8.20304748437);
  vehicle_param.set_steer_ratio(16.0);
  return vehicle_param;
}

void PopulateBoundarySegments(ParkingRoiGeometry* geometry) {
  geometry->boundary_segments.clear();
  for (size_t index = 0; index < geometry->union_polygon.size(); ++index) {
    geometry->boundary_segments.push_back(
        {geometry->union_polygon[index],
         geometry->union_polygon[(index + 1U) % geometry->union_polygon.size()]});
  }
}

ParkingSlot MakeSkewedRightSlot() {
  ParkingSlot slot;
  slot.type = ParkingSlotType::kPerpendicular;
  slot.corners.right_top = Vec2d(1.3955957541592015, -0.026966165716617307);
  slot.corners.left_top = Vec2d(-1.3955957541592015, 0.026966165716617307);
  slot.corners.left_down = Vec2d(-1.3130410679133526, 5.084535041701026);
  slot.corners.right_down = Vec2d(1.4914349694070408, 5.0500457738173425);
  slot.opening_center = (slot.corners.left_top + slot.corners.right_top) * 0.5;
  slot.rear_center = (slot.corners.left_down + slot.corners.right_down) * 0.5;
  slot.center = (slot.opening_center + slot.rear_center) * 0.5;
  slot.width =
      0.5 * ((slot.corners.right_top - slot.corners.left_top).Length() +
             (slot.corners.right_down - slot.corners.left_down).Length());
  slot.depth =
      0.5 * ((slot.corners.left_down - slot.corners.left_top).Length() +
             (slot.corners.right_down - slot.corners.right_top).Length());
  slot.heading = (slot.rear_center - slot.opening_center).Angle();
  return slot;
}

ParkingSlot MakeSanMateoEntryLaneSlot() {
  ParkingSlot slot;
  slot.type = ParkingSlotType::kPerpendicular;
  slot.corners.left_top = Vec2d(-1.4243994947911567, 0.014196710229636977);
  slot.corners.right_top = Vec2d(1.4243994951443355, -0.014196710533125101);
  slot.corners.right_down = Vec2d(-0.46000651238241863, -4.737384509839769);
  slot.corners.left_down = Vec2d(-3.222852066095146, -4.80492748058604);
  slot.opening_center = (slot.corners.left_top + slot.corners.right_top) * 0.5;
  slot.rear_center = (slot.corners.left_down + slot.corners.right_down) * 0.5;
  slot.center = (slot.opening_center + slot.rear_center) * 0.5;
  slot.width =
      0.5 * ((slot.corners.right_top - slot.corners.left_top).Length() +
             (slot.corners.right_down - slot.corners.left_down).Length());
  slot.depth =
      0.5 * ((slot.corners.left_down - slot.corners.left_top).Length() +
             (slot.corners.right_down - slot.corners.right_top).Length());
  slot.heading = (slot.rear_center - slot.opening_center).Angle();
  return slot;
}

ParkingSlot MakeFeasibleAngledRightSlot() {
  ParkingSlot slot;
  slot.type = ParkingSlotType::kAngled;
  slot.width = 3.4;
  slot.depth = 7.2;
  slot.heading = -1.12;
  slot.opening_center = Vec2d(6.0, -1.3);
  const Vec2d depth_axis = Vec2d::CreateUnitVec2d(slot.heading);
  const Vec2d width_axis(-depth_axis.y(), depth_axis.x());
  slot.rear_center = slot.opening_center + depth_axis * slot.depth;
  slot.center = (slot.opening_center + slot.rear_center) * 0.5;
  slot.corners.left_top = slot.opening_center - width_axis * (0.5 * slot.width);
  slot.corners.right_top = slot.opening_center + width_axis * (0.5 * slot.width);
  slot.corners.left_down = slot.rear_center - width_axis * (0.5 * slot.width);
  slot.corners.right_down = slot.rear_center + width_axis * (0.5 * slot.width);
  return slot;
}

ParkingRoiGeometry MakeSkewedRightGeometry() {
  ParkingRoiGeometry geometry;
  geometry.slot_polygon = {
      MakeSkewedRightSlot().corners.left_top,
      MakeSkewedRightSlot().corners.left_down,
      MakeSkewedRightSlot().corners.right_down,
      MakeSkewedRightSlot().corners.right_top,
  };
  geometry.union_polygon = {
      Vec2d(1.3960801858028236, -0.0013037556875108436),
      Vec2d(1.4914349694070408, 5.0500457738173425),
      Vec2d(-1.3130410679133526, 5.084535041701026),
      Vec2d(-1.3955957541592015, 0.026966165716617307),
      Vec2d(-0.06326255209561471, 0.0012223800899238552),
      Vec2d(-14.999949794599933, 0.02707792386298591),
      Vec2d(-14.999972724785644, -12.294085562562973),
      Vec2d(-0.9999937924379432, -12.44702776421623),
      Vec2d(7.43548040960107e-06, -7.4852946449276745),
      Vec2d(14.999976236886782, -7.661074022637162),
      Vec2d(14.999962322598297, -0.024852201494863113),
  };
  geometry.xy_boundary = {-14.999972724785644, 14.999976236886782,
                          -12.44702776421623, 5.084535041701026};
  geometry.aisle_width = 7.4852946449313675;
  PopulateBoundarySegments(&geometry);
  return geometry;
}

ParkingRoiGeometry MakeWideGeometryForSlot(const ParkingSlot& slot,
                                           const double aisle_width) {
  ParkingRoiGeometry geometry;
  geometry.union_polygon = {Vec2d(-30.0, -20.0), Vec2d(30.0, -20.0),
                            Vec2d(30.0, 20.0), Vec2d(-30.0, 20.0)};
  geometry.slot_polygon = {slot.corners.left_top, slot.corners.left_down,
                           slot.corners.right_down, slot.corners.right_top};
  geometry.xy_boundary = {-30.0, 30.0, -20.0, 20.0};
  geometry.aisle_width = aisle_width;
  PopulateBoundarySegments(&geometry);
  return geometry;
}

ParkingSlot MakePerpendicularRightSlot() {
  ParkingSlot slot;
  slot.type = ParkingSlotType::kPerpendicular;
  slot.heading = -M_PI_2;
  slot.depth = 6.0;
  slot.width = 2.8;
  slot.opening_center = Vec2d(0.0, 0.0);
  slot.rear_center = Vec2d(0.0, -6.0);
  slot.corners.left_top = Vec2d(-1.4, 0.0);
  slot.corners.right_top = Vec2d(1.4, 0.0);
  slot.corners.left_down = Vec2d(-1.4, -6.0);
  slot.corners.right_down = Vec2d(1.4, -6.0);
  slot.center = (slot.opening_center + slot.rear_center) * 0.5;
  return slot;
}

ParkingSlot MakeParallelRightSlot() {
  ParkingSlot slot;
  slot.type = ParkingSlotType::kParallel;
  slot.heading = 0.0;
  slot.depth = 2.8;
  slot.width = 9.0;
  slot.opening_center = Vec2d(6.0, -1.4);
  slot.rear_center = Vec2d(6.0, -4.2);
  slot.corners.left_top = Vec2d(1.5, -1.4);
  slot.corners.right_top = Vec2d(10.5, -1.4);
  slot.corners.left_down = Vec2d(1.5, -4.2);
  slot.corners.right_down = Vec2d(10.5, -4.2);
  slot.center = (slot.opening_center + slot.rear_center) * 0.5;
  return slot;
}

apollo::common::math::Polygon2d MakeSlotPolygon(const ParkingSlot& slot) {
  return apollo::common::math::Polygon2d(
      {slot.corners.left_top, slot.corners.left_down, slot.corners.right_down,
       slot.corners.right_top});
}

ParkingSlot MirrorAcrossXAxis(const ParkingSlot& slot) {
  ParkingSlot mirrored = slot;
  auto mirror = [](const Vec2d& point) { return Vec2d(point.x(), -point.y()); };
  mirrored.corners.left_top = mirror(slot.corners.left_top);
  mirrored.corners.right_top = mirror(slot.corners.right_top);
  mirrored.corners.left_down = mirror(slot.corners.left_down);
  mirrored.corners.right_down = mirror(slot.corners.right_down);
  mirrored.opening_center = mirror(slot.opening_center);
  mirrored.rear_center = mirror(slot.rear_center);
  mirrored.center = mirror(slot.center);
  mirrored.heading = -slot.heading;
  return mirrored;
}

ParkingRoiGeometry MirrorAcrossXAxis(const ParkingRoiGeometry& geometry) {
  ParkingRoiGeometry mirrored = geometry;
  auto mirror_polygon = [](std::vector<Vec2d>* polygon) {
    for (auto& point : *polygon) {
      point.set_y(-point.y());
    }
  };
  mirror_polygon(&mirrored.union_polygon);
  mirror_polygon(&mirrored.slot_polygon);
  mirror_polygon(&mirrored.bridge_polygon);
  mirror_polygon(&mirrored.attachment_polygon);
  mirror_polygon(&mirrored.corridor_polygon);
  mirror_polygon(&mirrored.connector_slice);
  mirror_polygon(&mirrored.outer_bridge_slice);
  for (auto& segment : mirrored.boundary_segments) {
    for (auto& point : segment) {
      point.set_y(-point.y());
    }
  }
  mirrored.xy_boundary = {geometry.xy_boundary[0], geometry.xy_boundary[1],
                          -geometry.xy_boundary[3], -geometry.xy_boundary[2]};
  PopulateBoundarySegments(&mirrored);
  return mirrored;
}

bool IsPhysicalInfeasibilityReason(const std::string& reason) {
  return reason == "no statically feasible vehicle box inside parking envelope" ||
         reason == "goal vehicle box exits parking envelope" ||
         reason == "goal vehicle box too close to parking envelope boundary" ||
         reason == "probe vehicle box exits roi polygon" ||
         reason == "probe vehicle box overlaps roi boundary";
}

bool DoesVehicleFitAnywhereInSlot(
    const ParkingSlot& slot,
    const apollo::common::VehicleParam& vehicle_param, const double heading) {
  const auto slot_polygon = MakeSlotPolygon(slot);
  constexpr int kLongitudinalSamples = 120;
  constexpr int kLateralSamples = 80;
  for (int longitudinal_index = 0; longitudinal_index <= kLongitudinalSamples;
       ++longitudinal_index) {
    const double longitudinal_ratio =
        static_cast<double>(longitudinal_index) / kLongitudinalSamples;
    const Vec2d left_edge_point =
        slot.corners.left_top +
        (slot.corners.left_down - slot.corners.left_top) * longitudinal_ratio;
    const Vec2d right_edge_point =
        slot.corners.right_top +
        (slot.corners.right_down - slot.corners.right_top) * longitudinal_ratio;
    for (int lateral_index = 0; lateral_index <= kLateralSamples;
         ++lateral_index) {
      const double lateral_ratio =
          static_cast<double>(lateral_index) / kLateralSamples;
      const Vec2d rear_axle =
          left_edge_point + (right_edge_point - left_edge_point) * lateral_ratio;
      const auto ego_box = Node3d::GetBoundingBox(vehicle_param, rear_axle.x(),
                                                  rear_axle.y(), heading);
      if (slot_polygon.Contains(apollo::common::math::Polygon2d(ego_box))) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

TEST(ParkingPoseSelectorTest, PreferTailInWhenHeadInAisleThresholdFails) {
  OpenSpaceRoiDeciderConfig config;
  config.set_candidate_min_aisle_width_head_in(6.0);
  config.set_candidate_min_aisle_width_tail_in(3.0);
  ParkingPoseSelector selector(config);

  ParkingSlot slot;
  slot.type = ParkingSlotType::kPerpendicular;
  slot.heading = -M_PI_2;
  slot.depth = 6.0;
  slot.width = 2.5;
  slot.opening_center = Vec2d(0.0, 0.0);
  slot.rear_center = Vec2d(0.0, -6.0);
  slot.corners.left_top = Vec2d(-1.25, 0.0);
  slot.corners.right_top = Vec2d(1.25, 0.0);
  slot.corners.left_down = Vec2d(-1.25, -6.0);
  slot.corners.right_down = Vec2d(1.25, -6.0);

  ParkingRoiGeometry geometry;
  geometry.union_polygon = {Vec2d(-60.0, 30.0), Vec2d(60.0, 30.0),
                            Vec2d(60.0, -60.0), Vec2d(-60.0, -60.0)};
  geometry.xy_boundary = {-60.0, 60.0, -60.0, 30.0};
  geometry.aisle_width = 4.0;
  geometry.boundary_segments = {
      {Vec2d(-60.0, 30.0), Vec2d(60.0, 30.0)},
      {Vec2d(60.0, 30.0), Vec2d(60.0, -60.0)},
      {Vec2d(60.0, -60.0), Vec2d(-60.0, -60.0)},
      {Vec2d(-60.0, -60.0), Vec2d(-60.0, 30.0)},
  };

  apollo::common::VehicleParam vehicle_param;
  vehicle_param.set_length(4.8);
  vehicle_param.set_width(2.0);
  vehicle_param.set_back_edge_to_center(1.0);
  vehicle_param.set_front_edge_to_center(3.8);
  vehicle_param.set_wheel_base(2.8);
  vehicle_param.set_max_steer_angle(8.20304748437);
  vehicle_param.set_steer_ratio(16.0);

  const auto selection =
      selector.Select(slot, geometry, vehicle_param, Vec2d(0.0, 15.0), -M_PI_2);
  ASSERT_TRUE(selection.has_feasible_candidate());
  EXPECT_EQ(selection.selected().approach, ParkingApproach::kTailIn);
}

TEST(ParkingPoseSelectorTest, PreferHeadInWhenTailInAisleThresholdFails) {
  OpenSpaceRoiDeciderConfig config;
  config.set_candidate_min_aisle_width_head_in(3.0);
  config.set_candidate_min_aisle_width_tail_in(6.0);
  ParkingPoseSelector selector(config);

  ParkingSlot slot;
  slot.type = ParkingSlotType::kPerpendicular;
  slot.heading = -M_PI_2;
  slot.depth = 6.0;
  slot.width = 2.8;
  slot.opening_center = Vec2d(0.0, 0.0);
  slot.rear_center = Vec2d(0.0, -6.0);
  slot.corners.left_top = Vec2d(-1.4, 0.0);
  slot.corners.right_top = Vec2d(1.4, 0.0);
  slot.corners.left_down = Vec2d(-1.4, -6.0);
  slot.corners.right_down = Vec2d(1.4, -6.0);

  ParkingRoiGeometry geometry;
  geometry.union_polygon = {Vec2d(-60.0, 30.0), Vec2d(60.0, 30.0),
                            Vec2d(60.0, -60.0), Vec2d(-60.0, -60.0)};
  geometry.slot_polygon = {slot.corners.left_top, slot.corners.left_down,
                           slot.corners.right_down, slot.corners.right_top};
  geometry.xy_boundary = {-60.0, 60.0, -60.0, 30.0};
  geometry.aisle_width = 4.0;
  geometry.boundary_segments = {
      {Vec2d(-60.0, 30.0), Vec2d(60.0, 30.0)},
      {Vec2d(60.0, 30.0), Vec2d(60.0, -60.0)},
      {Vec2d(60.0, -60.0), Vec2d(-60.0, -60.0)},
      {Vec2d(-60.0, -60.0), Vec2d(-60.0, 30.0)},
  };

  const auto selection =
      selector.Select(slot, geometry, MakeVehicleParam(), Vec2d(0.0, 15.0), -M_PI_2);
  ASSERT_TRUE(selection.has_feasible_candidate());
  EXPECT_EQ(selection.selected().approach, ParkingApproach::kHeadIn);
}

TEST(ParkingPoseSelectorTest, RejectCandidateVehicleBoxTooCloseToRoiBoundary) {
  OpenSpaceRoiDeciderConfig config;
  config.set_roi_min_goal_clearance(1.0);
  ParkingPoseSelector selector(config);

  ParkingSlot slot;
  slot.type = ParkingSlotType::kPerpendicular;
  slot.heading = -M_PI_2;
  slot.depth = 4.0;
  slot.width = 2.4;
  slot.opening_center = Vec2d(0.0, 0.0);
  slot.rear_center = Vec2d(0.0, 4.0);
  slot.corners.left_top = Vec2d(-1.2, 0.0);
  slot.corners.right_top = Vec2d(1.2, 0.0);
  slot.corners.left_down = Vec2d(-1.2, 4.0);
  slot.corners.right_down = Vec2d(1.2, 4.0);

  ParkingRoiGeometry geometry;
  geometry.union_polygon = {Vec2d(-0.7, -1.0), Vec2d(0.7, -1.0),
                            Vec2d(0.7, 6.0), Vec2d(-0.7, 6.0)};
  geometry.xy_boundary = {-0.7, 0.7, -1.0, 6.0};
  geometry.aisle_width = 5.0;
  geometry.boundary_segments = {
      {Vec2d(-0.7, -1.0), Vec2d(0.7, -1.0)},
      {Vec2d(0.7, -1.0), Vec2d(0.7, 6.0)},
      {Vec2d(0.7, 6.0), Vec2d(-0.7, 6.0)},
      {Vec2d(-0.7, 6.0), Vec2d(-0.7, -1.0)},
  };

  apollo::common::VehicleParam vehicle_param;
  vehicle_param.set_length(4.8);
  vehicle_param.set_width(2.0);
  vehicle_param.set_back_edge_to_center(1.0);
  vehicle_param.set_front_edge_to_center(3.8);
  vehicle_param.set_wheel_base(2.8);
  vehicle_param.set_max_steer_angle(8.20304748437);
  vehicle_param.set_steer_ratio(16.0);

  const auto selection =
      selector.Select(slot, geometry, vehicle_param, Vec2d(0.0, -3.0), -M_PI_2);
  EXPECT_FALSE(selection.has_feasible_candidate());
  ASSERT_EQ(selection.candidates.size(), 2U);
  EXPECT_EQ(selection.candidates[0].rejection_reason,
            "no statically feasible vehicle box inside parking envelope");
}

TEST(ParkingPoseSelectorTest,
     AcceptsLowClearanceCandidateAndReportsBoundaryMargin) {
  OpenSpaceRoiDeciderConfig config;
  config.set_roi_min_goal_clearance(0.2);
  ParkingPoseSelector selector(config);

  ParkingSlot slot;
  slot.type = ParkingSlotType::kPerpendicular;
  slot.heading = -M_PI_2;
  slot.depth = 5.1;
  slot.width = 2.8;
  slot.opening_center = Vec2d(0.0, 0.0);
  slot.rear_center = Vec2d(0.0, 5.1);
  slot.corners.left_top = Vec2d(-1.4, 0.0);
  slot.corners.right_top = Vec2d(1.4, 0.0);
  slot.corners.left_down = Vec2d(-1.3, 5.1);
  slot.corners.right_down = Vec2d(1.5, 5.1);

  ParkingRoiGeometry geometry;
  geometry.union_polygon = {Vec2d(-15.0, -12.0), Vec2d(15.0, -12.0),
                            Vec2d(15.0, 6.0), Vec2d(-15.0, 6.0)};
  geometry.slot_polygon = {slot.corners.left_top, slot.corners.left_down,
                           slot.corners.right_down, slot.corners.right_top};
  geometry.xy_boundary = {-15.0, 15.0, -12.0, 6.0};
  geometry.aisle_width = 7.5;
  PopulateBoundarySegments(&geometry);

  const auto selection =
      selector.Select(slot, geometry, MakeVehicleParam(), Vec2d(0.0, -6.0), 0.0);
  ASSERT_TRUE(selection.has_feasible_candidate());
  ASSERT_EQ(selection.candidates.size(), 2U);
  EXPECT_GT(selection.selected().min_clearance, 0.0);
  EXPECT_LT(selection.selected().min_clearance, 0.2);
}

TEST(ParkingPoseSelectorTest, SelectsParallelPoseInsideWideRoi) {
  OpenSpaceRoiDeciderConfig config;
  ParkingPoseSelector selector(config);

  ParkingSlot slot;
  slot.type = ParkingSlotType::kParallel;
  slot.heading = 0.0;
  slot.depth = 2.3;
  slot.width = 8.0;
  slot.opening_center = Vec2d(6.0, -1.5);
  slot.rear_center = Vec2d(6.0, -3.8);
  slot.corners.left_top = Vec2d(2.0, -1.5);
  slot.corners.right_top = Vec2d(10.0, -1.5);
  slot.corners.left_down = Vec2d(2.0, -3.8);
  slot.corners.right_down = Vec2d(10.0, -3.8);

  ParkingRoiGeometry geometry;
  geometry.union_polygon = {Vec2d(-20.0, 2.0), Vec2d(20.0, 2.0),
                            Vec2d(20.0, -8.0), Vec2d(-20.0, -8.0)};
  geometry.xy_boundary = {-20.0, 20.0, -8.0, 2.0};
  geometry.aisle_width = 5.0;
  PopulateBoundarySegments(&geometry);

  const auto selection =
      selector.Select(slot, geometry, MakeVehicleParam(), Vec2d(0.0, -2.5), 0.0);
  ASSERT_TRUE(selection.has_feasible_candidate());
  EXPECT_GT(selection.selected().min_clearance, 0.0);
}

TEST(ParkingPoseSelectorTest, AutoPrefersTailInForPerpendicularSlots) {
  OpenSpaceRoiDeciderConfig config;
  config.set_roi_min_goal_clearance(0.2);
  ParkingPoseSelector selector(config);

  const auto slot = MakePerpendicularRightSlot();
  const auto geometry = MakeWideGeometryForSlot(slot, 6.0);
  const auto selection =
      selector.Select(slot, geometry, Make4p6VehicleParam(), Vec2d(0.0, 12.0),
                      -M_PI_2);
  ASSERT_TRUE(selection.has_feasible_candidate());
  ASSERT_EQ(selection.candidates.size(), 2U);
  EXPECT_EQ(selection.selected().approach, ParkingApproach::kTailIn);
  EXPECT_TRUE(selection.candidates[0].was_probed);
  EXPECT_TRUE(selection.candidates[1].was_probed);
}

TEST(ParkingPoseSelectorTest, SelectsParallelLeftPoseInsideWideRoi) {
  OpenSpaceRoiDeciderConfig config;
  ParkingPoseSelector selector(config);

  ParkingSlot slot;
  slot.type = ParkingSlotType::kParallel;
  slot.heading = 0.0;
  slot.depth = 2.3;
  slot.width = 8.0;
  slot.opening_center = Vec2d(6.0, 1.5);
  slot.rear_center = Vec2d(6.0, 3.8);
  slot.corners.left_top = Vec2d(2.0, 1.5);
  slot.corners.right_top = Vec2d(10.0, 1.5);
  slot.corners.left_down = Vec2d(2.0, 3.8);
  slot.corners.right_down = Vec2d(10.0, 3.8);

  ParkingRoiGeometry geometry;
  geometry.union_polygon = {Vec2d(-20.0, -2.0), Vec2d(20.0, -2.0),
                            Vec2d(20.0, 8.0), Vec2d(-20.0, 8.0)};
  geometry.xy_boundary = {-20.0, 20.0, -2.0, 8.0};
  geometry.aisle_width = 5.0;
  PopulateBoundarySegments(&geometry);

  const auto selection =
      selector.Select(slot, geometry, MakeVehicleParam(), Vec2d(0.0, 2.5), 0.0);
  ASSERT_TRUE(selection.has_feasible_candidate());
  EXPECT_GT(selection.selected().min_clearance, 0.0);
}

TEST(ParkingPoseSelectorTest, AutoPrefersTailInForParallelSlots) {
  OpenSpaceRoiDeciderConfig config;
  config.set_roi_min_goal_clearance(0.2);
  ParkingPoseSelector selector(config);

  const auto slot = MakeParallelRightSlot();
  const auto geometry = MakeWideGeometryForSlot(slot, 6.0);
  const auto selection =
      selector.Select(slot, geometry, Make4p6VehicleParam(), Vec2d(0.0, -2.5),
                      0.0);
  ASSERT_TRUE(selection.has_feasible_candidate());
  EXPECT_EQ(selection.selected().approach, ParkingApproach::kTailIn);
}

TEST(ParkingPoseSelectorTest, SelectsAngledRightHeadInInsideWideRoi) {
  OpenSpaceRoiDeciderConfig config;
  config.set_candidate_min_aisle_width_head_in(3.0);
  config.set_candidate_min_aisle_width_tail_in(6.0);
  config.set_roi_min_goal_clearance(0.2);
  ParkingPoseSelector selector(config);

  const ParkingSlot slot = MakeFeasibleAngledRightSlot();
  const ParkingRoiGeometry geometry = MakeWideGeometryForSlot(slot, 5.0);
  const auto selection =
      selector.Select(slot, geometry, MakeVehicleParam(), Vec2d(0.0, 1.5), 0.0);
  ASSERT_TRUE(selection.has_feasible_candidate());
  EXPECT_EQ(selection.selected().approach, ParkingApproach::kHeadIn);
  EXPECT_GT(selection.selected().min_clearance, 0.2);
}

TEST(ParkingPoseSelectorTest, AutoPrefersHeadInForAngledSlots) {
  OpenSpaceRoiDeciderConfig config;
  config.set_roi_min_goal_clearance(0.2);
  ParkingPoseSelector selector(config);

  const ParkingSlot slot = MakeFeasibleAngledRightSlot();
  const ParkingRoiGeometry geometry = MakeWideGeometryForSlot(slot, 6.0);
  const auto selection =
      selector.Select(slot, geometry, MakeVehicleParam(), Vec2d(0.0, 1.5), 0.0);
  ASSERT_TRUE(selection.has_feasible_candidate());
  EXPECT_EQ(selection.selected().approach, ParkingApproach::kHeadIn);
}

TEST(ParkingPoseSelectorTest, SelectsAngledLeftTailInInsideWideRoi) {
  OpenSpaceRoiDeciderConfig config;
  config.set_candidate_min_aisle_width_head_in(6.0);
  config.set_candidate_min_aisle_width_tail_in(3.0);
  config.set_roi_min_goal_clearance(0.2);
  ParkingPoseSelector selector(config);

  const ParkingSlot slot = MirrorAcrossXAxis(MakeFeasibleAngledRightSlot());
  const ParkingRoiGeometry geometry = MirrorAcrossXAxis(MakeWideGeometryForSlot(
      MakeFeasibleAngledRightSlot(), 5.0));
  const auto selection =
      selector.Select(slot, geometry, MakeVehicleParam(), Vec2d(0.0, -1.5), 0.0);
  ASSERT_TRUE(selection.has_feasible_candidate());
  EXPECT_EQ(selection.selected().approach, ParkingApproach::kTailIn);
  EXPECT_GT(selection.selected().min_clearance, 0.0);
}

TEST(ParkingPoseSelectorTest, Vehicle4p6CoversParkingMatrixWithPositiveClearance) {
  const auto vehicle_param = Make4p6VehicleParam();

  struct CoverageCase {
    const char* name;
    ParkingSlot slot;
    ParkingRoiGeometry geometry;
    ParkingApproach preferred;
    double min_head_in_aisle_width;
    double min_tail_in_aisle_width;
    Vec2d vehicle_position;
    double vehicle_heading;
  };

  OpenSpaceRoiDeciderConfig base_config;
  base_config.set_roi_min_goal_clearance(0.2);

  const ParkingSlot perpendicular_right = MakePerpendicularRightSlot();
  const ParkingSlot perpendicular_left = MirrorAcrossXAxis(perpendicular_right);
  const ParkingSlot parallel_right = MakeParallelRightSlot();
  const ParkingSlot parallel_left = MirrorAcrossXAxis(parallel_right);
  const ParkingSlot angled_right = MakeFeasibleAngledRightSlot();
  const ParkingSlot angled_left = MirrorAcrossXAxis(angled_right);

  const std::vector<CoverageCase> cases = {
      {"perpendicular-right-tail-in", perpendicular_right,
       MakeWideGeometryForSlot(perpendicular_right, 4.0),
       ParkingApproach::kTailIn, 6.0, 3.0, Vec2d(0.0, 12.0), -M_PI_2},
      {"perpendicular-left-head-in", perpendicular_left,
       MakeWideGeometryForSlot(perpendicular_left, 4.0),
       ParkingApproach::kHeadIn, 3.0, 6.0, Vec2d(0.0, -12.0), M_PI_2},
      {"parallel-right-tail-in", parallel_right,
       MakeWideGeometryForSlot(parallel_right, 5.0),
       ParkingApproach::kTailIn, 6.0, 3.0, Vec2d(0.0, -2.5), 0.0},
      {"parallel-left-head-in", parallel_left,
       MakeWideGeometryForSlot(parallel_left, 5.0),
       ParkingApproach::kHeadIn, 3.0, 6.0, Vec2d(0.0, 2.5), 0.0},
      {"angled-right-head-in", angled_right,
       MakeWideGeometryForSlot(angled_right, 5.0),
       ParkingApproach::kHeadIn, 3.0, 6.0, Vec2d(0.0, 1.5), 0.0},
      {"angled-left-tail-in", angled_left,
       MakeWideGeometryForSlot(angled_left, 5.0),
       ParkingApproach::kTailIn, 6.0, 3.0, Vec2d(0.0, -1.5), 0.0},
  };

  for (const auto& test_case : cases) {
    OpenSpaceRoiDeciderConfig config = base_config;
    config.set_candidate_min_aisle_width_head_in(
        test_case.min_head_in_aisle_width);
    config.set_candidate_min_aisle_width_tail_in(
        test_case.min_tail_in_aisle_width);
    ParkingPoseSelector selector(config);

    SCOPED_TRACE(test_case.name);
    const auto selection =
        selector.Select(test_case.slot, test_case.geometry, vehicle_param,
                        test_case.vehicle_position, test_case.vehicle_heading);
    ASSERT_TRUE(selection.has_feasible_candidate());
    EXPECT_EQ(selection.selected().approach, test_case.preferred);
    EXPECT_GT(selection.selected().min_clearance, 0.0);
  }
}

TEST(ParkingPoseSelectorTest,
     RejectsSunnyvaleSkewedSlotWhenPathFeasibilityFailsAtProductionClearance) {
  OpenSpaceRoiDeciderConfig config;
  config.set_roi_min_goal_clearance(0.2);
  ParkingPoseSelector selector(config);

  const auto selection =
      selector.Select(MakeSkewedRightSlot(), MakeSkewedRightGeometry(),
                      Make4p6VehicleParam(), Vec2d(0.0, 0.0), 0.0);
  ASSERT_FALSE(selection.has_feasible_candidate())
      << "head_in=" << selection.candidates[0].rejection_reason << " clearance="
      << selection.candidates[0].min_clearance << ", tail_in="
      << selection.candidates[1].rejection_reason << " clearance="
      << selection.candidates[1].min_clearance;
  ASSERT_EQ(selection.candidates.size(), 2U);
  EXPECT_TRUE(
      IsPhysicalInfeasibilityReason(selection.candidates[0].rejection_reason));
  EXPECT_TRUE(
      IsPhysicalInfeasibilityReason(selection.candidates[1].rejection_reason));
}

TEST(ParkingPoseSelectorTest, RejectsSkewedRightSlotWhenVehicleBoxCannotFit) {
  OpenSpaceRoiDeciderConfig config;
  config.set_roi_min_goal_clearance(0.2);
  ParkingPoseSelector selector(config);

  const auto selection =
      selector.Select(MakeSkewedRightSlot(), MakeSkewedRightGeometry(),
                      MakeVehicleParam(), Vec2d(-0.23087395123878895, 3.9296748253017255),
                      -1.5883970034463442);
  ASSERT_FALSE(selection.has_feasible_candidate());
  ASSERT_EQ(selection.candidates.size(), 2U);
  EXPECT_TRUE(
      IsPhysicalInfeasibilityReason(selection.candidates[0].rejection_reason));
  EXPECT_TRUE(
      IsPhysicalInfeasibilityReason(selection.candidates[1].rejection_reason));
}

TEST(ParkingPoseSelectorTest, RejectsSkewedLeftSlotWhenVehicleBoxCannotFit) {
  OpenSpaceRoiDeciderConfig config;
  config.set_roi_min_goal_clearance(0.2);
  ParkingPoseSelector selector(config);

  const auto slot = MirrorAcrossXAxis(MakeSkewedRightSlot());
  const auto geometry = MirrorAcrossXAxis(MakeSkewedRightGeometry());
  const auto selection =
      selector.Select(slot, geometry, MakeVehicleParam(),
                      Vec2d(-0.23087395123878895, -3.9296748253017255),
                      1.5883970034463442);
  ASSERT_FALSE(selection.has_feasible_candidate());
  ASSERT_EQ(selection.candidates.size(), 2U);
  EXPECT_TRUE(
      IsPhysicalInfeasibilityReason(selection.candidates[0].rejection_reason));
  EXPECT_TRUE(
      IsPhysicalInfeasibilityReason(selection.candidates[1].rejection_reason));
}

TEST(ParkingPoseSelectorTest, SanMateoSlotFitsSmallerVehicleOnly) {
  const ParkingSlot slot = MakeSanMateoEntryLaneSlot();
  const double head_in_heading = slot.heading;
  const double tail_in_heading =
      common::math::NormalizeAngle(slot.heading + M_PI);

  EXPECT_FALSE(DoesVehicleFitAnywhereInSlot(slot, MakeVehicleParam(),
                                            head_in_heading));
  EXPECT_FALSE(DoesVehicleFitAnywhereInSlot(slot, MakeVehicleParam(),
                                            tail_in_heading));
  EXPECT_FALSE(DoesVehicleFitAnywhereInSlot(slot, MakeSmallerVehicleParam(),
                                            head_in_heading));
  EXPECT_FALSE(DoesVehicleFitAnywhereInSlot(slot, MakeSmallerVehicleParam(),
                                            tail_in_heading));
}

TEST(ParkingPoseSelectorTest, SanMateoTailInCandidateRefinesTowardOpening) {
  OpenSpaceRoiDeciderConfig config;
  ParkingPoseSelector selector(config);

  ParkingRoiGeometry geometry;
  geometry.union_polygon = {Vec2d(-40.0, -20.0), Vec2d(40.0, -20.0),
                            Vec2d(40.0, 20.0), Vec2d(-40.0, 20.0)};
  geometry.xy_boundary = {-40.0, 40.0, -20.0, 20.0};
  geometry.aisle_width = 8.0;
  const ParkingSlot slot = MakeSanMateoEntryLaneSlot();
  geometry.slot_polygon = {slot.corners.left_top, slot.corners.left_down,
                           slot.corners.right_down, slot.corners.right_top};
  PopulateBoundarySegments(&geometry);
  const auto selection =
      selector.Select(slot, geometry, MakeVehicleParam(), Vec2d(0.0, 8.0), 0.0);
  ASSERT_EQ(selection.candidates.size(), 2U);

  const auto tail_candidate = std::find_if(
      selection.candidates.begin(), selection.candidates.end(),
      [](const ParkingPoseCandidate& candidate) {
        return candidate.approach == ParkingApproach::kTailIn;
      });
  ASSERT_NE(tail_candidate, selection.candidates.end());

  Vec2d depth_axis = slot.rear_center - slot.opening_center;
  depth_axis.Normalize();
  const double tail_depth =
      (Vec2d(tail_candidate->end_pose[0], tail_candidate->end_pose[1]) -
       slot.opening_center)
          .InnerProd(depth_axis);
  EXPECT_GT(tail_depth, -0.5);
  EXPECT_LT(tail_depth, 0.5);
}

TEST(ParkingPoseSelectorTest,
     SanMateoOpeningFallbackEnvelopeAllowsFeasibleSelectionFor4p0Vehicle) {
  OpenSpaceRoiDeciderConfig config;
  ParkingPoseSelector selector(config);

  ParkingRoiGeometry geometry;
  geometry.union_polygon = {Vec2d(-40.0, -20.0), Vec2d(40.0, -20.0),
                            Vec2d(40.0, 20.0), Vec2d(-40.0, 20.0)};
  geometry.xy_boundary = {-40.0, 40.0, -20.0, 20.0};
  geometry.aisle_width = 8.0;
  const ParkingSlot slot = MakeSanMateoEntryLaneSlot();
  geometry.slot_polygon = {slot.corners.left_top, slot.corners.left_down,
                           slot.corners.right_down, slot.corners.right_top};
  PopulateBoundarySegments(&geometry);

  const auto selection =
      selector.Select(slot, geometry, Make4p0VehicleParam(), Vec2d(0.0, 8.0), 0.0);
  ASSERT_TRUE(selection.has_feasible_candidate());
  EXPECT_GT(selection.selected().min_clearance, 0.0);
  EXPECT_TRUE(std::any_of(selection.candidates.begin(), selection.candidates.end(),
                          [](const ParkingPoseCandidate& candidate) {
                            return candidate.feasible;
                          }));
}



}  // namespace parking
}  // namespace planning
}  // namespace apollo
