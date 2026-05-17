/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/planning/open_space/parking/parking_pose_selector.h"

#include <cmath>

#include "gtest/gtest.h"

#include "modules/common/math/math_utils.h"

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

ParkingRoiGeometry MakeGeometry(const double aisle_width = 6.0) {
  ParkingRoiGeometry geometry;
  geometry.aisle_width = aisle_width;
  geometry.union_polygon = {Vec2d(-1.0, -1.0), Vec2d(1.0, -1.0),
                            Vec2d(1.0, 1.0), Vec2d(-1.0, 1.0)};
  geometry.xy_boundary = {-1.0, 1.0, -1.0, 1.0};
  return geometry;
}

ParkingSlot MakePerpendicularRightSlot() {
  ParkingSlot slot;
  slot.id = "perpendicular";
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
  slot.id = "parallel";
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

ParkingSlot MakeAngledRightSlot() {
  ParkingSlot slot;
  slot.id = "angled";
  slot.type = ParkingSlotType::kAngled;
  slot.heading = -1.12;
  slot.depth = 7.2;
  slot.width = 3.4;
  slot.opening_center = Vec2d(6.0, -1.3);
  const Vec2d depth_axis = Vec2d::CreateUnitVec2d(slot.heading);
  const Vec2d width_axis(-depth_axis.y(), depth_axis.x());
  slot.rear_center = slot.opening_center + depth_axis * slot.depth;
  slot.center = (slot.opening_center + slot.rear_center) * 0.5;
  slot.corners.left_top = slot.opening_center - width_axis * (0.5 * slot.width);
  slot.corners.right_top =
      slot.opening_center + width_axis * (0.5 * slot.width);
  slot.corners.left_down = slot.rear_center - width_axis * (0.5 * slot.width);
  slot.corners.right_down = slot.rear_center + width_axis * (0.5 * slot.width);
  return slot;
}

}  // namespace

TEST(ParkingPoseSelectorTest, AutoPrefersTailInForPerpendicularSlots) {
  ParkingPoseSelector selector{OpenSpaceRoiDeciderConfig()};

  const auto selection =
      selector.Select(MakePerpendicularRightSlot(), MakeGeometry(),
                      MakeVehicleParam(), Vec2d(0.0, 3.0), 0.0);

  ASSERT_TRUE(selection.has_feasible_candidate());
  EXPECT_EQ(selection.selected().approach, ParkingApproach::kTailIn);
  EXPECT_NEAR(selection.selected().end_pose[2], 0.5 * M_PI, 1e-6);
}

TEST(ParkingPoseSelectorTest, AutoPrefersTailInForParallelSlots) {
  ParkingPoseSelector selector{OpenSpaceRoiDeciderConfig()};

  const auto selection =
      selector.Select(MakeParallelRightSlot(), MakeGeometry(),
                      MakeVehicleParam(), Vec2d(0.0, 0.0), 0.0);

  ASSERT_TRUE(selection.has_feasible_candidate());
  EXPECT_EQ(selection.selected().approach, ParkingApproach::kTailIn);
  EXPECT_NEAR(selection.selected().end_pose[2], 0.0, 1e-6);
}

TEST(ParkingPoseSelectorTest, AutoPrefersHeadInForAngledSlots) {
  ParkingPoseSelector selector{OpenSpaceRoiDeciderConfig()};

  const auto selection =
      selector.Select(MakeAngledRightSlot(), MakeGeometry(), MakeVehicleParam(),
                      Vec2d(0.0, 0.0), 0.0);

  ASSERT_TRUE(selection.has_feasible_candidate());
  EXPECT_EQ(selection.selected().approach, ParkingApproach::kHeadIn);
  EXPECT_NEAR(selection.selected().end_pose[2], MakeAngledRightSlot().heading,
              1e-6);
}

TEST(ParkingPoseSelectorTest, ExplicitPreferenceOverridesAutoPolicy) {
  OpenSpaceRoiDeciderConfig config;
  config.set_parking_approach_preference(
      OpenSpaceRoiDeciderConfig::PARKING_APPROACH_PREFER_HEAD_IN);
  ParkingPoseSelector selector(config);

  const auto selection =
      selector.Select(MakePerpendicularRightSlot(), MakeGeometry(),
                      MakeVehicleParam(), Vec2d(0.0, 0.0), 0.0);

  ASSERT_TRUE(selection.has_feasible_candidate());
  EXPECT_EQ(selection.selected().approach, ParkingApproach::kHeadIn);
}

TEST(ParkingPoseSelectorTest, ExplicitTailInPreferenceOverridesAngledAutoPolicy) {
  OpenSpaceRoiDeciderConfig config;
  config.set_parking_approach_preference(
      OpenSpaceRoiDeciderConfig::PARKING_APPROACH_PREFER_TAIL_IN);
  ParkingPoseSelector selector(config);

  const ParkingSlot slot = MakeAngledRightSlot();
  const auto selection = selector.Select(slot, MakeGeometry(),
                                        MakeVehicleParam(), Vec2d(0.0, 0.0),
                                        0.0);

  ASSERT_TRUE(selection.has_feasible_candidate());
  EXPECT_EQ(selection.selected().approach, ParkingApproach::kTailIn);
  EXPECT_NEAR(common::math::AngleDiff(selection.selected().end_pose[2],
                                      slot.heading + M_PI),
              0.0, 1e-6);

  const double tail_in_depth =
      (Vec2d(selection.selected().end_pose[0], selection.selected().end_pose[1]) -
       slot.opening_center)
          .InnerProd(Vec2d::CreateUnitVec2d(slot.heading));
  EXPECT_NEAR(tail_in_depth,
              slot.depth - config.parking_depth_buffer() -
                  MakeVehicleParam().back_edge_to_center(),
              1e-6);
}

TEST(ParkingPoseSelectorTest,
     IgnoresRoiContainmentAndLeavesReachabilityToWarmStart) {
  ParkingPoseSelector selector{OpenSpaceRoiDeciderConfig()};
  ParkingRoiGeometry tiny_geometry = MakeGeometry();
  tiny_geometry.union_polygon = {Vec2d(100.0, 100.0), Vec2d(101.0, 100.0),
                                 Vec2d(101.0, 101.0), Vec2d(100.0, 101.0)};
  tiny_geometry.xy_boundary = {100.0, 101.0, 100.0, 101.0};

  const auto selection =
      selector.Select(MakeAngledRightSlot(), tiny_geometry, MakeVehicleParam(),
                      Vec2d(0.0, 0.0), 0.0);

  ASSERT_TRUE(selection.has_feasible_candidate());
  EXPECT_EQ(selection.selected().approach, ParkingApproach::kHeadIn);
  EXPECT_TRUE(selection.selected().rejection_reason.empty());
}

TEST(ParkingPoseSelectorTest, RejectsSlotThatCannotContainVehicleFootprint) {
  ParkingPoseSelector selector{OpenSpaceRoiDeciderConfig()};
  ParkingSlot slot = MakeAngledRightSlot();
  slot.id = "too-small";
  slot.width = 2.0;
  slot.depth = 3.5;
  slot.opening_center = Vec2d(0.0, 0.0);
  const Vec2d depth_axis = Vec2d::CreateUnitVec2d(slot.heading);
  const Vec2d width_axis(-depth_axis.y(), depth_axis.x());
  slot.rear_center = slot.opening_center + depth_axis * slot.depth;
  slot.corners.left_top = slot.opening_center - width_axis * (0.5 * slot.width);
  slot.corners.right_top = slot.opening_center + width_axis * (0.5 * slot.width);
  slot.corners.left_down = slot.rear_center - width_axis * (0.5 * slot.width);
  slot.corners.right_down = slot.rear_center + width_axis * (0.5 * slot.width);

  const auto selection =
      selector.Select(slot, MakeGeometry(), MakeVehicleParam(), Vec2d(0.0, 0.0),
                      0.0);

  EXPECT_FALSE(selection.has_feasible_candidate());
}

TEST(ParkingPoseSelectorTest, StartPosePerturbationDoesNotChangeDeterministicGoal) {
  ParkingPoseSelector selector{OpenSpaceRoiDeciderConfig()};
  const ParkingSlot slot = MakePerpendicularRightSlot();
  const auto nominal =
      selector.Select(slot, MakeGeometry(), MakeVehicleParam(), Vec2d(0.0, 3.0),
                      0.0);
  const auto perturbed = selector.Select(slot, MakeGeometry(), MakeVehicleParam(),
                                        Vec2d(0.6, 2.7), 0.15);

  ASSERT_TRUE(nominal.has_feasible_candidate());
  ASSERT_TRUE(perturbed.has_feasible_candidate());
  EXPECT_EQ(nominal.selected().approach, perturbed.selected().approach);
  ASSERT_EQ(nominal.selected().end_pose.size(),
            perturbed.selected().end_pose.size());
  for (std::size_t i = 0; i < nominal.selected().end_pose.size(); ++i) {
    EXPECT_NEAR(nominal.selected().end_pose[i], perturbed.selected().end_pose[i],
                1e-9);
  }
}

}  // namespace parking
}  // namespace planning
}  // namespace apollo
