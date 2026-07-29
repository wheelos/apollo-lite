/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/planning/open_space/parking/parking_roi_geometry.h"

#include <array>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "modules/common/math/box2d.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/polygon2d.h"

namespace apollo {
namespace planning {
namespace parking {

using apollo::common::math::Vec2d;

namespace {

ParkingSlot MakeSlot(const Vec2d& left_top, const Vec2d& right_top,
                     const Vec2d& right_down, const Vec2d& left_down) {
  ParkingSlot slot;
  slot.corners.left_top = left_top;
  slot.corners.right_top = right_top;
  slot.corners.right_down = right_down;
  slot.corners.left_down = left_down;
  slot.opening_center = (left_top + right_top) * 0.5;
  slot.rear_center = (left_down + right_down) * 0.5;
  slot.center = (slot.opening_center + slot.rear_center) * 0.5;
  return slot;
}

ParkingRoiBuildInput MakeStraightInput(
    const double left_boundary_y, const double right_boundary_y,
    const std::vector<Vec2d>& connector_boundary, const bool slot_on_left,
    const ParkingSlot& slot) {
  ParkingRoiBuildInput input;
  input.left_boundary = {Vec2d(0.0, left_boundary_y), Vec2d(12.0, left_boundary_y)};
  input.right_boundary = {Vec2d(0.0, right_boundary_y), Vec2d(12.0, right_boundary_y)};
  input.slot_side_connector_boundary = connector_boundary;
  input.connection_start_index = 0U;
  input.connection_end_index = connector_boundary.size() - 1U;
  input.slot_on_left = slot_on_left;
  input.slot = slot;
  return input;
}

double CrossProd2d(const Vec2d& lhs, const Vec2d& rhs) {
  return lhs.x() * rhs.y() - lhs.y() * rhs.x();
}

void ExpectNoConsecutiveDuplicateVertices(const std::vector<Vec2d>& polygon) {
  for (std::size_t index = 1; index < polygon.size(); ++index) {
    EXPECT_GT(polygon[index - 1U].DistanceTo(polygon[index]), 1e-6)
        << "duplicate vertex at index " << index;
  }
}

}  // namespace

TEST(ParkingRoiGeometryTest, BuildPerpendicularRightTemplatesAcrossGaps) {
  std::array<double, 2U> gaps = {1.0, 3.5};
  double previous_area = 0.0;
  for (const double gap : gaps) {
    const ParkingSlot slot =
        MakeSlot(Vec2d(4.75, -gap), Vec2d(7.25, -gap), Vec2d(7.25, -gap - 5.0),
                 Vec2d(4.75, -gap - 5.0));
    const auto input = MakeStraightInput(
        3.0, 0.0, {Vec2d(0.0, -gap), Vec2d(12.0, -gap)}, false, slot);

    ParkingRoiGeometry geometry;
    std::string error;
    ASSERT_TRUE(BuildParkingRoiGeometry(input, &geometry, &error)) << error;
    ASSERT_EQ(geometry.corridor_polygon.size(), 4U);
    EXPECT_NEAR(geometry.corridor_polygon[0].y(), -gap, 1e-6);
    EXPECT_NEAR(geometry.corridor_polygon[1].y(), -gap, 1e-6);
    ASSERT_GE(geometry.connector_slice.size(), 2U);
    EXPECT_NEAR(geometry.connector_slice.front().y(), -gap, 1e-6);
    EXPECT_NEAR(geometry.connector_slice.back().y(), -gap, 1e-6);
    EXPECT_TRUE(geometry.bridge_polygon.empty());
    EXPECT_LT(geometry.xy_boundary[2], -gap - 4.9);
    EXPECT_GT(geometry.area, previous_area);
    previous_area = geometry.area;
  }
}

TEST(ParkingRoiGeometryTest, BuildPerpendicularLeftTemplateWithoutBridge) {
  const ParkingSlot slot =
      MakeSlot(Vec2d(4.5, 2.5), Vec2d(7.0, 2.5), Vec2d(7.0, 7.2), Vec2d(4.5, 7.2));
  const auto input = MakeStraightInput(
      4.0, 0.0, {Vec2d(0.0, 2.5), Vec2d(12.0, 2.5)}, true, slot);

  ParkingRoiGeometry geometry;
  std::string error;
  ASSERT_TRUE(BuildParkingRoiGeometry(input, &geometry, &error)) << error;
  ASSERT_EQ(geometry.corridor_polygon.size(), 4U);
  EXPECT_NEAR(geometry.corridor_polygon[0].y(), 0.0, 1e-6);
  EXPECT_NEAR(geometry.corridor_polygon[1].y(), 0.0, 1e-6);
  EXPECT_NEAR(geometry.corridor_polygon[2].y(), 2.5, 1e-6);
  EXPECT_NEAR(geometry.corridor_polygon[3].y(), 2.5, 1e-6);
  EXPECT_TRUE(geometry.bridge_polygon.empty());
  EXPECT_GT(geometry.xy_boundary[3], 7.1);
}

TEST(ParkingRoiGeometryTest, NormalizesRepeatedBoundarySamplesBeforeUnion) {
  const ParkingSlot slot =
      MakeSlot(Vec2d(4.75, -1.0), Vec2d(7.25, -1.0), Vec2d(7.25, -6.0),
               Vec2d(4.75, -6.0));
  ParkingRoiBuildInput input;
  input.left_boundary = {Vec2d(0.0, 3.0), Vec2d(0.0, 3.0), Vec2d(0.0, 3.0),
                         Vec2d(6.0, 3.0), Vec2d(12.0, 3.0)};
  input.right_boundary = {Vec2d(0.0, -1.0), Vec2d(0.0, -1.0), Vec2d(0.0, -1.0),
                          Vec2d(6.0, -1.0), Vec2d(12.0, -1.0)};
  input.slot_side_connector_boundary = {
      Vec2d(0.0, -1.0),
      Vec2d(0.0, -1.0),
      Vec2d(0.0, -1.0),
      Vec2d(6.0, -1.0),
      Vec2d(12.0, -1.0),
  };
  input.connection_start_index = 0U;
  input.connection_end_index = input.slot_side_connector_boundary.size() - 1U;
  input.slot_on_left = false;
  input.slot = slot;

  ParkingRoiGeometry geometry;
  std::string error;
  ASSERT_TRUE(BuildParkingRoiGeometry(input, &geometry, &error)) << error;
  ExpectNoConsecutiveDuplicateVertices(geometry.corridor_polygon);
  ExpectNoConsecutiveDuplicateVertices(geometry.slot_polygon);
  ExpectNoConsecutiveDuplicateVertices(geometry.union_polygon);
  EXPECT_GT(geometry.area, 0.0);
}

TEST(ParkingRoiGeometryTest, BuildParallelSlotTemplateWithoutBridge) {
  const ParkingSlot slot =
      MakeSlot(Vec2d(2.0, -1.5), Vec2d(10.0, -1.5), Vec2d(10.0, -3.7),
               Vec2d(2.0, -3.7));
  const auto input = MakeStraightInput(
      3.0, 0.0, {Vec2d(0.0, -1.5), Vec2d(12.0, -1.5)}, false, slot);

  ParkingRoiGeometry geometry;
  std::string error;
  ASSERT_TRUE(BuildParkingRoiGeometry(input, &geometry, &error)) << error;
  EXPECT_TRUE(geometry.bridge_polygon.empty());
  EXPECT_GT(slot.corners.left_top.DistanceTo(slot.corners.right_top),
            slot.corners.left_top.DistanceTo(slot.corners.left_down));
  EXPECT_GT(geometry.area, 0.0);
  EXPECT_LT(geometry.xy_boundary[2], -3.6);
}

TEST(ParkingRoiGeometryTest, BuildParallelLeftSlotTemplateWithoutBridge) {
  const ParkingSlot slot =
      MakeSlot(Vec2d(2.0, 1.5), Vec2d(10.0, 1.5), Vec2d(10.0, 3.7),
               Vec2d(2.0, 3.7));
  const auto input = MakeStraightInput(
      3.0, 0.0, {Vec2d(0.0, 1.5), Vec2d(12.0, 1.5)}, true, slot);

  ParkingRoiGeometry geometry;
  std::string error;
  ASSERT_TRUE(BuildParkingRoiGeometry(input, &geometry, &error)) << error;
  EXPECT_TRUE(geometry.bridge_polygon.empty());
  EXPECT_GT(geometry.area, 0.0);
  EXPECT_GT(geometry.xy_boundary[3], 3.6);
  ASSERT_GE(geometry.connector_slice.size(), 2U);
  EXPECT_NEAR(geometry.connector_slice.front().y(), 1.5, 1e-6);
  EXPECT_NEAR(geometry.connector_slice.back().y(), 1.5, 1e-6);
}

TEST(ParkingRoiGeometryTest, BuildAngledRightBridgeParallelToOpeningAcrossGap) {
  const ParkingSlot slot =
      MakeSlot(Vec2d(4.0, -1.0), Vec2d(8.5, -2.4), Vec2d(7.3, -6.6),
               Vec2d(2.8, -5.2));
  ParkingRoiBuildInput input;
  input.left_boundary = {Vec2d(0.0, 3.0), Vec2d(6.0, 3.0), Vec2d(12.0, 3.0)};
  input.right_boundary = {Vec2d(0.0, 0.0), Vec2d(6.0, 0.0), Vec2d(12.0, 0.0)};
  input.slot_side_connector_boundary = {Vec2d(0.0, 1.08), Vec2d(6.0, -0.79),
                                        Vec2d(12.0, -2.65)};
  input.connection_start_index = 0U;
  input.connection_end_index = 2U;
  input.slot_on_left = false;
  input.slot = slot;

  ParkingRoiGeometry geometry;
  std::string error;
  ASSERT_TRUE(BuildParkingRoiGeometry(input, &geometry, &error)) << error;
  ASSERT_GE(geometry.connector_slice.size(), 2U);
  const Vec2d opening_vec = slot.corners.right_top - slot.corners.left_top;
  const Vec2d connector_vec =
      geometry.connector_slice.back() - geometry.connector_slice.front();
  EXPECT_NEAR(CrossProd2d(opening_vec, connector_vec), 0.0, 1e-2);
  EXPECT_GT(geometry.connector_slice.front().DistanceTo(slot.corners.left_top),
            0.5);
  EXPECT_GT(geometry.connector_slice.back().DistanceTo(slot.corners.right_top),
            0.5);
  EXPECT_GT(geometry.bridge_polygon.size(), 3U);
  EXPECT_GT(geometry.area, 0.0);
}

TEST(ParkingRoiGeometryTest, BuildAngledLeftBridgeParallelToOpeningAcrossGap) {
  const ParkingSlot slot =
      MakeSlot(Vec2d(3.5, 1.0), Vec2d(8.0, 2.4), Vec2d(6.8, 6.6),
               Vec2d(2.3, 5.2));
  ParkingRoiBuildInput input;
  input.left_boundary = {Vec2d(0.0, 3.0), Vec2d(6.0, 3.0), Vec2d(12.0, 3.0)};
  input.right_boundary = {Vec2d(0.0, 0.0), Vec2d(6.0, 0.0), Vec2d(12.0, 0.0)};
  input.slot_side_connector_boundary = {Vec2d(0.0, -1.08), Vec2d(6.0, 0.79),
                                        Vec2d(12.0, 2.65)};
  input.connection_start_index = 0U;
  input.connection_end_index = 2U;
  input.slot_on_left = true;
  input.slot = slot;

  ParkingRoiGeometry geometry;
  std::string error;
  ASSERT_TRUE(BuildParkingRoiGeometry(input, &geometry, &error)) << error;
  ASSERT_GE(geometry.connector_slice.size(), 2U);
  const Vec2d opening_vec = slot.corners.right_top - slot.corners.left_top;
  const Vec2d connector_vec =
      geometry.connector_slice.back() - geometry.connector_slice.front();
  EXPECT_NEAR(CrossProd2d(opening_vec, connector_vec), 0.0, 1e-2);
  EXPECT_GT(geometry.connector_slice.front().DistanceTo(slot.corners.left_top),
            0.5);
  EXPECT_GT(geometry.connector_slice.back().DistanceTo(slot.corners.right_top),
            0.5);
  EXPECT_GT(geometry.bridge_polygon.size(), 3U);
  EXPECT_GT(geometry.xy_boundary[3], 6.5);
}

TEST(ParkingRoiGeometryTest, KeepOpeningAttachmentWhenConnectorEndpointsMatch) {
  const ParkingSlot slot =
      MakeSlot(Vec2d(4.0, -1.0), Vec2d(8.5, -2.4), Vec2d(7.3, -6.6),
               Vec2d(2.8, -5.2));
  ParkingRoiBuildInput input;
  input.left_boundary = {Vec2d(0.0, 3.0), Vec2d(6.0, 3.0), Vec2d(12.0, 3.0)};
  input.right_boundary = {Vec2d(0.0, 0.0), Vec2d(6.0, 0.0), Vec2d(12.0, 0.0)};
  input.slot_side_connector_boundary = {
      slot.corners.left_top,
      Vec2d(6.25, -0.8),
      slot.corners.right_top,
  };
  input.connection_start_index = 0U;
  input.connection_end_index = 2U;
  input.slot_on_left = false;
  input.slot = slot;

  ParkingRoiGeometry geometry;
  std::string error;
  ASSERT_TRUE(BuildParkingRoiGeometry(input, &geometry, &error)) << error;
  EXPECT_GE(geometry.bridge_polygon.size(), 3U);
  EXPECT_GE(geometry.attachment_polygon.size(), 3U);
  EXPECT_NE(geometry.bridge_polygon, geometry.attachment_polygon);
  const auto envelope = BuildParkingEnvelopePolygon(geometry);
  ASSERT_GE(envelope.size(), 3U);
  const Vec2d opening_buffer_point(6.25, -0.8);
  EXPECT_FALSE(apollo::common::math::Polygon2d(geometry.slot_polygon).IsPointIn(
      opening_buffer_point));
  EXPECT_TRUE(apollo::common::math::Polygon2d(envelope).IsPointIn(
      opening_buffer_point));
}

TEST(ParkingRoiGeometryTest,
     ExpandsUnionPolygonToIncludeCurrentVehicleWarmStartPatch) {
  const ParkingSlot slot =
      MakeSlot(Vec2d(4.75, 0.0), Vec2d(7.25, 0.0), Vec2d(7.25, 5.0),
               Vec2d(4.75, 5.0));
  const auto input = MakeStraightInput(
      -3.0, 0.0, {Vec2d(0.0, 0.0), Vec2d(12.0, 0.0)}, false, slot);

  ParkingRoiGeometry geometry;
  std::string error;
  ASSERT_TRUE(BuildParkingRoiGeometry(input, &geometry, &error)) << error;

  apollo::common::VehicleParam vehicle_param;
  vehicle_param.set_length(4.6);
  vehicle_param.set_width(2.11);
  vehicle_param.set_back_edge_to_center(0.9725927427528887);
  vehicle_param.set_front_edge_to_center(3.627407257247111);

  const double ego_length = vehicle_param.length();
  const double shift_distance =
      0.5 * ego_length - vehicle_param.back_edge_to_center();
  const apollo::common::math::Box2d ego_box(
      Vec2d(6.0 + shift_distance * std::cos(0.0), 0.0), 0.0, ego_length,
      vehicle_param.width());
  auto buffered_ego_box = ego_box;
  buffered_ego_box.LongitudinalExtend(0.2);
  buffered_ego_box.LateralExtend(0.2);
  const apollo::common::math::Box2d forward_box(
      Vec2d(6.5 + shift_distance * std::cos(0.0), 0.0), 0.0, ego_length,
      vehicle_param.width());
  auto buffered_forward_box = forward_box;
  buffered_forward_box.LongitudinalExtend(0.2);
  buffered_forward_box.LateralExtend(0.2);
  const apollo::common::math::Box2d reverse_box(
      Vec2d(5.5 + shift_distance * std::cos(0.0), 0.0), 0.0, ego_length,
      vehicle_param.width());
  auto buffered_reverse_box = reverse_box;
  buffered_reverse_box.LongitudinalExtend(0.2);
  buffered_reverse_box.LateralExtend(0.2);
  const auto has_boundary_overlap = [&](const apollo::common::math::Box2d& box) {
    return std::any_of(
        geometry.boundary_segments.begin(), geometry.boundary_segments.end(),
        [&](const std::vector<Vec2d>& obstacle_vertices) {
          if (obstacle_vertices.size() < 2U) {
            return false;
          }
          const apollo::common::math::LineSegment2d segment(
              obstacle_vertices.front(), obstacle_vertices.back());
          return box.HasOverlap(segment);
        });
  };
  EXPECT_FALSE(
      apollo::common::math::Polygon2d(geometry.union_polygon).Contains(
          apollo::common::math::Polygon2d(buffered_ego_box)));

  ASSERT_TRUE(ExpandParkingRoiToIncludeVehicleFootprint(
      Vec2d(6.0, 0.0), 0.0, vehicle_param, 0.1, 0.5, &geometry, &error))
      << error;
  EXPECT_TRUE(
      apollo::common::math::Polygon2d(geometry.union_polygon).Contains(
          apollo::common::math::Polygon2d(buffered_ego_box)));
  EXPECT_TRUE(
      apollo::common::math::Polygon2d(geometry.union_polygon).Contains(
          apollo::common::math::Polygon2d(buffered_forward_box)));
  EXPECT_TRUE(
      apollo::common::math::Polygon2d(geometry.union_polygon).Contains(
          apollo::common::math::Polygon2d(buffered_reverse_box)));
  EXPECT_FALSE(has_boundary_overlap(ego_box));
  EXPECT_FALSE(has_boundary_overlap(forward_box));
  EXPECT_FALSE(has_boundary_overlap(reverse_box));
}

TEST(ParkingRoiGeometryTest,
     ExpandsUnionPolygonToIncludeDisconnectedVehicleWarmStartPatch) {
  const ParkingSlot slot =
      MakeSlot(Vec2d(4.75, 0.0), Vec2d(7.25, 0.0), Vec2d(7.25, 5.0),
               Vec2d(4.75, 5.0));
  const auto input = MakeStraightInput(
      -3.0, 0.0, {Vec2d(0.0, 0.0), Vec2d(12.0, 0.0)}, false, slot);

  ParkingRoiGeometry geometry;
  std::string error;
  ASSERT_TRUE(BuildParkingRoiGeometry(input, &geometry, &error)) << error;

  apollo::common::VehicleParam vehicle_param;
  vehicle_param.set_length(4.6);
  vehicle_param.set_width(2.11);
  vehicle_param.set_back_edge_to_center(0.9725927427528887);
  vehicle_param.set_front_edge_to_center(3.627407257247111);

  const double ego_length = vehicle_param.length();
  const double shift_distance =
      0.5 * ego_length - vehicle_param.back_edge_to_center();
  const double start_pose_buffer = 0.1;
  const Vec2d vehicle_position(geometry.xy_boundary[0] - 3.5, 0.0);
  const apollo::common::math::Box2d ego_box(
      Vec2d(vehicle_position.x() + shift_distance * std::cos(0.0),
            vehicle_position.y() + shift_distance * std::sin(0.0)),
      0.0, ego_length, vehicle_param.width());
  auto buffered_ego_box = ego_box;
  buffered_ego_box.LongitudinalExtend(start_pose_buffer);
  buffered_ego_box.LateralExtend(start_pose_buffer);

  EXPECT_FALSE(
      apollo::common::math::Polygon2d(geometry.union_polygon).Contains(
          apollo::common::math::Polygon2d(buffered_ego_box)));

  ASSERT_TRUE(ExpandParkingRoiToIncludeVehicleFootprint(
      vehicle_position, 0.0, vehicle_param, start_pose_buffer, 0.5, &geometry,
      &error))
      << error;
  EXPECT_TRUE(
      apollo::common::math::Polygon2d(geometry.union_polygon).Contains(
          apollo::common::math::Polygon2d(buffered_ego_box)));
  EXPECT_LT(geometry.xy_boundary[0], vehicle_position.x());
}

TEST(ParkingRoiGeometryTest, StartGoalTemplateKeepsDirectParkingRoiTight) {
  ParkingRoiGeometry geometry;
  geometry.union_polygon = {Vec2d(-20.0, -30.0), Vec2d(30.0, -30.0),
                            Vec2d(30.0, 10.0), Vec2d(-20.0, 10.0)};
  geometry.slot_polygon = {Vec2d(-1.4, 0.0), Vec2d(1.4, 0.0),
                           Vec2d(1.4, -6.0), Vec2d(-1.4, -6.0)};
  geometry.xy_boundary = {-20.0, 30.0, -30.0, 10.0};

  apollo::common::VehicleParam vehicle_param;
  vehicle_param.set_length(4.933);
  vehicle_param.set_width(2.11);
  vehicle_param.set_back_edge_to_center(1.043);
  vehicle_param.set_front_edge_to_center(3.89);

  std::string error;
  ParkingSlot slot;
  slot.width = 2.8;
  slot.depth = 6.0;
  slot.heading = -M_PI_2;
  slot.opening_center = Vec2d(0.0, 0.0);
  slot.rear_center = Vec2d(0.0, -6.0);
  slot.corners.left_top = Vec2d(-1.4, 0.0);
  slot.corners.right_top = Vec2d(1.4, 0.0);
  slot.corners.left_down = Vec2d(-1.4, -6.0);
  slot.corners.right_down = Vec2d(1.4, -6.0);
  ASSERT_TRUE(ApplyStartGoalParkingRoiTemplate(
      Vec2d(4.0, 8.0), 0.0, slot, {0.0, -3.8, M_PI_2, 0.0},
      vehicle_param, 1.0, &geometry, &error))
      << error;

  const apollo::common::math::Polygon2d roi_polygon(geometry.union_polygon);
  EXPECT_GE(geometry.union_polygon.size(), 6U);
  EXPECT_TRUE(roi_polygon.IsPointIn(Vec2d(4.0, 8.0)));
  EXPECT_TRUE(roi_polygon.IsPointIn(Vec2d(0.0, -3.8)));
  const apollo::common::math::Box2d ego_box(
      Vec2d(4.0 + (0.5 * vehicle_param.length() -
                   vehicle_param.back_edge_to_center()),
            8.0),
      0.0, vehicle_param.length(), vehicle_param.width());
  const apollo::common::math::Box2d goal_box(
      Vec2d(0.0, -3.8 + (0.5 * vehicle_param.length() -
                         vehicle_param.back_edge_to_center())),
      M_PI_2, vehicle_param.length(), vehicle_param.width());
  EXPECT_TRUE(roi_polygon.Contains(apollo::common::math::Polygon2d(ego_box)));
  EXPECT_TRUE(roi_polygon.Contains(apollo::common::math::Polygon2d(goal_box)));
  EXPECT_LT(geometry.xy_boundary[1] - geometry.xy_boundary[0], 25.0);
  EXPECT_LT(geometry.xy_boundary[3] - geometry.xy_boundary[2], 30.0);
  EXPECT_FALSE(roi_polygon.IsPointIn(Vec2d(13.0, -3.0)));
}

TEST(ParkingRoiGeometryTest, StartGoalTemplateKeepsNeighborSlotAreaOutside) {
  ParkingRoiGeometry geometry;
  geometry.union_polygon = {Vec2d(-20.0, -30.0), Vec2d(30.0, -30.0),
                            Vec2d(30.0, 10.0), Vec2d(-20.0, 10.0)};
  geometry.slot_polygon = {Vec2d(-1.5, 0.0), Vec2d(1.5, 0.0),
                           Vec2d(1.5, -6.0), Vec2d(-1.5, -6.0)};
  geometry.xy_boundary = {-20.0, 30.0, -30.0, 10.0};

  apollo::common::VehicleParam vehicle_param;
  vehicle_param.set_length(4.933);
  vehicle_param.set_width(2.11);
  vehicle_param.set_back_edge_to_center(1.043);
  vehicle_param.set_front_edge_to_center(3.89);

  ParkingSlot slot;
  slot.width = 3.0;
  slot.depth = 6.0;
  slot.heading = -M_PI_2;
  slot.opening_center = Vec2d(0.0, 0.0);
  slot.rear_center = Vec2d(0.0, -6.0);
  slot.corners.left_top = Vec2d(-1.5, 0.0);
  slot.corners.right_top = Vec2d(1.5, 0.0);
  slot.corners.left_down = Vec2d(-1.5, -6.0);
  slot.corners.right_down = Vec2d(1.5, -6.0);

  std::string error;
  ASSERT_TRUE(ApplyStartGoalParkingRoiTemplate(
      Vec2d(4.0, 8.0), 0.0, slot, {0.0, -3.8, M_PI_2, 0.0},
      vehicle_param, 0.2, &geometry, &error))
      << error;

  const apollo::common::math::Polygon2d roi_polygon(geometry.union_polygon);
  EXPECT_TRUE(roi_polygon.IsPointIn(Vec2d(4.0, 1.0)));
  EXPECT_TRUE(roi_polygon.IsPointIn(Vec2d(0.0, -3.8)));
  EXPECT_TRUE(roi_polygon.IsPointIn(Vec2d(1.39, -3.0)));
  EXPECT_TRUE(roi_polygon.IsPointIn(Vec2d(-1.39, -3.0)));
  EXPECT_FALSE(roi_polygon.IsPointIn(Vec2d(1.41, -3.0)));
  EXPECT_FALSE(roi_polygon.IsPointIn(Vec2d(-1.41, -3.0)));
  EXPECT_FALSE(roi_polygon.IsPointIn(Vec2d(1.8, -3.0)));
  EXPECT_FALSE(roi_polygon.IsPointIn(Vec2d(-1.8, -3.0)));
  EXPECT_FALSE(roi_polygon.IsPointIn(Vec2d(0.0, -6.01)));
}

TEST(ParkingRoiGeometryTest, StartGoalTemplateStaysWithinMapEnvelope) {
  ParkingRoiGeometry geometry;
  geometry.union_polygon = {Vec2d(-5.0, -12.0), Vec2d(5.0, -12.0),
                            Vec2d(5.0, 10.0), Vec2d(-5.0, 10.0)};
  geometry.slot_polygon = {Vec2d(-1.5, 0.0), Vec2d(1.5, 0.0),
                           Vec2d(1.5, -6.0), Vec2d(-1.5, -6.0)};
  geometry.xy_boundary = {-5.0, 5.0, -12.0, 10.0};

  apollo::common::VehicleParam vehicle_param;
  vehicle_param.set_length(4.933);
  vehicle_param.set_width(2.11);
  vehicle_param.set_back_edge_to_center(1.043);
  vehicle_param.set_front_edge_to_center(3.89);
  vehicle_param.set_wheel_base(2.8448);

  ParkingSlot slot;
  slot.width = 3.0;
  slot.depth = 6.0;
  slot.heading = -M_PI_2;
  slot.opening_center = Vec2d(0.0, 0.0);
  slot.rear_center = Vec2d(0.0, -6.0);
  slot.corners.left_top = Vec2d(-1.5, 0.0);
  slot.corners.right_top = Vec2d(1.5, 0.0);
  slot.corners.left_down = Vec2d(-1.5, -6.0);
  slot.corners.right_down = Vec2d(1.5, -6.0);

  std::string error;
  ASSERT_TRUE(ApplyStartGoalParkingRoiTemplate(
      Vec2d(0.0, 8.0), 0.0, slot, {0.0, -3.8, M_PI_2, 0.0},
      vehicle_param, 0.2, &geometry, &error))
      << error;

  const apollo::common::math::Polygon2d map_polygon(
      {Vec2d(-5.0, -12.0), Vec2d(5.0, -12.0), Vec2d(5.0, 10.0),
       Vec2d(-5.0, 10.0)});
  const apollo::common::math::Polygon2d roi_polygon(geometry.union_polygon);
  EXPECT_TRUE(map_polygon.Contains(roi_polygon));
  EXPECT_LE(geometry.xy_boundary[1], 5.0 + 1e-6);
  EXPECT_GE(geometry.xy_boundary[0], -5.0 - 1e-6);
}

TEST(ParkingRoiGeometryTest,
     StartGoalTemplateIgnoresMapClipThatExcludesRequiredFootprints) {
  ParkingRoiGeometry geometry;
  geometry.union_polygon = {Vec2d(-0.2, -0.2), Vec2d(0.2, -0.2),
                            Vec2d(0.2, 0.2), Vec2d(-0.2, 0.2)};
  geometry.slot_polygon = {Vec2d(-1.5, 0.0), Vec2d(1.5, 0.0),
                           Vec2d(1.5, -6.0), Vec2d(-1.5, -6.0)};
  geometry.xy_boundary = {-0.2, 0.2, -0.2, 0.2};

  apollo::common::VehicleParam vehicle_param;
  vehicle_param.set_length(4.933);
  vehicle_param.set_width(2.11);
  vehicle_param.set_back_edge_to_center(1.043);
  vehicle_param.set_front_edge_to_center(3.89);
  vehicle_param.set_wheel_base(2.8448);

  ParkingSlot slot;
  slot.width = 3.0;
  slot.depth = 6.0;
  slot.heading = -M_PI_2;
  slot.opening_center = Vec2d(0.0, 0.0);
  slot.rear_center = Vec2d(0.0, -6.0);
  slot.corners.left_top = Vec2d(-1.5, 0.0);
  slot.corners.right_top = Vec2d(1.5, 0.0);
  slot.corners.left_down = Vec2d(-1.5, -6.0);
  slot.corners.right_down = Vec2d(1.5, -6.0);

  std::string error;
  ASSERT_TRUE(ApplyStartGoalParkingRoiTemplate(
      Vec2d(4.0, 8.0), 0.0, slot, {0.0, -3.8, M_PI_2, 0.0},
      vehicle_param, 0.2, &geometry, &error))
      << error;

  const apollo::common::math::Polygon2d roi_polygon(geometry.union_polygon);
  EXPECT_TRUE(roi_polygon.IsPointIn(Vec2d(4.0, 8.0)));
  EXPECT_TRUE(roi_polygon.IsPointIn(Vec2d(0.0, -3.8)));
  EXPECT_GT(geometry.xy_boundary[1], 0.2);
}

}  // namespace parking
}  // namespace planning
}  // namespace apollo
