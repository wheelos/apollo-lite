/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *****************************************************************************/

#include <memory>
#include <vector>

#include "modules/common_msgs/map_msgs/map_parking_space.pb.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/open_space/parking/parking_slot.h"
#include "modules/planning/open_space/parking/parking_slot_provider.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {
namespace parking {

using apollo::common::math::Vec2d;
using apollo::hdmap::MapPathPoint;
using apollo::hdmap::ParkingSpace;
using apollo::hdmap::ParkingSpaceInfo;
using apollo::hdmap::Path;

namespace {

ParkingSpace MakeParkingSpace(const std::string& id,
                              const std::vector<Vec2d>& points,
                              const double heading) {
  ParkingSpace parking_space;
  parking_space.mutable_id()->set_id(id);
  for (const auto& point : points) {
    auto* polygon_point = parking_space.mutable_polygon()->add_point();
    polygon_point->set_x(point.x());
    polygon_point->set_y(point.y());
  }
  parking_space.set_heading(heading);
  return parking_space;
}

Path MakeStraightPath() {
  std::vector<MapPathPoint> path_points;
  path_points.emplace_back(Vec2d(0.0, 0.0), 0.0);
  path_points.emplace_back(Vec2d(10.0, 0.0), 0.0);
  return Path(std::move(path_points));
}

}  // namespace

TEST(ParkingSlotTest, NormalizePerpendicularSlot) {
  ParkingSlot slot;
  std::string error;
  const std::vector<Vec2d> polygon = {
      Vec2d(2.0, -6.0), Vec2d(0.0, 0.0), Vec2d(2.0, 0.0), Vec2d(0.0, -6.0)};
  EXPECT_TRUE(NormalizeParkingSlot(polygon, "slot-1", -M_PI_2, 0.0, -3.0, &slot,
                                   &error))
      << error;
  EXPECT_EQ(slot.type, ParkingSlotType::kPerpendicular);
  EXPECT_FALSE(slot.on_left_lane_side);
  EXPECT_NEAR(slot.width, 2.0, 1e-6);
  EXPECT_NEAR(slot.depth, 6.0, 1e-6);
  EXPECT_NEAR(slot.corners.left_top.x(), 0.0, 1e-6);
  EXPECT_NEAR(slot.corners.right_top.x(), 2.0, 1e-6);
}

TEST(ParkingSlotTest, NormalizeParallelSlot) {
  ParkingSlot slot;
  std::string error;
  const std::vector<Vec2d> polygon = {
      Vec2d(0.0, 0.0), Vec2d(6.0, 0.0), Vec2d(6.0, -2.5), Vec2d(0.0, -2.5)};
  EXPECT_TRUE(NormalizeParkingSlot(polygon, "slot-2", 0.0, 0.0, -2.0, &slot,
                                   &error))
      << error;
  EXPECT_EQ(slot.type, ParkingSlotType::kParallel);
  EXPECT_NEAR(slot.width, 6.0, 1e-6);
  EXPECT_NEAR(slot.depth, 2.5, 1e-6);
  EXPECT_NEAR(slot.heading, 0.0, 1e-6);
}

TEST(ParkingSlotProviderTest, UsesNearestEdgeAsEntranceForPerpendicularSlot) {
  const Path nearby_path = MakeStraightPath();
  const std::vector<Vec2d> polygon = {
      Vec2d(6.0, -5.0), Vec2d(6.0, -1.0), Vec2d(4.0, -1.0), Vec2d(4.0, -5.0)};
  ParkingSpace parking_space = MakeParkingSpace("slot-perp", polygon, -M_PI_2);
  auto parking_info = std::make_shared<ParkingSpaceInfo>(parking_space);

  ParkingSlotProvider provider;
  ParkingSlot slot;
  std::string error;
  ASSERT_TRUE(provider.BuildFromMap(parking_info, nearby_path, &slot, &error))
      << error;

  EXPECT_EQ(slot.type, ParkingSlotType::kPerpendicular);
  EXPECT_NEAR(slot.corners.left_top.x(), 4.0, 1e-6);
  EXPECT_NEAR(slot.corners.left_top.y(), -1.0, 1e-6);
  EXPECT_NEAR(slot.corners.right_top.x(), 6.0, 1e-6);
  EXPECT_NEAR(slot.corners.right_top.y(), -1.0, 1e-6);
  EXPECT_NEAR(slot.opening_center.x(), 5.0, 1e-6);
  EXPECT_NEAR(slot.opening_center.y(), -1.0, 1e-6);
}

TEST(ParkingSlotProviderTest, UsesNearestEdgeAsEntranceForParallelSlot) {
  const Path nearby_path = MakeStraightPath();
  const std::vector<Vec2d> polygon = {
      Vec2d(7.0, -3.0), Vec2d(7.0, -1.0), Vec2d(1.0, -1.0), Vec2d(1.0, -3.0)};
  ParkingSpace parking_space = MakeParkingSpace("slot-parallel", polygon, 0.0);
  auto parking_info = std::make_shared<ParkingSpaceInfo>(parking_space);

  ParkingSlotProvider provider;
  ParkingSlot slot;
  std::string error;
  ASSERT_TRUE(provider.BuildFromMap(parking_info, nearby_path, &slot, &error))
      << error;

  EXPECT_EQ(slot.type, ParkingSlotType::kParallel);
  EXPECT_NEAR(slot.corners.left_top.x(), 1.0, 1e-6);
  EXPECT_NEAR(slot.corners.left_top.y(), -1.0, 1e-6);
  EXPECT_NEAR(slot.corners.right_top.x(), 7.0, 1e-6);
  EXPECT_NEAR(slot.corners.right_top.y(), -1.0, 1e-6);
  EXPECT_NEAR(slot.width, 6.0, 1e-6);
  EXPECT_NEAR(slot.depth, 2.0, 1e-6);
}

TEST(ParkingSlotProviderTest, UsesNearestEdgeAsEntranceForLeftSideParallelSlot) {
  const Path nearby_path = MakeStraightPath();
  const std::vector<Vec2d> polygon = {
      Vec2d(1.0, 1.0), Vec2d(7.0, 1.0), Vec2d(7.0, 3.0), Vec2d(1.0, 3.0)};
  ParkingSpace parking_space =
      MakeParkingSpace("slot-parallel-left", polygon, 0.0);
  auto parking_info = std::make_shared<ParkingSpaceInfo>(parking_space);

  ParkingSlotProvider provider;
  ParkingSlot slot;
  std::string error;
  ASSERT_TRUE(provider.BuildFromMap(parking_info, nearby_path, &slot, &error))
      << error;

  EXPECT_EQ(slot.type, ParkingSlotType::kParallel);
  EXPECT_TRUE(slot.on_left_lane_side);
  EXPECT_NEAR(slot.corners.left_top.x(), 1.0, 1e-6);
  EXPECT_NEAR(slot.corners.left_top.y(), 1.0, 1e-6);
  EXPECT_NEAR(slot.corners.right_top.x(), 7.0, 1e-6);
  EXPECT_NEAR(slot.corners.right_top.y(), 1.0, 1e-6);
  EXPECT_NEAR(slot.opening_center.y(), 1.0, 1e-6);
  EXPECT_NEAR(slot.rear_center.y(), 3.0, 1e-6);
}

TEST(ParkingSlotProviderTest, UsesNearestEdgeAsEntranceForRightSideAngledSlot) {
  const Path nearby_path = MakeStraightPath();
  const std::vector<Vec2d> polygon = {
      Vec2d(6.8, -4.1), Vec2d(9.4, -5.9), Vec2d(6.8, -2.2), Vec2d(4.2, -0.4)};
  ParkingSpace parking_space =
      MakeParkingSpace("slot-angled-right", polygon, -0.95);
  auto parking_info = std::make_shared<ParkingSpaceInfo>(parking_space);

  ParkingSlotProvider provider;
  ParkingSlot slot;
  std::string error;
  ASSERT_TRUE(provider.BuildFromMap(parking_info, nearby_path, &slot, &error))
      << error;

  EXPECT_EQ(slot.type, ParkingSlotType::kAngled);
  EXPECT_FALSE(slot.on_left_lane_side);
  EXPECT_NEAR(slot.corners.left_top.x(), 4.2, 1e-6);
  EXPECT_NEAR(slot.corners.left_top.y(), -0.4, 1e-6);
  EXPECT_NEAR(slot.corners.right_top.x(), 6.8, 1e-6);
  EXPECT_NEAR(slot.corners.right_top.y(), -2.2, 1e-6);
  EXPECT_LT(slot.rear_center.y(), slot.opening_center.y());
}

TEST(ParkingSlotProviderTest, UsesNearestEdgeAsEntranceForLeftSideAngledSlot) {
  const Path nearby_path = MakeStraightPath();
  const std::vector<Vec2d> polygon = {
      Vec2d(6.8, 4.1), Vec2d(9.4, 5.9), Vec2d(6.8, 2.2), Vec2d(4.2, 0.4)};
  ParkingSpace parking_space =
      MakeParkingSpace("slot-angled-left", polygon, 0.95);
  auto parking_info = std::make_shared<ParkingSpaceInfo>(parking_space);

  ParkingSlotProvider provider;
  ParkingSlot slot;
  std::string error;
  ASSERT_TRUE(provider.BuildFromMap(parking_info, nearby_path, &slot, &error))
      << error;

  EXPECT_EQ(slot.type, ParkingSlotType::kAngled);
  EXPECT_TRUE(slot.on_left_lane_side);
  EXPECT_NEAR(slot.corners.left_top.x(), 4.2, 1e-6);
  EXPECT_NEAR(slot.corners.left_top.y(), 0.4, 1e-6);
  EXPECT_NEAR(slot.corners.right_top.x(), 6.8, 1e-6);
  EXPECT_NEAR(slot.corners.right_top.y(), 2.2, 1e-6);
  EXPECT_GT(slot.rear_center.y(), slot.opening_center.y());
}

TEST(ParkingSlotTest, NormalizeAngledSlot) {
  ParkingSlot slot;
  std::string error;
  const std::vector<Vec2d> polygon = {
      Vec2d(3.0, -4.0), Vec2d(1.0, -1.0), Vec2d(3.4, 0.6), Vec2d(5.4, -2.4)};
  EXPECT_TRUE(NormalizeParkingSlot(polygon, "slot-angled", -0.98, 0.0, -2.0,
                                   &slot, &error))
      << error;
  EXPECT_EQ(slot.type, ParkingSlotType::kAngled);
  EXPECT_GT(slot.width, 2.5);
  EXPECT_GT(slot.depth, 3.0);
}

TEST(ParkingSlotTest, TransformUsesOpeningCenterAndLaneHeadingOrigin) {
  ParkingSlot right_slot;
  ParkingSlot left_slot;
  std::string error;
  ASSERT_TRUE(NormalizeParkingSlot(
      {Vec2d(2.0, -6.0), Vec2d(0.0, 0.0), Vec2d(2.0, 0.0), Vec2d(0.0, -6.0)},
      "slot-right", -M_PI_2, 0.0, -3.0, &right_slot, &error))
      << error;
  ASSERT_TRUE(NormalizeParkingSlot(
      {Vec2d(0.0, 0.0), Vec2d(2.0, 0.0), Vec2d(2.0, 6.0), Vec2d(0.0, 6.0)},
      "slot-left", M_PI_2, 0.0, 3.0, &left_slot, &error))
      << error;

  const ParkingSlot transformed_right = TransformParkingSlot(
      right_slot, right_slot.opening_center, right_slot.lane_heading);
  const ParkingSlot transformed_left = TransformParkingSlot(
      left_slot, left_slot.opening_center, left_slot.lane_heading);

  EXPECT_NEAR(transformed_right.opening_center.x(), 0.0, 1e-6);
  EXPECT_NEAR(transformed_right.opening_center.y(), 0.0, 1e-6);
  EXPECT_NEAR(transformed_right.corners.left_top.y(), 0.0, 1e-6);
  EXPECT_NEAR(transformed_right.corners.right_top.y(), 0.0, 1e-6);
  EXPECT_LT(transformed_right.rear_center.y(), 0.0);

  EXPECT_NEAR(transformed_left.opening_center.x(), 0.0, 1e-6);
  EXPECT_NEAR(transformed_left.opening_center.y(), 0.0, 1e-6);
  EXPECT_NEAR(transformed_left.corners.left_top.y(), 0.0, 1e-6);
  EXPECT_NEAR(transformed_left.corners.right_top.y(), 0.0, 1e-6);
  EXPECT_GT(transformed_left.rear_center.y(), 0.0);
}

}  // namespace parking
}  // namespace planning
}  // namespace apollo
