// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/common/vehicle_state/vehicle_geometry_model.h"

#include <cmath>

#include "gtest/gtest.h"

namespace apollo {
namespace common {

class VehicleGeometryModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_.mutable_vehicle_param()->set_wheel_base(2.8);
    config_.mutable_vehicle_param()->set_length(4.8);
    config_.mutable_vehicle_param()->set_width(2.0);
    config_.mutable_vehicle_param()->set_front_edge_to_center(3.8);
    config_.mutable_vehicle_param()->set_back_edge_to_center(1.0);
    config_.mutable_vehicle_param()->set_left_edge_to_center(1.0);
    config_.mutable_vehicle_param()->set_right_edge_to_center(1.0);
    geometry_model_ = VehicleGeometryModel(VehicleDescription(config_));
  }

  VehicleConfig config_;
  VehicleGeometryModel geometry_model_;
};

TEST_F(VehicleGeometryModelTest, BuildBoxFromRearAxle) {
  ReferenceState state;
  state.set_x(10.0);
  state.set_y(20.0);
  state.set_heading(0.0);
  state.set_reference_point(REAR_AXLE_CENTER);

  math::Box2d box;
  ASSERT_TRUE(geometry_model_.BuildBox(state, &box).ok());

  // (front_edge - back_edge) / 2 = (3.8 - 1.0) / 2 = 1.4
  // heading = 0: x-shift = 1.4, y-shift = 0
  EXPECT_NEAR(box.center_x(), 11.4, 1e-6);
  EXPECT_NEAR(box.center_y(), 20.0, 1e-6);
  EXPECT_NEAR(box.length(), 4.8, 1e-6);
  EXPECT_NEAR(box.width(), 2.0, 1e-6);
}

TEST_F(VehicleGeometryModelTest, ConsistentBoxAcrossReferencePoints) {
  // Rear axle at (10, 20) with heading = pi / 2 (pointing North)
  ReferenceState rear_state;
  rear_state.set_x(10.0);
  rear_state.set_y(20.0);
  rear_state.set_heading(M_PI_2);
  rear_state.set_reference_point(REAR_AXLE_CENTER);

  math::Box2d rear_box;
  ASSERT_TRUE(geometry_model_.BuildBox(rear_state, &rear_box).ok());

  // Front axle is wheel_base (2.8) ahead of rear axle.
  // With heading = pi/2, dx = cos(pi/2)*2.8 = 0, dy = sin(pi/2)*2.8 = 2.8.
  ReferenceState front_state;
  front_state.set_x(10.0);
  front_state.set_y(22.8);
  front_state.set_heading(M_PI_2);
  front_state.set_reference_point(FRONT_AXLE_CENTER);

  math::Box2d front_box;
  ASSERT_TRUE(geometry_model_.BuildBox(front_state, &front_box).ok());

  EXPECT_NEAR(rear_box.center_x(), front_box.center_x(), 1e-6);
  EXPECT_NEAR(rear_box.center_y(), front_box.center_y(), 1e-6);
  EXPECT_NEAR(rear_box.heading(), front_box.heading(), 1e-6);
}

TEST_F(VehicleGeometryModelTest, BuildBoxFromPathPoint) {
  PathPoint point;
  point.set_x(5.0);
  point.set_y(5.0);
  point.set_theta(0.0);

  math::Box2d box;
  ASSERT_TRUE(geometry_model_.BuildBox(point, REAR_AXLE_CENTER, &box).ok());
  EXPECT_NEAR(box.center_x(), 6.4, 1e-6);
  EXPECT_NEAR(box.center_y(), 5.0, 1e-6);

  // Test direct return and buffer overload
  math::Box2d direct_box = geometry_model_.BuildBox(point);
  EXPECT_NEAR(direct_box.center_x(), 6.4, 1e-6);
  EXPECT_NEAR(direct_box.center_y(), 5.0, 1e-6);
  EXPECT_NEAR(direct_box.length(), 4.8, 1e-6);
  EXPECT_NEAR(direct_box.width(), 2.0, 1e-6);

  math::Box2d buffered_box = geometry_model_.BuildBox(point, 0.2, 0.5);
  EXPECT_NEAR(buffered_box.center_x(), 6.4, 1e-6);
  EXPECT_NEAR(buffered_box.center_y(), 5.0, 1e-6);
  EXPECT_NEAR(buffered_box.length(), 4.8 + 1.0, 1e-6);
  EXPECT_NEAR(buffered_box.width(), 2.0 + 0.4, 1e-6);
}

TEST_F(VehicleGeometryModelTest, BuildFrontRegion) {
  ReferenceState state;
  state.set_x(0.0);
  state.set_y(0.0);
  state.set_heading(0.0);
  state.set_reference_point(REAR_AXLE_CENTER);

  math::Box2d front_region;
  ASSERT_TRUE(geometry_model_.BuildFrontRegion(state, 50.0, 0.1, &front_region).ok());

  // center is at 1.4 + 50.0 / 2 = 26.4
  EXPECT_NEAR(front_region.center_x(), 26.4, 1e-6);
  EXPECT_NEAR(front_region.center_y(), 0.0, 1e-6);
  EXPECT_NEAR(front_region.length(), 4.8 + 0.1 + 50.0, 1e-6);
  EXPECT_NEAR(front_region.width(), 2.0 + 0.1, 1e-6);
}

TEST_F(VehicleGeometryModelTest, StopAlignmentSemantics) {
  // front_edge_to_center = 3.8, back_edge_to_center = 1.0, wheel_base = 2.8
  // Forward stop at 100.0 with margin 1.0
  // Target reference s should be 100.0 - 1.0 - 3.8 = 95.2
  double forward_s = geometry_model_.ComputeStopReferenceS(
      100.0, 1.0, TravelDirection::TRAVEL_DIRECTION_FORWARD, REAR_AXLE_CENTER);
  EXPECT_NEAR(forward_s, 95.2, 1e-6);

  // Reverse stop at 20.0 with margin 0.5
  // Target reference s should be 20.0 + 0.5 + 1.0 = 21.5
  double reverse_s = geometry_model_.ComputeStopReferenceS(
      20.0, 0.5, TravelDirection::TRAVEL_DIRECTION_REVERSE, REAR_AXLE_CENTER);
  EXPECT_NEAR(reverse_s, 21.5, 1e-6);

  // Front axle forward stop: front edge distance from front axle is 3.8 - 2.8 = 1.0
  // Target reference s should be 100.0 - 1.0 - 1.0 = 98.0
  double front_axle_s = geometry_model_.ComputeStopReferenceS(
      100.0, 1.0, TravelDirection::TRAVEL_DIRECTION_FORWARD, FRONT_AXLE_CENTER);
  EXPECT_NEAR(front_axle_s, 98.0, 1e-6);
}

TEST_F(VehicleGeometryModelTest, OccupancySemantics) {
  // ref_s = 50.0, rear axle
  // s_min = 50.0 - 1.0 = 49.0, s_max = 50.0 + 3.8 = 53.8
  auto s_range = geometry_model_.GetOccupancySRange(50.0, REAR_AXLE_CENTER);
  EXPECT_NEAR(s_range.first, 49.0, 1e-6);
  EXPECT_NEAR(s_range.second, 53.8, 1e-6);

  // ref_l = 2.0, width = 2.0 (left = 1.0, right = 1.0)
  // l_min = 2.0 - 1.0 = 1.0, l_max = 2.0 + 1.0 = 3.0
  auto l_range = geometry_model_.GetOccupancyLRange(2.0);
  EXPECT_NEAR(l_range.first, 1.0, 1e-6);
  EXPECT_NEAR(l_range.second, 3.0, 1e-6);
}

TEST_F(VehicleGeometryModelTest, CollisionAndClearanceSemantics) {
  PathPoint ego_pt;
  ego_pt.set_x(0.0);
  ego_pt.set_y(0.0);
  ego_pt.set_theta(0.0);

  // Vehicle footprint: x in [-1.0, 3.8], y in [-1.0, 1.0]
  // Obstacle box at x=5.0, y=0.0, length=2.0, width=2.0 (x in [4.0, 6.0])
  math::Box2d obs_box(math::Vec2d(5.0, 0.0), 0.0, 2.0, 2.0);

  // No collision without buffer
  EXPECT_FALSE(geometry_model_.CheckCollision(ego_pt, obs_box));

  // Clearance is from x=3.8 to x=4.0 -> 0.2
  EXPECT_NEAR(geometry_model_.ComputeClearance(ego_pt, obs_box), 0.2, 1e-6);

  // Collision with longitudinal_buffer >= 0.2
  EXPECT_TRUE(geometry_model_.CheckCollision(ego_pt, obs_box, 0.0, 0.25));
}
}

}  // namespace common
}  // namespace apollo
