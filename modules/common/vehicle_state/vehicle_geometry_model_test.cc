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
  VehicleState state;
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
  VehicleState rear_state;
  rear_state.set_x(10.0);
  rear_state.set_y(20.0);
  rear_state.set_heading(M_PI_2);
  rear_state.set_reference_point(REAR_AXLE_CENTER);

  math::Box2d rear_box;
  ASSERT_TRUE(geometry_model_.BuildBox(rear_state, &rear_box).ok());

  // Front axle is wheel_base (2.8) ahead of rear axle.
  // With heading = pi/2, dx = cos(pi/2)*2.8 = 0, dy = sin(pi/2)*2.8 = 2.8.
  VehicleState front_state;
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

TEST_F(VehicleGeometryModelTest, BuildBoxFromPose) {
  VehiclePose2d pose(math::Vec2d(5.0, 5.0), 0.0, REAR_AXLE_CENTER);

  math::Box2d box;
  ASSERT_TRUE(geometry_model_.BuildBox(pose, &box).ok());
  EXPECT_NEAR(box.center_x(), 6.4, 1e-6);
  EXPECT_NEAR(box.center_y(), 5.0, 1e-6);

  math::Box2d buffered_box;
  ASSERT_TRUE(geometry_model_.BuildBox(pose, 0.2, 0.5, &buffered_box).ok());
  EXPECT_NEAR(buffered_box.center_x(), 6.4, 1e-6);
  EXPECT_NEAR(buffered_box.center_y(), 5.0, 1e-6);
  EXPECT_NEAR(buffered_box.length(), 4.8 + 1.0, 1e-6);
  EXPECT_NEAR(buffered_box.width(), 2.0 + 0.4, 1e-6);
}

TEST_F(VehicleGeometryModelTest, BuildFrontRegion) {
  VehicleState state;
  state.set_x(0.0);
  state.set_y(0.0);
  state.set_heading(0.0);
  state.set_reference_point(REAR_AXLE_CENTER);

  math::Box2d front_region;
  ASSERT_TRUE(
      geometry_model_.BuildFrontRegion(state, 50.0, 0.1, &front_region).ok());

  // center is at 1.4 + 50.0 / 2 = 26.4
  EXPECT_NEAR(front_region.center_x(), 26.4, 1e-6);
  EXPECT_NEAR(front_region.center_y(), 0.0, 1e-6);
  // The state overload's buffer is a lateral buffer. It expands both sides,
  // while the distance threshold extends the region only forward.
  EXPECT_NEAR(front_region.length(), 4.8 + 50.0, 1e-6);
  EXPECT_NEAR(front_region.width(), 2.0 + 2.0 * 0.1, 1e-6);
}

TEST_F(VehicleGeometryModelTest, CollisionAndClearanceSemantics) {
  VehiclePose2d ego_pose(math::Vec2d(0.0, 0.0), 0.0, REAR_AXLE_CENTER);

  // Vehicle footprint: x in [-1.0, 3.8], y in [-1.0, 1.0]
  // Obstacle box at x=5.0, y=0.0, length=2.0, width=2.0 (x in [4.0, 6.0])
  math::Box2d obs_box(math::Vec2d(5.0, 0.0), 0.0, 2.0, 2.0);

  // No collision without buffer
  bool collision = true;
  ASSERT_TRUE(
      geometry_model_.CheckCollision(ego_pose, obs_box, &collision).ok());
  EXPECT_FALSE(collision);

  // Clearance is from x=3.8 to x=4.0 -> 0.2
  double clearance = 0.0;
  ASSERT_TRUE(
      geometry_model_.ComputeClearance(ego_pose, obs_box, &clearance).ok());
  EXPECT_NEAR(clearance, 0.2, 1e-6);

  // Collision with longitudinal_buffer >= 0.2
  ASSERT_TRUE(
      geometry_model_.CheckCollision(ego_pose, obs_box, &collision, 0.0, 0.25)
          .ok());
  EXPECT_TRUE(collision);
}

TEST_F(VehicleGeometryModelTest, RejectsUnknownReferencePointAndInvalidBuffer) {
  VehiclePose2d invalid_pose(math::Vec2d(0.0, 0.0), 0.0,
                             static_cast<ReferencePoint>(99));
  math::Box2d box;
  EXPECT_FALSE(geometry_model_.BuildBox(invalid_pose, &box).ok());

  VehiclePose2d valid_pose(math::Vec2d(0.0, 0.0), 0.0, REAR_AXLE_CENTER);
  EXPECT_FALSE(geometry_model_.BuildBox(valid_pose, -0.1, 0.0, &box).ok());
}

}  // namespace common
}  // namespace apollo
