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

#include "modules/common/vehicle_state/reference_point_transformer.h"

#include <cmath>

#include "gtest/gtest.h"

namespace apollo {
namespace common {

TEST(ReferencePointTransformerTest, ResolvesFrontAxleFromRearAxleState) {
  VehicleConfig config;
  config.mutable_vehicle_param()->set_wheel_base(2.8);
  ReferencePointTransformer transformer{VehicleDescription(config)};

  VehicleState state;
  state.set_x(1.0);
  state.set_y(2.0);
  state.set_heading(0.0);
  state.set_linear_velocity(5.0);
  state.set_reference_point(REAR_AXLE_CENTER);

  VehicleState front_axle;
  ASSERT_TRUE(
      transformer.TransformState(state, FRONT_AXLE_CENTER, &front_axle).ok());
  EXPECT_DOUBLE_EQ(front_axle.x(), 3.8);
  EXPECT_DOUBLE_EQ(front_axle.y(), 2.0);
  EXPECT_EQ(front_axle.reference_point(), FRONT_AXLE_CENTER);
}

TEST(ReferencePointTransformerTest, PoseOrientationMatchesHeading) {
  VehicleConfig config;
  config.mutable_vehicle_param()->set_wheel_base(2.8);
  ReferencePointTransformer transformer{VehicleDescription(config)};

  const double heading = M_PI / 4.0;  // 45 degrees
  VehicleState state_heading;
  state_heading.set_x(0.0);
  state_heading.set_y(0.0);
  state_heading.set_heading(heading);
  state_heading.set_reference_point(REAR_AXLE_CENTER);

  VehicleState front_heading;
  ASSERT_TRUE(
      transformer
          .TransformState(state_heading, FRONT_AXLE_CENTER, &front_heading)
          .ok());

  VehicleState state_pose;
  state_pose.set_x(0.0);
  state_pose.set_y(0.0);
  state_pose.set_heading(heading);
  state_pose.set_reference_point(REAR_AXLE_CENTER);
  state_pose.mutable_pose()->mutable_orientation()->set_qw(
      std::cos(heading / 2.0));
  state_pose.mutable_pose()->mutable_orientation()->set_qx(0.0);
  state_pose.mutable_pose()->mutable_orientation()->set_qy(0.0);
  state_pose.mutable_pose()->mutable_orientation()->set_qz(
      std::sin(heading / 2.0));

  VehicleState front_pose;
  ASSERT_TRUE(
      transformer.TransformState(state_pose, FRONT_AXLE_CENTER, &front_pose)
          .ok());

  EXPECT_NEAR(front_heading.x(), front_pose.x(), 1e-6);
  EXPECT_NEAR(front_heading.y(), front_pose.y(), 1e-6);
  EXPECT_NEAR(front_pose.x(), 2.8 * std::cos(heading), 1e-6);
  EXPECT_NEAR(front_pose.y(), 2.8 * std::sin(heading), 1e-6);
}

TEST(ReferencePointTransformerTest, IncompletePoseOrientationUsesHeading) {
  VehicleConfig config;
  config.mutable_vehicle_param()->set_wheel_base(2.8);
  ReferencePointTransformer transformer{VehicleDescription(config)};

  VehicleState state;
  state.set_heading(M_PI / 4.0);
  state.set_reference_point(REAR_AXLE_CENTER);
  state.mutable_pose()->mutable_orientation()->set_qw(1.0);

  VehicleState front;
  ASSERT_TRUE(
      transformer.TransformState(state, FRONT_AXLE_CENTER, &front).ok());
  EXPECT_NEAR(front.x(), 2.8 * std::cos(M_PI / 4.0), 1e-6);
  EXPECT_NEAR(front.y(), 2.8 * std::sin(M_PI / 4.0), 1e-6);
}

TEST(ReferencePointTransformerTest, ZeroNormOrientationUsesHeading) {
  VehicleConfig config;
  config.mutable_vehicle_param()->set_wheel_base(2.8);
  ReferencePointTransformer transformer{VehicleDescription(config)};

  VehicleState state;
  state.set_heading(M_PI / 4.0);
  state.set_reference_point(REAR_AXLE_CENTER);
  auto* orientation = state.mutable_pose()->mutable_orientation();
  orientation->set_qw(0.0);
  orientation->set_qx(0.0);
  orientation->set_qy(0.0);
  orientation->set_qz(0.0);

  VehicleState front;
  ASSERT_TRUE(
      transformer.TransformState(state, FRONT_AXLE_CENTER, &front).ok());
  EXPECT_NEAR(front.x(), 2.8 * std::cos(M_PI / 4.0), 1e-6);
  EXPECT_NEAR(front.y(), 2.8 * std::sin(M_PI / 4.0), 1e-6);
}

TEST(ReferencePointTransformerTest, RoundTripTransform) {
  VehicleConfig config;
  config.mutable_vehicle_param()->set_wheel_base(2.8);

  VehicleState initial;
  initial.set_x(12.34);
  initial.set_y(56.78);
  initial.set_heading(1.23);
  initial.set_linear_velocity(10.0);
  initial.set_reference_point(REAR_AXLE_CENTER);

  VehicleState com;
  const double com_offset = 1.35;
  VehicleDescription configured_description(config, com_offset);
  ReferencePointTransformer configured_transformer(configured_description);
  ASSERT_TRUE(
      configured_transformer.TransformState(initial, CENTER_OF_MASS, &com)
          .ok());

  VehicleState restored;
  ASSERT_TRUE(
      configured_transformer.TransformState(com, REAR_AXLE_CENTER, &restored)
          .ok());

  EXPECT_NEAR(restored.x(), initial.x(), 1e-9);
  EXPECT_NEAR(restored.y(), initial.y(), 1e-9);
  EXPECT_DOUBLE_EQ(restored.heading(), initial.heading());
  EXPECT_EQ(restored.reference_point(), REAR_AXLE_CENTER);
}

TEST(ReferencePointTransformerTest, KinematicTransformWithYawRate) {
  VehicleConfig config;
  config.mutable_vehicle_param()->set_wheel_base(2.8);
  ReferencePointTransformer transformer{VehicleDescription(config)};

  VehicleState state;
  state.set_x(0.0);
  state.set_y(0.0);
  state.set_heading(0.0);
  state.set_linear_velocity(10.0);
  state.set_lateral_velocity(0.5);
  state.set_angular_velocity(0.5);  // rad/s
  state.set_linear_acceleration(1.0);
  state.set_reference_point(REAR_AXLE_CENTER);

  VehicleState front;
  ASSERT_TRUE(
      transformer.TransformState(state, FRONT_AXLE_CENTER, &front).ok());

  // linear_velocity is the body-frame longitudinal component. The front
  // axle's additional lateral velocity is not folded into this scalar.
  EXPECT_DOUBLE_EQ(front.linear_velocity(), 10.0);
  EXPECT_NEAR(front.lateral_velocity(), 1.9, 1e-6);

  // a_front = 1.0 - (0.5)^2 * 2.8 = 1.0 - 0.7 = 0.3
  EXPECT_NEAR(front.linear_acceleration(), 0.3, 1e-4);

  // omega invariant
  EXPECT_DOUBLE_EQ(front.angular_velocity(), 0.5);
}

TEST(ReferencePointTransformerTest, RejectsNullOutput) {
  VehicleConfig config;
  ReferencePointTransformer transformer{VehicleDescription(config)};
  VehicleState state;
  EXPECT_FALSE(transformer
                   .TransformState(state, REAR_AXLE_CENTER,
                                   static_cast<VehicleState*>(nullptr))
                   .ok());
}

}  // namespace common
}  // namespace apollo
