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

//  Created Date: 2026-09-16
//  Author: daohu527

#include "modules/simulation/adapter/cyber_adapter.h"

#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "modules/common/configs/config_gflags.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"

namespace apollo {
namespace simulation {

TEST(CyberAdapterTest, ConvertsControlCommandUnitsAndGear) {
  apollo::control::ControlCommand message;
  message.mutable_header()->set_timestamp_sec(12.5);
  message.mutable_header()->set_sequence_num(7);
  message.set_throttle(50.0);
  message.set_brake(25.0);
  message.set_steering_target(-50.0);
  message.set_steering_rate(20.0);
  message.set_acceleration(1.5);
  message.set_speed(4.0);
  message.set_gear_location(apollo::canbus::Chassis::GEAR_REVERSE);

  VehicleCommand command;
  CyberAdapter::FromControlCommand(message, &command, 0.6);

  EXPECT_DOUBLE_EQ(command.timestamp_sec, 12.5);
  EXPECT_EQ(command.sequence_num, 7U);
  EXPECT_DOUBLE_EQ(command.throttle, 0.5);
  EXPECT_DOUBLE_EQ(command.brake, 0.25);
  EXPECT_DOUBLE_EQ(command.front_steering_rad, -0.3);
  EXPECT_DOUBLE_EQ(command.steering_rate_radps, 0.12);
  EXPECT_DOUBLE_EQ(command.target_acceleration_mps2, 1.5);
  EXPECT_DOUBLE_EQ(command.target_speed_mps, 4.0);
  EXPECT_EQ(command.gear, VehicleCommand::Gear::GEAR_REVERSE);
  EXPECT_FALSE(command.emergency_stop);
}

TEST(CyberAdapterTest, InvalidControlCommandRequestsEmergencyBrake) {
  apollo::control::ControlCommand message;
  message.set_throttle(std::numeric_limits<double>::quiet_NaN());

  VehicleCommand command;
  CyberAdapter::FromControlCommand(message, &command);

  EXPECT_TRUE(command.emergency_stop);
  EXPECT_DOUBLE_EQ(command.throttle, 0.0);
  EXPECT_DOUBLE_EQ(command.brake, 1.0);
}

TEST(CyberAdapterTest, FeedsCommonVehicleStateFromGroundTruth) {
  const bool previous_map_reference_mode = FLAGS_enable_map_reference_unify;
  FLAGS_enable_map_reference_unify = true;

  VehicleState state;
  state.timestamp_sec = 12.5;
  state.x = 10.0;
  state.y = -3.0;
  state.z = 0.335;
  state.qw = 1.0;
  state.yaw = 0.25;
  state.linear_velocity_mps = -2.0;
  state.angular_velocity_yaw_radps = 0.4;
  state.angular_velocity_body_radps = {0.0, 0.0, 0.4};
  state.angular_velocity_world_radps = {0.0, 0.0, 0.4};
  state.linear_acceleration_mps2 = 1.5;
  state.linear_acceleration_body_mps2 = {1.5, 0.0, 0.0};
  state.current_gear = VehicleCommand::Gear::GEAR_REVERSE;

  apollo::canbus::Chassis chassis;
  apollo::localization::LocalizationEstimate localization;
  CyberAdapter::ToChassis(state, &chassis);
  CyberAdapter::ToLocalization(state, &localization);

  common::VehicleStateProvider provider;
  ASSERT_TRUE(provider.Update(localization, chassis).ok());
  const auto& common_state = provider.state();
  EXPECT_DOUBLE_EQ(chassis.speed_mps(), 2.0F);
  EXPECT_EQ(chassis.gear_location(), apollo::canbus::Chassis::GEAR_REVERSE);
  EXPECT_EQ(common_state.gear(), apollo::canbus::Chassis::GEAR_REVERSE);
  EXPECT_DOUBLE_EQ(common_state.linear_velocity(), -2.0);
  EXPECT_NEAR(common_state.angular_velocity(), 0.4, 1e-12);
  EXPECT_NEAR(common_state.kappa(), -0.2, 1e-12);
  EXPECT_NEAR(common_state.linear_acceleration(), 1.5, 1e-12);
  EXPECT_NEAR(common_state.x(), 10.0, 1e-12);
  EXPECT_NEAR(common_state.y(), -3.0, 1e-12);

  FLAGS_enable_map_reference_unify = previous_map_reference_mode;
}

TEST(CyberAdapterTest, ConvertsOrientationToApolloVehicleFrame) {
  VehicleState state;
  state.qw = 1.0;

  apollo::localization::LocalizationEstimate localization;
  CyberAdapter::ToLocalization(state, &localization);

  const double sqrt_half = std::sqrt(0.5);
  EXPECT_NEAR(localization.pose().orientation().qw(), sqrt_half, 1e-12);
  EXPECT_NEAR(localization.pose().orientation().qz(), -sqrt_half, 1e-12);
}

}  // namespace simulation
}  // namespace apollo
