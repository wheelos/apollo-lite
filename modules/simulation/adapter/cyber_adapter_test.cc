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

#include "gtest/gtest.h"

#include "modules/common/configs/config_gflags.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"

namespace apollo {
namespace simulation {

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
  const auto& common_state = provider.vehicle_state();
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

}  // namespace simulation
}  // namespace apollo
