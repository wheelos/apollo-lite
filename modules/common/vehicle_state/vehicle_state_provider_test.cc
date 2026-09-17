/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/common/vehicle_state/vehicle_state_provider.h"

#include "gtest/gtest.h"

#include "wheelos_msgs/chassis_msgs/chassis.pb.h"
#include "wheelos_msgs/localization_msgs/localization.pb.h"

#include "modules/common/configs/config_gflags.h"

namespace apollo {
namespace common {
namespace vehicle_state_provider {

using apollo::canbus::Chassis;
using apollo::localization::LocalizationEstimate;

class VehicleStateProviderTest : public ::testing::Test {
 public:
  void SetUp() override {
    auto* header = localization_.mutable_header();
    header->set_timestamp_sec(1173545122.22);
    auto* pose = localization_.mutable_pose();
    pose->mutable_position()->set_x(357.51331791372041);
    pose->mutable_position()->set_y(96.165912376788725);
    pose->mutable_position()->set_z(-31.983237908221781);
    pose->mutable_orientation()->set_qx(0.024015498296453403);
    pose->mutable_orientation()->set_qy(0.0021656820647661572);
    pose->mutable_orientation()->set_qz(-0.99072964388722151);
    pose->mutable_orientation()->set_qw(-0.13369120534226134);
    pose->mutable_linear_acceleration()->set_y(-0.079383290718229638);
    pose->mutable_linear_velocity()->set_x(-1.5626866011382312);
    pose->mutable_linear_velocity()->set_y(-5.9852188341040344);
    pose->mutable_angular_velocity()->set_z(-0.0079623083093763921);
    pose->set_heading(-1.8388082455104939);
    chassis_.set_speed_mps(3.0);
    chassis_.set_gear_location(canbus::Chassis::GEAR_DRIVE);
    FLAGS_enable_map_reference_unify = false;
  }

 protected:
  LocalizationEstimate localization_;
  Chassis chassis_;
};

TEST_F(VehicleStateProviderTest, Accessors) {
  VehicleStateProvider vehicle_state_provider;
  ASSERT_TRUE(vehicle_state_provider.Update(localization_, chassis_).ok());
  const auto& state = vehicle_state_provider.state();
  EXPECT_DOUBLE_EQ(state.x(), 357.51331791372041);
  EXPECT_DOUBLE_EQ(state.y(), 96.165912376788725);
  EXPECT_DOUBLE_EQ(state.heading(), -1.8388082455104939);
  EXPECT_DOUBLE_EQ(state.roll(), 0.047026695713820919);
  EXPECT_DOUBLE_EQ(state.pitch(), -0.010712737572581465);
  EXPECT_DOUBLE_EQ(state.yaw(), 2.8735807348741953);
  EXPECT_DOUBLE_EQ(state.linear_velocity(), 3.0);
  EXPECT_NEAR(state.lateral_velocity(), 0.0780773, 1e-6);
  EXPECT_DOUBLE_EQ(state.angular_velocity(), -0.0079623083093763921);
  EXPECT_DOUBLE_EQ(state.linear_acceleration(), -0.079383290718229638);
  EXPECT_EQ(state.gear(), canbus::Chassis::GEAR_DRIVE);
}

TEST_F(VehicleStateProviderTest, LowGearIsForward) {
  chassis_.set_gear_location(canbus::Chassis::GEAR_LOW);
  VehicleStateProvider vehicle_state_provider;

  ASSERT_TRUE(vehicle_state_provider.Update(localization_, chassis_).ok());
  EXPECT_EQ(vehicle_state_provider.state().travel_direction(),
            TRAVEL_DIRECTION_FORWARD);
}

TEST_F(VehicleStateProviderTest, StateContainsAlignedOperatingFields) {
  VehicleStateProvider vehicle_state_provider;
  ASSERT_TRUE(vehicle_state_provider.Update(localization_, chassis_).ok());

  const auto& state = vehicle_state_provider.state();
  EXPECT_DOUBLE_EQ(state.timestamp(), localization_.header().timestamp_sec());
  EXPECT_EQ(state.gear(), canbus::Chassis::GEAR_DRIVE);
  EXPECT_EQ(state.travel_direction(), TRAVEL_DIRECTION_FORWARD);
}

}  // namespace vehicle_state_provider
}  // namespace common
}  // namespace apollo
