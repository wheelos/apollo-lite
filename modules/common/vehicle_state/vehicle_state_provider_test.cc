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

#include <cstdlib>
#include <string>

#include "gtest/gtest.h"

#include "Eigen/Core"

#include "wheelos_msgs/chassis_msgs/chassis.pb.h"
#include "wheelos_msgs/localization_msgs/localization.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/vehicle_state/vehicle_motion_model.h"

namespace apollo {
namespace common {
namespace vehicle_state_provider {

using apollo::canbus::Chassis;
using apollo::localization::LocalizationEstimate;

namespace {

std::string RunfilePath(const std::string& relative_path) {
  const char* test_srcdir = std::getenv("TEST_SRCDIR");
  const char* test_workspace = std::getenv("TEST_WORKSPACE");
  if (test_srcdir != nullptr && test_workspace != nullptr) {
    return std::string(test_srcdir) + "/" + test_workspace + "/" +
           relative_path;
  }
  return relative_path;
}

}  // namespace

class VehicleStateProviderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string localization_file = RunfilePath(
        "modules/common/vehicle_state/testdata/3_localization_result_1.pb.txt");
    ACHECK(cyber::common::GetProtoFromFile(localization_file, &localization_));
    chassis_.set_speed_mps(3.0);
    chassis_.set_gear_location(canbus::Chassis::GEAR_DRIVE);
    FLAGS_enable_map_reference_unify = false;
  }

 protected:
  LocalizationEstimate localization_;
  Chassis chassis_;
};

TEST_F(VehicleStateProviderTest, Accessors) {
  auto vehicle_state_provider = std::make_shared<VehicleStateProvider>();
  vehicle_state_provider->Update(localization_, chassis_);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->x(), 357.51331791372041);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->y(), 96.165912376788725);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->heading(), -1.8388082455104939);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->roll(), 0.047026695713820919);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->pitch(), -0.010712737572581465);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->yaw(), 2.8735807348741953);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->linear_velocity(), 3.0);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->angular_velocity(),
                   -0.0079623083093763921);
  EXPECT_DOUBLE_EQ(vehicle_state_provider->linear_acceleration(),
                   -0.079383290718229638);
  EXPECT_EQ(vehicle_state_provider->gear(), canbus::Chassis::GEAR_DRIVE);
}

TEST_F(VehicleStateProviderTest, EstimateFuturePosition) {
  auto vehicle_state_provider = std::make_shared<VehicleStateProvider>();
  vehicle_state_provider->Update(localization_, chassis_);
  common::math::Vec2d future_position =
      VehicleMotionModel::EstimateFuturePosition(
          vehicle_state_provider->vehicle_state(), 1.0);
  EXPECT_NEAR(future_position.x(), 356.707, 1e-3);
  EXPECT_NEAR(future_position.y(), 93.276, 1e-3);
  future_position = VehicleMotionModel::EstimateFuturePosition(
      vehicle_state_provider->vehicle_state(), 2.0);
  EXPECT_NEAR(future_position.x(), 355.879, 1e-3);
  EXPECT_NEAR(future_position.y(), 90.393, 1e-3);
}

TEST(VehicleMotionModelTest, UsesHeadingWithoutPoseOrientation) {
  VehicleState state;
  state.set_x(10.0);
  state.set_y(20.0);
  state.set_heading(1.5707963267948966);
  state.set_linear_velocity(2.0);

  const auto future_position =
      VehicleMotionModel::EstimateFuturePosition(state, 1.0);

  EXPECT_NEAR(future_position.x(), 10.0, 1e-9);
  EXPECT_NEAR(future_position.y(), 22.0, 1e-9);
}

TEST(VehicleMotionModelTest, UsesHeadingConsistentlyForTurningWithoutPose) {
  VehicleState state;
  state.set_y(20.0);
  state.set_heading(1.5707963267948966);
  state.set_linear_velocity(2.0);
  state.set_angular_velocity(0.5);

  const auto future_position =
      VehicleMotionModel::EstimateFuturePosition(state, 1.0);

  const double local_x = -2.0 / 0.5 * (1.0 - std::cos(0.5));
  const double local_y = std::sin(0.5) * 2.0 / 0.5;
  EXPECT_NEAR(future_position.x(), local_x, 1e-9);
  EXPECT_NEAR(future_position.y(), 20.0 + local_y, 1e-9);
}

TEST_F(VehicleStateProviderTest, LowGearIsForward) {
  chassis_.set_gear_location(canbus::Chassis::GEAR_LOW);
  auto vehicle_state_provider = std::make_shared<VehicleStateProvider>();

  ASSERT_TRUE(vehicle_state_provider->Update(localization_, chassis_).ok());
  EXPECT_EQ(vehicle_state_provider->operating_state().travel_direction(),
            TRAVEL_DIRECTION_FORWARD);
}

}  // namespace vehicle_state_provider
}  // namespace common
}  // namespace apollo
