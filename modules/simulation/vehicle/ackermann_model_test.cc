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

//  Created Date: 2026-09-10
//  Author: daohu527

#include "modules/simulation/vehicle/ackermann_model.h"

#include <cmath>

#include "gtest/gtest.h"

namespace apollo {
namespace simulation {

TEST(AckermannModelTest, ComputesInnerAndOuterSteeringAngles) {
  AckermannModel model;
  model.SetGeometry(2.8448, 1.58, 0.33);

  VehicleCommand command;
  command.front_steering_rad = 0.25;
  command.gear = VehicleCommand::Gear::GEAR_DRIVE;

  const VehicleActuation actuation =
      model.ComputeActuation(command, VehicleState{}, 0.02);

  EXPECT_GT(actuation.wheel_steer_rad[0], actuation.wheel_steer_rad[1]);
  EXPECT_NEAR(actuation.wheel_steer_rad[2], 0.0, 1e-12);
  EXPECT_NEAR(actuation.wheel_steer_rad[3], 0.0, 1e-12);
  EXPECT_NEAR(actuation.wheel_steer_rad[0],
              std::atan(2.8448 / (2.8448 / std::tan(0.25) - 1.58 * 0.5)),
              1e-12);
}

TEST(AckermannModelTest, DistributesDriveTorqueAcrossAllWheels) {
  AckermannModel model;
  model.SetTorqueLimits(2400.0, 4800.0);

  VehicleCommand command;
  command.throttle = 0.5;
  command.gear = VehicleCommand::Gear::GEAR_DRIVE;

  const VehicleActuation actuation =
      model.ComputeActuation(command, VehicleState{}, 0.02);

  for (double torque : actuation.drive_torque_nm) {
    EXPECT_DOUBLE_EQ(torque, 300.0);
  }
  for (double torque : actuation.brake_torque_nm) {
    EXPECT_DOUBLE_EQ(torque, 0.0);
  }
}

TEST(AckermannModelTest, AppliesGearAndBrakeSemantics) {
  AckermannModel model;
  model.SetTorqueLimits(2400.0, 4800.0);

  VehicleCommand reverse;
  reverse.throttle = 0.5;
  reverse.gear = VehicleCommand::Gear::GEAR_REVERSE;
  const VehicleActuation reverse_actuation =
      model.ComputeActuation(reverse, VehicleState{}, 0.02);
  EXPECT_DOUBLE_EQ(reverse_actuation.drive_torque_nm[0], -300.0);

  VehicleCommand neutral = reverse;
  neutral.gear = VehicleCommand::Gear::GEAR_NEUTRAL;
  const VehicleActuation neutral_actuation =
      model.ComputeActuation(neutral, VehicleState{}, 0.02);
  for (double torque : neutral_actuation.drive_torque_nm) {
    EXPECT_DOUBLE_EQ(torque, 0.0);
  }

  VehicleCommand brake;
  brake.brake = 0.5;
  brake.gear = VehicleCommand::Gear::GEAR_DRIVE;
  const VehicleActuation brake_actuation =
      model.ComputeActuation(brake, VehicleState{}, 0.02);
  EXPECT_DOUBLE_EQ(brake_actuation.brake_torque_nm[0], 720.0);
  EXPECT_DOUBLE_EQ(brake_actuation.brake_torque_nm[1], 720.0);
  EXPECT_DOUBLE_EQ(brake_actuation.brake_torque_nm[2], 480.0);
  EXPECT_DOUBLE_EQ(brake_actuation.brake_torque_nm[3], 480.0);
}

TEST(AckermannModelTest, ClampsSteeringAndAppliesParkingBrake) {
  AckermannModel model;
  model.SetMaxSteerAngle(0.50);

  VehicleCommand command;
  command.front_steering_rad = 1.0;
  command.gear = VehicleCommand::Gear::GEAR_PARKING;

  const VehicleActuation actuation =
      model.ComputeActuation(command, VehicleState{}, 0.02);

  EXPECT_LE(actuation.wheel_steer_rad[0], 0.50);
  EXPECT_LE(actuation.wheel_steer_rad[1], 0.50);
  for (int i = 0; i < 4; ++i) {
    EXPECT_DOUBLE_EQ(actuation.drive_torque_nm[i], 0.0);
    EXPECT_DOUBLE_EQ(actuation.brake_torque_nm[i], 1200.0);
  }
}

TEST(AckermannModelTest, ComputesCounterPhaseFourWheelSteeringAngles) {
  AckermannModel model;
  model.SetGeometry(2.8448, 1.58, 0.33);
  model.SetMaxSteerAngle(0.50);
  model.SetMaxRearSteerAngle(0.25);

  VehicleCommand command;
  command.front_steering_rad = 0.25;

  const VehicleActuation actuation =
      model.ComputeActuation(command, VehicleState{}, 0.02);

  EXPECT_GT(actuation.wheel_steer_rad[0], actuation.wheel_steer_rad[1]);
  EXPECT_LT(actuation.wheel_steer_rad[2], 0.0);
  EXPECT_LT(actuation.wheel_steer_rad[3], 0.0);
  EXPECT_LT(actuation.wheel_steer_rad[2], actuation.wheel_steer_rad[3]);
  EXPECT_LE(std::abs(actuation.wheel_steer_rad[2]), 0.25);
  EXPECT_LE(std::abs(actuation.wheel_steer_rad[3]), 0.25);
}

}  // namespace simulation
}  // namespace apollo
