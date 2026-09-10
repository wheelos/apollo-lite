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

#include "modules/simulation/core/simulation_engine.h"

#include <cmath>
#include <cstdio>
#include <fstream>

#include "gtest/gtest.h"

namespace apollo {
namespace simulation {

TEST(SimulationEngineTest, InitializationAndReset) {
  SimulationEngine engine;
  EXPECT_TRUE(engine.Init("kinematic", ""));

  engine.Reset(10.0, 20.0, M_PI_2);

  VehicleState state{};
  EXPECT_TRUE(engine.GetVehicleState(&state));
  EXPECT_DOUBLE_EQ(state.x, 10.0);
  EXPECT_DOUBLE_EQ(state.y, 20.0);
  EXPECT_DOUBLE_EQ(state.yaw, M_PI_2);
  EXPECT_DOUBLE_EQ(state.linear_velocity_mps, 0.0);
}

TEST(SimulationEngineTest, ForwardAcceleration) {
  SimulationEngine engine;
  EXPECT_TRUE(engine.Init("kinematic", ""));
  engine.Reset(0.0, 0.0, 0.0);

  VehicleCommand cmd{};
  cmd.throttle = 0.5;
  cmd.gear = VehicleCommand::Gear::GEAR_DRIVE;
  cmd.timestamp_sec = 1.0;

  // Step 10 cycles of 20ms = 0.2s
  for (int i = 0; i < 10; ++i) {
    engine.Step(cmd, 0.02);
  }

  VehicleState state{};
  EXPECT_TRUE(engine.GetVehicleState(&state));
  EXPECT_GT(state.linear_velocity_mps, 0.0);
  EXPECT_GT(state.x, 0.0);
  EXPECT_NEAR(state.y, 0.0, 1e-3);
  EXPECT_NEAR(state.yaw, 0.0, 1e-3);
}

TEST(SimulationEngineTest, TargetSpeedProducesClosedLoopMotion) {
  SimulationEngine engine;
  ASSERT_TRUE(engine.Init("kinematic", ""));
  engine.SetControlMode("speed");
  engine.SetSpeedKp(1.5);
  ASSERT_TRUE(engine.SetCommandTimeout(10.0));
  engine.Reset(0.0, 0.0, 0.0);

  VehicleCommand cmd{};
  cmd.target_speed_mps = 3.0;
  cmd.gear = VehicleCommand::Gear::GEAR_DRIVE;
  for (int i = 0; i < 250; ++i) {
    ASSERT_TRUE(engine.Step(cmd, 0.02, i == 0));
  }

  VehicleState state{};
  ASSERT_TRUE(engine.GetVehicleState(&state));
  EXPECT_GT(state.x, 1.0);
  EXPECT_GT(state.linear_velocity_mps, 0.0);
}

TEST(SimulationEngineTest, SteeringTurn) {
  SimulationEngine engine;
  EXPECT_TRUE(engine.Init("kinematic", ""));
  engine.Reset(0.0, 0.0, 0.0);

  VehicleCommand cmd{};
  cmd.throttle = 0.5;
  cmd.front_steering_rad = 0.2;  // Turn left
  cmd.gear = VehicleCommand::Gear::GEAR_DRIVE;
  cmd.timestamp_sec = 1.0;

  // Step 50 cycles of 20ms = 1.0s
  for (int i = 0; i < 50; ++i) {
    engine.Step(cmd, 0.02);
  }

  VehicleState state{};
  EXPECT_TRUE(engine.GetVehicleState(&state));
  EXPECT_GT(state.linear_velocity_mps, 0.0);
  EXPECT_GT(state.x, 0.0);
  EXPECT_GT(state.y, 0.0);    // Left turn in ENU yields positive Y
  EXPECT_GT(state.yaw, 0.0);  // Positive yaw (CCW)
}

TEST(SimulationEngineTest, BrakingStopsVehicle) {
  SimulationEngine engine;
  EXPECT_TRUE(engine.Init("kinematic", ""));
  engine.Reset(0.0, 0.0, 0.0);

  // Accelerate
  VehicleCommand cmd{};
  cmd.throttle = 0.8;
  cmd.gear = VehicleCommand::Gear::GEAR_DRIVE;
  cmd.timestamp_sec = 1.0;
  for (int i = 0; i < 20; ++i) {
    engine.Step(cmd, 0.02);
  }

  VehicleState moving_state{};
  EXPECT_TRUE(engine.GetVehicleState(&moving_state));
  EXPECT_GT(moving_state.linear_velocity_mps, 1.0);

  // Brake
  cmd.throttle = 0.0;
  cmd.brake = 1.0;
  for (int i = 0; i < 50; ++i) {
    engine.Step(cmd, 0.02);
  }

  VehicleState stopped_state{};
  EXPECT_TRUE(engine.GetVehicleState(&stopped_state));
  EXPECT_NEAR(stopped_state.linear_velocity_mps, 0.0, 0.05);
}

TEST(SimulationEngineTest, MujocoBackendLifecycle) {
  const std::string tmp_xml = "/tmp/test_mujoco_vehicle.xml";
  {
    std::ofstream ofs(tmp_xml);
    ofs << "<mujoco model=\"test_vehicle\">\n"
        << "  <worldbody>\n"
        << "    <body name=\"ego_vehicle\" pos=\"0 0 0.5\">\n"
        << "      <freejoint/>\n"
        << "      <geom type=\"box\" size=\"2 1 0.5\" mass=\"1500\"/>\n"
        << "    </body>\n"
        << "  </worldbody>\n"
        << "</mujoco>\n";
  }

  SimulationEngine engine;
  // A model without the required Ackermann actuators must be rejected during
  // initialization rather than failing later when control is applied.
  EXPECT_FALSE(engine.Init("mujoco", tmp_xml));

  std::remove(tmp_xml.c_str());
}

TEST(SimulationEngineTest, RejectsInvalidPhysicsDt) {
  SimulationEngine engine;
  EXPECT_FALSE(engine.SetPhysicsDt(0.0));
  EXPECT_FALSE(engine.SetPhysicsDt(-0.001));
  EXPECT_TRUE(engine.SetPhysicsDt(0.001));
}

TEST(SimulationEngineTest, AppliesFailsafeAfterCommandTimeout) {
  SimulationEngine engine;
  ASSERT_TRUE(engine.Init("kinematic", ""));
  ASSERT_TRUE(engine.SetCommandTimeout(0.04));

  VehicleCommand cmd{};
  cmd.gear = VehicleCommand::Gear::GEAR_DRIVE;
  cmd.throttle = 0.8;
  ASSERT_TRUE(engine.Step(cmd, 0.02, true));
  ASSERT_TRUE(engine.Step(cmd, 0.02, false));
  ASSERT_TRUE(engine.Step(cmd, 0.02, false));
  ASSERT_TRUE(engine.Step(cmd, 0.02, false));

  VehicleState state{};
  ASSERT_TRUE(engine.GetVehicleState(&state));
  EXPECT_DOUBLE_EQ(state.throttle_percentage, 0.0);
  EXPECT_DOUBLE_EQ(state.brake_percentage, 50.0);
}

}  // namespace simulation
}  // namespace apollo
