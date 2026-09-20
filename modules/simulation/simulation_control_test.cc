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

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "modules/control/proto/control_conf.pb.h"
#include "wheelos_msgs/chassis_msgs/chassis.pb.h"
#include "wheelos_msgs/control_msgs/control_cmd.pb.h"
#include "wheelos_msgs/localization_msgs/localization.pb.h"
#include "wheelos_msgs/planning_msgs/planning.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/common/dependency_injector.h"
#include "modules/control/controller/controller_agent.h"
#include "modules/simulation/adapter/cyber_adapter.h"
#include "modules/simulation/core/simulation_engine.h"

namespace apollo {
namespace simulation {
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

using apollo::control::ControlCommand;
using apollo::control::ControlConf;
using apollo::control::ControllerAgent;
using apollo::control::DependencyInjector;
using apollo::planning::ADCTrajectory;

class SimulationControlTest : public ::testing::Test {
 protected:
  void SetUp() override {
    FLAGS_enable_map_reference_unify = false;
    model_path_ = RunfilePath("modules/simulation/model/ackermann_vehicle.xml");
  }

  ADCTrajectory BuildStraightTrajectory(double start_x, double start_y,
                                        double target_v, double total_time) {
    ADCTrajectory traj;
    auto* header = traj.mutable_header();
    header->set_timestamp_sec(0.0);
    header->set_module_name("planning");

    traj.set_total_path_length(target_v * total_time);
    traj.set_total_path_time(total_time);
    traj.set_gear(apollo::canbus::Chassis::GEAR_DRIVE);

    int num_points = static_cast<int>(total_time / 0.1);
    for (int i = 0; i <= num_points; ++i) {
      double t = i * 0.1;
      double v = std::min(target_v, target_v * (t / (total_time * 0.5)));
      double s = v * t;

      auto* pt = traj.add_trajectory_point();
      auto* path_pt = pt->mutable_path_point();
      path_pt->set_x(start_x + s);
      path_pt->set_y(start_y);
      path_pt->set_z(0.0);
      path_pt->set_theta(0.0);
      path_pt->set_kappa(0.0);
      path_pt->set_s(s);

      pt->set_v(v);
      pt->set_a(v < target_v ? 1.0 : 0.0);
      pt->set_relative_time(t);
    }
    return traj;
  }

  std::string model_path_;
};

TEST_F(SimulationControlTest, MujocoAckermannVehicleDynamics) {
  SimulationEngine engine;
  ASSERT_TRUE(engine.Init("mujoco", model_path_));

  engine.Reset(0.0, 0.0, 0.0);

  VehicleState state{};
  ASSERT_TRUE(engine.GetVehicleState(&state));
  const double initial_x = state.x;
  AINFO << "Initial state: x=" << state.x << ", y=" << state.y
        << ", z=" << state.z << ", yaw=" << state.yaw;

  // 1. Accelerate forward
  VehicleCommand cmd{};
  cmd.gear = VehicleCommand::Gear::GEAR_DRIVE;
  cmd.throttle = 0.6;
  cmd.front_steering_rad = 0.0;
  cmd.timestamp_sec = 0.0;

  for (int i = 0; i < 50; ++i) {
    cmd.timestamp_sec += 0.02;
    engine.Step(cmd, 0.02);
    if (i < 5 || i == 49) {
      engine.GetVehicleState(&state);
      AINFO << "Step " << i << ": x=" << state.x << ", y=" << state.y
            << ", z=" << state.z << ", v=" << state.linear_velocity_mps;
    }
  }

  ASSERT_TRUE(engine.GetVehicleState(&state));
  EXPECT_GT(state.linear_velocity_mps, 0.5);
  EXPECT_GT(state.x - initial_x, 0.5);
  EXPECT_NEAR(state.y, 0.0, 0.2);

  // 2. Steer left
  cmd.front_steering_rad = 0.25;
  for (int i = 0; i < 50; ++i) {
    cmd.timestamp_sec += 0.02;
    engine.Step(cmd, 0.02);
  }

  ASSERT_TRUE(engine.GetVehicleState(&state));
  EXPECT_GT(state.y, 0.0);
  EXPECT_GT(state.yaw, 0.0);

  // 3. Brake to stop
  cmd.throttle = 0.0;
  cmd.brake = 1.0;
  cmd.front_steering_rad = 0.0;
  for (int i = 0; i < 50; ++i) {
    cmd.timestamp_sec += 0.02;
    engine.Step(cmd, 0.02);
  }

  ASSERT_TRUE(engine.GetVehicleState(&state));
  EXPECT_LT(state.linear_velocity_mps, 0.1);
}

TEST_F(SimulationControlTest, MujocoTargetSpeedControlProducesMotion) {
  SimulationEngine engine;
  ASSERT_TRUE(engine.Init("mujoco", model_path_));
  engine.SetControlMode("speed");
  engine.SetSpeedKp(1.5);
  engine.Reset(0.0, 0.0, 0.0);
  VehicleState state{};
  ASSERT_TRUE(engine.GetVehicleState(&state));
  const double initial_x = state.x;

  VehicleCommand cmd{};
  cmd.gear = VehicleCommand::Gear::GEAR_DRIVE;
  cmd.target_speed_mps = 2.0;
  for (int i = 0; i < 250; ++i) {
    ASSERT_TRUE(engine.Step(cmd, 0.02));
  }

  ASSERT_TRUE(engine.GetVehicleState(&state));
  EXPECT_GT(state.x - initial_x, 0.1);
  EXPECT_GT(state.linear_velocity_mps, 0.0);
}

TEST_F(SimulationControlTest, ClosedLoopControlSimulation) {
  // 1. Initialize MuJoCo Simulation Engine
  SimulationEngine engine;
  ASSERT_TRUE(engine.Init("mujoco", model_path_));
  engine.Reset(0.0, 0.0, 0.0);

  // 2. Initialize Apollo Controller Agent
  std::string control_conf_file =
      RunfilePath("modules/control/testdata/conf/control_conf.pb.txt");
  ControlConf control_conf;
  ASSERT_TRUE(
      cyber::common::GetProtoFromFile(control_conf_file, &control_conf));

  auto injector = std::make_shared<DependencyInjector>();
  ControllerAgent controller_agent;
  ASSERT_TRUE(controller_agent.Init(injector, &control_conf).ok());
  controller_agent.Reset();

  // 3. Build reference trajectory (accelerating up to 3 m/s along straight
  // line)
  ADCTrajectory trajectory = BuildStraightTrajectory(0.0, 0.0, 3.0, 5.0);

  // 4. Closed-loop control simulation: 100 cycles = 2.0 seconds
  double dt = 0.02;
  double sim_time = 0.0;
  VehicleState state{};
  ASSERT_TRUE(engine.GetVehicleState(&state));
  const double initial_x = state.x;
  apollo::canbus::Chassis chassis;
  apollo::localization::LocalizationEstimate localization;
  ControlCommand control_cmd;
  VehicleCommand vehicle_cmd{};

  for (int step = 0; step < 100; ++step) {
    sim_time += dt;

    // a. Get simulation feedback
    ASSERT_TRUE(engine.GetVehicleState(&state));

    // b. Convert to Cyber/Apollo messages
    const double clock_now = sim_time;
    CyberAdapter::ToChassis(state, &chassis, clock_now);
    CyberAdapter::ToLocalization(state, &localization, clock_now);
    trajectory.mutable_header()->set_timestamp_sec(0.0);

    // Update vehicle state provider in injector
    ASSERT_TRUE(injector->UpdateVehicleState(localization, chassis).ok());

    // c. Compute Control Command
    auto status = controller_agent.ComputeControlCommand(
        &localization, &chassis, &trajectory, &control_cmd);
    EXPECT_TRUE(status.ok());

    // d. Convert Control Command to Vehicle Command
    CyberAdapter::FromControlCommand(control_cmd, &vehicle_cmd);

    if (step < 5 || step % 20 == 0) {
      AINFO << "Step " << step << ": throttle=" << control_cmd.throttle()
            << ", brake=" << control_cmd.brake()
            << ", steer=" << control_cmd.steering_target()
            << ", gear=" << static_cast<int>(control_cmd.gear_location())
            << ", vcmd.gear=" << static_cast<int>(vehicle_cmd.gear)
            << ", vcmd.throttle=" << vehicle_cmd.throttle
            << ", vcmd.brake=" << vehicle_cmd.brake
            << ", vcmd.estop=" << vehicle_cmd.emergency_stop
            << ", x=" << state.x << ", v=" << state.linear_velocity_mps;
    }

    // e. Step physics simulation in MuJoCo
    engine.Step(vehicle_cmd, dt);
  }

  // 5. Verify final closed-loop performance
  ASSERT_TRUE(engine.GetVehicleState(&state));
  AINFO << "Final closed-loop vehicle position: (" << state.x << ", " << state.y
        << "), speed: " << state.linear_velocity_mps
        << " m/s, yaw: " << state.yaw;

  EXPECT_GT(state.x - initial_x, 0.5);
  EXPECT_GT(state.linear_velocity_mps, 0.3);
  EXPECT_NEAR(state.y, 0.0, 0.3);
}

}  // namespace simulation
}  // namespace apollo
