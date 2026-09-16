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

#include "modules/simulation/backend/mujoco_backend.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace simulation {
namespace {

std::string WriteCollisionModel() {
  const std::string path = "mujoco_backend_collision_test.xml";
  std::ofstream model(path);
  model << R"xml(
<mujoco model="collision_test">
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="ego_vehicle" pos="0 0 0.5">
      <freejoint name="vehicle_freejoint"/>
      <geom name="vehicle_geom" type="box" size="0.5 0.5 0.5" mass="10"/>
      <body name="steer_fl" pos="0 0 0">
        <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
        <joint name="steer_fl_joint" type="hinge" axis="0 0 1"/>
      </body>
      <body name="steer_fr" pos="0 0 0">
        <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
        <joint name="steer_fr_joint" type="hinge" axis="0 0 1"/>
      </body>
      <body name="wheel_fl" pos="0 0 0">
        <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
        <joint name="wheel_fl_joint" type="hinge" axis="0 1 0"/>
      </body>
      <body name="wheel_fr" pos="0 0 0">
        <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
        <joint name="wheel_fr_joint" type="hinge" axis="0 1 0"/>
      </body>
      <body name="wheel_rl" pos="0 0 0">
        <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
        <joint name="wheel_rl_joint" type="hinge" axis="0 1 0"/>
      </body>
      <body name="wheel_rr" pos="0 0 0">
        <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
        <joint name="wheel_rr_joint" type="hinge" axis="0 1 0"/>
      </body>
    </body>
    <body name="obstacle" pos="0 0 0.5">
      <geom name="obstacle_geom" type="box" size="0.25 0.25 0.25" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <position name="steer_fl" joint="steer_fl_joint"/>
    <position name="steer_fr" joint="steer_fr_joint"/>
    <motor name="drive_fl" joint="wheel_fl_joint"/>
    <motor name="drive_fr" joint="wheel_fr_joint"/>
    <motor name="drive_rl" joint="wheel_rl_joint"/>
    <motor name="drive_rr" joint="wheel_rr_joint"/>
  </actuator>
</mujoco>
)xml";
  return path;
}

std::string WriteInvalidModel() {
  const std::string path = "mujoco_backend_invalid_test.xml";
  std::ofstream model(path);
  model << R"xml(
<mujoco model="invalid_test">
  <worldbody>
    <body name="ego_vehicle">
      <freejoint name="vehicle_freejoint"/>
      <geom type="box" size="0.5 0.5 0.5"/>
    </body>
  </worldbody>
</mujoco>
)xml";
  return path;
}

std::string AckermannModelPath() {
  const std::string relative = "modules/simulation/model/ackermann_vehicle.xml";
  std::ifstream source_tree(relative);
  if (source_tree.good()) {
    return relative;
  }

  const char* test_srcdir = std::getenv("TEST_SRCDIR");
  const char* test_workspace = std::getenv("TEST_WORKSPACE");
  if (test_srcdir != nullptr && test_workspace != nullptr) {
    const std::string runfile_path =
        std::string(test_srcdir) + "/" + test_workspace + "/" + relative;
    std::ifstream runfile(runfile_path);
    if (runfile.good()) {
      return runfile_path;
    }
  }
  return "/apollo/" + relative;
}

}  // namespace

TEST(MujocoBackendTest, RejectsMissingModelAndBindsRequiredModelParts) {
  MujocoBackend backend;
  EXPECT_FALSE(backend.Init("does-not-exist.xml"));
  const std::string invalid_model = WriteInvalidModel();
  EXPECT_FALSE(backend.Init(invalid_model));
  std::remove(invalid_model.c_str());
  EXPECT_TRUE(backend.Init(AckermannModelPath()));
  EXPECT_EQ(backend.Name(), "MuJoCo");
}

TEST(MujocoBackendTest, ExtractsResetStateAndAdvancesSimulationTime) {
  MujocoBackend backend;
  ASSERT_TRUE(backend.Init(AckermannModelPath()));
  EXPECT_TRUE(backend.SetVehicleGeometry(2.8448, 1.58, 0.335));
  EXPECT_TRUE(backend.SetMaxSteerAngle(0.6108652382));
  EXPECT_TRUE(backend.SetMaxRearSteerAngle(0.25));
  EXPECT_FALSE(backend.SetVehicleGeometry(2.8, 1.58, 0.33));
  EXPECT_FALSE(backend.SetVehicleGeometry(2.8448, 1.6, 0.33));
  EXPECT_FALSE(backend.SetVehicleGeometry(2.8448, 1.58, 0.34));
  EXPECT_FALSE(backend.SetMaxSteerAngle(0.62));
  EXPECT_FALSE(backend.SetMaxRearSteerAngle(0.62));
  backend.Reset(3.0, -2.0, M_PI_2);

  VehicleState state;
  ASSERT_TRUE(backend.GetVehicleState(&state));
  EXPECT_NEAR(state.x, 3.0, 1e-6);
  EXPECT_NEAR(state.y, -2.0, 1e-6);
  EXPECT_NEAR(state.yaw, M_PI_2, 1e-6);
  EXPECT_NEAR(state.qw, std::cos(M_PI_2 * 0.5), 1e-6);
  EXPECT_NEAR(state.qz, std::sin(M_PI_2 * 0.5), 1e-6);

  VehicleActuation actuation;
  actuation.wheel_steer_rad[0] = 0.2;
  actuation.wheel_steer_rad[1] = 0.1;
  actuation.wheel_steer_rad[2] = -0.1;
  actuation.wheel_steer_rad[3] = -0.2;
  ASSERT_TRUE(backend.ApplyActuation(actuation));
  ASSERT_TRUE(backend.Step(0.01));
  EXPECT_NEAR(backend.SimulationTime(), 0.01, 1e-12);
  ASSERT_TRUE(backend.GetVehicleState(&state));
  EXPECT_NEAR(state.timestamp_sec, 0.01, 1e-12);
}

TEST(MujocoBackendTest, RejectsInvalidStepAndNullState) {
  MujocoBackend backend;
  ASSERT_TRUE(backend.Init(AckermannModelPath()));
  EXPECT_FALSE(backend.Step(0.0));
  EXPECT_FALSE(backend.Step(-0.01));
  EXPECT_FALSE(backend.GetVehicleState(nullptr));
}

TEST(MujocoBackendTest, ReportsVehicleContactWithExternalBody) {
  const std::string model_path = WriteCollisionModel();
  MujocoBackend backend;
  ASSERT_TRUE(backend.Init(model_path));

  VehicleState state;
  ASSERT_TRUE(backend.GetVehicleState(&state));
  EXPECT_TRUE(state.is_collision);
  std::remove(model_path.c_str());
}

}  // namespace simulation
}  // namespace apollo
