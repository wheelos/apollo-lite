// Copyright 2025 WheelOS. All Rights Reserved.
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

//  Created Date: 2026-01-03
//  Author: daohu527

#include "modules/control/safety/safety_manager.h"

#include <cmath>

#include <gtest/gtest.h>

#include "modules/control/proto/control_conf.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace control {

class SafetyManagerTest : public ::testing::Test {
 protected:
  /**
   * @brief SetUp runs before each test case.
   * Uses Static Injection to mock VehicleConfig without file I/O.
   */
  void SetUp() override {
    // 1. Prepare Mock Vehicle Configuration
    apollo::common::VehicleConfig mock_config;
    auto* param = mock_config.mutable_vehicle_param();
    param->set_max_steer_angle_rate(100.0);
    param->set_max_acceleration(5.0);

    // 2. Inject mock config into Singleton (Bypasses file loading)
    apollo::common::VehicleConfigHelper::Init(mock_config);

    // 3. Initialize Control Configuration
    ControlConf conf;
    conf.set_control_period(0.01);
    conf.set_soft_estop_brake(20.0);

    // 4. Initialize SafetyManager
    ASSERT_TRUE(safety_manager_.Init(conf));
  }

  SafetyManager safety_manager_;
};

/**
 * @test Verify that the system starts in kNormal state.
 */
TEST_F(SafetyManagerTest, InitialStateIsNormal) {
  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kNormal);
}

/**
 * @test Verify fault arbitration: Multiple faults should result in the highest
 * severity.
 */
TEST_F(SafetyManagerTest, ArbitrateHighestSeverity) {
  LocalView view;
  // Trigger Trajectory Loss (Soft Stop level after 3 counts via debouncer)
  for (int i = 0; i < 3; ++i) {
    safety_manager_.CheckInput(view);
  }
  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kSoftStop);

  // Inject a NaN velocity (Hard Estop level)
  auto* pt = view.mutable_trajectory()->add_trajectory_point();
  pt->set_v(std::nan(""));

  safety_manager_.CheckInput(view);
  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kHardEstop);
}

/**
 * @test Verify steering rate protection during CheckOutput.
 */
TEST_F(SafetyManagerTest, SteeringRateProtection) {
  ControlCommand current_cmd;
  ControlCommand prev_cmd;

  prev_cmd.set_steering_target(0.0);
  // Max steer rate is 100.0 rad/s. At dt=0.01, max delta is 1.0.
  // Set target to 2.0 to trigger violation (rate = 200.0 rad/s).
  current_cmd.set_steering_target(2.0);

  safety_manager_.CheckOutput(current_cmd, prev_cmd);
  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kHardEstop);
}

/**
 * @test Verify Safety Policy Application for Hard Estop.
 */
TEST_F(SafetyManagerTest, ApplyHardEstopPolicy) {
  LocalView view;
  auto* pt = view.mutable_trajectory()->add_trajectory_point();
  pt->set_v(std::nan(""));  // Trigger Hard Estop

  safety_manager_.CheckInput(view);
  ASSERT_EQ(safety_manager_.GetState(), SafetyState::kHardEstop);

  ControlCommand cmd;
  cmd.set_speed(10.0);
  cmd.set_throttle(50.0);

  safety_manager_.ApplySafetyPolicy(&cmd);

  // Command should be overridden by emergency safety values
  EXPECT_DOUBLE_EQ(cmd.speed(), 0.0);
  EXPECT_DOUBLE_EQ(cmd.throttle(), 0.0);
  EXPECT_DOUBLE_EQ(cmd.brake(), 100.0);
  EXPECT_TRUE(cmd.parking_brake());
  EXPECT_EQ(cmd.gear_location(), apollo::canbus::Chassis::GEAR_PARKING);
  EXPECT_TRUE(cmd.signal().emergency_light());
}

/**
 * @test Verify State Latching: System should not recover from HardEstop
 * automatically even if the fault is cleared.
 */
TEST_F(SafetyManagerTest, StateLatchingLogic) {
  LocalView view;
  // Trigger Hard Estop via NaN
  auto* pt = view.mutable_trajectory()->add_trajectory_point();
  pt->set_v(std::nan(""));
  safety_manager_.CheckInput(view);
  ASSERT_EQ(safety_manager_.GetState(), SafetyState::kHardEstop);

  // Clear faults in input (set valid velocity)
  pt->set_v(1.0);
  safety_manager_.CheckInput(view);

  // State should remain latched at HardEstop until manual reset
  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kHardEstop);
}

/**
 * @test Verify that the system automatically recovers from kSoftStop
 * when the fault is cleared, but remains latched in kHardEstop.
 */
TEST_F(SafetyManagerTest, SoftStopAutoRecoveryAndHardStopLatching) {
  LocalView view;

  // 1. Trigger Soft Stop via Trajectory Loss (3 counts)
  for (int i = 0; i < 3; ++i) {
    safety_manager_.CheckInput(view);
  }
  ASSERT_EQ(safety_manager_.GetState(), SafetyState::kSoftStop);

  // 2. Clear the fault (Add a valid trajectory point)
  auto* pt = view.mutable_trajectory()->add_trajectory_point();
  pt->set_v(1.0);
  pt->set_a(0.0);

  // 3. Check arbitration - SoftStop should recover to Normal automatically
  safety_manager_.CheckInput(view);
  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kNormal);

  // 4. Now trigger a Hard Estop (NaN check)
  pt->set_v(std::nan(""));
  safety_manager_.CheckInput(view);
  ASSERT_EQ(safety_manager_.GetState(), SafetyState::kHardEstop);

  // 5. Clear the Hard Estop fault
  pt->set_v(1.0);
  safety_manager_.CheckInput(view);

  // 6. Hard Estop MUST NOT recover automatically (Latching logic)
  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kHardEstop);
}

/**
 * @test Verify Reset functionality via PadMessage.
 */
TEST_F(SafetyManagerTest, ManualResetSuccessful) {
  LocalView view;
  auto* pt = view.mutable_trajectory()->add_trajectory_point();
  pt->set_v(std::nan(""));
  safety_manager_.CheckInput(view);

  // Try reset while fault is still present (NaN). Should fail to reset.
  PadMessage pad;
  pad.set_action(DrivingAction::RESET);
  safety_manager_.TryReset(pad);
  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kHardEstop);

  // Fix fault and then trigger reset.
  pt->set_v(1.0);
  safety_manager_.CheckInput(view);  // Clears active_faults_ buffer
  safety_manager_.TryReset(pad);

  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kNormal);
}

/**
 * @test Verify CheckInput return value for skipping computation on critical
 * failure.
 */
TEST_F(SafetyManagerTest, SkipComputationOnCriticalFailure) {
  LocalView view;
  auto* pt = view.mutable_trajectory()->add_trajectory_point();
  pt->set_v(std::nan(""));

  // HardEstop state should return true to skip control solver computation
  bool skip = safety_manager_.CheckInput(view);
  EXPECT_TRUE(skip);
}

}  // namespace control
}  // namespace apollo
