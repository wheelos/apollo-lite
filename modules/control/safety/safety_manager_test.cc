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
 * @test Verify fault arbitration: Multiple faults result in the highest
 * severity.
 */
TEST_F(SafetyManagerTest, ArbitrateHighestSeverity) {
  LocalView view;
  ControlCommand cmd;

  // 1. Soft Stop triggered (track lost 3 times)
  for (int i = 0; i < 30; ++i) {
    safety_manager_.PreCheck(view);
  }
  safety_manager_.ApplySafetyPolicy(&cmd);
  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kSoftStop);

  // 2. Trigger Hard Estop (Dynamics Verification: DRIVE mode + negative speed)
  view.mutable_chassis()->set_gear_location(
      apollo::canbus::Chassis::GEAR_DRIVE);
  auto* pt = view.mutable_trajectory()->add_trajectory_point();
  pt->set_v(-1.0);  // Serious conflicts, without debouncing, directly report to
                    // fault.

  safety_manager_.PreCheck(view);
  safety_manager_.ApplySafetyPolicy(&cmd);
  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kHardEstop);
}

/**
 * @test Verify the 0-latency bypass flag for invalid trajectory inputs.
 */
TEST_F(SafetyManagerTest, ImmediateBypassOnInvalidInput) {
  LocalView view;
  auto* pt = view.mutable_trajectory()->add_trajectory_point();
  pt->set_v(std::nan(""));  // NaN is a physical failure

  // result.must_bypass should be true even on the first frame
  SafetyResult res = safety_manager_.PreCheck(view);
  EXPECT_TRUE(res.must_bypass);
}

/**
 * @test Verify Safety Policy Application for Hard Estop.
 */
TEST_F(SafetyManagerTest, ApplyHardEstopPolicy) {
  LocalView view;
  ControlCommand cmd;

  // Hard Estop triggered by: Acceleration exceeding limit (5.0 is the upper
  // limit)
  cmd.set_acceleration(7.0);
  ControlCommand prev_cmd;
  prev_cmd.set_acceleration(0.0);

  // PostCheck debouncing triggers HardEstop (0x0302) 3 times.
  for (int i = 0; i < 3; ++i) {
    safety_manager_.PostCheck(cmd, prev_cmd);
  }

  safety_manager_.ApplySafetyPolicy(&cmd);
  ASSERT_EQ(safety_manager_.GetState(), SafetyState::kHardEstop);

  // Verify emergency braking strategy execution
  EXPECT_DOUBLE_EQ(cmd.brake(), 100.0);
  EXPECT_EQ(cmd.gear_location(), apollo::canbus::Chassis::GEAR_PARKING);
  EXPECT_TRUE(cmd.signal().emergency_light());
}

/**
 * @test Verify State Latching: System stays in HardEstop until manual reset.
 */
TEST_F(SafetyManagerTest, StateLatchingLogic) {
  LocalView view;
  ControlCommand cmd;

  // 1. Trigger HardEstop: Set DRIVE mode + negative speed (CheckKinematics
  // logic)
  view.mutable_chassis()->set_gear_location(
      apollo::canbus::Chassis::GEAR_DRIVE);
  auto* pt = view.mutable_trajectory()->add_trajectory_point();
  pt->set_v(-1.0);  // Triggered 0x0205 (517) LEVEL_HARD_ESTOP

  safety_manager_.PreCheck(view);
  safety_manager_.ApplySafetyPolicy(&cmd);  // Driver Arbitration
  ASSERT_EQ(safety_manager_.GetState(), SafetyState::kHardEstop);

  // 2. Clear fault: Set speed to a positive value, keep gear in DRIVE
  pt->set_v(1.0);
  safety_manager_.PreCheck(view);  // active_faults_ will be cleared.

  // 3. Re-drive arbitration to verify whether the state remains in HardEstop
  // due to the latching logic.
  safety_manager_.ApplySafetyPolicy(&cmd);

  // The state should remain locked in HardEstop until manually triedReset.
  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kHardEstop);
}

/**
 * @test Verify SoftStop auto-recovery (debouncer reset).
 */
TEST_F(SafetyManagerTest, SoftStopAutoRecovery) {
  LocalView view;
  ControlCommand cmd;

  // 1. Enter SoftStop (3-step debouncing)
  for (int i = 0; i < 30; ++i) {
    safety_manager_.PreCheck(view);
  }
  safety_manager_.ApplySafetyPolicy(&cmd);
  ASSERT_EQ(safety_manager_.GetState(), SafetyState::kSoftStop);

  // 2. Restore input
  auto* pt = view.mutable_trajectory()->add_trajectory_point();
  pt->set_v(1.0);
  pt->set_a(0.0);

  safety_manager_.PreCheck(view);
  safety_manager_.ApplySafetyPolicy(
      &cmd);  // The driver checks and executes the Normal policy (direct
              // return).

  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kNormal);
}

/**
 * @test Verify Manual Reset functionality.
 */
TEST_F(SafetyManagerTest, ManualResetSuccessful) {
  LocalView view;
  ControlCommand cmd;

  view.mutable_chassis()->set_gear_location(
      apollo::canbus::Chassis::GEAR_DRIVE);
  view.mutable_trajectory()->add_trajectory_point()->set_v(-1.0);

  safety_manager_.PreCheck(view);
  safety_manager_.ApplySafetyPolicy(&cmd);
  ASSERT_EQ(safety_manager_.GetState(), SafetyState::kHardEstop);

  PadMessage pad;
  pad.set_action(DrivingAction::RESET);

  // 2. The fault persists (PreCheck will continue to report), Reset should
  // fail.
  safety_manager_.TryReset(pad);
  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kHardEstop);

  // 3. Eliminate the fault (revert to a legal trajectory and gear).
  view.mutable_chassis()->set_gear_location(
      apollo::canbus::Chassis::GEAR_NEUTRAL);
  view.mutable_trajectory()->clear_trajectory_point();
  view.mutable_trajectory()->add_trajectory_point()->set_v(0.0);

  safety_manager_.PreCheck(view);  // active_faults_ was cleared

  // 4. Reset successful
  safety_manager_.TryReset(pad);
  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kNormal);
}

/**
 * @test NEW: Verify that the debouncer resets if a single valid frame appears.
 */
TEST_F(SafetyManagerTest, DebouncerResetsOnValidFrame) {
  LocalView view;  // Invalid

  // 2 consecutive faults
  safety_manager_.PreCheck(view);
  safety_manager_.PreCheck(view);
  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kNormal);

  // 1 valid frame
  auto* pt = view.mutable_trajectory()->add_trajectory_point();
  pt->set_v(1.0);
  safety_manager_.PreCheck(view);

  // 1 more fault (total was 2 before, but should have reset to 0)
  view.mutable_trajectory()->clear_trajectory_point();
  safety_manager_.PreCheck(view);

  // State should still be Normal because counter reset to 0
  EXPECT_EQ(safety_manager_.GetState(), SafetyState::kNormal);
}

}  // namespace control
}  // namespace apollo
