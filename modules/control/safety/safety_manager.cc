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

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace control {

bool SafetyManager::Init(const ControlConf& conf) {
  conf_ = conf;
  current_state_ = SafetyState::kNormal;

  {
    std::lock_guard<std::mutex> lk(mutex_);
    active_faults_.clear();
  }

  // Control logic runs at 100Hz (10ms/cycle).
  // Planning logic runs at 10Hz (100ms/cycle).
  // Setting debouncer to 30 allows for a maximum of 300ms (3 missing planning
  // frames) before triggering a trajectory loss fault.
  trajectory_loss_debouncer_ = std::make_unique<CounterDebouncer>(30);
  output_fault_debouncer_ = std::make_unique<CounterDebouncer>(3);
  return true;
}

SafetyResult SafetyManager::PreCheck(const LocalView& view) {
  std::lock_guard<std::mutex> lk(mutex_);
  active_faults_.clear();
  SafetyResult result;

  CheckPlanningTrajectory(view, &result);

  CheckKinematics(view, &result);

  return result;
}

SafetyResult SafetyManager::PostCheck(const ControlCommand& cmd,
                                      const ControlCommand& prev_cmd) {
  std::lock_guard<std::mutex> lk(mutex_);
  SafetyResult result;

  CheckControlOutputDynamic(cmd, prev_cmd, &result);

  return result;
}

void SafetyManager::CheckPlanningTrajectory(const LocalView& view,
                                            SafetyResult* result) {
  bool traj_invalid = false;

  if (view.trajectory().trajectory_point().empty()) {
    traj_invalid = true;
  } else {
    for (const auto& pt : view.trajectory().trajectory_point()) {
      if (std::isnan(pt.v()) || std::isnan(pt.a()) || std::isinf(pt.v()) ||
          std::isinf(pt.a())) {
        traj_invalid = true;
        break;
      }
    }
  }

  if (traj_invalid) {
    result->must_bypass = true;
  }

  if (trajectory_loss_debouncer_->Update(traj_invalid)) {
    ReportFault(0x0203, FaultLevel::LEVEL_SOFT_STOP, FaultSource::SOURCE_INPUT);
  }
}

void SafetyManager::CheckKinematics(const LocalView& view,
                                    SafetyResult* result) {
  if (view.trajectory().trajectory_point().empty()) return;

  const double kEpsilon = 0.001;
  auto first_pt = view.trajectory().trajectory_point(0);

  // Check if the gear selection conflicts with the intended speed (e.g., a
  // forward gear is planned for reverse speed).
  if (view.chassis().gear_location() == canbus::Chassis::GEAR_DRIVE &&
      first_pt.v() < -kEpsilon) {
    ReportFault(0x0205, FaultLevel::LEVEL_HARD_ESTOP,
                FaultSource::SOURCE_INPUT);
  }
}

void SafetyManager::CheckControlOutputDynamic(const ControlCommand& cmd,
                                              const ControlCommand& prev_cmd,
                                              SafetyResult* result) {
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  // 1. Physical inspection logic: Acceleration over-limit check
  bool is_accel_insane =
      std::abs(cmd.acceleration()) > vehicle_param.max_acceleration();

  if (is_accel_insane) {
    result->need_freeze = true;  // Intercept this frame
  }

  if (output_fault_debouncer_->Update(is_accel_insane)) {
    ReportFault(0x0302, FaultLevel::LEVEL_HARD_ESTOP,
                FaultSource::SOURCE_OUTPUT);
  }
}

void SafetyManager::ReportFault(uint32_t id, FaultLevel level,
                                FaultSource source) {
  if (active_faults_.size() >= active_faults_.capacity()) {
    AERROR_EVERY(10) << "Critical: Fault buffer overflow! ID " << id
                     << " ignored.";
    return;
  }

  active_faults_.emplace_back();
  auto& e = active_faults_.back();
  e.set_fault_id(id);
  e.set_level(level);
  e.set_source(source);
  e.set_timestamp_sec(apollo::cyber::Clock::NowInSeconds());
}

void SafetyManager::Arbitrate() {
  FaultLevel max_level = FaultLevel::LEVEL_NONE;

  // Find the highest severity level among all active faults
  for (const auto& event : active_faults_) {
    AINFO << event.DebugString();
    if (event.has_level() && event.level() > max_level) {
      max_level = event.level();
    }
  }

  // State Machine Latching Logic:
  // State can ONLY escalate automatically.
  // State downgrade MUST be triggered by manual Reset.
  if (max_level == FaultLevel::LEVEL_NONE) {
    if (current_state_ < SafetyState::kHardEstop) {
      current_state_ = SafetyState::kNormal;  // Automatic recovery
    }
  } else if (static_cast<int>(max_level) > static_cast<int>(current_state_)) {
    current_state_ = static_cast<SafetyState>(max_level);  // upgrade
  }
}

void SafetyManager::ExecuteWarningPolicy(ControlCommand* cmd) {
  // Limited speed for minor issues
  const double kWarningMaxSpeed = 5.0;
  if (cmd->speed() > kWarningMaxSpeed) {
    cmd->set_speed(kWarningMaxSpeed);
  }
}

void SafetyManager::ExecuteSoftStop(ControlCommand* cmd) {
  // Comfortable deceleration, keep steering control if possible
  cmd->set_throttle(0.0);
  cmd->set_brake(conf_.soft_estop_brake());
  cmd->set_gear_location(apollo::canbus::Chassis::GEAR_DRIVE);
}

void SafetyManager::ExecuteHardEstop(ControlCommand* cmd) {
  // Immediate emergency braking
  cmd->set_throttle(0.0);
  cmd->set_brake(100.0);
  cmd->set_speed(0.0);
  cmd->set_parking_brake(true);
  cmd->set_steering_rate(0.0);
  cmd->set_gear_location(apollo::canbus::Chassis::GEAR_PARKING);

  // Hazard warning
  cmd->mutable_signal()->set_emergency_light(true);
}

void SafetyManager::ApplySafetyPolicy(ControlCommand* cmd) {
  Arbitrate();

  if (current_state_ == SafetyState::kNormal) return;

  switch (current_state_) {
    case SafetyState::kWarning:
      ExecuteWarningPolicy(cmd);
      break;

    case SafetyState::kSoftStop:
      ExecuteSoftStop(cmd);
      break;

    case SafetyState::kHardEstop:
    case SafetyState::kFatal:
      ExecuteHardEstop(cmd);
      break;

    default:
      break;
  }
}

void SafetyManager::TryReset(const PadMessage& pad_msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  if (pad_msg.action() != DrivingAction::RESET) return;

  // Industrial Reset Conditions:
  // 1. No active instantaneous faults (active_faults_ is empty).
  // 2. Debouncers have cooled down.
  // 3. (Optional) Vehicle speed is near zero.

  if (active_faults_.empty()) {
    // Reset FSM State
    current_state_ = SafetyState::kNormal;

    AINFO << "Safety System RESET by Operator.";
  } else {
    AWARN << "Reset Request Denied: Faults are still active.";
    for (const auto& f : active_faults_) {
      AWARN << "Active Fault ID: " << f.fault_id();
    }
  }
}

}  // namespace control
}  // namespace apollo
