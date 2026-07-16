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

#include <chrono>
#include <cmath>

#include "cyber/common/log.h"
#include "modules/common/util/message_util.h"

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
  last_trajectory_sequence_num_ = std::numeric_limits<uint32_t>::max();
  return true;
}

SafetyResult SafetyManager::PreCheck(const LocalView& view) {
  std::lock_guard<std::mutex> lk(mutex_);
  active_faults_.clear();
  SafetyResult result;

  CheckPlanningInput(view, &result);

  CheckKinematics(view, &result);

  return result;
}

SafetyResult SafetyManager::PostCheck(const ControlCommand& cmd) {
  std::lock_guard<std::mutex> lk(mutex_);
  SafetyResult result;

  CheckControlOutput(cmd, &result);

  return result;
}

void SafetyManager::CheckPlanningInput(const LocalView& view,
                                       SafetyResult* result) {
  const auto& trajectory = view.trajectory();
  const bool trajectory_stale = !common::util::IsHeaderSequenceNumUpdated(
      trajectory.header(), &last_trajectory_sequence_num_);

  const bool trajectory_empty = trajectory.trajectory_point().empty();
  bool trajectory_point_invalid = false;
  if (!trajectory_empty) {
    for (const auto& pt : trajectory.trajectory_point()) {
      if (!std::isfinite(pt.v()) || !std::isfinite(pt.a()) ||
          !std::isfinite(pt.relative_time()) ||
          !std::isfinite(pt.path_point().x()) ||
          !std::isfinite(pt.path_point().y()) ||
          !std::isfinite(pt.path_point().theta()) ||
          !std::isfinite(pt.path_point().s()) ||
          !std::isfinite(pt.path_point().kappa())) {
        trajectory_point_invalid = true;
        break;
      }
    }
  }

  // Missing trajectory points or non-finite trajectory geometry are immediate
  // bypass conditions. Stale sequence numbers are debounced separately because
  // planning runs at lower frequency than control.
  if (trajectory_empty || trajectory_point_invalid) {
    result->must_bypass = true;
  }

  const bool trajectory_lost =
      trajectory_stale || trajectory_empty || trajectory_point_invalid;
  if (trajectory_loss_debouncer_->Update(trajectory_lost)) {
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

void SafetyManager::CheckControlOutput(const ControlCommand& cmd,
                                       SafetyResult* result) {
  bool invalid = false;
  constexpr double kMaxDouble = std::numeric_limits<double>::max();

  auto validate_field = [&](bool has_field, double value, double min_value,
                            double max_value, const char* name) {
    // Optional field.
    if (!has_field) {
      return;
    }

    if (!std::isfinite(value)) {
      AERROR_EVERY(10) << "Invalid ControlCommand field: " << name << " = "
                       << value;
      invalid = true;
      return;
    }

    if (value < min_value || value > max_value) {
      AERROR_EVERY(10) << "Out-of-range ControlCommand field: " << name << " = "
                       << value << " (expected [" << min_value << ", "
                       << max_value << "])";
      invalid = true;
    }
  };

  validate_field(cmd.has_throttle(), cmd.throttle(), 0.0, 100.0, "throttle");
  validate_field(cmd.has_brake(), cmd.brake(), 0.0, 100.0, "brake");
  validate_field(cmd.has_steering_target(), cmd.steering_target(), -100.0,
                 100.0, "steering_target");
  validate_field(cmd.has_steering_rate(), cmd.steering_rate(), 0.0, 100.0,
                 "steering_rate");
  validate_field(cmd.has_speed(), cmd.speed(), -kMaxDouble, kMaxDouble,
                 "speed");
  validate_field(cmd.has_acceleration(), cmd.acceleration(), -kMaxDouble,
                 kMaxDouble, "acceleration");

  if (invalid) {
    result->need_freeze = true;
  }

  if (output_fault_debouncer_->Update(invalid)) {
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
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  e.set_timestamp_sec(std::chrono::duration<double>(now).count());
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
  // TODO(daohu527): Define warning mitigation strategy.
  // For example, triggering packet recording via the monitor module.
}

void SafetyManager::ExecuteSoftStop(ControlCommand* cmd) {
  // Comfortable deceleration, keep steering control if possible
  cmd->set_throttle(0.0);
  cmd->set_brake(conf_.soft_estop_brake());
  cmd->set_speed(0.0);

  // TODO: Replace actuator override with controlled-stop trajectory generation.

  // Hazard warning
  cmd->mutable_signal()->set_emergency_light(true);
}

void SafetyManager::ExecuteHardEstop(ControlCommand* cmd) {
  // Immediate emergency braking
  cmd->set_throttle(0.0);
  cmd->set_brake(100.0);
  cmd->set_speed(0.0);

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
