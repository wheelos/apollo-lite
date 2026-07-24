// Copyright 2025 WheelOS All Rights Reserved.
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

//  Created Date: 2025-12-07
//  Author: daohu527

#include "modules/planning/scenarios/deciders/emergency_decider.h"

#include "wheelos_msgs/planning_msgs/pad_msg.pb.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace scenario {

ScenarioDecisionResult EmergencyDecider::MakeDecision(
    const DeciderContext& context) {
  const auto& frame = context.frame;
  const auto& emergency_pull_over_config = config_.emergency_pull_over_config();
  const auto& emergency_stop_config = config_.emergency_stop_config();
  emergency_pull_over_score_ =
      emergency_pull_over_config.scenario_entry_score();
  emergency_stop_score_ = emergency_stop_config.scenario_entry_score();

  // 1. Human Interface (HMI) Override
  // This is the highest priority. If a human hits "STOP", we STOP.
  auto decision = CheckPadMsg(frame);
  if (decision.IsValid()) {
    return decision;
  }

  // 2. System Integrity Check
  // If the car is physically unsafe (chassis dead) or lost (localization lost),
  // we must degrade immediately.
  decision = CheckSystemFaults(frame);
  if (decision.IsValid()) {
    return decision;
  }

  // 3. Internal Watchdog
  // Logic from Monitor or other safety modules.
  decision = CheckInternalRequest(frame);
  if (decision.IsValid()) {
    return decision;
  }

  // No emergency detected.
  return ScenarioDecisionResult();
}

ScenarioDecisionResult EmergencyDecider::CheckPadMsg(const Frame* frame) {
  const auto& pad_msg_driving_action = frame->GetPadMsgDrivingAction();

  // Action: Pull Over (Controlled Stop)
  if (pad_msg_driving_action == PadMessage::PULL_OVER) {
    return ScenarioDecisionResult(
        ScenarioType::EMERGENCY_PULL_OVER, ScenarioGrade::CRITICAL,
        emergency_pull_over_score_, "PadMsg Request: PULL_OVER");
  }

  // Action: Immediate Stop (E-Stop)
  if (pad_msg_driving_action == PadMessage::STOP) {
    return ScenarioDecisionResult(
        ScenarioType::EMERGENCY_STOP, ScenarioGrade::CRITICAL,
        emergency_stop_score_, "PadMsg Request: STOP");
  }

  return ScenarioDecisionResult();
}

ScenarioDecisionResult EmergencyDecider::CheckSystemFaults(const Frame* frame) {
  // Access global planning status
  // const auto& status = injector_->planning_context()->planning_status();

  // TODO(zero): Need to be complete!
  // 1. Chassis Fault (Brake/Steering/Throttle failure)
  // 2. Localization Fault (GPS/Lidar matching failure)
  // bool is_chassis_error = status.has_chassis_error() &&
  // status.chassis_error(); bool is_localization_unstable =
  //     status.has_localization_unstable() && status.localization_unstable();

  // if (is_chassis_error || is_localization_unstable) {
  //   // If we have a dedicated fault handling scenario
  //   std::string reason =
  //       is_chassis_error ? "Chassis Error" : "Localization Unstable";
  //   return ScenarioDecisionResult(
  //       ScenarioType::EMERGENCY_STOP,  // Or FAULT_MANAGEMENT
  //       ScenarioGrade::CRITICAL, emergency_stop_score_, reason);
  // }

  return ScenarioDecisionResult();
}

ScenarioDecisionResult EmergencyDecider::CheckInternalRequest(
    const Frame* frame) {
  const auto& planning_status =
      injector_->planning_context()->planning_status();

  // TODO(zero): need complete reason!
  // Check if the Monitor module has flagged an emergency
  // (e.g., HDMap mismatch, Prediction latency high)
  if (planning_status.has_emergency_stop() &&
      planning_status.emergency_stop().has_stop_fence_point()) {
    return ScenarioDecisionResult(ScenarioType::EMERGENCY_STOP,
                                  ScenarioGrade::CRITICAL,
                                  emergency_stop_score_, "Internal Monitor: ");
  }

  return ScenarioDecisionResult();
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
