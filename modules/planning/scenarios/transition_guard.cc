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

#include "modules/planning/scenarios/transition_guard.h"

#include "cyber/common/log.h"

namespace apollo {
namespace planning {
namespace scenario {

void TransitionGuard::Init(const PlanningConfig& config) {
  allow_list_.clear();

  if (config.has_scenario_transition_config()) {
    for (const auto& rule :
         config.scenario_transition_config().transition_rules()) {
      ScenarioType from = rule.from_scenario_type();
      // Protobuf stores enums as ints in repeated fields
      for (const auto& to_int : rule.to_scenario_types()) {
        ScenarioType to = static_cast<ScenarioType>(to_int);
        allow_list_[from].insert(to);
      }
    }
  }
}

bool TransitionGuard::IsTransitionAllowed(
    const ScenarioType current_type,
    const Scenario::ScenarioStatus current_status,
    const ScenarioType target_type, const ScenarioGrade target_grade) {
  // 1. Self-loop is always allowed (Keep Current).
  if (current_type == target_type) {
    return true;
  }

  // 2. CRITICAL scenarios (Grade 3) have highest privilege.
  // e.g., EmergencyStop triggers should interrupt anything.
  if (target_grade == ScenarioGrade::CRITICAL) {
    return true;
  }

  // 3. Static Whitelist Check (Config).
  auto it = allow_list_.find(current_type);
  if (it == allow_list_.end()) {
    // Safety Net: If the source scenario has no defined exit rules,
    // explicitly allow switching back to LANE_FOLLOW to prevent deadlocks.
    if (target_type == ScenarioType::LANE_FOLLOW) {
      return true;
    }
    ADEBUG << "Transition rejected (No rules found): "
           << ScenarioType_Name(current_type) << " -> "
           << ScenarioType_Name(target_type);
    return false;
  }

  // Check if target is in the allowed set.
  if (it->second.count(target_type) == 0) {
    ADEBUG << "Transition rejected (Not in whitelist): "
           << ScenarioType_Name(current_type) << " -> "
           << ScenarioType_Name(target_type);
    return false;
  }

  // 4. Dynamic State Check (Runtime Protection).
  // If the current scenario is not finished, check if it allows interruption.
  if (current_status != Scenario::ScenarioStatus::STATUS_DONE) {
    // Specific check for Emergency scenarios (Double check, though Grade check
    // handles most)
    if (target_type == ScenarioType::EMERGENCY_PULL_OVER ||
        target_type == ScenarioType::EMERGENCY_STOP) {
      return true;
    }

    // If current scenario is atomic (Maneuver) and target is not Critical,
    // reject.
    if (!IsInterruptible(current_type)) {
      ADEBUG << "Transition rejected (Scenario In-Progress): "
             << ScenarioType_Name(current_type)
             << " is not done yet and cannot be interrupted by "
             << ScenarioType_Name(target_type);
      return false;
    }
  }

  return true;
}

bool TransitionGuard::IsInterruptible(const ScenarioType scenario_type) const {
  switch (scenario_type) {
    case ScenarioType::LANE_FOLLOW:
      return true;  // Always interruptible

    // Atomic Maneuvers: Must complete before switching (unless Emergency)
    case ScenarioType::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioType::STOP_SIGN_PROTECTED:
    case ScenarioType::STOP_SIGN_UNPROTECTED:
    case ScenarioType::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioType::VALET_PARKING:
    case ScenarioType::PULL_OVER:
    case ScenarioType::YIELD_SIGN:
      return false;

    default:
      return true;
  }
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
