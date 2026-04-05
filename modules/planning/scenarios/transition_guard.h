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

#pragma once

#include <map>
#include <set>

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {
namespace scenario {

class TransitionGuard {
 public:
  TransitionGuard() = default;

  /**
   * @brief Initialize the transition whitelist from configuration.
   * @param config The planning configuration containing scenario transition
   * rules.
   */
  void Init(const PlanningConfig& config);

  /**
   * @brief Check if a transition from one scenario to another is allowed.
   *        Combines static whitelist checks and dynamic state checks.
   *
   * @param current_type Current scenario type.
   * @param current_status Current scenario execution status (PROCESSING, DONE,
   * etc.).
   * @param target_type Target scenario type.
   * @param target_grade Grade of the target scenario (used for priority
   * overrides).
   * @return true if transition is allowed, false otherwise.
   */
  bool IsTransitionAllowed(const ScenarioType current_type,
                           const Scenario::ScenarioStatus current_status,
                           const ScenarioType target_type,
                           const ScenarioGrade target_grade);

 private:
  /**
   * @brief Check if the current scenario can be interrupted while it is still
   * running.
   * @param scenario_type The scenario type to check.
   * @return true if it can be interrupted, false if it must run to completion.
   */
  bool IsInterruptible(const ScenarioType scenario_type) const;

 private:
  // Key: Current Scenario, Value: Set of allowed Target Scenarios.
  std::map<ScenarioType, std::set<ScenarioType>> allow_list_;
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
