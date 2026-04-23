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

#include <memory>
#include <string>
#include <unordered_map>

#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/scenarios/deciders/scenario_decider.h"

namespace apollo {
namespace planning {
namespace scenario {

class EmergencyDecider : public ScenarioDecider {
 public:
  EmergencyDecider(const ScenarioConfig& config,
                   const std::shared_ptr<DependencyInjector>& injector)
      : ScenarioDecider(config, injector) {}

  ScenarioDecisionResult MakeDecision(const DeciderContext& context) override;

  std::string_view Name() const override { return "EmergencyDecider"; }

 private:
  /**
   * @brief Check for manual intervention commands from HMI/Pad.
   * Priority: High (Human Override)
   */
  ScenarioDecisionResult CheckPadMsg(const Frame* frame);

  /**
   * @brief Check for critical hardware or platform faults.
   * Priority: Critical (Safety Integrity)
   */
  ScenarioDecisionResult CheckSystemFaults(const Frame* frame);

  /**
   * @brief Check for internal software requests (e.g., from Monitor).
   * Priority: Medium (Software Failsafe)
   */
  ScenarioDecisionResult CheckInternalRequest(const Frame* frame);

 private:
  uint32_t emergency_pull_over_score_;
  uint32_t emergency_stop_score_;
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
