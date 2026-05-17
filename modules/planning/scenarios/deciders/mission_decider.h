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

class MissionDecider : public ScenarioDecider {
 public:
  MissionDecider(const ScenarioConfig& config,
                 const std::shared_ptr<DependencyInjector>& injector)
      : ScenarioDecider(config, injector) {}

  ScenarioDecisionResult MakeDecision(const DeciderContext& context) override;

  std::string_view Name() const override { return "MissionDecider"; }

 private:
  /**
   * @brief Check if the vehicle has arrived at the destination or has no valid
   * routing. Priority: Very High (L1)
   */
  ScenarioDecisionResult CheckMissionIdle(const DeciderContext& context);

  /**
   * @brief Check if the vehicle is entering a narrow street zone based on map
   * data. Priority: High (L1 - Mission Mode)
   */
  ScenarioDecisionResult CheckNarrowStreet(const DeciderContext& context);

  /**
   * @brief Check if the upcoming junction requires a U-Turn maneuver based on
   * routing. Priority: Medium-High (L2 - Complex Maneuver)
   */
  ScenarioDecisionResult CheckUTurn(const DeciderContext& context);

  bool IsCloseToDestination(const Frame* frame);
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
