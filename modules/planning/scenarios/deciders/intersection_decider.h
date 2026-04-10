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
#include <vector>

#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/scenarios/deciders/scenario_decider.h"

namespace apollo {
namespace planning {
namespace scenario {

class IntersectionDecider : public ScenarioDecider {
 public:
  IntersectionDecider(const ScenarioConfig& config,
                      const std::shared_ptr<DependencyInjector>& injector)
      : ScenarioDecider(config, injector) {}

  ScenarioDecisionResult MakeDecision(const DeciderContext& context) override;

  std::string_view Name() const override { return "IntersectionDecider"; }

 private:
  /**
   * @brief Check for Traffic Lights (Red/Yellow).
   * Priority: High (Regulatory)
   */
  ScenarioDecisionResult CheckTrafficLight(const DeciderContext& context);

  /**
   * @brief Check for Stop Signs.
   * Priority: Medium-High (Regulatory)
   */
  ScenarioDecisionResult CheckStopSign(const DeciderContext& context);

  /**
   * @brief Check for Yield Signs.
   * Priority: Medium (Regulatory)
   */
  ScenarioDecisionResult CheckYieldSign(const DeciderContext& context);

  /**
   * @brief Check for Uncontrolled Intersections (No lights/signs).
   * Priority: Low (Geometric)
   */
  ScenarioDecisionResult CheckBareIntersection(const DeciderContext& context);

  /**
   * @brief Helper to calculate dynamic score based on proximity.
   */
  double CalculateDistanceScore(double distance, double max_distance,
                                double base_score);
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
