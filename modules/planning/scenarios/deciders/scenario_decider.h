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
#include <string_view>
#include <unordered_map>

#include "modules/common_msgs/planning_msgs/scenario_type.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {
namespace scenario {

using FirstEncounteredOverlapMap =
    std::unordered_map<ReferenceLineInfo::OverlapType, hdmap::PathOverlap>;

/**
 * @brief Context structure passed to the ScenarioDecider for decision making.
 */
struct DeciderContext {
  const Frame* frame = nullptr;
  Scenario* current_scenario = nullptr;
  const std::unordered_map<ReferenceLineInfo::OverlapType, hdmap::PathOverlap,
                           std::hash<int>>* first_encountered_overlaps =
      nullptr;
};

/**
 * @brief Abstract base class for scenario decision making.
 * * Subclasses implement the core logic to transition between different
 * planning scenarios based on the current context.
 */
class ScenarioDecider {
 public:
  ScenarioDecider(const ScenarioConfig& config,
                  const std::shared_ptr<DependencyInjector>& injector)
      : config_(config), injector_(injector) {}

  virtual ~ScenarioDecider() = default;

  ScenarioDecider(const ScenarioDecider&) = delete;
  ScenarioDecider& operator=(const ScenarioDecider&) = delete;
  ScenarioDecider(ScenarioDecider&&) = delete;
  ScenarioDecider& operator=(ScenarioDecider&&) = delete;

  /**
   * @brief Initialization interface: loads configuration.
   * @param config The configuration protobuf for this specific decider.
   * @return True if initialization succeeds, false otherwise.
   */
  // virtual bool Init(const ScenarioDeciderConfig& config) { return true; }

  /**
   * @brief Core decision interface to select the next scenario.
   * @param context The current planning frame context and overlaps->
   * @return The result of the decision, including the next scenario type.
   */
  virtual ScenarioDecisionResult MakeDecision(
      const DeciderContext& context) = 0;

  /**
   * @brief Returns the name of the decider for logging/performance tracking.
   * @return The name as a string view for performance.
   */
  virtual std::string_view Name() const = 0;

 protected:
  ScenarioConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
