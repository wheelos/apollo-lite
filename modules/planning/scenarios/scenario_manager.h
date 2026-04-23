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

#include "gtest/gtest_prod.h"

#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/scenarios/deciders/scenario_decider.h"
#include "modules/planning/scenarios/scenario.h"
#include "modules/planning/scenarios/transition_guard.h"

namespace apollo {
namespace planning {
namespace scenario {

class ScenarioManager final {
 public:
  FRIEND_TEST(ScenarioManagerTest, VerifyScenarioTransitions);

  ScenarioManager() = delete;
  explicit ScenarioManager(const std::shared_ptr<DependencyInjector>& injector);

  /**
   * @brief Initialize: Load configurations, init transition guard, register
   * deciders.
   */
  bool Init(const PlanningConfig& planning_config);

  /**
   * @brief Main entry point for every planning frame.
   */
  void Update(const common::TrajectoryPoint& ego_point, const Frame& frame);

  Scenario* mutable_scenario() { return current_scenario_.get(); }
  DependencyInjector* injector() { return injector_.get(); }

 private:
  /**
   * @brief Core dispatch logic: Iterate through deciders, compare scores,
   *        and determine if a scenario transition is needed.
   */
  void ScenarioDispatch(const Frame& frame);

  ScenarioType ScenarioDispatchLearning(const Frame& frame);

  /**
   * @brief Helper: Handle scenario switching life-cycle
   *        (OnExit Old -> Create New -> OnEnter New).
   */
  void SwitchToScenario(ScenarioType new_scenario_type, const Frame& frame);

  /**
   * @brief Helper: Pre-process map information (e.g., first encountered
   * overlaps) to be shared among all deciders for optimization.
   */
  void Observe(const Frame& frame);

  /**
   * @brief Helper: Update PlanningContext side-effects (e.g., traffic light ID)
   *        based on the determined scenario.
   */
  void UpdatePlanningContext(const Frame& frame, const ScenarioType& type);

  void UpdateContextBareIntersection(const ScenarioType& type,
                                     const ScenarioType& current_running_type);

  void UpdateContextStopSign(const ScenarioType& type,
                             const ScenarioType& current_running_type);

  void UpdateContextYieldSign(const ScenarioType& type,
                              const ScenarioType& current_running_type);

  void UpdateContextTrafficLight(const Frame& frame, const ScenarioType& type,
                                 const ScenarioType& current_running_type);

  void UpdateContextPullOver(const Frame& frame, const ScenarioType& type);

  void UpdateContextEmergencyStop(const ScenarioType& type);

  /**
   * @brief Factory: Create specific scenario instance based on type.
   */
  std::unique_ptr<Scenario> CreateScenario(ScenarioType scenario_type);

  /**
   * @brief Load configuration files for all supported scenarios.
   */
  void RegisterScenarios();

  /**
   * @brief Register all available scenario deciders.
   */
  void RegisterDeciders();

 private:
  std::shared_ptr<DependencyInjector> injector_;
  PlanningConfig planning_config_;

  // 1. Transition Control (Allowlist)
  TransitionGuard transition_guard_;

  // 2. Collection of Deciders (replaces the old monolithic logic)
  std::vector<std::unique_ptr<ScenarioDecider>> deciders_;

  // 3. Scenario Configuration Map (Loaded during Init)
  std::unordered_map<ScenarioType, ScenarioConfig, std::hash<int>> config_map_;

  // 4. Current active scenario
  std::shared_ptr<Scenario> current_scenario_;

  // 5. Default scenario type (usually LANE_FOLLOW)
  ScenarioType default_scenario_type_;

  // 6. Shared Data: Cache for first encountered overlaps to avoid repeated
  // lookups
  std::unordered_map<ReferenceLineInfo::OverlapType, hdmap::PathOverlap,
                     std::hash<int>>
      first_encountered_overlap_map_;

  // 7. Last decision result for debugging and hysteresis
  ScenarioDecisionResult last_decision_result_;
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
