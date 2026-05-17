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

class ParkDecider : public ScenarioDecider {
 public:
  ParkDecider(const ScenarioConfig& config,
              const std::shared_ptr<DependencyInjector>& injector)
      : ScenarioDecider(config, injector) {}

  ScenarioDecisionResult MakeDecision(const DeciderContext& context) override;

  std::string_view Name() const override { return "ParkDecider"; }

 private:
  /**
   * @brief Check if Valet Parking is requested and feasible in current area.
   * Priority: Very High (Specialized Mission)
   */
  ScenarioDecisionResult CheckValetParking(const DeciderContext& context);

  /**
   * @brief Check if vehicle is approaching destination and needs to pull over.
   * Priority: High (Mission Completion)
   */
  ScenarioDecisionResult CheckPullOver(const DeciderContext& context);

  /**
   * @brief Check if vehicle is stationary off-road and needs to merge in.
   * Priority: Medium (Maneuver)
   */
  ScenarioDecisionResult CheckParkAndGo(const DeciderContext& context);

  bool SearchTargetParkingSpotOnPath(const hdmap::Path& nearby_path,
                                     const std::string& target_parking_id,
                                     hdmap::PathOverlap* parking_space_overlap);

  bool CheckDistanceToParkingSpot(
      const Frame* frame, const hdmap::Path& nearby_path,
      const double parking_start_range,
      const hdmap::PathOverlap& parking_space_overlap);
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
