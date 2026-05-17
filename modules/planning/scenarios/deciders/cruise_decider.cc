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

#include "modules/planning/scenarios/deciders/cruise_decider.h"

namespace apollo {
namespace planning {
namespace scenario {

ScenarioDecisionResult CruiseDecider::MakeDecision(
    const DeciderContext& context) {
  // 1. Basic Validity Check
  // The core assumption for the LaneFollow scenario is the existence of a
  // drivable ReferenceLine. If the reference line list is empty (e.g., due to
  // map request or routing failure), entering LaneFollow is meaningless.
  // Returning an empty result means the CruiseDecider relinquishes the
  // decision, which might trigger error handling within
  // ScenarioManager::Update.
  const auto& frame = context.frame;
  if (frame->reference_line_info().empty()) {
    return ScenarioDecisionResult();
  }

  const auto& config = config_.lane_follow_config();

  // 2. Default Cruising Mode: Lane Follow
  ScenarioDecisionResult result;
  result.type = ScenarioType::LANE_FOLLOW;
  result.grade = ScenarioGrade::CRUISE;
  result.score = config.scenario_entry_score();
  result.reason = "Default Cruise (Lane Follow)";
  result.can_enter = true;

  return result;
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
