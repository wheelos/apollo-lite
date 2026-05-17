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

#include "modules/planning/scenarios/park/mission_idle/mission_idle_scenario.h"

#include "modules/planning/scenarios/park/mission_idle/mission_idle_stage.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace mission_idle {

void MissionIdleScenario::Init() {
  if (init_) {
    return;
  }

  Scenario::Init();

  if (GetStageType() == StageType::NO_STAGE) {
    return;
  }

  init_ = true;
}

std::unique_ptr<Stage> MissionIdleScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (stage_config.stage_type() == StageType::MISSION_IDLE_DEFAULT_STAGE) {
    return std::unique_ptr<Stage>(new MissionIdleStage(stage_config, injector));
  }
  return nullptr;
}

}  // namespace mission_idle
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
