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

#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace narrow_street {

class NarrowStreetScenario : public Scenario {
 public:
  NarrowStreetScenario(const ScenarioConfig& config, ScenarioContext* context,
                       const std::shared_ptr<DependencyInjector>& injector)
      : Scenario(config, context, injector) {}

  std::unique_ptr<Stage> CreateStage(
      const ScenarioConfig::StageConfig& stage_config,
      const std::shared_ptr<DependencyInjector>& injector) override;

  ScenarioGrade Grade() const override { return ScenarioGrade::MISSION; }
};

}  // namespace narrow_street
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
