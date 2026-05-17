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

#include "modules/planning/scenarios/stage.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace mission_idle {

class MissionIdleStage : public Stage {
 public:
  MissionIdleStage(const ScenarioConfig::StageConfig& config,
                   const std::shared_ptr<DependencyInjector>& injector)
      : Stage(config, injector) {}

  Stage::StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                             Frame* frame) override;

 private:
  /**
   * @brief Helper to generate a standstill trajectory to keep the vehicle safe.
   */
  void GenerateStopTrajectory(
      const common::TrajectoryPoint& planning_init_point, Frame* frame);
};

}  // namespace mission_idle
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
