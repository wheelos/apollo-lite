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

#include "modules/planning/scenarios/maneuver/escape/escape_scenario.h"

#include <cmath>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace escape {

namespace {

constexpr double kEscapeReleaseSpeedThreshold = 0.2;

class EscapeStage final : public Stage {
 public:
  EscapeStage(const ScenarioConfig::StageConfig& config,
              const std::shared_ptr<DependencyInjector>& injector)
      : Stage(config, injector) {}

  StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override {
    CHECK_NOTNULL(frame);

    const double speed = std::abs(frame->vehicle_state().linear_velocity());
    auto* blocking_status = injector_->history()->mutable_blocking_status();
    if (speed > kEscapeReleaseSpeedThreshold ||
        !blocking_status->has_start_stuck_time()) {
      blocking_status->clear_start_stuck_time();
      return FinishScenario();
    }

    const bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
    if (!plan_ok) {
      AERROR << "EscapeStage failed to produce a reference-line plan.";
    }
    return Stage::RUNNING;
  }
};

}  // namespace

std::unique_ptr<Stage> EscapeScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (stage_config.stage_type() == StageType::ESCAPE_DEFAULT_STAGE) {
    return std::make_unique<EscapeStage>(stage_config, injector);
  }
  return nullptr;
}

}  // namespace escape
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
