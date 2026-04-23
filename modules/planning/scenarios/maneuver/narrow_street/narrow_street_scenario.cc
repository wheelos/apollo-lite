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

#include "modules/planning/scenarios/maneuver/narrow_street/narrow_street_scenario.h"

#include "cyber/common/log.h"

#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace narrow_street {

namespace {

constexpr double kNarrowStreetWidthThreshold = 0.5;

bool IsStillNarrowStreet(const Frame& frame) {
  if (frame.reference_line_info().empty()) {
    return false;
  }

  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  const double adc_s = reference_line_info.AdcSlBoundary().end_s();

  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!reference_line.GetLaneWidth(adc_s, &lane_left_width,
                                   &lane_right_width)) {
    return false;
  }

  const double vehicle_width = common::VehicleConfigHelper::Instance()
                                   ->GetConfig()
                                   .vehicle_param()
                                   .width();
  const double buffer = lane_left_width + lane_right_width - vehicle_width;
  return buffer > 0.0 && buffer < kNarrowStreetWidthThreshold;
}

class NarrowStreetStage final : public Stage {
 public:
  NarrowStreetStage(const ScenarioConfig::StageConfig& config,
                    const std::shared_ptr<DependencyInjector>& injector)
      : Stage(config, injector) {}

  StageStatus Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override {
    CHECK_NOTNULL(frame);

    if (!IsStillNarrowStreet(*frame)) {
      return FinishScenario();
    }

    const bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
    if (!plan_ok) {
      AERROR << "NarrowStreetStage failed to produce a reference-line plan.";
    }
    return Stage::RUNNING;
  }
};

}  // namespace

std::unique_ptr<Stage> NarrowStreetScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (stage_config.stage_type() ==
      StageType::NARROW_STREET_MANEUVER_DEFAULT_STAGE) {
    return std::make_unique<NarrowStreetStage>(stage_config, injector);
  }
  return nullptr;
}

}  // namespace narrow_street
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
