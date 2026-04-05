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

#include "modules/planning/scenarios/scenario.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace planning {
namespace scenario {

using apollo::cyber::common::GetProtoFromFile;

Scenario::Scenario(const ScenarioConfig& config, const ScenarioContext* context,
                   const std::shared_ptr<DependencyInjector>& injector)
    : config_(config), scenario_context_(context), injector_(injector) {
  name_ = ScenarioType_Name(config.scenario_type());
}

bool Scenario::LoadConfig(const std::string& config_file,
                          ScenarioConfig* config) {
  return GetProtoFromFile(config_file, config);
}

void Scenario::Init() {
  if (config_.stage_type().empty()) {
    AERROR << "Scenario " << Name() << " has no stages configured!";
    return;
  }

  // 2. Update global state
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_scenario()
      ->set_scenario_type(config_.scenario_type());

  // 3. Build Stage mapping table
  stage_config_map_.clear();
  for (const auto& stage_config : config_.stage_config()) {
    stage_config_map_[stage_config.stage_type()] = &stage_config;
  }

  // 4. Verify Stage Integrity
  for (int i = 0; i < config_.stage_type_size(); ++i) {
    auto stage_type = config_.stage_type(i);
    if (stage_config_map_.find(stage_type) == stage_config_map_.end()) {
      AERROR << "Stage " << StageType_Name(stage_type)
             << " declared but no config found in " << Name();
    }
  }

  // 5. Create the initial Stage
  const auto& first_stage_type = config_.stage_type(0);
  if (stage_config_map_.count(first_stage_type)) {
    ADEBUG << "Initializing first stage: " << StageType_Name(first_stage_type);
    current_stage_ =
        CreateStage(*stage_config_map_[first_stage_type], injector_);
  }
}

void Scenario::OnEnter(Frame* frame) {
  scenario_status_ = STATUS_PROCESSING;

  // Record start time and other statistical information
  AINFO << "Entering Scenario: " << Name();

  // Reset Stage to the beginning
  if (!config_.stage_type().empty()) {
    auto first_stage = config_.stage_type(0);
    if (current_stage_ == nullptr ||
        current_stage_->stage_type() != first_stage) {
      current_stage_ = CreateStage(*stage_config_map_[first_stage], injector_);
    }
  }
}

void Scenario::OnExit(Frame* frame) { AINFO << "Exiting Scenario: " << Name(); }

Scenario::ScenarioStatus Scenario::Process(Frame* frame) {
  if (current_stage_ == nullptr) {
    AERROR << "Current stage is null in scenario " << Name();
    return STATUS_UNKNOWN;
  }

  // 1. Check if scenario is finished by Logic (e.g. Stage NO_STAGE)
  if (current_stage_->stage_type() == StageType::NO_STAGE) {
    scenario_status_ = STATUS_DONE;
    return scenario_status_;
  }

  // 2. Execute Stage Process
  auto planning_init_point = frame->PlanningStartPoint();
  auto stage_ret = current_stage_->Process(planning_init_point, frame);

  // 3. Handle Stage Transition
  switch (stage_ret) {
    case Stage::ERROR: {
      AERROR << "Stage " << current_stage_->Name() << " error.";
      scenario_status_ = STATUS_UNKNOWN;
      break;
    }
    case Stage::RUNNING: {
      scenario_status_ = STATUS_PROCESSING;
      break;
    }
    case Stage::FINISHED: {
      auto next_stage_type = current_stage_->NextStage();

      if (next_stage_type != current_stage_->stage_type()) {
        AINFO << "Switching stage: " << current_stage_->Name() << " -> "
              << StageType_Name(next_stage_type);

        // Case A: Scenario Done
        if (next_stage_type == StageType::NO_STAGE) {
          scenario_status_ = STATUS_DONE;
          return scenario_status_;
        }

        // Case B: Transition to Next Stage
        if (stage_config_map_.find(next_stage_type) ==
            stage_config_map_.end()) {
          AERROR << "Next stage " << StageType_Name(next_stage_type)
                 << " config not found!";
          scenario_status_ = STATUS_UNKNOWN;
          return scenario_status_;
        }

        current_stage_ =
            CreateStage(*stage_config_map_[next_stage_type], injector_);
        if (!current_stage_) {
          AERROR << "Failed to create next stage instance.";
          return STATUS_UNKNOWN;
        }

        // The state remains PROCESSING only after switching to a valid new
        // Stage.
        scenario_status_ = STATUS_PROCESSING;
      }
      break;
    }
    default: {
      AWARN << "Unknown stage return type.";
      scenario_status_ = STATUS_UNKNOWN;
    }
  }
  return scenario_status_;
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
