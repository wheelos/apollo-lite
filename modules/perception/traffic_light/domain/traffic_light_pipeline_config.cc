/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/traffic_light/domain/traffic_light_pipeline_config.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace domain {

namespace {

constexpr char kDefaultTrafficLightDetectorType[] = "TrafficLightDetectorStage";
constexpr char kDefaultTrafficLightTrackerType[] = "TrafficLightTrackerStage";

}  // namespace

TrafficLightStageBundle ResolveTrafficLightStageBundle(
    const pipeline::PipelineConfig& pipeline_config) {
  TrafficLightStageBundle stage_bundle;
  for (const auto& stage_config : pipeline_config.stage_config()) {
    switch (stage_config.stage_type()) {
      case pipeline::StageType::TRAFFIC_LIGHT_DETECTION:
        stage_bundle.detector_stage = &stage_config;
        break;
      case pipeline::StageType::SEMANTIC_REVISER:
        stage_bundle.tracker_stage = &stage_config;
        break;
      default:
        break;
    }
  }
  return stage_bundle;
}

std::string ResolveTrafficLightDetectorType(
    const pipeline::StageConfig& stage_config) {
  return stage_config.type().empty() ? kDefaultTrafficLightDetectorType
                                     : stage_config.type();
}

std::string ResolveTrafficLightTrackerType(
    const pipeline::StageConfig& stage_config) {
  return stage_config.type().empty() ? kDefaultTrafficLightTrackerType
                                     : stage_config.type();
}

}  // namespace domain
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
