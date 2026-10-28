// Copyright 2026 The Apollo Authors. All Rights Reserved.
#pragma once

#include <memory>

#include "modules/traffic_light/pipeline/pipeline.h"
#include "modules/traffic_light/ports/provider_ports.h"
#include "modules/traffic_light/stages/binder_stage.h"
#include "modules/traffic_light/stages/detector_stage.h"
#include "modules/traffic_light/stages/fusion_stage.h"
#include "modules/traffic_light/stages/heuristic_stage.h"
#include "modules/traffic_light/stages/prompter_stage.h"
#include "modules/traffic_light/stages/tracker_prediction_stage.h"

namespace apollo {
namespace traffic_light {

inline std::unique_ptr<PerceptionPipeline> CreateTrafficLightPipeline(
    const ComponentOptions& options,
    const std::shared_ptr<IDetectorProviderPort>& detector) {
  auto pipeline = std::make_unique<PerceptionPipeline>();
  auto tracker = std::make_shared<TrackerStage>(options.tracker);
  pipeline->RegisterStage(std::make_shared<TrackerPredictionStage>(tracker));
  pipeline->RegisterStage(std::make_shared<PrompterStage>(options.prompter));
  pipeline->RegisterStage(
      std::make_shared<DetectorStage>(options.detector, detector));
  pipeline->RegisterStage(tracker);
  pipeline->RegisterStage(std::make_shared<BinderStage>(options.binder));
  if (options.enable_heuristic_stage) {
    pipeline->RegisterStage(
        std::make_shared<HeuristicStage>(options.heuristic));
  }
  pipeline->RegisterStage(std::make_shared<FusionStage>(options.fusion));
  return pipeline;
}

}  // namespace traffic_light
}  // namespace apollo
