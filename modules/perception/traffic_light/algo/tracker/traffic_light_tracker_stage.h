/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "modules/perception/camera/lib/interface/base_traffic_light_tracker.h"
#include "modules/perception/camera/lib/traffic_light/tracker/semantic_decision.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace algo {

class TrafficLightTrackerStage final : public camera::BaseTrafficLightTracker {
 public:
  TrafficLightTrackerStage();
  ~TrafficLightTrackerStage() override = default;

  bool Init(const camera::TrafficLightTrackerInitOptions& options) override;

  bool Init(const pipeline::StageConfig& stage_config) override;

  bool ValidateStageConfig(
      const pipeline::StageConfig& stage_config) const override;

  bool Track(const camera::TrafficLightTrackerOptions& options,
             camera::CameraFrame* frame) override;

  bool Process(pipeline::DataFrame* data_frame) override;

  bool IsEnabled() const override;

  std::string Name() const override;

 private:
  std::unique_ptr<camera::SemanticReviser> impl_;
};

}  // namespace algo
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
