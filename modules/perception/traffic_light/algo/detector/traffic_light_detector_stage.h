/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/camera/lib/interface/base_traffic_light_detector.h"
#include "modules/perception/camera/lib/traffic_light/detector/detection.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace algo {

class TrafficLightDetectorStage final : public camera::BaseTrafficLightDetector {
 public:
  TrafficLightDetectorStage();
  ~TrafficLightDetectorStage() override = default;

  bool Init(const camera::TrafficLightDetectorInitOptions& options) override;

  bool Init(const pipeline::StageConfig& stage_config) override;

  bool ValidateStageConfig(
      const pipeline::StageConfig& stage_config) const override;

  bool Detect(const camera::TrafficLightDetectorOptions& options,
              camera::CameraFrame* frame) override;

  std::string Name() const override;

  bool Process(pipeline::DataFrame* data_frame) override;

  bool IsEnabled() const override;

 private:
  std::unique_ptr<camera::TrafficLightDetection> impl_;
};

}  // namespace algo
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
