/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/perception/traffic_light/algo/detector/traffic_light_detector_stage.h"

#include <utility>

namespace apollo {
namespace perception {
namespace traffic_light {
namespace algo {

TrafficLightDetectorStage::TrafficLightDetectorStage()
    : impl_(new camera::TrafficLightDetection()) {}

bool TrafficLightDetectorStage::Init(
    const camera::TrafficLightDetectorInitOptions& options) {
  return impl_ != nullptr && impl_->Init(options);
}

bool TrafficLightDetectorStage::Init(const pipeline::StageConfig& stage_config) {
  if (!ValidateStageConfig(stage_config) || impl_ == nullptr) {
    return false;
  }
  return impl_->Init(stage_config);
}

bool TrafficLightDetectorStage::ValidateStageConfig(
    const pipeline::StageConfig& stage_config) const {
  (void)stage_config;
  return impl_ != nullptr;
}

bool TrafficLightDetectorStage::Detect(
    const camera::TrafficLightDetectorOptions& options,
    camera::CameraFrame* frame) {
  return impl_ != nullptr && impl_->Detect(options, frame);
}

std::string TrafficLightDetectorStage::Name() const {
  return "TrafficLightDetectorStage";
}

bool TrafficLightDetectorStage::Process(pipeline::DataFrame* data_frame) {
  return impl_ != nullptr && impl_->Process(data_frame);
}

bool TrafficLightDetectorStage::IsEnabled() const {
  return impl_ != nullptr && impl_->IsEnabled();
}

}  // namespace algo
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
