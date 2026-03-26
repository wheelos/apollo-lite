/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/perception/traffic_light/algo/tracker/traffic_light_tracker_stage.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace algo {

TrafficLightTrackerStage::TrafficLightTrackerStage()
    : impl_(new camera::SemanticReviser()) {}

bool TrafficLightTrackerStage::Init(
    const camera::TrafficLightTrackerInitOptions& options) {
  return impl_ != nullptr && impl_->Init(options);
}

bool TrafficLightTrackerStage::Init(const pipeline::StageConfig& stage_config) {
  if (!ValidateStageConfig(stage_config) || impl_ == nullptr) {
    return false;
  }
  return impl_->Init(stage_config);
}

bool TrafficLightTrackerStage::ValidateStageConfig(
    const pipeline::StageConfig& stage_config) const {
  (void)stage_config;
  return impl_ != nullptr;
}

bool TrafficLightTrackerStage::Track(
    const camera::TrafficLightTrackerOptions& options,
    camera::CameraFrame* frame) {
  return impl_ != nullptr && impl_->Track(options, frame);
}

bool TrafficLightTrackerStage::Process(pipeline::DataFrame* data_frame) {
  return impl_ != nullptr && impl_->Process(data_frame);
}

bool TrafficLightTrackerStage::IsEnabled() const {
  return impl_ != nullptr && impl_->IsEnabled();
}

std::string TrafficLightTrackerStage::Name() const {
  return "TrafficLightTrackerStage";
}

}  // namespace algo
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
