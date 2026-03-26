/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/perception/traffic_light/algo/preprocessor/traffic_light_preprocessor_stage.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace algo {

TrafficLightPreprocessorStage::TrafficLightPreprocessorStage()
    : impl_(new camera::TLPreprocessor()) {}

bool TrafficLightPreprocessorStage::Init(
    const camera::TrafficLightPreprocessorInitOptions& options) {
  return impl_ != nullptr && impl_->Init(options);
}

bool TrafficLightPreprocessorStage::SyncInformation(
    double timestamp, const std::string& camera_name) {
  return impl_ != nullptr && impl_->SyncInformation(timestamp, camera_name);
}

bool TrafficLightPreprocessorStage::UpdateCameraSelection(
    const camera::CarPose& pose, const camera::TLPreprocessorOption& option,
    std::vector<base::TrafficLightPtr>* lights) {
  return impl_ != nullptr &&
         impl_->UpdateCameraSelection(pose, option, lights);
}

bool TrafficLightPreprocessorStage::UpdateLightsProjection(
    const camera::CarPose& pose, const camera::TLPreprocessorOption& option,
    const std::string& camera_name, std::vector<base::TrafficLightPtr>* lights) {
  return impl_ != nullptr &&
         impl_->UpdateLightsProjection(pose, option, camera_name, lights);
}

bool TrafficLightPreprocessorStage::SetCameraWorkingFlag(
    const std::string& camera_name, bool is_working) {
  return impl_ != nullptr &&
         impl_->SetCameraWorkingFlag(camera_name, is_working);
}

bool TrafficLightPreprocessorStage::GetCameraWorkingFlag(
    const std::string& camera_name, bool* is_working) const {
  return impl_ != nullptr &&
         impl_->GetCameraWorkingFlag(camera_name, is_working);
}

const std::vector<std::string>&
TrafficLightPreprocessorStage::GetCameraNamesByDescendingFocalLen() const {
  return impl_->GetCameraNamesByDescendingFocalLen();
}

std::string TrafficLightPreprocessorStage::Name() const {
  return "TrafficLightPreprocessorStage";
}

bool TrafficLightPreprocessorStage::ValidateConfiguration() const {
  return impl_ != nullptr;
}

bool TrafficLightPreprocessorStage::Reset() {
  if (impl_ == nullptr) {
    return false;
  }
  impl_.reset(new camera::TLPreprocessor());
  return true;
}

bool TrafficLightPreprocessorStage::Shutdown() {
  return true;
}

}  // namespace algo
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
