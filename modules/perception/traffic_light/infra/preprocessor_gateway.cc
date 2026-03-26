/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/
#include "modules/perception/traffic_light/infra/preprocessor_gateway.h"

#include "cyber/common/log.h"
#include "modules/perception/traffic_light/algo/preprocessor/traffic_light_preprocessor_stage.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace infra {

namespace {

constexpr char kModulePreprocessorType[] = "TrafficLightPreprocessorStage";
constexpr char kLegacyPreprocessorAlias[] = "TLPreprocessor";

}  // namespace

bool PreprocessorGateway::Init(
    const std::string& preprocessor_name,
    const camera::TrafficLightPreprocessorInitOptions& options) {
  if (preprocessor_name != kModulePreprocessorType &&
      preprocessor_name != kLegacyPreprocessorAlias) {
    AERROR << "Unsupported traffic light preprocessor type: "
           << preprocessor_name << ", expected: " << kModulePreprocessorType;
    return false;
  }

  preprocessor_.reset(new algo::TrafficLightPreprocessorStage());
  if (preprocessor_ == nullptr) {
    return false;
  }
  if (!preprocessor_->ValidateConfiguration()) {
    AERROR << "Traffic light preprocessor validation failed: "
           << preprocessor_->Name();
    return false;
  }
  return preprocessor_->Init(options);
}

bool PreprocessorGateway::SyncInformation(double timestamp,
                                          const std::string& camera_name) {
  return preprocessor_ != nullptr &&
         preprocessor_->SyncInformation(timestamp, camera_name);
}

bool PreprocessorGateway::UpdateCameraSelection(
    const camera::CarPose& pose, const camera::TLPreprocessorOption& option,
    std::vector<base::TrafficLightPtr>* lights) {
  return preprocessor_ != nullptr &&
         preprocessor_->UpdateCameraSelection(pose, option, lights);
}

bool PreprocessorGateway::UpdateLightsProjection(
    const camera::CarPose& pose, const camera::TLPreprocessorOption& option,
    const std::string& camera_name,
    std::vector<base::TrafficLightPtr>* lights) {
  return preprocessor_ != nullptr &&
         preprocessor_->UpdateLightsProjection(pose, option, camera_name,
                                               lights);
}

bool PreprocessorGateway::SetCameraWorkingFlag(const std::string& camera_name,
                                               bool is_working) {
  return preprocessor_ != nullptr &&
         preprocessor_->SetCameraWorkingFlag(camera_name, is_working);
}

bool PreprocessorGateway::GetCameraWorkingFlag(const std::string& camera_name,
                                               bool* is_working) const {
  return preprocessor_ != nullptr &&
         preprocessor_->GetCameraWorkingFlag(camera_name, is_working);
}

const std::vector<std::string>&
PreprocessorGateway::GetCameraNamesByDescendingFocalLen() const {
  return preprocessor_->GetCameraNamesByDescendingFocalLen();
}

}  // namespace infra
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
