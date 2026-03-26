/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/camera/lib/interface/base_tl_preprocessor.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace infra {

class PreprocessorGateway {
 public:
  PreprocessorGateway() = default;
  ~PreprocessorGateway() = default;

  bool Init(const std::string& preprocessor_name,
            const camera::TrafficLightPreprocessorInitOptions& options);
  bool SyncInformation(double timestamp, const std::string& camera_name);
  bool UpdateCameraSelection(const camera::CarPose& pose,
                             const camera::TLPreprocessorOption& option,
                             std::vector<base::TrafficLightPtr>* lights);
  bool UpdateLightsProjection(const camera::CarPose& pose,
                              const camera::TLPreprocessorOption& option,
                              const std::string& camera_name,
                              std::vector<base::TrafficLightPtr>* lights);
  bool SetCameraWorkingFlag(const std::string& camera_name, bool is_working);
  bool GetCameraWorkingFlag(const std::string& camera_name,
                            bool* is_working) const;
  const std::vector<std::string>& GetCameraNamesByDescendingFocalLen() const;

 private:
  std::shared_ptr<camera::BaseTLPreprocessor> preprocessor_;
};

}  // namespace infra
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
