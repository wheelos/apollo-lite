/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/camera/lib/interface/base_tl_preprocessor.h"
#include "modules/perception/camera/lib/traffic_light/preprocessor/tl_preprocessor.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace algo {

class TrafficLightPreprocessorStage final : public camera::BaseTLPreprocessor {
 public:
  TrafficLightPreprocessorStage();
  ~TrafficLightPreprocessorStage() override = default;

  bool Init(const camera::TrafficLightPreprocessorInitOptions& options) override;

  bool SyncInformation(double timestamp,
                       const std::string& camera_name) override;

  bool UpdateCameraSelection(const camera::CarPose& pose,
                             const camera::TLPreprocessorOption& option,
                             std::vector<base::TrafficLightPtr>* lights) override;

  bool UpdateLightsProjection(const camera::CarPose& pose,
                              const camera::TLPreprocessorOption& option,
                              const std::string& camera_name,
                              std::vector<base::TrafficLightPtr>* lights)
      override;

  bool SetCameraWorkingFlag(const std::string& camera_name,
                            bool is_working) override;

  bool GetCameraWorkingFlag(const std::string& camera_name,
                            bool* is_working) const override;

  const std::vector<std::string>& GetCameraNamesByDescendingFocalLen()
      const override;

  std::string Name() const override;

    bool ValidateConfiguration() const override;

    bool Reset() override;

    bool Shutdown() override;

 private:
  std::unique_ptr<camera::TLPreprocessor> impl_;
};

}  // namespace algo
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
