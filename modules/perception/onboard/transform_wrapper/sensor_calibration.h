// Copyright 2026 WheelOS. All Rights Reserved.
#pragma once

#include <string>
#include <vector>

#include "Eigen/Core"

namespace apollo {
namespace perception {
namespace common {
class SensorManager;
}  // namespace common
namespace onboard {

class TransformWrapper;

bool QueryStaticTransform(TransformWrapper* wrapper, const std::string& target,
                          const std::string& source,
                          Eigen::Matrix4d* transform);
bool SetCameraHeight(TransformWrapper* wrapper, const std::string& lidar,
                     const std::string& camera, float* height);

bool GetCommonCameraImageSize(const common::SensorManager& sensors,
                              const std::vector<std::string>& cameras,
                              int* width, int* height);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
