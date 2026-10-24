// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_object_detection/types/pointpillars_input.h"

#include <cmath>
#include <utility>

namespace apollo::lidar_object_detection {
bool BuildPointPillarsInput(const apollo::drivers::PointCloud& cloud,
                            const std::string& frame, float scale,
                            std::vector<float>* points, std::string* error) {
  if (points == nullptr || frame.empty() || cloud.frame_id() != frame ||
      !std::isfinite(scale) || scale <= 0 ||
      !std::isfinite(cloud.measurement_time()) ||
      cloud.measurement_time() <= 0 || cloud.point_size() == 0 ||
      cloud.point_size() > 300000) {
    if (error != nullptr) {
      *error =
          "Invalid PointPillars frame, timestamp, intensity scale or point "
          "count";
    }
    return false;
  }
  std::vector<float> result;
  result.reserve(static_cast<std::size_t>(cloud.point_size()) * 4);
  for (const auto& point : cloud.point()) {
    const float intensity = static_cast<float>(point.intensity()) * scale;
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z()) || !std::isfinite(intensity)) {
      if (error != nullptr) *error = "Nonfinite PointPillars point feature";
      return false;
    }
    result.insert(result.end(), {point.x(), point.y(), point.z(), intensity});
  }
  *points = std::move(result);
  return true;
}
}  // namespace apollo::lidar_object_detection
