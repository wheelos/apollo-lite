// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_semantic_segmentation/types/cylinder_projection.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace apollo {
namespace lidar_semantic_segmentation {

namespace {
bool Fail(const char* message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}
}  // namespace

bool CylinderProjector::Init(const CylinderProjectionOptions& options,
                             std::string* error) {
  initialized_ = false;
  if (options.max_points == 0 ||
      options.max_points > std::numeric_limits<uint32_t>::max() ||
      !std::isfinite(options.intensity_scale) ||
      options.intensity_scale <= 0.0F) {
    return Fail("invalid Cylinder3D point limit or intensity scale", error);
  }
  int64_t volume = 1;
  for (int axis = 0; axis < 3; ++axis) {
    if (options.grid[axis] < 2 ||
        options.grid[axis] > std::numeric_limits<int32_t>::max() ||
        volume > std::numeric_limits<int64_t>::max() / options.grid[axis] ||
        !std::isfinite(options.minimum[axis]) ||
        !std::isfinite(options.maximum[axis]) ||
        options.maximum[axis] <= options.minimum[axis]) {
      return Fail("invalid Cylinder3D grid or volume bounds", error);
    }
    volume *= options.grid[axis];
    intervals_[axis] = (options.maximum[axis] - options.minimum[axis]) /
                       static_cast<double>(options.grid[axis] - 1);
    if (!std::isfinite(intervals_[axis]) || intervals_[axis] <= 0) {
      return Fail("invalid Cylinder3D grid interval", error);
    }
  }
  options_ = options;
  initialized_ = true;
  return true;
}

bool CylinderProjector::Project(const apollo::drivers::PointCloud& cloud,
                                int flip, CylinderInput* input,
                                std::string* error) const {
  if (!initialized_ || input == nullptr || flip < 0 || flip > 3) {
    return Fail("uninitialized Cylinder3D projector or invalid output/flip",
                error);
  }
  if (static_cast<std::size_t>(cloud.point_size()) > options_.max_points) {
    return Fail("Cylinder3D point limit exceeded; refusing truncation", error);
  }
  CylinderInput projected;
  projected.features.reserve(static_cast<std::size_t>(cloud.point_size()) * 9);
  for (int index = 0; index < cloud.point_size(); ++index) {
    const auto& point = cloud.point(index);
    const float x = point.x() * ((flip & 1) != 0 ? -1.0F : 1.0F);
    const float y = point.y() * ((flip & 2) != 0 ? -1.0F : 1.0F);
    const float z = point.z();
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        (x == 0.0F && y == 0.0F && z == 0.0F)) {
      continue;
    }
    // Match NumPy's float32 cart2polar before float64 bound/center math.
    const float radius = std::sqrt(x * x + y * y);
    const std::array<float, 3> polar = {radius, std::atan2(y, x), z};
    if (!std::isfinite(radius)) {
      return Fail("Cylinder3D polar conversion overflow", error);
    }
    CylinderCoordinate coordinate{};
    std::array<float, 9> features{};
    for (int axis = 0; axis < 3; ++axis) {
      const double clipped =
          std::clamp(static_cast<double>(polar[axis]), options_.minimum[axis],
                     options_.maximum[axis]);
      coordinate[axis] = std::clamp(
          static_cast<int64_t>(std::floor((clipped - options_.minimum[axis]) /
                                          intervals_[axis])),
          int64_t{0}, options_.grid[axis] - 1);
      const double center =
          (static_cast<float>(coordinate[axis]) + 0.5F) * intervals_[axis] +
          options_.minimum[axis];
      features[axis] = static_cast<float>(polar[axis] - center);
      features[axis + 3] = polar[axis];
    }
    features[6] = x;
    features[7] = y;
    features[8] = point.intensity() * options_.intensity_scale;
    if (!std::all_of(features.begin(), features.end(),
                     [](float value) { return std::isfinite(value); })) {
      return Fail("Cylinder3D features contain non-finite values", error);
    }
    projected.coordinates.push_back(coordinate);
    projected.point_indices.push_back(static_cast<uint32_t>(index));
    projected.features.insert(projected.features.end(), features.begin(),
                              features.end());
  }
  *input = std::move(projected);
  return true;
}

}  // namespace lidar_semantic_segmentation
}  // namespace apollo
