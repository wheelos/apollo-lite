// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace lidar_semantic_segmentation {

using CylinderCoordinate = std::array<int64_t, 3>;

struct CylinderProjectionOptions {
  CylinderCoordinate grid = {480, 360, 32};
  std::array<double, 3> minimum = {0.0, -3.1415926, -4.0};
  std::array<double, 3> maximum = {50.0, 3.1415926, 2.0};
  std::size_t max_points = 150000;
  float intensity_scale = 1.0F / 255.0F;
};

struct CylinderInput {
  std::vector<float> features;  // [N,9], in valid source-point order.
  std::vector<CylinderCoordinate> coordinates;
  std::vector<uint32_t> point_indices;
};

class CylinderProjector {
 public:
  bool Init(const CylinderProjectionOptions& options, std::string* error);
  bool Project(const apollo::drivers::PointCloud& cloud, int flip,
               CylinderInput* input, std::string* error) const;

 private:
  CylinderProjectionOptions options_;
  std::array<double, 3> intervals_{};
  bool initialized_ = false;
};

}  // namespace lidar_semantic_segmentation
}  // namespace apollo
