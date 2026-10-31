// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include <vector>

#include "modules/lidar_fusion/core/evidence/point_geometry.h"

namespace apollo {
namespace lidar_fusion {
namespace fusion_detail {

using Detection = lidar_object_detection::LidarObjectDetection;

std::vector<OrientedBox> Associate(
    const DetectionFrame* detection, const FusionConfig& config,
    const std::vector<Eigen::Vector3d>& positions, const SpatialIndex& index,
    const std::vector<int>& components, FusionOutput* output);

}  // namespace fusion_detail
}  // namespace lidar_fusion
}  // namespace apollo
