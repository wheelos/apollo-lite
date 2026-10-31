// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include <unordered_map>
#include <vector>

#include "modules/lidar_fusion/core/evidence/point_geometry.h"

namespace apollo {
namespace lidar_fusion {
namespace fusion_detail {

struct GroundPlane {
  Eigen::Vector3d coefficients;
  double origin_x = 0;
  double origin_y = 0;
  double residual = 0;
  uint32_t count = 0;
};

using GroundPlanes = std::unordered_map<CellKey, GroundPlane, CellHash>;

GroundPlanes FitGround(const std::vector<Eigen::Vector3d>& positions,
                       const SpatialIndex& seed_tiles,
                       const FusionConfig& config);

}  // namespace fusion_detail
}  // namespace lidar_fusion
}  // namespace apollo
