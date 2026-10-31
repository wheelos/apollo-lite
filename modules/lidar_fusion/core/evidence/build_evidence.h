// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include <vector>

#include "modules/lidar_fusion/core/evidence/ground_plane.h"

namespace apollo {
namespace lidar_fusion {
namespace fusion_detail {

void BuildEvidence(const drivers::PointCloud& cloud,
                   const SemanticFrame* segmentation,
                   const FusionConfig& config,
                   std::vector<Eigen::Vector3d>* positions, SpatialIndex* index,
                   FusionOutput* output, const GroundPrior* prior);

}  // namespace fusion_detail
}  // namespace lidar_fusion
}  // namespace apollo
