// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include <vector>

#include "modules/lidar_fusion/core/association/candidates.h"
#include "modules/lidar_fusion/core/spatial/connected_voxels.h"

namespace apollo {
namespace lidar_fusion {
namespace fusion_detail {

void BuildCandidates(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<int>& scene_components, const ConnectedVoxels& voxels,
    const FusionConfig& config,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    google::protobuf::RepeatedPtrField<PointDecision>* decisions,
    CandidateResult* candidates);

}  // namespace fusion_detail
}  // namespace lidar_fusion
}  // namespace apollo
