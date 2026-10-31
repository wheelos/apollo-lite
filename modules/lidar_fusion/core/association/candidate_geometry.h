// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include "modules/lidar_fusion/core/association/candidates.h"

namespace apollo {
namespace lidar_fusion {
namespace candidate_detail {

void RefineObjectGeometry(
    const FusionConfig& config,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    ObjectCandidate* object, const PointIndex* index);
void RefineUnknownGeometry(
    const FusionConfig& config,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    UnknownCandidate* unknown);

}  // namespace candidate_detail
}  // namespace lidar_fusion
}  // namespace apollo
