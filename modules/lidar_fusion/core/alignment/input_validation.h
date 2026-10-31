// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include <map>
#include <set>

#include "modules/lidar_fusion/core/runtime/single_frame_fusion.h"

namespace apollo {
namespace lidar_fusion {
namespace alignment_internal {

bool Probability(double value);
std::map<uint32_t, ClassMapping> Mappings(
    const google::protobuf::RepeatedPtrField<ClassMapping>& entries);
bool ValidMappings(
    const google::protobuf::RepeatedPtrField<ClassMapping>& entries,
    bool detection);
Reason ValidateDetection(const drivers::PointCloud& cloud,
                         const DetectionFrame* input,
                         const FusionConfig& config);
Reason ValidateSemantic(const drivers::PointCloud& cloud,
                        const SemanticFrame* input, const FusionConfig& config);

}  // namespace alignment_internal
}  // namespace lidar_fusion
}  // namespace apollo
