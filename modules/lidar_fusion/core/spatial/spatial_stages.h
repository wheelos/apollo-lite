// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include "modules/lidar_fusion/core/spatial/spatial.h"

namespace apollo {
namespace lidar_fusion {
namespace spatial_internal {

void BuildPointViews(const FusionConfig& config, FusionOutput* output);
void TraceRay(const FusionConfig& config, const PointEvidence& point,
              const SpaceMap& map, SpaceCells* cells);
void ClassifyVisibleTraversability(const FusionConfig& config,
                                   const SpaceMap& map, SpaceCells* cells);

}  // namespace spatial_internal
}  // namespace lidar_fusion
}  // namespace apollo
