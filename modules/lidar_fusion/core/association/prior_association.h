// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include "modules/lidar_fusion/core/evidence/point_geometry.h"

namespace apollo {
namespace lidar_fusion {
namespace fusion_detail {

void AssociatePrior(const ObjectPrior* prior, const FusionConfig& config,
                    FusionOutput* output);

}  // namespace fusion_detail
}  // namespace lidar_fusion
}  // namespace apollo
