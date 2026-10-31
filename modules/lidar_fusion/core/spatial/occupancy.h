// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include <vector>

#include "modules/lidar_fusion/core/evidence/point_geometry.h"
#include "modules/lidar_fusion/core/spatial/spatial.h"

namespace apollo {
namespace lidar_fusion {
namespace fusion_detail {

SpaceCells BuildSpace(const FusionConfig& config, const FusionOutput& frame,
                      const std::vector<OrientedBox>& detection_boxes,
                      SpaceMap* map);

}  // namespace fusion_detail
}  // namespace lidar_fusion
}  // namespace apollo
