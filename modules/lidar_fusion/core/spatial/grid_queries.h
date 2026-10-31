// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include <cmath>

#include "modules/lidar_fusion/core/spatial/spatial.h"

namespace apollo {
namespace lidar_fusion {
namespace spatial_internal {

using Key = SpaceKey;
using Cells = SpaceCells;

inline Key Index(const SpaceMap& map, double x, double y) {
  return {
      static_cast<int>(std::floor((x - map.origin_x()) / map.resolution())),
      static_cast<int>(std::floor((y - map.origin_y()) / map.resolution()))};
}

inline bool Inside(const SpaceMap& map, const Key& key) {
  return key.first >= 0 && key.second >= 0 &&
         key.first < static_cast<int>(map.width()) &&
         key.second < static_cast<int>(map.height());
}

inline bool Free(const SpaceCell& cell) {
  return cell.traversability() == FREE || cell.traversability() == FREE_SOFT;
}

inline bool Occupied(const SpaceCell& cell) {
  return cell.occupancy_obs() || cell.occupancy_det() ||
         cell.occupancy_unknown();
}

}  // namespace spatial_internal
}  // namespace lidar_fusion
}  // namespace apollo
