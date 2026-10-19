// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstddef>
#include <vector>

#include "modules/open_space_planning/common/status.h"
#include "modules/open_space_planning/common/types.h"

namespace apollo {
namespace open_space_planning {

struct ReachableRegion {
  std::vector<bool> reachable;
  std::vector<double> accumulated_cost;
  std::vector<int> parent_index;
  std::size_t seed_index = 0;
};

bool WorldToGrid(const GridMap& map, double x, double y, int* grid_x,
                 int* grid_y);

Pose2d GridCellCenter(const GridMap& map, int grid_x, int grid_y);

bool IsWithinDrivableRegions(const GridMap& map, double x, double y,
                             double boundary_clearance = 0.0);

enum class PoseTraversability {
  kFree = 0,
  kOutsideMap,
  kOutsideDrivableRegion,
  kBlocked,
};

PoseTraversability CheckVehiclePose(const GridMap& map,
                                   const VehicleModel& vehicle,
                                   const Pose2d& pose);

Status ComputeReachableRegion(const GridMap& map, const Pose2d& seed,
                              double obstacle_clearance,
                              ReachableRegion* region);

}  // namespace open_space_planning
}  // namespace apollo
