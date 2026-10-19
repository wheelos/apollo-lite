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

#include "modules/open_space_planning/map/reachable_region.h"

#include <memory>

#include "gtest/gtest.h"

namespace apollo {
namespace open_space_planning {
namespace {

GridMap CreateGrid() {
  GridMap map;
  map.frame_id = "map";
  map.resolution = 1.0;
  map.width = 7;
  map.height = 5;
  map.cell_state.assign(map.cell_count(), CellState::kFree);
  return map;
}

TEST(ReachableRegionTest, DiffusesWithinExplicitIrregularRegion) {
  GridMap map = CreateGrid();
  GridMap::DrivableRegion roi;
  roi.id = "l_shape";
  roi.boundary = {{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0},
                  {5.0, 2.0, 0.0}, {2.0, 2.0, 0.0},
                  {2.0, 5.0, 0.0}, {0.0, 5.0, 0.0}};
  map.drivable_regions.push_back(roi);

  ReachableRegion reachable;
  const Status status =
      ComputeReachableRegion(map, {1.5, 1.5, 0.0}, 0.0, &reachable);

  ASSERT_TRUE(status.ok()) << status.message();
  ASSERT_EQ(reachable.reachable.size(), map.cell_count());
  EXPECT_TRUE(reachable.reachable[1 * map.width + 1]);
  EXPECT_FALSE(reachable.reachable[3 * map.width + 3]);
}

TEST(ReachableRegionTest, UnknownCellsAndDiagonalCornerCuttingAreBlocked) {
  GridMap map = CreateGrid();
  map.cell_state[1] = CellState::kUnknown;
  map.cell_state[map.width] = CellState::kOccupied;

  ReachableRegion reachable;
  const Status status =
      ComputeReachableRegion(map, {0.5, 0.5, 0.0}, 0.0, &reachable);

  ASSERT_TRUE(status.ok()) << status.message();
  EXPECT_FALSE(reachable.reachable[1]);
  EXPECT_FALSE(reachable.reachable[map.width]);
  EXPECT_FALSE(reachable.reachable[map.width + 1]);
}

TEST(ReachableRegionTest, AppliesRotatedMapOriginTransform) {
  GridMap map = CreateGrid();
  map.origin = {10.0, 20.0, 1.57079632679};
  int grid_x = -1;
  int grid_y = -1;

  const Pose2d center = GridCellCenter(map, 2, 3);
  EXPECT_TRUE(WorldToGrid(map, center.x, center.y, &grid_x, &grid_y));
  EXPECT_EQ(grid_x, 2);
  EXPECT_EQ(grid_y, 3);
}

}  // namespace
}  // namespace open_space_planning
}  // namespace apollo
