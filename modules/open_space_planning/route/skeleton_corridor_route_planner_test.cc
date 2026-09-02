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

#include "modules/open_space_planning/route/skeleton_corridor_route_planner.h"

#include <memory>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace open_space_planning {
namespace {

PlanningProblem CreateGridProblemWithObstacle() {
  auto grid = std::make_shared<GridMap>();
  grid->frame_id = "map";
  grid->resolution = 0.2;
  grid->width = 60;
  grid->height = 50;
  grid->origin = {-2.0, -5.0, 0.0};
  grid->revision = 1;
  grid->cell_state.assign(grid->width * grid->height, CellState::kFree);

  // Obstacle in the middle: x=[3.0, 4.0], y=[-1.0, 1.0]
  for (std::size_t r = 0; r < grid->height; ++r) {
    for (std::size_t c = 0; c < grid->width; ++c) {
      const double x = grid->origin.x + c * grid->resolution;
      const double y = grid->origin.y + r * grid->resolution;
      if (x >= 3.0 && x <= 4.0 && y >= -1.0 && y <= 1.0) {
        grid->cell_state[r * grid->width + c] = CellState::kOccupied;
      }
    }
  }

  PlanningProblem problem;
  problem.grid_map = grid;
  problem.start.pose = {0.0, 0.0, 0.0};
  problem.goal.pose = {7.0, 0.0, 0.0};
  problem.goal.revision = 1;
  return problem;
}

TEST(SkeletonCorridorRoutePlannerTest, PlanAroundObstacleWithCorridor) {
  const PlanningProblem problem = CreateGridProblemWithObstacle();
  SkeletonCorridorRoutePlanner planner;

  std::vector<RouteCandidate> candidates;
  RoutePlanningRequest request{
      problem, 1, RouteSearchParadigm::kSkeletonCorridor};
  const Status status = planner.Plan(request, &candidates);

  EXPECT_TRUE(status.ok());
  ASSERT_FALSE(candidates.empty());
  const auto& candidate = candidates.front();

  EXPECT_FALSE(candidate.skeleton.empty());
  EXPECT_EQ(candidate.corridor.size(), candidate.skeleton.size());

  // Check start and goal reachability
  EXPECT_NEAR(candidate.skeleton.front().pose.x, 0.0, 0.3);
  EXPECT_NEAR(candidate.skeleton.front().pose.y, 0.0, 0.3);
  EXPECT_NEAR(candidate.skeleton.back().pose.x, 7.0, 0.3);
  EXPECT_NEAR(candidate.skeleton.back().pose.y, 0.0, 0.3);

  // Check that corridor bounds are positive and valid
  for (const auto& sample : candidate.corridor) {
    EXPECT_LT(sample.minimum_lateral_offset, 0.0);
    EXPECT_GT(sample.maximum_lateral_offset, 0.0);
  }
}

}  // namespace
}  // namespace open_space_planning
}  // namespace apollo
