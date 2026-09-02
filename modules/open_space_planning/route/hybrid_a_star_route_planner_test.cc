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

#include "modules/open_space_planning/route/hybrid_a_star_route_planner.h"

#include <memory>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace open_space_planning {
namespace {

PlanningProblem CreateTestProblem() {
  auto grid = std::make_shared<GridMap>();
  grid->frame_id = "map";
  grid->resolution = 0.25;
  grid->width = 100;
  grid->height = 100;
  grid->origin = {-12.5, -12.5, 0.0};
  grid->revision = 1;
  grid->cell_state.assign(grid->width * grid->height, CellState::kFree);

  PlanningProblem problem;
  problem.grid_map = grid;
  problem.start.pose = {0.0, 0.0, 0.0};
  problem.start.longitudinal_velocity = 0.0;
  problem.start.gear = Gear::kDrive;
  problem.vehicle.wheel_base = 2.8448;
  problem.vehicle.maximum_curvature = 0.2;
  problem.goal.pose = {8.0, 0.0, 0.0};
  problem.goal.revision = 1;
  return problem;
}

TEST(HybridAStarRoutePlannerTest, PlanStraightLineRoute) {
  const PlanningProblem problem = CreateTestProblem();
  HybridAStarRoutePlanner planner;

  std::vector<RouteCandidate> candidates;
  RoutePlanningRequest request{
      problem, 1, RouteSearchParadigm::kCruisingForward};
  const Status status = planner.Plan(request, &candidates);

  EXPECT_TRUE(status.ok());
  ASSERT_FALSE(candidates.empty());
  const auto& candidate = candidates.front();
  EXPECT_FALSE(candidate.skeleton.empty());
  EXPECT_EQ(candidate.map_revision, problem.grid_map->revision);
  EXPECT_EQ(candidate.goal_revision, problem.goal.revision);

  // Cruising forward must produce pure drive gear
  for (const auto& pt : candidate.skeleton) {
    EXPECT_EQ(pt.gear, Gear::kDrive);
  }
}

TEST(HybridAStarRoutePlannerTest, PlanWithSkeletonCorridorParadigm) {
  const PlanningProblem problem = CreateTestProblem();
  HybridAStarRoutePlanner planner;

  std::vector<RouteCandidate> candidates;
  RoutePlanningRequest request{
      problem, 1, RouteSearchParadigm::kSkeletonCorridor};
  const Status status = planner.Plan(request, &candidates);

  EXPECT_TRUE(status.ok());
  ASSERT_FALSE(candidates.empty());
  const auto& candidate = candidates.front();
  EXPECT_FALSE(candidate.skeleton.empty());
  EXPECT_EQ(candidate.corridor.size(), candidate.skeleton.size());
}

TEST(HybridAStarRoutePlannerTest, ExtractBoundsAndObstacles) {
  const PlanningProblem problem = CreateTestProblem();
  std::vector<double> xy_bounds;
  HybridAStarRoutePlanner::ExtractXYBounds(problem, &xy_bounds);
  ASSERT_EQ(xy_bounds.size(), 4U);
  EXPECT_DOUBLE_EQ(xy_bounds[0], -12.5);
  EXPECT_DOUBLE_EQ(xy_bounds[1], 12.5);
}

}  // namespace
}  // namespace open_space_planning
}  // namespace apollo

