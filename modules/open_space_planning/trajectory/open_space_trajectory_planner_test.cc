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

#include "modules/open_space_planning/trajectory/open_space_trajectory_planner.h"

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
  grid->revision = 5;
  grid->cell_state.assign(grid->width * grid->height, CellState::kFree);

  PlanningProblem problem;
  problem.grid_map = grid;
  problem.start.pose = {0.0, 0.0, 0.0};
  problem.start.longitudinal_velocity = 0.0;
  problem.start.gear = Gear::kDrive;
  problem.vehicle.wheel_base = 2.8448;
  problem.vehicle.maximum_curvature = 0.2;
  problem.goal.pose = {5.0, 0.0, 0.0};
  problem.goal.revision = 5;
  return problem;
}

RouteCandidate CreateTestRoute(const PlanningProblem& problem) {
  RouteCandidate candidate;
  candidate.id = 1;
  candidate.map_revision = problem.grid_map->revision;
  candidate.goal_revision = problem.goal.revision;

  for (int i = 0; i <= 10; ++i) {
    GeometricPathPoint pt;
    pt.pose.x = 0.5 * i;
    pt.pose.y = 0.0;
    pt.pose.heading = 0.0;
    pt.s = 0.5 * i;
    pt.curvature = 0.0;
    pt.gear = Gear::kDrive;
    candidate.skeleton.push_back(pt);
  }
  return candidate;
}

TEST(OpenSpaceTrajectoryPlannerTest, PlanTrajectoryFromRoute) {
  const PlanningProblem problem = CreateTestProblem();
  const RouteCandidate route = CreateTestRoute(problem);
  OpenSpaceTrajectoryPlanner planner;

  PhysicalTrajectory trajectory;
  TrajectoryPlanningRequest request{problem, route};
  const Status status = planner.Plan(request, &trajectory);

  EXPECT_TRUE(status.ok());
  EXPECT_FALSE(trajectory.points.empty());
  EXPECT_EQ(trajectory.map_revision, problem.grid_map->revision);
  EXPECT_EQ(trajectory.goal_revision, problem.goal.revision);
}

TEST(OpenSpaceTrajectoryPlannerTest, AvoidsDynamicObstacleByYielding) {
  PlanningProblem problem = CreateTestProblem();

  DynamicObstacle dynamic_obs;
  dynamic_obs.id = "pedestrian_1";
  dynamic_obs.footprint = {
      {2.5, -0.5, 0.0}, {2.5, 0.5, 0.0}, {3.5, 0.5, 0.0}, {3.5, -0.5, 0.0}};

  // Pedestrian crossing the path at s = 3.0m at t = 1.0s to 3.0s
  dynamic_obs.prediction = {
      {0.0, {3.0, -2.0, M_PI_2}, 1.0},
      {1.0, {3.0, 0.0, M_PI_2}, 1.0},
      {2.0, {3.0, 1.0, M_PI_2}, 1.0},
      {3.0, {3.0, 2.0, M_PI_2}, 1.0},
  };
  problem.dynamic_obstacles.push_back(dynamic_obs);

  const RouteCandidate route = CreateTestRoute(problem);
  OpenSpaceTrajectoryPlanner planner;

  PhysicalTrajectory trajectory;
  TrajectoryPlanningRequest request{problem, route};
  const Status status = planner.Plan(request, &trajectory);

  EXPECT_TRUE(status.ok());
  EXPECT_FALSE(trajectory.points.empty());

  // Trajectory should respect dynamic obstacle and yield safely
  for (const auto& pt : trajectory.points) {
    if (pt.relative_time >= 0.8 && pt.relative_time <= 2.2) {
      EXPECT_LE(pt.s, 2.5);  // Yields before s = 3.0m
    }
  }
}

}  // namespace
}  // namespace open_space_planning
}  // namespace apollo

