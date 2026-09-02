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

#include "modules/open_space_planning/safety/default_fallback_planner.h"

#include <memory>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace open_space_planning {
namespace {

TEST(DefaultFallbackPlannerTest, GeneratesSafeStopTrajectory) {
  DefaultFallbackPlanner fallback_planner;

  auto grid = std::make_shared<GridMap>();
  grid->revision = 3;

  PlanningProblem problem;
  problem.grid_map = grid;
  problem.goal.revision = 7;
  problem.start.pose = {0.0, 0.0, 0.0};
  problem.start.longitudinal_velocity = 2.0;
  problem.start.gear = Gear::kDrive;

  PhysicalTrajectory trajectory;
  FallbackPlanningRequest request{problem};
  const Status status = fallback_planner.Plan(request, &trajectory);

  EXPECT_TRUE(status.ok());
  EXPECT_FALSE(trajectory.points.empty());
  EXPECT_EQ(trajectory.map_revision, 3U);
  EXPECT_EQ(trajectory.goal_revision, 7U);
  EXPECT_DOUBLE_EQ(trajectory.points.back().velocity, 0.0);
}

}  // namespace
}  // namespace open_space_planning
}  // namespace apollo
