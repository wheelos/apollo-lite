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

#include "modules/open_space_planning/runtime/open_space_planner.h"

#include <memory>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace open_space_planning {
namespace {

PlanningProblem ValidProblem() {
  auto grid = std::make_shared<GridMap>();
  grid->frame_id = "map";
  grid->resolution = 0.2;
  grid->width = 2;
  grid->height = 2;
  grid->revision = 7;
  grid->cell_state.assign(4, CellState::kFree);

  PlanningProblem problem;
  problem.grid_map = std::move(grid);
  problem.vehicle.wheel_base = 2.8;
  problem.vehicle.maximum_curvature = 0.2;
  problem.goal.revision = 11;
  return problem;
}

PhysicalTrajectory ValidTrajectory(const PlanningProblem& problem,
                                   CandidateId candidate_id) {
  PhysicalTrajectory trajectory;
  trajectory.source_candidate_id = candidate_id;
  trajectory.map_revision = problem.grid_map->revision;
  trajectory.goal_revision = problem.goal.revision;
  trajectory.points.emplace_back();
  return trajectory;
}

class FakeRoutePlanner final : public RoutePlanner {
 public:
  explicit FakeRoutePlanner(bool succeed) : succeed_(succeed) {}

  Status Plan(const RoutePlanningRequest& request,
              std::vector<RouteCandidate>* candidates) override {
    if (!succeed_) {
      return {StatusCode::kRouteSearchFailed, "route failed"};
    }
    RouteCandidate candidate;
    candidate.id = 42;
    candidate.map_revision = request.problem.grid_map->revision;
    candidate.goal_revision = request.problem.goal.revision;
    candidate.skeleton.emplace_back();
    candidates->push_back(std::move(candidate));
    return Status::Ok();
  }

 private:
  bool succeed_;
};

class FakeTrajectoryPlanner final : public TrajectoryPlanner {
 public:
  Status Plan(const TrajectoryPlanningRequest& request,
              PhysicalTrajectory* trajectory) override {
    *trajectory = ValidTrajectory(request.problem, request.route.id);
    return Status::Ok();
  }
};

class FakeValidator final : public TrajectoryValidator {
 public:
  ValidationReport Validate(
      const TrajectoryValidationRequest& request) const override {
    ValidationReport report;
    report.safe = !request.trajectory.points.empty();
    return report;
  }
};

class FakeFallbackPlanner final : public FallbackPlanner {
 public:
  Status Plan(const FallbackPlanningRequest& request,
              PhysicalTrajectory* trajectory) override {
    *trajectory = ValidTrajectory(request.problem, 0);
    return Status::Ok();
  }
};

std::unique_ptr<OpenSpacePlanner> MakePlanner(bool route_succeeds) {
  return std::unique_ptr<OpenSpacePlanner>(new OpenSpacePlanner(
      OpenSpacePlannerConfig{},
      std::unique_ptr<RoutePlanner>(new FakeRoutePlanner(route_succeeds)),
      std::unique_ptr<TrajectoryPlanner>(new FakeTrajectoryPlanner()),
      std::unique_ptr<TrajectoryValidator>(new FakeValidator()),
      std::unique_ptr<FallbackPlanner>(new FakeFallbackPlanner())));
}

TEST(OpenSpacePlannerTest, ReturnsValidatedCandidateTrajectory) {
  const PlanningProblem problem = ValidProblem();
  auto planner = MakePlanner(true);
  PlanningResult result;

  const Status status = planner->Plan(problem, &result);

  EXPECT_TRUE(status.ok());
  EXPECT_EQ(result.outcome, PlanningOutcome::kTrajectory);
  EXPECT_EQ(result.active_route.id, 42);
  EXPECT_TRUE(result.validation.safe);
}

TEST(OpenSpacePlannerTest, ValidatesFallbackThroughSameSafetyGate) {
  const PlanningProblem problem = ValidProblem();
  auto planner = MakePlanner(false);
  PlanningResult result;

  const Status status = planner->Plan(problem, &result);

  EXPECT_TRUE(status.ok());
  EXPECT_EQ(result.outcome, PlanningOutcome::kFallbackTrajectory);
  EXPECT_TRUE(result.validation.safe);
}

}  // namespace
}  // namespace open_space_planning
}  // namespace apollo
