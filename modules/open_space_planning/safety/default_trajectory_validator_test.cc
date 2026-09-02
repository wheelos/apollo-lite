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

#include "modules/open_space_planning/safety/default_trajectory_validator.h"

#include <memory>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace open_space_planning {
namespace {

TEST(DefaultTrajectoryValidatorTest, RejectsEmptyTrajectory) {
  DefaultTrajectoryValidator validator;
  PlanningProblem problem;
  PhysicalTrajectory trajectory;

  TrajectoryValidationRequest request{problem, trajectory};
  ValidationReport report = validator.Validate(request);

  EXPECT_FALSE(report.safe);
  ASSERT_FALSE(report.issues.empty());
  EXPECT_EQ(report.issues.front().rule, "EmptyTrajectory");
}

TEST(DefaultTrajectoryValidatorTest, AcceptsValidTrajectory) {
  DefaultTrajectoryValidator validator;
  PlanningProblem problem;
  PhysicalTrajectory trajectory;

  PhysicalTrajectoryPoint p1;
  p1.pose = {0.0, 0.0, 0.0};
  p1.velocity = 1.0;
  p1.acceleration = 0.0;
  p1.curvature = 0.05;
  p1.relative_time = 0.0;
  p1.s = 0.0;

  PhysicalTrajectoryPoint p2;
  p2.pose = {1.0, 0.0, 0.0};
  p2.velocity = 1.0;
  p2.acceleration = 0.0;
  p2.curvature = 0.05;
  p2.relative_time = 1.0;
  p2.s = 1.0;

  trajectory.points = {p1, p2};

  TrajectoryValidationRequest request{problem, trajectory};
  ValidationReport report = validator.Validate(request);

  EXPECT_TRUE(report.safe);
  EXPECT_TRUE(report.issues.empty());
}

}  // namespace
}  // namespace open_space_planning
}  // namespace apollo
