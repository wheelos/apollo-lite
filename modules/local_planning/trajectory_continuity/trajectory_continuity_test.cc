// Copyright 2026 WheelOS All Rights Reserved.
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

#include "modules/local_planning/trajectory_continuity/trajectory_continuity.h"

#include "gtest/gtest.h"

namespace apollo {
namespace local_planning {
namespace {

CycleInput Cycle(double now) {
  CycleInput input;
  input.planning_time = now;
  input.odometry.stamp.frame_id = "odom";
  input.odometry.stamp.clock_id = "clock";
  input.odometry.stamp.epoch = {"session", 1};
  input.odometry.stamp.measurement_time = now;
  input.corridor.lane_id = 7;
  input.odometry.x = 1.0;
  input.odometry.speed_mps = 1.0;
  return input;
}

LocalTrajectory Previous() {
  LocalTrajectory trajectory;
  trajectory.valid = true;
  trajectory.stamp = Cycle(1.0).odometry.stamp;
  trajectory.stamp.valid_until = 1.25;
  trajectory.lane_id = 7;
  trajectory.points = {{0.0, 0.9, 0.0, 0.0, 1.0, -0.2, 0.01},
                       {0.2, 1.1, 0.0, 0.0, 1.0, -0.4, 0.03}};
  return trajectory;
}

TEST(TrajectoryContinuityTest, AlignsAbsoluteTimeWithValidatedHistory) {
  auto input = Cycle(1.1);
  const auto history = Previous();
  TrajectoryStart start;
  ASSERT_TRUE(
      AlignTrajectoryStart(input, &history, {0.1, 0.1, 0.2}, &start).ok());
  EXPECT_EQ(TrajectoryContinuityState::STITCHED, start.continuity);
  EXPECT_NEAR(1.0, start.point.x, 1e-9);
  EXPECT_NEAR(-0.3, start.point.acceleration, 1e-9);
  EXPECT_NEAR(0.02, start.point.curvature, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, start.point.time);

  input.odometry.x = 1.4;
  ASSERT_TRUE(
      AlignTrajectoryStart(input, &history, {0.1, 0.1, 0.2}, &start).ok());
  EXPECT_EQ(TrajectoryContinuityState::REANCHORED, start.continuity);
  EXPECT_DOUBLE_EQ(1.4, start.point.x);
  EXPECT_NEAR(-0.3, start.point.acceleration, 1e-9);
}

TEST(TrajectoryContinuityTest, NeverReusesExpiredOrMismatchedOutput) {
  auto input = Cycle(1.1);
  auto history = Previous();
  TrajectoryStart start;
  input.odometry.stamp.epoch.generation = 2;
  ASSERT_TRUE(
      AlignTrajectoryStart(input, &history, {0.1, 0.1, 0.2}, &start).ok());
  EXPECT_EQ(TrajectoryContinuityState::SOURCE_CHANGED, start.continuity);
  EXPECT_DOUBLE_EQ(0.0, start.point.acceleration);
  input = Cycle(1.25);
  ASSERT_TRUE(
      AlignTrajectoryStart(input, &history, {0.1, 0.1, 0.2}, &start).ok());
  EXPECT_EQ(TrajectoryContinuityState::EXPIRED, start.continuity);
  input = Cycle(0.9);
  ASSERT_TRUE(
      AlignTrajectoryStart(input, &history, {0.1, 0.1, 0.2}, &start).ok());
  EXPECT_EQ(TrajectoryContinuityState::OUTSIDE_HORIZON, start.continuity);
}

TEST(TrajectoryContinuityTest, RejectsInvalidInputsAndDoesNotRetainFailure) {
  auto input = Cycle(1.1);
  TrajectoryStart start;
  EXPECT_FALSE(
      AlignTrajectoryStart(input, nullptr, {-1, 0.1, 0.2}, &start).ok());
  TrajectoryContinuity continuity;
  LocalTrajectory invalid;
  EXPECT_FALSE(continuity.Accept(input, LaneFollowConfig{}, invalid).ok());
  ASSERT_TRUE(continuity.Start(input, {0.1, 0.1, 0.2}, &start).ok());
  EXPECT_EQ(TrajectoryContinuityState::NO_HISTORY, start.continuity);
}

}  // namespace
}  // namespace local_planning
}  // namespace apollo
