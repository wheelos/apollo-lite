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

#include "modules/local_planning/lane_follow_planner.h"

#include <cmath>
#include <functional>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

#include "modules/local_planning/simulation/closed_loop.h"

namespace apollo {
namespace local_planning {
namespace {
CycleInput Scene() {
  CycleInput input;
  input.planning_time = 1.0;
  SourceStamp stamp;
  stamp.frame_id = "odom";
  stamp.clock_id = "simulation";
  stamp.epoch = {"test-session", 1};
  stamp.sequence = 1;
  stamp.measurement_time = 1.0;
  stamp.publication_time = 1.0;
  stamp.valid_until = 1.4;
  stamp.health = InputHealth::HEALTHY;
  input.odometry.stamp = stamp;
  input.graph = stamp;
  input.prediction.stamp = stamp;
  input.prediction.graph_sequence = 1;
  input.corridor.lane_id = 1;
  input.corridor.confirmed_forward = true;
  input.corridor.half_width = 2.5;
  input.corridor.position_error_bound = 0.02;
  for (double x = -8.0; x <= 45.0; x += 0.5) {
    input.corridor.centerline.push_back({x, 0.0});
  }
  input.occupancy.corridor_fully_observed = true;
  input.occupancy.horizon = 6.2;
  input.environment.source = {"odom",
                              "simulation",
                              "test-session",
                              1,
                              1,
                              1.0,
                              1.0,
                              1.4,
                              world_model::SceneHealth::HEALTHY};
  input.environment.position_error = 0.02;
  input.environment.free_space = {{-10, -4}, {50, -4}, {50, 4}, {-10, 4}};
  return input;
}

void Advance(CycleInput* input) {
  input->planning_time += 0.1;
  auto& stamp = input->odometry.stamp;
  ++stamp.sequence;
  stamp.measurement_time = input->planning_time;
  stamp.publication_time = input->planning_time;
  stamp.valid_until = input->planning_time + 0.4;
  input->graph = stamp;
  input->environment.source = {stamp.frame_id,
                               stamp.clock_id,
                               stamp.epoch.producer_session,
                               stamp.epoch.generation,
                               stamp.sequence,
                               stamp.measurement_time,
                               stamp.publication_time,
                               stamp.valid_until,
                               world_model::SceneHealth::HEALTHY};
  input->prediction.stamp = stamp;
  input->prediction.graph_sequence = stamp.sequence;
}

TEST(LaneFollowPlannerTest, ProducesBoundedTaggedTrajectory) {
  const auto c = SimulationPlannerConfig();
  LaneFollowPlanner planner(SimulationInputPolicy(), c);
  const auto input = Scene();
  ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1.0).ok());
  const auto status = planner.Plan(input);
  ASSERT_TRUE(status.ok()) << status.ToString();
  const auto& t = planner.trajectory();
  ASSERT_TRUE(t.valid);
  ASSERT_EQ(61U, t.points.size());
  EXPECT_EQ("odom", t.stamp.frame_id);
  EXPECT_TRUE(t.stamp.epoch == input.odometry.stamp.epoch);
  EXPECT_EQ(input.graph.sequence, t.graph_sequence);
  EXPECT_EQ(input.prediction.stamp.sequence, t.prediction_sequence);
  EXPECT_DOUBLE_EQ(1.2, t.stamp.valid_until);
  EXPECT_DOUBLE_EQ(0.0, t.points.front().x);
  EXPECT_DOUBLE_EQ(0.0, t.points.front().speed);
  EXPECT_GT(t.points.back().x, 5.0);
  for (size_t i = 0; i < t.points.size(); ++i) {
    const auto& p = t.points[i];
    EXPECT_NEAR(i * c.step, p.time, 1e-9);
    EXPECT_GE(p.speed, 0.0);
    EXPECT_LE(p.speed, c.cruise_speed + 1e-6);
    EXPECT_LE(std::abs(p.curvature), c.max_curvature);
    EXPECT_NEAR(0.0, p.y, 1e-9);
    if (i > 0) {
      const auto& before = t.points[i - 1];
      EXPECT_NEAR(before.speed + before.acceleration * c.step, p.speed, 1e-9);
      EXPECT_NEAR((before.speed + p.speed) * 0.5 * c.step, p.x - before.x,
                  1e-9);
    }
  }
}

TEST(LaneFollowPlannerTest, StopsBeforeEndAndObstacle) {
  for (bool obstacle : {false, true}) {
    auto input = Scene();
    LaneFollowPlanner planner(SimulationInputPolicy(),
                              SimulationPlannerConfig());
    ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1.0).ok());
    input.odometry.x = obstacle ? 10.0 : 34.0;
    Advance(&input);
    if (obstacle) {
      input.occupancy.obstacles.push_back({1, {18.0, 0.0}, {0.0, 0.0}, 1.0});
    }
    const auto status = planner.Plan(input);
    ASSERT_TRUE(status.ok()) << status.ToString();
    const auto& last = planner.trajectory().points.back();
    EXPECT_LT(last.speed, 0.1);
    EXPECT_LE(last.x + 3.8, obstacle ? 17.0 : 45.0);
    EXPECT_GT(last.x, input.odometry.x);
  }
}

TEST(LaneFollowPlannerTest, InvalidPayloadClearsPreviouslyValidTrajectory) {
  const std::vector<std::function<void(CycleInput*)>> mutations = {
      [](CycleInput* s) { s->corridor.confirmed_forward = false; },
      [](CycleInput* s) { s->corridor.lane_id = 0; },
      [](CycleInput* s) { s->corridor.half_width = 1.5; },
      [](CycleInput* s) { s->corridor.centerline.clear(); },
      [](CycleInput* s) {
        s->corridor.centerline[1] = s->corridor.centerline[0];
      },
      [](CycleInput* s) { s->corridor.centerline[4].y = 4.0; },
      [](CycleInput* s) {
        s->corridor.centerline[2].x = std::numeric_limits<double>::quiet_NaN();
      },
      [](CycleInput* s) { s->corridor.position_error_bound = 0.2; },
      [](CycleInput* s) { s->occupancy.corridor_fully_observed = false; },
      [](CycleInput* s) { s->occupancy.horizon = 1.0; },
      [](CycleInput* s) {
        s->occupancy.obstacles.push_back({0, {10.0, 0.0}, {}, 1.0});
      },
      [](CycleInput* s) {
        s->occupancy.obstacles.push_back({1, {10.0, 0.0}, {}, -1.0});
      },
      [](CycleInput* s) {
        s->occupancy.obstacles.push_back({1, {0.0, 0.0}, {}, 1.0});
      },
      [](CycleInput* s) { s->odometry.heading = 3.14; },
      [](CycleInput* s) { s->odometry.speed_mps = -1.0; },
      [](CycleInput* s) { s->odometry.speed_mps = 10.0; },
      [](CycleInput* s) { s->odometry.x = 44.0; },
      [](CycleInput* s) { s->odometry.y = 2.0; },
      [](CycleInput* s) { s->odometry.stamp.measurement_time -= 0.01; }};
  for (size_t i = 0; i < mutations.size(); ++i) {
    SCOPED_TRACE(i);
    auto input = Scene();
    LaneFollowPlanner planner(SimulationInputPolicy(),
                              SimulationPlannerConfig());
    ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1.0).ok());
    ASSERT_TRUE(planner.Plan(input).ok());
    Advance(&input);
    mutations[i](&input);
    const auto status = planner.Plan(input);
    EXPECT_FALSE(status.ok());
    EXPECT_FALSE(status.error_message().empty());
    EXPECT_FALSE(planner.trajectory().valid);
    EXPECT_TRUE(planner.trajectory().points.empty());
  }
}

TEST(LaneFollowPlannerTest, VersionMutationRejectedAndResetClearsOutput) {
  auto input = Scene();
  LaneFollowPlanner planner(SimulationInputPolicy(), SimulationPlannerConfig());
  ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1.0).ok());
  ASSERT_TRUE(planner.Plan(input).ok());
  const auto graph = input.graph;
  const auto environment = input.environment;
  Advance(&input);
  input.graph = graph;
  input.environment = environment;
  input.prediction.graph_sequence = graph.sequence;
  input.corridor.half_width = 2.6;
  EXPECT_FALSE(planner.Plan(input).ok());
  EXPECT_FALSE(planner.trajectory().valid);
  input.corridor.half_width = 2.5;
  ASSERT_TRUE(planner.Plan(input).ok());
  input.odometry.stamp.epoch.generation = 2;
  ASSERT_TRUE(planner.BeginEpoch(input.odometry, input.planning_time).ok());
  EXPECT_FALSE(planner.trajectory().valid);
  EXPECT_TRUE(planner.trajectory().points.empty());
}

TEST(LaneFollowPlannerTest, RejectsInfeasibleBrakingAndCrossingActor) {
  auto input = Scene();
  LaneFollowPlanner planner(SimulationInputPolicy(), SimulationPlannerConfig());
  ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1.0).ok());
  Advance(&input);
  input.odometry.speed_mps = 2.0;
  input.occupancy.obstacles.push_back({1, {7.0, 0.0}, {}, 1.0});
  EXPECT_FALSE(planner.Plan(input).ok());
  input = Scene();
  LaneFollowPlanner crossing(SimulationInputPolicy(),
                             SimulationPlannerConfig());
  ASSERT_TRUE(crossing.BeginEpoch(input.odometry, 1.0).ok());
  input.occupancy.obstacles.push_back({1, {7.0, 5.0}, {0.0, -1.0}, 1.0});
  EXPECT_FALSE(crossing.Plan(input).ok());
}

TEST(LaneFollowPlannerTest, FailedTaskCannotAllowInPlacePayloadRepair) {
  for (bool graph_failure : {false, true}) {
    auto input = Scene();
    LaneFollowPlanner planner(SimulationInputPolicy(),
                              SimulationPlannerConfig());
    ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1.0).ok());
    if (graph_failure) {
      input.corridor.confirmed_forward = false;
    } else {
      input.occupancy.corridor_fully_observed = false;
    }
    ASSERT_FALSE(planner.Plan(input).ok());
    const auto old_graph = input.graph;
    const auto old_prediction = input.prediction;
    Advance(&input);
    input.graph = old_graph;
    input.prediction = old_prediction;
    input.corridor.confirmed_forward = true;
    input.occupancy.corridor_fully_observed = true;
    const auto rejected = planner.Plan(input);
    EXPECT_FALSE(rejected.ok());
    EXPECT_NE(std::string::npos,
              rejected.error_message().find("payload mutated"));
    EXPECT_FALSE(planner.trajectory().valid);
    Advance(&input);
    EXPECT_TRUE(planner.Plan(input).ok());
  }
}

TEST(LaneFollowPlannerTest, RejectsBadConfiguration) {
  for (int field = 0; field < 4; ++field) {
    auto config = SimulationPlannerConfig();
    if (field == 0) config.braking = 0.0;
    if (field == 1) config.step = 0.5;
    if (field == 2) config.cruise_speed = 20.0;
    if (field == 3) config.horizon = std::numeric_limits<double>::infinity();
    LaneFollowPlanner planner(SimulationInputPolicy(), config);
    EXPECT_FALSE(planner.BeginEpoch(Scene().odometry, 1.0).ok());
    EXPECT_FALSE(planner.Plan(Scene()).ok());
    EXPECT_FALSE(planner.trajectory().valid);
  }
}

TEST(LaneFollowPlannerTest, StitchesAtSameAbsoluteTimeAndBoundsControlChanges) {
  auto input = Scene();
  const auto config = SimulationPlannerConfig();
  LaneFollowPlanner planner(SimulationInputPolicy(), config);
  ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1.0).ok());
  ASSERT_TRUE(planner.Plan(input).ok());
  const auto old = planner.trajectory();
  Advance(&input);
  const auto& expected = old.points[1];
  input.odometry.x = expected.x;
  input.odometry.y = expected.y;
  input.odometry.heading = expected.heading;
  input.odometry.speed_mps = expected.speed;
  for (auto& p : input.corridor.centerline) p.y += 0.01;
  auto status = planner.Plan(input);
  ASSERT_TRUE(status.ok()) << status.ToString();
  ASSERT_TRUE(planner.trajectory().stitched);
  const auto& points = planner.trajectory().points;
  EXPECT_NEAR(expected.x, points[0].x, 1e-8);
  EXPECT_NEAR(expected.heading, points[0].heading, 1e-8);
  EXPECT_NEAR(expected.acceleration, points[0].acceleration, 1e-8);
  EXPECT_NEAR(expected.curvature, points[0].curvature, 1e-8);
  for (size_t i = 1; i < points.size(); ++i) {
    EXPECT_LE(std::abs(points[i].acceleration - points[i - 1].acceleration),
              config.max_jerk * config.step + 1e-8);
    EXPECT_LE(std::abs(points[i].curvature - points[i - 1].curvature),
              config.max_curvature_rate * config.step + 1e-8);
  }
  Advance(&input);
  input.occupancy.obstacles.push_back({1, {1, 0}, {}, 1});
  EXPECT_FALSE(planner.Plan(input).ok());
  EXPECT_FALSE(planner.trajectory().valid);
  Advance(&input);
  input.occupancy.obstacles.clear();
  ASSERT_TRUE(planner.Plan(input).ok());
  EXPECT_FALSE(planner.trajectory().stitched);
}

TEST(LaneFollowPlannerTest, TrackingDivergenceDoesNotResetBraking) {
  auto input = Scene();
  const auto config = SimulationPlannerConfig();
  LaneFollowPlanner planner(SimulationInputPolicy(), config);
  ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1.0).ok());
  Advance(&input);
  input.odometry.x = 37.5;
  input.odometry.speed_mps = 1.7;
  auto status = planner.Plan(input);
  ASSERT_TRUE(status.ok()) << status.ToString();
  const auto expected = planner.trajectory().points[1];
  ASSERT_LT(expected.acceleration, -config.max_jerk * config.step);
  Advance(&input);
  input.odometry.x = expected.x + config.stitch_position_tolerance + 0.01;
  input.odometry.speed_mps = expected.speed;
  status = planner.Plan(input);
  ASSERT_TRUE(status.ok()) << status.ToString();
  const auto& trajectory = planner.trajectory();
  EXPECT_FALSE(trajectory.stitched);
  EXPECT_DOUBLE_EQ(input.odometry.x, trajectory.points.front().x);
  EXPECT_LT(trajectory.points.front().acceleration,
            -config.max_jerk * config.step);
  EXPECT_LE(
      std::abs(trajectory.points.front().acceleration - expected.acceleration),
      config.max_jerk * config.step + 1e-8);
}

TEST(LaneFollowPlannerTest, StopBoundaryDoesNotCollapseLateralLookahead) {
  auto input = Scene();
  LaneFollowPlanner planner(SimulationInputPolicy(), SimulationPlannerConfig());
  ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1.0).ok());
  Advance(&input);
  input.odometry.x = 39.0;
  for (auto& p : input.corridor.centerline) p.y = 0.015;
  const auto status = planner.Plan(input);
  ASSERT_TRUE(status.ok()) << status.ToString();
  EXPECT_LT(planner.trajectory().points.back().speed, 0.1);
  EXPECT_LT(std::abs(planner.trajectory().points.front().curvature), 0.01);
}

TEST(LaneFollowPlannerTest, StitchCannotHideUnsafeMeasuredStoppingState) {
  auto input = Scene();
  LaneFollowPlanner planner(SimulationInputPolicy(), SimulationPlannerConfig());
  ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1.0).ok());
  Advance(&input);
  input.odometry.x = 39.58;
  ASSERT_TRUE(planner.Plan(input).ok());
  Advance(&input);
  // Inside stitch speed tolerance, but outside measured stopping feasibility.
  input.odometry.speed_mps = 0.24;
  const auto status = planner.Plan(input);
  EXPECT_FALSE(status.ok());
  EXPECT_NE(std::string::npos, status.error_message().find("actual ODOM"));
  EXPECT_FALSE(planner.trajectory().valid);
}

}  // namespace
}  // namespace local_planning
}  // namespace apollo
