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

#include "modules/local_planning/planning_context/planning_input_builder.h"

#include <functional>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace local_planning {
namespace {
using world_model::EvidenceState;
using world_model::LocalScene;
using world_model::SceneMode;

InputPolicy Policy() { return {"odom", "test_clock", 0.5, 0.5, 0.5, 0.01}; }

PlanningInputConfig Config() {
  PlanningInputConfig config;
  config.vehicle = {1.5, 0.4, 0.6, 0.1};
  config.reference_rear = 1.0;
  config.reference_front = 8.0;
  config.continuity_tolerance = 0.1;
  config.minimum_prediction_horizon = 2.0;
  return config;
}

SourceStamp Stamp(double time, uint64_t sequence) {
  return {"odom", "test_clock", {"session", 1}, sequence,
          time,   time,         time + 1.0,     InputHealth::HEALTHY};
}

OdometryInput Odom(double time, uint64_t sequence) {
  OdometryInput odometry;
  odometry.stamp = Stamp(time, sequence);
  return odometry;
}

LocalScene Scene(double time, uint64_t sequence,
                 SceneMode mode = SceneMode::LANE) {
  LocalScene scene;
  scene.source = {"odom", "test_clock", "session",
                  1,      sequence,     time,
                  time,   time + 1.0,   world_model::SceneHealth::HEALTHY};
  scene.mode = mode;
  scene.reference = world_model::VehicleReference::REAR_AXLE;
  scene.capabilities.drivable_region = true;
  scene.capabilities.lane_follow = mode == SceneMode::LANE;
  if (mode == SceneMode::LANE) {
    world_model::LaneBoundary left;
    left.id = 1;
    world_model::LaneBoundary right;
    right.id = 2;
    for (double x = -8.0; x <= 30.0; x += 0.5) {
      left.points.push_back({x, 2.5});
      right.points.push_back({x, -2.5});
      left.observation_times.push_back(time);
      right.observation_times.push_back(time);
      left.valid_until.push_back(time + 1.1);
      right.valid_until.push_back(time + 1.1);
    }
    scene.boundaries = {left, right};
    scene.lanes.push_back(
        {1, left.id, right.id, EvidenceState::CONFIRMED, 0.02});
  } else {
    scene.mode_reason = "no lane entities available";
  }
  scene.environment.source = scene.source;
  scene.environment.position_error = 0.02;
  scene.environment.free_space = {
      {-10.0, -4.0}, {32.0, -4.0}, {32.0, 4.0}, {-10.0, 4.0}};
  return scene;
}

void AdvanceScene(double time, uint64_t sequence, LocalScene* scene) {
  scene->source.sequence = sequence;
  scene->source.measurement_time = time;
  scene->source.publication_time = time;
  scene->source.valid_until = time + 1.0;
  scene->environment.source = scene->source;
  for (auto& boundary : scene->boundaries) {
    for (auto& observation_time : boundary.observation_times)
      observation_time = time;
    for (auto& deadline : boundary.valid_until) deadline = time + 1.1;
  }
}

PredictionInput Prediction(double time, uint64_t sequence,
                           uint64_t graph_sequence) {
  return {Stamp(time, sequence), graph_sequence};
}

LocalOccupancy Occupancy() {
  LocalOccupancy occupancy;
  occupancy.corridor_fully_observed = true;
  occupancy.drivable_region_fully_observed = true;
  occupancy.horizon = 4.0;
  return occupancy;
}

common::Status Arm(PlanningInputBuilder* builder) {
  return builder->BeginEpoch(Odom(9.9, 1), 9.9);
}

TEST(PlanningInputBuilderTest, BuildsLaneAndAreaConditionsFromMeasuredOdom) {
  PlanningInputBuilder builder(Policy(), Config());
  ASSERT_TRUE(Arm(&builder).ok());
  auto lane = Scene(10.0, 1);
  auto status = builder.Build(lane, Odom(10.0, 2), Prediction(10.0, 1, 1),
                              Occupancy(), 10.0);
  ASSERT_TRUE(status.ok()) << status.ToString();
  const auto* conditions = builder.conditions();
  ASSERT_NE(nullptr, conditions);
  EXPECT_TRUE(conditions->executable_conditions);
  EXPECT_EQ(10.0, conditions->measured_start.measured_at);
  EXPECT_EQ(0.0, conditions->measured_start.x);
  EXPECT_EQ(SceneMode::LANE, conditions->geometry.mode);
  ASSERT_TRUE(conditions->geometry.reference.has_value());
  EXPECT_EQ(conditions->cycle.corridor,
            conditions->geometry.reference->corridor);
  EXPECT_EQ(1U, conditions->dynamic_occupancy.graph_sequence);
  EXPECT_DOUBLE_EQ(14.0, conditions->dynamic_occupancy.coverage_end);
  ASSERT_EQ(conditions->geometry.reference->line.size(),
            conditions->geometry.reference->left.size());
  EXPECT_EQ(conditions->geometry.reference->line.size(),
            conditions->geometry.reference->right.size());

  auto area = Scene(10.1, 2, SceneMode::AREA);
  status = builder.Build(area, Odom(10.1, 3), Prediction(10.1, 2, 2),
                         Occupancy(), 10.1);
  ASSERT_TRUE(status.ok()) << status.ToString();
  conditions = builder.conditions();
  ASSERT_NE(nullptr, conditions);
  EXPECT_EQ(SceneMode::AREA, conditions->geometry.mode);
  EXPECT_FALSE(conditions->geometry.reference.has_value());
  EXPECT_TRUE(conditions->geometry.environment.free_space.size() >= 3);

  auto recovered = Scene(10.2, 3);
  status = builder.Build(recovered, Odom(10.2, 4), Prediction(10.2, 3, 3),
                         Occupancy(), 10.2);
  ASSERT_TRUE(status.ok()) << status.ToString();
  ASSERT_TRUE(builder.conditions()->geometry.reference.has_value());
  EXPECT_EQ(ContinuityReason::MODE_CHANGED,
            builder.conditions()->geometry.reference->continuity.reason);
}

TEST(PlanningInputBuilderTest, RejectsBadAreaOdometryFrameTimeAndEpoch) {
  const std::vector<std::function<void(OdometryInput*)>> mutations = {
      [](OdometryInput* odom) { odom->stamp.frame_id = "map"; },
      [](OdometryInput* odom) { odom->stamp.clock_id = "other_clock"; },
      [](OdometryInput* odom) { odom->stamp.measurement_time = 9.4; },
      [](OdometryInput* odom) {
        odom->stamp.publication_time = 10.1;
        odom->stamp.measurement_time = 10.1;
      },
      [](OdometryInput* odom) { ++odom->stamp.epoch.generation; }};
  for (size_t i = 0; i < mutations.size(); ++i) {
    SCOPED_TRACE(i);
    PlanningInputBuilder builder(Policy(), Config());
    ASSERT_TRUE(Arm(&builder).ok());
    auto odometry = Odom(10.0, 2);
    mutations[i](&odometry);
    const auto status =
        builder.Build(Scene(10.0, 1, SceneMode::AREA), odometry,
                      Prediction(10.0, 1, 1), Occupancy(), 10.0);
    EXPECT_FALSE(status.ok());
    EXPECT_EQ(nullptr, builder.conditions());
  }
}

TEST(PlanningInputBuilderTest, ReportsEpochResetAndUsesNewOdomAnchor) {
  PlanningInputBuilder builder(Policy(), Config());
  ASSERT_TRUE(Arm(&builder).ok());
  ASSERT_TRUE(builder
                  .Build(Scene(10.0, 1), Odom(10.0, 2), Prediction(10.0, 1, 1),
                         Occupancy(), 10.0)
                  .ok());

  auto epoch_start = Odom(10.1, 1);
  epoch_start.stamp.epoch.generation = 2;
  ASSERT_TRUE(builder.BeginEpoch(epoch_start, 10.1).ok());
  auto odometry = Odom(10.2, 2);
  odometry.stamp.epoch.generation = 2;
  odometry.x = 2.0;
  auto prediction = Prediction(10.2, 1, 1);
  prediction.stamp.epoch.generation = 2;
  auto scene = Scene(10.2, 1);
  scene.source.generation = 2;
  scene.environment.source = scene.source;
  const auto status =
      builder.Build(scene, odometry, prediction, Occupancy(), 10.2);
  ASSERT_TRUE(status.ok()) << status.ToString();
  ASSERT_TRUE(builder.conditions()->geometry.reference.has_value());
  EXPECT_EQ(2.0, builder.conditions()->measured_start.x);
  EXPECT_EQ(ContinuityReason::ODOM_EPOCH_CHANGED,
            builder.conditions()->geometry.reference->continuity.reason);
}

TEST(PlanningInputBuilderTest, RejectsMutationOfUnusedAcceptedSceneEvidence) {
  PlanningInputBuilder builder(Policy(), Config());
  ASSERT_TRUE(Arm(&builder).ok());
  auto scene = Scene(10.0, 1);
  ASSERT_TRUE(builder
                  .Build(scene, Odom(10.0, 2), Prediction(10.0, 1, 1),
                         Occupancy(), 10.0)
                  .ok());

  auto mutated = scene;
  mutated.boundaries[0].crossing = EvidenceState::CONFIRMED;
  const auto status = builder.Build(mutated, Odom(10.1, 3),
                                    Prediction(10.0, 1, 1), Occupancy(), 10.1);
  EXPECT_FALSE(status.ok());
  EXPECT_NE(std::string::npos, status.error_message().find("mutated"));
  EXPECT_EQ(nullptr, builder.conditions());
}

TEST(PlanningInputBuilderTest, RejectedFutureGraphDoesNotAdvanceSceneFrontier) {
  PlanningInputBuilder builder(Policy(), Config());
  ASSERT_TRUE(Arm(&builder).ok());
  ASSERT_TRUE(builder
                  .Build(Scene(10.0, 1), Odom(10.0, 2), Prediction(10.0, 1, 1),
                         Occupancy(), 10.0)
                  .ok());

  auto future = Scene(10.2, 2);
  future.source.publication_time = 10.3;
  future.environment.source = future.source;
  EXPECT_FALSE(builder
                   .Build(future, Odom(10.1, 3), Prediction(10.1, 2, 2),
                          Occupancy(), 10.1)
                   .ok());
  AdvanceScene(10.1, 2, &future);
  const auto status = builder.Build(future, Odom(10.1, 3),
                                    Prediction(10.1, 2, 2), Occupancy(), 10.1);
  EXPECT_TRUE(status.ok()) << status.ToString();
}

TEST(PlanningInputBuilderTest,
     AdmittedGeometryFailureStillFreezesSceneVersion) {
  PlanningInputBuilder builder(Policy(), Config());
  ASSERT_TRUE(Arm(&builder).ok());
  ASSERT_TRUE(builder
                  .Build(Scene(10.0, 1), Odom(10.0, 2), Prediction(10.0, 1, 1),
                         Occupancy(), 10.0)
                  .ok());

  auto invalid = Scene(10.1, 2);
  invalid.lanes[0].left_boundary = 999;
  EXPECT_FALSE(builder
                   .Build(invalid, Odom(10.1, 3), Prediction(10.1, 2, 2),
                          Occupancy(), 10.1)
                   .ok());
  invalid.lanes[0].left_boundary = 1;
  const auto status = builder.Build(invalid, Odom(10.2, 4),
                                    Prediction(10.2, 3, 2), Occupancy(), 10.2);
  EXPECT_FALSE(status.ok());
  EXPECT_NE(std::string::npos, status.error_message().find("mutated"));
}

TEST(PlanningInputBuilderTest,
     RequiresAffirmativeSpaceAndCurrentFootprintClear) {
  PlanningInputBuilder unknown_builder(Policy(), Config());
  ASSERT_TRUE(Arm(&unknown_builder).ok());
  auto unknown = Scene(10.0, 1, SceneMode::AREA);
  unknown.environment.free_space.clear();
  EXPECT_FALSE(unknown_builder
                   .Build(unknown, Odom(10.0, 2), Prediction(10.0, 1, 1),
                          Occupancy(), 10.0)
                   .ok());

  PlanningInputBuilder blocked_builder(Policy(), Config());
  ASSERT_TRUE(Arm(&blocked_builder).ok());
  auto blocked = Scene(10.0, 1, SceneMode::AREA);
  blocked.environment.obstacles.push_back(
      {1, {{-0.1, -0.1}, {0.1, -0.1}, {0.1, 0.1}, {-0.1, 0.1}}});
  EXPECT_FALSE(blocked_builder
                   .Build(blocked, Odom(10.0, 2), Prediction(10.0, 1, 1),
                          Occupancy(), 10.0)
                   .ok());
}

TEST(PlanningInputBuilderTest, RequiresDeclaredDynamicCoverageForAreaToo) {
  PlanningInputBuilder builder(Policy(), Config());
  ASSERT_TRUE(Arm(&builder).ok());
  auto missing = Occupancy();
  missing.drivable_region_fully_observed = false;
  EXPECT_FALSE(builder
                   .Build(Scene(10.0, 1, SceneMode::AREA), Odom(10.0, 2),
                          Prediction(10.0, 1, 1), missing, 10.0)
                   .ok());
  EXPECT_EQ(nullptr, builder.conditions());
}

}  // namespace
}  // namespace local_planning
}  // namespace apollo
