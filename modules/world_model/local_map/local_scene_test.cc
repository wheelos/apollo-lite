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

#include <cmath>

#include "gtest/gtest.h"

#include "modules/world_model/local_map/local_scene_builder.h"
#include "modules/world_model/local_map/measured_lane_geometry.h"

namespace apollo {
namespace world_model {
namespace {
TemporalLanePolicy Policy() {
  return {0.5, 2.0, 0.2, 0.6, 1.0, 0.22, 0.3, 2.0, 3.0, 6.0, 0.1, 0.3};
}
SceneSource Source(double t, uint64_t seq, const std::string& frame) {
  return {frame, "test",  "session",           1, seq, t,
          t,     t + 0.5, SceneHealth::HEALTHY};
}
LaneObservation Lane(double t, uint64_t seq) {
  LaneObservation lane;
  lane.session = "session";
  lane.generation = 1;
  lane.sequence = seq;
  lane.measurement_time = t;
  lane.position_error = 0.02;
  lane.forward_confirmed = true;
  for (double x = -8.0; x <= 25.0; x += 0.5) {
    lane.left.push_back({x, 2.5});
    lane.right.push_back({x, -2.5});
  }
  return lane;
}
EnvironmentObservation Environment(double t, uint64_t seq) {
  EnvironmentObservation environment;
  environment.session = "session";
  environment.generation = 1;
  environment.sequence = seq;
  environment.measurement_time = t;
  environment.position_error = 0.02;
  environment.free_space = {{-10, -4}, {30, -4}, {30, 4}, {-10, 4}};
  environment.curbs = {{1, {{-10, -3}, {30, -3}}}, {2, {{-10, 3}, {30, 3}}}};
  environment.obstacles = {{3, {{15, -0.5}, {16, -0.5}, {16, 0.5}, {15, 0.5}}}};
  return environment;
}
void ObserveAll(LocalSceneBuilder* builder, double t, uint64_t seq) {
  ASSERT_TRUE(
      builder->Observe(Source(t, seq, "base_link"), Lane(t, seq), t).ok());
  ASSERT_TRUE(builder
                  ->ObserveEnvironment(Source(t, seq, "base_link"),
                                       Environment(t, seq), t)
                  .ok());
}

TEST(LocalSceneTest, PreservesEntitiesEvidenceAndDisablesUnsupportedManeuvers) {
  LocalSceneBuilder builder(Policy(), "odom", "base_link", "test");
  ASSERT_TRUE(
      builder.BeginEpoch(Source(1, 1, "odom"), {1, 0, 0, 0, 0.01}, 1, true)
          .ok());
  ObserveAll(&builder, 1, 1);
  LocalScene first, second;
  ASSERT_TRUE(builder.Build(1, &first).ok());
  ASSERT_TRUE(builder.Build(1.1, &second).ok());
  ASSERT_EQ(2U, second.boundaries.size());
  EXPECT_EQ(first.boundaries[0].id, second.boundaries[0].id);
  EXPECT_NE(second.boundaries[0].id, second.boundaries[1].id);
  EXPECT_EQ(first.lanes[0].id, second.lanes[0].id);
  EXPECT_GT(second.source.sequence, first.source.sequence);
  EXPECT_EQ(first.source.measurement_time, second.source.measurement_time);
  EXPECT_EQ("odom", second.source.frame_id);
  EXPECT_EQ("test", second.source.clock_id);
  EXPECT_EQ(VehicleReference::REAR_AXLE, second.reference);
  EXPECT_EQ(SceneMode::LANE, second.mode);
  EXPECT_TRUE(second.capabilities.drivable_region);
  EXPECT_TRUE(second.capabilities.lane_follow);
  EXPECT_FALSE(second.capabilities.lane_change);
  EXPECT_FALSE(second.capabilities.intersection);
  EXPECT_EQ(EvidenceState::UNKNOWN, second.boundaries[0].crossing);
  EXPECT_EQ(4U, second.environment.free_space.size());
  EXPECT_EQ(2U, second.environment.curbs.size());
  EXPECT_EQ(1U, second.environment.obstacles.size());
}

TEST(LocalSceneTest, RestrictiveOrInvalidCurrentObservationRevokesPublication) {
  for (int failure = 0; failure < 6; ++failure) {
    SCOPED_TRACE(failure);
    LocalSceneBuilder builder(Policy(), "odom", "base_link", "test");
    ASSERT_TRUE(
        builder.BeginEpoch(Source(1, 1, "odom"), {1, 0, 0, 0, 0}, 1, true)
            .ok());
    ObserveAll(&builder, 1, 1);
    LocalScene previous;
    ASSERT_TRUE(builder.Build(1, &previous).ok());
    ASSERT_TRUE(
        builder.AddOdometry(Source(1.1, 2, "odom"), {1.1, 0, 0, 0, 0}, 1.1)
            .ok());
    auto source = Source(1.1, 2, "base_link");
    auto lane = Lane(1.1, 2);
    if (failure == 0) source.frame_id = "camera";
    if (failure == 1) source.clock_id = "other";
    if (failure == 2) source.health = SceneHealth::UNKNOWN;
    if (failure == 3) lane.forward_confirmed = false;
    if (failure == 4) lane.right.clear();
    if (failure == 5) source.measurement_time -= 0.01;
    EXPECT_FALSE(builder.Observe(source, lane, 1.1).ok());
    LocalScene rejected;
    ASSERT_TRUE(builder.Build(1.1, &rejected).ok());
    EXPECT_EQ(SceneMode::AREA, rejected.mode);
    EXPECT_TRUE(rejected.capabilities.drivable_region);
    EXPECT_FALSE(rejected.capabilities.lane_follow);
    EXPECT_TRUE(rejected.lanes.empty());
    EXPECT_FALSE(rejected.mode_reason.empty());
    ASSERT_TRUE(
        builder.AddOdometry(Source(1.2, 3, "odom"), {1.2, 0, 0, 0, 0}, 1.2)
            .ok());
    ObserveAll(&builder, 1.2, 3);
    ASSERT_TRUE(builder.Build(1.2, &rejected).ok());
    EXPECT_EQ(SceneMode::LANE, rejected.mode);
    EXPECT_NE(previous.lanes[0].id, rejected.lanes[0].id);
    EXPECT_NE(previous.boundaries[0].id, rejected.boundaries[0].id);
  }
}

TEST(LocalSceneTest, OdomLossRequiresNewEpochButLateOldPacketsDoNotPoisonIt) {
  LocalSceneBuilder builder(Policy(), "odom", "base_link", "test");
  ASSERT_TRUE(
      builder.BeginEpoch(Source(1, 1, "odom"), {1, 0, 0, 0, 0}, 1, true).ok());
  ObserveAll(&builder, 1, 1);
  auto invalid = Source(1.1, 2, "odom");
  invalid.health = SceneHealth::INVALID;
  EXPECT_FALSE(builder.AddOdometry(invalid, {1.1, 0, 0, 0, 0}, 1.1).ok());
  LocalScene scene;
  EXPECT_FALSE(builder.Build(1.1, &scene).ok());
  EXPECT_FALSE(
      builder.AddOdometry(Source(1.2, 3, "odom"), {1.2, 0, 0, 0, 0}, 1.2).ok());
  auto odom = Source(2, 1, "odom");
  odom.generation = 2;
  ASSERT_TRUE(builder.BeginEpoch(odom, {2, 0, 0, 0, 0}, 2, true).ok());
  auto source = Source(2, 1, "base_link");
  source.generation = 2;
  auto lane = Lane(2, 1);
  lane.generation = 2;
  ASSERT_TRUE(builder.Observe(source, lane, 2).ok());
  auto environment = Environment(2, 1);
  environment.generation = 2;
  ASSERT_TRUE(builder.ObserveEnvironment(source, environment, 2).ok());
  EXPECT_FALSE(builder.Observe(Source(1, 1, "base_link"), Lane(1, 1), 2).ok());
  EXPECT_TRUE(builder.Build(2, &scene).ok());
  EXPECT_EQ(2U, scene.source.generation);
}

TEST(LocalSceneTest, OdomTimeoutRevokesEvenStillFreshLaneEvidence) {
  LocalSceneBuilder builder(Policy(), "odom", "base_link", "test");
  ASSERT_TRUE(
      builder.BeginEpoch(Source(1, 1, "odom"), {1, 0, 0, 0, 0}, 1, true).ok());
  ObserveAll(&builder, 1, 1);
  LocalScene scene;
  EXPECT_FALSE(builder.Build(1.3, &scene).ok());
  EXPECT_EQ(SceneMode::INVALID, scene.mode);
  EXPECT_EQ(SceneHealth::INVALID, scene.source.health);
  EXPECT_FALSE(scene.capabilities.lane_follow);
}

TEST(LocalSceneTest, EnvironmentExpiryInvalidatesOtherwiseHealthyOdom) {
  LocalSceneBuilder builder(Policy(), "odom", "base_link", "test");
  ASSERT_TRUE(
      builder.BeginEpoch(Source(1, 1, "odom"), {1, 0, 0, 0, 0}, 1, true).ok());
  auto environment_source = Source(1, 1, "base_link");
  environment_source.valid_until = 1.15;
  ASSERT_TRUE(
      builder.ObserveEnvironment(environment_source, Environment(1, 1), 1)
          .ok());
  LocalScene scene;
  ASSERT_TRUE(builder.Build(1, &scene).ok());
  EXPECT_EQ(SceneMode::AREA, scene.mode);
  EXPECT_FALSE(builder.Build(1.15, &scene).ok());
  EXPECT_EQ(SceneMode::INVALID, scene.mode);
}

TEST(LocalSceneTest, InvalidEnvironmentCannotReopenAcceptedSourceFrontier) {
  LocalSceneBuilder builder(Policy(), "odom", "base_link", "test");
  ASSERT_TRUE(
      builder.BeginEpoch(Source(1, 1, "odom"), {1, 0, 0, 0, 0}, 1, true).ok());
  ASSERT_TRUE(
      builder
          .ObserveEnvironment(Source(1, 1, "base_link"), Environment(1, 1), 1)
          .ok());

  ASSERT_TRUE(
      builder.AddOdometry(Source(1.1, 2, "odom"), {1.1, 0, 0, 0, 0}, 1.1).ok());
  auto invalid_environment = Environment(1.1, 2);
  invalid_environment.free_space.clear();
  EXPECT_FALSE(builder
                   .ObserveEnvironment(Source(1.1, 2, "base_link"),
                                       invalid_environment, 1.1)
                   .ok());

  EXPECT_FALSE(
      builder
          .ObserveEnvironment(Source(1, 1, "base_link"), Environment(1, 1), 1.1)
          .ok());
  LocalScene scene;
  EXPECT_FALSE(builder.Build(1.1, &scene).ok());
  EXPECT_EQ(SceneMode::INVALID, scene.mode);

  ASSERT_TRUE(
      builder.AddOdometry(Source(1.2, 3, "odom"), {1.2, 0, 0, 0, 0}, 1.2).ok());
  ASSERT_TRUE(builder
                  .ObserveEnvironment(Source(1.2, 3, "base_link"),
                                      Environment(1.2, 3), 1.2)
                  .ok());
  EXPECT_TRUE(builder.Build(1.2, &scene).ok());
  EXPECT_EQ(SceneMode::AREA, scene.mode);

  ASSERT_TRUE(
      builder.AddOdometry(Source(1.3, 4, "odom"), {1.3, 0, 0, 0, 0}, 1.3).ok());
  EXPECT_FALSE(builder
                   .ObserveEnvironment(Source(1.15, 4, "base_link"),
                                       Environment(1.15, 4), 1.3)
                   .ok());
  EXPECT_FALSE(builder.Build(1.3, &scene).ok());
}

TEST(LocalSceneTest, SwitchesAreaLaneAreaLaneWithoutFabricatedGeometry) {
  LocalSceneBuilder builder(Policy(), "odom", "base_link", "test");
  ASSERT_TRUE(
      builder.BeginEpoch(Source(1, 1, "odom"), {1, 0, 0, 0, 0}, 1, true).ok());
  ASSERT_TRUE(
      builder
          .ObserveEnvironment(Source(1, 1, "base_link"), Environment(1, 1), 1)
          .ok());
  LocalScene scene;
  ASSERT_TRUE(builder.Build(1, &scene).ok());
  EXPECT_EQ(SceneMode::AREA, scene.mode);
  EXPECT_TRUE(scene.capabilities.drivable_region);
  EXPECT_FALSE(scene.capabilities.lane_follow);
  EXPECT_TRUE(scene.boundaries.empty());
  EXPECT_TRUE(scene.lanes.empty());

  ASSERT_TRUE(
      builder.AddOdometry(Source(1.1, 2, "odom"), {1.1, 0, 0, 0, 0}, 1.1).ok());
  ASSERT_TRUE(
      builder.Observe(Source(1.1, 2, "base_link"), Lane(1.1, 2), 1.1).ok());
  ASSERT_TRUE(builder
                  .ObserveEnvironment(Source(1.1, 2, "base_link"),
                                      Environment(1.1, 2), 1.1)
                  .ok());
  ASSERT_TRUE(builder.Build(1.1, &scene).ok());
  ASSERT_EQ(SceneMode::LANE, scene.mode);
  ASSERT_EQ(1U, scene.lanes.size());
  const uint64_t first_lane = scene.lanes[0].id;

  ASSERT_TRUE(
      builder.AddOdometry(Source(1.2, 3, "odom"), {1.2, 0, 0, 0, 0}, 1.2).ok());
  ASSERT_TRUE(
      builder.ObserveLaneAbsence(Source(1.2, 3, "base_link"), 1.2).ok());
  ASSERT_TRUE(builder
                  .ObserveEnvironment(Source(1.2, 3, "base_link"),
                                      Environment(1.2, 3), 1.2)
                  .ok());
  ASSERT_TRUE(builder.Build(1.2, &scene).ok());
  EXPECT_EQ(SceneMode::AREA, scene.mode);
  EXPECT_TRUE(scene.lanes.empty());
  EXPECT_FALSE(scene.mode_reason.empty());

  ASSERT_TRUE(
      builder.AddOdometry(Source(1.3, 4, "odom"), {1.3, 0, 0, 0, 0}, 1.3).ok());
  ASSERT_TRUE(
      builder.Observe(Source(1.3, 4, "base_link"), Lane(1.3, 4), 1.3).ok());
  ASSERT_TRUE(builder
                  .ObserveEnvironment(Source(1.3, 4, "base_link"),
                                      Environment(1.3, 4), 1.3)
                  .ok());
  ASSERT_TRUE(builder.Build(1.3, &scene).ok());
  ASSERT_EQ(SceneMode::LANE, scene.mode);
  ASSERT_EQ(1U, scene.lanes.size());
  EXPECT_NE(first_lane, scene.lanes[0].id);

  EXPECT_FALSE(builder.Build(1.8, &scene).ok());
  EXPECT_EQ(SceneMode::INVALID, scene.mode);
}

TEST(LocalSceneTest, SamplesMeasuredCubicBoundariesOnlyInsideObservedRange) {
  MeasuredCubicLane measured;
  measured.session = "session";
  measured.generation = 1;
  measured.sequence = 1;
  measured.measurement_time = 1;
  measured.position_error = 0.02;
  measured.forward_confirmed = true;
  measured.sample_spacing = 0.5;
  measured.left = {2.5, 0.1, 0.01, -0.0002, -2.0, 12.0};
  measured.right = {-2.5, 0.1, 0.01, -0.0002, 0.0, 10.0};
  LaneObservation lane;
  ASSERT_TRUE(BuildMeasuredLaneObservation(measured, &lane).ok());
  ASSERT_FALSE(lane.left.empty());
  EXPECT_DOUBLE_EQ(0.0, lane.left.front().x);
  EXPECT_DOUBLE_EQ(10.0, lane.left.back().x);
  EXPECT_EQ(lane.left.size(), lane.right.size());

  std::vector<MeasuredBoundarySample> samples;
  ASSERT_TRUE(
      SampleMeasuredCubicBoundary(measured.left, 0.0, 1.0, 0.5, &samples).ok());
  ASSERT_EQ(3U, samples.size());
  EXPECT_GT(samples.back().station, 0.99);
  EXPECT_NEAR(std::atan2(0.1194, 1.0), samples.back().heading, 1e-9);
  EXPECT_NEAR(0.0188 / std::pow(1.0 + 0.1194 * 0.1194, 1.5),
              samples.back().curvature, 1e-9);

  LocalSceneBuilder builder(Policy(), "odom", "base_link", "test");
  ASSERT_TRUE(
      builder.BeginEpoch(Source(1, 1, "odom"), {1, 0, 0, 0, 0}, 1, true).ok());
  ASSERT_TRUE(builder.Observe(Source(1, 1, "base_link"), lane, 1).ok());
  ASSERT_TRUE(
      builder
          .ObserveEnvironment(Source(1, 1, "base_link"), Environment(1, 1), 1)
          .ok());
  LocalScene scene;
  ASSERT_TRUE(builder.Build(1, &scene).ok());
  EXPECT_EQ(SceneMode::LANE, scene.mode);
  EXPECT_LT(scene.boundaries[0].points.back().x, 10.01);
}

TEST(LocalSceneTest, UnobservedNavigationCannotCreateLaneOrConnection) {
  LocalSceneBuilder builder(Policy(), "odom", "base_link", "test");
  ASSERT_TRUE(
      builder.BeginEpoch(Source(1, 1, "odom"), {1, 0, 0, 0, 0}, 1, true).ok());
  ObserveAll(&builder, 1, 1);
  LocalScene scene;
  ASSERT_TRUE(builder.Build(1, &scene).ok());
  NavigationPrior prior{Source(1, 1, "odom"), scene.lanes[0].id, 999};
  EXPECT_FALSE(builder.Build(1, &scene, &prior).ok());
  EXPECT_EQ(SceneHealth::INVALID, scene.source.health);
  EXPECT_TRUE(scene.lanes.empty());
  ASSERT_TRUE(builder.Build(1, &scene).ok());
  EXPECT_TRUE(scene.edges.empty());
  EXPECT_TRUE(scene.rules.empty());
  EXPECT_FALSE(scene.navigation.has_value());
}

TEST(LocalSceneTest, EnvironmentRequiresFreshMeasuredCoverageAndTransforms) {
  LocalSceneBuilder builder(Policy(), "odom", "base_link", "test");
  ASSERT_TRUE(
      builder.BeginEpoch(Source(1, 1, "odom"), {1, 5, 2, 0, 0.01}, 1, true)
          .ok());
  ASSERT_TRUE(builder.Observe(Source(1, 1, "base_link"), Lane(1, 1), 1).ok());
  LocalScene scene;
  EXPECT_FALSE(builder.Build(1, &scene).ok());
  auto environment = Environment(1, 1);
  ASSERT_TRUE(
      builder.ObserveEnvironment(Source(1, 1, "base_link"), environment, 1)
          .ok());
  ASSERT_TRUE(builder.Build(1, &scene).ok());
  EXPECT_NEAR(-5, scene.environment.free_space[0].x, 1e-9);
  EXPECT_NEAR(-2, scene.environment.free_space[0].y, 1e-9);
  environment.sequence = 2;
  environment.measurement_time = 1.1;
  environment.free_space.clear();
  ASSERT_TRUE(
      builder.AddOdometry(Source(1.1, 2, "odom"), {1.1, 5, 2, 0, 0.01}, 1.1)
          .ok());
  EXPECT_FALSE(
      builder.ObserveEnvironment(Source(1.1, 2, "base_link"), environment, 1.1)
          .ok());
  EXPECT_FALSE(builder.Build(1.1, &scene).ok());
}

TEST(LocalSceneTest, EnvironmentRejectsHealthEpochTimeAndUnsupportedShape) {
  for (int failure = 0; failure < 4; ++failure) {
    SCOPED_TRACE(failure);
    LocalSceneBuilder builder(Policy(), "odom", "base_link", "test");
    ASSERT_TRUE(
        builder.BeginEpoch(Source(1, 1, "odom"), {1, 0, 0, 0, 0}, 1, true)
            .ok());
    ASSERT_TRUE(builder.Observe(Source(1, 1, "base_link"), Lane(1, 1), 1).ok());
    auto source = Source(1, 1, "base_link");
    auto environment = Environment(1, 1);
    if (failure == 0) source.health = SceneHealth::INVALID;
    if (failure == 1) environment.generation = 2;
    if (failure == 2) environment.measurement_time = 0.9;
    if (failure == 3)
      environment.free_space = {{-1, -1}, {1, 1}, {1, -1}, {-1, 1}};
    EXPECT_FALSE(builder.ObserveEnvironment(source, environment, 1).ok());
    LocalScene scene;
    EXPECT_FALSE(builder.Build(1, &scene).ok());
    EXPECT_FALSE(scene.capabilities.lane_follow);
  }
}
}  // namespace
}  // namespace world_model
}  // namespace apollo
