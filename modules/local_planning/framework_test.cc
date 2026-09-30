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
#include <functional>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

#include "modules/local_planning/lane_follow_planner.h"
#include "modules/local_planning/planning_context/corridor_selector.h"
#include "modules/local_planning/planning_context/environment_geometry.h"
#include "modules/local_planning/planning_context/planning_geometry.h"
#include "modules/local_planning/planning_context/reference_line.h"
#include "modules/local_planning/planning_context/trajectory_geometry.h"
#include "modules/local_planning/simulation/closed_loop.h"
#include "modules/local_planning/trajectory_continuity/trajectory_validation.h"
#include "modules/world_model/local_map/local_scene_builder.h"
#include "modules/world_model/local_map/navigation_prior.h"

namespace apollo {
namespace local_planning {
namespace {
using world_model::EvidenceState;
using world_model::LocalScene;

SourceStamp Stamp(double time = 1.0, uint64_t sequence = 1) {
  return {"odom", "simulation", {"session", 1}, sequence,
          time,   time,         time + 0.4,     InputHealth::HEALTHY};
}
void AddLane(uint64_t id, double start, double end, double y,
             LocalScene* scene) {
  world_model::LaneBoundary left, right;
  left.id = 2 * id;
  right.id = 2 * id + 1;
  for (double x = start; x <= end; x += 0.5) {
    left.points.push_back({x, y + 2.5});
    right.points.push_back({x, y - 2.5});
    left.observation_times.push_back(1.0);
    right.observation_times.push_back(1.0);
    left.valid_until.push_back(2.0);
    right.valid_until.push_back(2.0);
  }
  scene->boundaries.push_back(left);
  scene->boundaries.push_back(right);
  scene->lanes.push_back(
      {id, left.id, right.id, EvidenceState::CONFIRMED, 0.02});
}
LocalScene Scene() {
  LocalScene scene;
  scene.source = {"odom",
                  "simulation",
                  "session",
                  1,
                  1,
                  1,
                  1,
                  1.4,
                  world_model::SceneHealth::HEALTHY};
  scene.mode = world_model::SceneMode::LANE;
  scene.reference = world_model::VehicleReference::REAR_AXLE;
  scene.capabilities.drivable_region = true;
  scene.capabilities.lane_follow = true;
  AddLane(1, -8, 45, 0, &scene);
  scene.environment.source = scene.source;
  scene.environment.position_error = 0.02;
  scene.environment.free_space = {{-10, -4}, {50, -4}, {50, 4}, {-10, 4}};
  return scene;
}
CycleInput Input() {
  CycleInput input;
  input.planning_time = 1.0;
  input.odometry.stamp = Stamp();
  input.graph = Stamp();
  input.prediction = {Stamp(), 1};
  input.occupancy.corridor_fully_observed = true;
  input.occupancy.drivable_region_fully_observed = true;
  input.occupancy.horizon = 6.2;
  EXPECT_TRUE(
      SelectCorridor(Scene(), input.odometry, 1.0, &input.corridor).ok());
  input.environment = Scene().environment;
  return input;
}

TEST(CoreFrameworkTest, PlannerRejectsMixedEntryFrontiersWithinEpoch) {
  const auto config = SimulationPlannerConfig();
  auto direct = Input();
  LaneFollowPlanner planner(SimulationInputPolicy(), config);
  ASSERT_TRUE(planner.BeginEpoch(direct.odometry, 1.0).ok());
  ASSERT_TRUE(planner.Plan(direct).ok());
  auto scene = Scene();
  EXPECT_FALSE(planner
                   .PlanScene(scene, direct.odometry, direct.prediction,
                              direct.occupancy, 1.0)
                   .ok());
  EXPECT_FALSE(planner.trajectory().valid);
  EXPECT_EQ(nullptr, planner.planning_geometry());
  EXPECT_NE(std::string::npos,
            planner.invalid_reason().find("cannot be mixed"));

  LaneFollowPlanner other(SimulationInputPolicy(), config);
  ASSERT_TRUE(other.BeginEpoch(direct.odometry, 1.0).ok());
  ASSERT_TRUE(other
                  .PlanScene(scene, direct.odometry, direct.prediction,
                             direct.occupancy, 1.0)
                  .ok());
  EXPECT_FALSE(other.Plan(direct).ok());
  EXPECT_FALSE(other.trajectory().valid);
  EXPECT_EQ(nullptr, other.planning_geometry());
}

world_model::SceneSource WorldSource(double t, uint64_t sequence,
                                     const std::string& frame) {
  return {frame, "simulation", "session",
          1,     sequence,     t,
          t,     t + 0.5,      world_model::SceneHealth::HEALTHY};
}

world_model::LaneObservation ObservedLane(double t, uint64_t sequence) {
  world_model::LaneObservation lane;
  lane.session = "session";
  lane.generation = 1;
  lane.sequence = sequence;
  lane.measurement_time = t;
  lane.position_error = 0.02;
  lane.forward_confirmed = true;
  for (double x = -8; x <= 25; x += 0.5) {
    lane.left.push_back({x, 2.5});
    lane.right.push_back({x, -2.5});
  }
  return lane;
}

world_model::EnvironmentObservation ObservedEnvironment(double t,
                                                        uint64_t sequence) {
  world_model::EnvironmentObservation environment;
  environment.session = "session";
  environment.generation = 1;
  environment.sequence = sequence;
  environment.measurement_time = t;
  environment.position_error = 0.02;
  environment.free_space = {{-10, -4}, {30, -4}, {30, 4}, {-10, 4}};
  environment.curbs = {{1, {{-10, -3}, {30, -3}}}, {2, {{-10, 3}, {30, 3}}}};
  return environment;
}

TEST(CoreFrameworkTest, SelectsOnlyPermittedUnambiguousSuccessor) {
  auto scene = Scene();
  scene.boundaries.clear();
  scene.lanes.clear();
  AddLane(1, -8, 10, 0, &scene);
  AddLane(2, 10, 45, 0, &scene);
  scene.edges.push_back({1, 2, world_model::EdgeType::SUCCESSOR,
                         EvidenceState::CONFIRMED, EvidenceState::UNKNOWN});
  auto input = Input();
  LocalCorridor corridor;
  ASSERT_TRUE(SelectCorridor(scene, input.odometry, 1, &corridor).ok());
  EXPECT_DOUBLE_EQ(10, corridor.centerline.back().x);
  scene.edges[0].permission = EvidenceState::CONFIRMED;
  ASSERT_TRUE(SelectCorridor(scene, input.odometry, 1, &corridor).ok());
  EXPECT_DOUBLE_EQ(45, corridor.centerline.back().x);
  EXPECT_EQ(1U, corridor.lane_id);
  scene.edges.push_back({1, 2, world_model::EdgeType::SPLIT,
                         EvidenceState::HYPOTHESIS, EvidenceState::UNKNOWN});
  ASSERT_TRUE(SelectCorridor(scene, input.odometry, 1, &corridor).ok());
  EXPECT_DOUBLE_EQ(10, corridor.centerline.back().x);
  scene.edges[0].to = 999;
  EXPECT_FALSE(SelectCorridor(scene, input.odometry, 1, &corridor).ok());
  EXPECT_TRUE(corridor.centerline.empty());
}

TEST(CoreFrameworkTest, NavigationSelectsOnlyObservedPermittedSuccessor) {
  auto scene = Scene();
  scene.boundaries.clear();
  scene.lanes.clear();
  AddLane(1, -8, 10, 0, &scene);
  AddLane(2, 10, 45, 0, &scene);
  AddLane(3, 10, 45, 0, &scene);
  scene.edges = {{1, 2, world_model::EdgeType::SUCCESSOR,
                  EvidenceState::CONFIRMED, EvidenceState::CONFIRMED},
                 {1, 3, world_model::EdgeType::SUCCESSOR,
                  EvidenceState::CONFIRMED, EvidenceState::CONFIRMED}};
  const auto ego = Input().odometry;
  LocalCorridor corridor;
  ASSERT_TRUE(SelectCorridor(scene, ego, 1, &corridor).ok());
  EXPECT_DOUBLE_EQ(10, corridor.centerline.back().x);
  world_model::NavigationPrior prior{scene.source, 1, 3};
  const auto environment = scene.environment;
  ASSERT_TRUE(world_model::FuseNavigationPrior(prior, 1, &scene).ok());
  EXPECT_EQ(environment, scene.environment);
  ASSERT_TRUE(SelectCorridor(scene, ego, 1, &corridor).ok());
  EXPECT_DOUBLE_EQ(45, corridor.centerline.back().x);
  scene.edges[1].permission = EvidenceState::UNKNOWN;
  ASSERT_TRUE(SelectCorridor(scene, ego, 1, &corridor).ok());
  EXPECT_DOUBLE_EQ(10, corridor.centerline.back().x);
  scene.edges[1].permission = EvidenceState::CONFIRMED;
  scene.edges[1].connection = EvidenceState::HYPOTHESIS;
  ASSERT_TRUE(SelectCorridor(scene, ego, 1, &corridor).ok());
  EXPECT_DOUBLE_EQ(10, corridor.centerline.back().x);
  scene.edges[1].connection = EvidenceState::CONFIRMED;
  scene.edges.push_back(scene.edges[1]);
  scene.navigation.reset();
  EXPECT_FALSE(world_model::FuseNavigationPrior(prior, 1, &scene).ok());
  scene.edges.pop_back();
  prior.to_lane = 999;
  EXPECT_FALSE(world_model::FuseNavigationPrior(prior, 1, &scene).ok());
  prior.to_lane = 3;
  prior.source.generation = 2;
  EXPECT_FALSE(world_model::FuseNavigationPrior(prior, 1, &scene).ok());
  prior.source.generation = 1;
  prior.source.valid_until = 1.01;
  ASSERT_TRUE(world_model::FuseNavigationPrior(prior, 1, &scene).ok());
  EXPECT_FALSE(SelectCorridor(scene, ego, 1.01, &corridor).ok());
}

TEST(CoreFrameworkTest,
     FreeSpaceCurbsAndStaticObstaclesRestrictContinuousVehicleSweep) {
  const auto config = SimulationPlannerConfig();
  const VehicleEnvelope vehicle{config.front_extent, config.rear_extent,
                                config.half_width, config.clearance};
  auto environment = Scene().environment;
  const EnvironmentPose at_origin{0, 0, 0};
  EXPECT_TRUE(
      CheckEnvironmentSweep(environment, vehicle, at_origin, at_origin).ok());

  auto unknown = environment;
  unknown.free_space.clear();
  EXPECT_FALSE(ValidateEnvironment(unknown, Scene().source, 1).ok());

  auto narrow = environment;
  narrow.free_space = {{-10, -1.5}, {50, -1.5}, {50, 1.5}, {-10, 1.5}};
  EXPECT_FALSE(
      CheckEnvironmentSweep(narrow, vehicle, at_origin, at_origin).ok());

  auto curb = environment;
  curb.curbs = {{1, {{0.5, -3}, {0.5, 3}}}};
  EXPECT_FALSE(
      CheckEnvironmentSweep(curb, vehicle, {-1, 0, 0}, {2, 0, 0}).ok());

  auto obstacle = environment;
  obstacle.obstacles = {
      {1, {{0.49, -0.1}, {0.51, -0.1}, {0.51, 0.1}, {0.49, 0.1}}}};
  EXPECT_FALSE(
      CheckEnvironmentSweep(obstacle, vehicle, {0, 0, 0}, {1, 0, 0}).ok());

  obstacle.curbs = curb.curbs;
  EXPECT_FALSE(
      CheckEnvironmentSweep(obstacle, vehicle, {-1, 0, 0}, {2, 0, 0}).ok());
}

TEST(CoreFrameworkTest, ReferenceExtractsSignedProjectionAndCurrentBoundaries) {
  auto scene = Scene();
  auto ego = Input().odometry;
  ego.y = 0.2;
  ReferenceGeometry first, outward, narrow;
  ASSERT_TRUE(
      BuildReferenceGeometry(scene, ego, 1, 10, 12, 0.1, nullptr, &first).ok());
  EXPECT_NEAR(8, first.ego_s, 1e-9);
  EXPECT_NEAR(0.2, first.ego_l, 1e-9);
  EXPECT_DOUBLE_EQ(0, first.start_s);
  EXPECT_DOUBLE_EQ(20, first.end_s);
  EXPECT_EQ(first.line.size(), first.left.size());
  EXPECT_EQ(first.line.size(), first.right.size());
  EXPECT_EQ(first.line.size(), first.stations.size());
  EXPECT_DOUBLE_EQ(-8, first.line.front().x);
  EXPECT_DOUBLE_EQ(12, first.line.back().x);
  scene.source.sequence++;
  scene.source.publication_time = 1.1;
  for (auto& point : scene.boundaries[0].points) point.y += 0.06;
  ASSERT_TRUE(
      BuildReferenceGeometry(scene, ego, 1.1, 10, 12, 0.1, &first, &outward)
          .ok());
  EXPECT_TRUE(outward.continuous);
  EXPECT_NEAR(0, outward.corridor.centerline[20].y, 1e-9);
  EXPECT_NEAR(2.56, outward.corridor.left_boundary[20].y, 1e-9);
  EXPECT_LT(outward.corridor.half_width, outward.observed_half_width);
  auto moved_ego = ego;
  moved_ego.x = 1;
  moved_ego.stamp = Stamp(1.1, 2);
  ReferenceGeometry delayed;
  ASSERT_TRUE(BuildReferenceGeometry(scene, moved_ego, 1.1, 10, 12, 0.1,
                                     &outward, &delayed)
                  .ok());
  EXPECT_TRUE(delayed.continuous);
  // The graph is unchanged: retain its accepted reference, including the
  // endpoint segment, rather than repeatedly projecting interior points.
  const double expected_station = std::hypot(0.5, 0.03) + 8.5;
  EXPECT_NEAR(expected_station, delayed.ego_s, 1e-12);
  EXPECT_NEAR(-7.5, delayed.corridor.centerline[1].x, 1e-12);
  EXPECT_NEAR(0.0, delayed.corridor.centerline[1].y, 1e-12);
  EXPECT_EQ(outward.corridor.centerline, delayed.corridor.centerline);
  EXPECT_DOUBLE_EQ(outward.corridor.half_width, delayed.corridor.half_width);
  EXPECT_GT(delayed.continuity.reused_points, 0U);
  EXPECT_EQ(ContinuityReason::REUSED_OVERLAP, delayed.continuity.reason);
  EXPECT_NEAR(0.03, delayed.corridor.centerline.front().y, 1e-12);
  EXPECT_NEAR(2.56, delayed.corridor.left_boundary.front().y, 1e-12);
  ReferenceGeometry repeated;
  ASSERT_TRUE(BuildReferenceGeometry(scene, moved_ego, 1.1, 10, 12, 0.1,
                                     &delayed, &repeated)
                  .ok());
  EXPECT_NEAR(delayed.corridor.centerline[1].x,
              repeated.corridor.centerline[1].x, 1e-12);
  EXPECT_NEAR(delayed.corridor.centerline[1].y,
              repeated.corridor.centerline[1].y, 1e-12);
  for (size_t i = 1; i < delayed.corridor.centerline.size(); ++i) {
    const auto& previous = delayed.corridor.centerline[i - 1];
    const auto& current = delayed.corridor.centerline[i];
    EXPECT_GT(std::hypot(current.x - previous.x, current.y - previous.y), 0.4);
  }
  EXPECT_NEAR(1.1, delayed.source.publication_time, 1e-9);
  for (auto& point : scene.boundaries[0].points) point.y -= 0.12;
  scene.source.sequence++;
  ASSERT_TRUE(
      BuildReferenceGeometry(scene, ego, 1.1, 10, 12, 0.1, &outward, &narrow)
          .ok());
  EXPECT_FALSE(narrow.continuous);
  EXPECT_NEAR(-0.03, narrow.corridor.centerline[20].y, 1e-9);
  EXPECT_NEAR(2.44, narrow.corridor.left_boundary[20].y, 1e-9);
  scene.environment.free_space[0].x += 0.1;
  ReferenceGeometry restricted;
  ASSERT_TRUE(
      BuildReferenceGeometry(scene, ego, 1.1, 10, 12, 0.1, &narrow, &restricted)
          .ok());
  EXPECT_FALSE(restricted.continuous);
  scene.source.valid_until = 1.1;
  EXPECT_FALSE(
      BuildReferenceGeometry(scene, ego, 1.1, 10, 12, 0.1, &narrow, &first)
          .ok());
  scene.source.valid_until = 1.4;
  scene.source.generation++;
  ego.stamp.epoch.generation++;
  scene.source.sequence++;
  ASSERT_TRUE(
      BuildReferenceGeometry(scene, ego, 1.1, 10, 12, 0.1, &narrow, &first)
          .ok());
  EXPECT_FALSE(first.continuous);
}

TEST(CoreFrameworkTest, PlanScenePublishesOnlyAcceptedReference) {
  auto input = Input();
  auto scene = Scene();
  LaneFollowPlanner planner(SimulationInputPolicy(), SimulationPlannerConfig());
  ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1).ok());
  ASSERT_TRUE(planner
                  .PlanScene(scene, input.odometry, input.prediction,
                             input.occupancy, 1)
                  .ok());
  ASSERT_NE(nullptr, planner.reference_geometry());
  EXPECT_EQ(scene.source.sequence,
            planner.reference_geometry()->source.sequence);
  scene.capabilities.lane_follow = false;
  EXPECT_FALSE(planner
                   .PlanScene(scene, input.odometry, input.prediction,
                              input.occupancy, 1.1)
                   .ok());
  EXPECT_EQ(nullptr, planner.reference_geometry());
}

TEST(CoreFrameworkTest,
     AreaGeometryRemainsUsableWhileLaneFollowStopsAndRecoveryIsFresh) {
  auto input = Input();
  auto lane_scene = Scene();
  LaneFollowPlanner planner(SimulationInputPolicy(), SimulationPlannerConfig());
  ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1).ok());
  ASSERT_TRUE(planner
                  .PlanScene(lane_scene, input.odometry, input.prediction,
                             input.occupancy, 1)
                  .ok());
  ASSERT_NE(nullptr, planner.planning_geometry());
  EXPECT_EQ(world_model::SceneMode::LANE, planner.planning_geometry()->mode);

  auto area_scene = lane_scene;
  area_scene.source.sequence = 2;
  area_scene.source.measurement_time = 1.1;
  area_scene.source.publication_time = 1.1;
  area_scene.source.valid_until = 1.5;
  area_scene.mode = world_model::SceneMode::AREA;
  area_scene.capabilities.lane_follow = false;
  area_scene.boundaries.clear();
  area_scene.lanes.clear();
  area_scene.edges.clear();
  area_scene.rules.clear();
  area_scene.navigation.reset();
  area_scene.mode_reason = "fresh observation reports no usable lane";
  area_scene.environment.source = area_scene.source;
  input.odometry.stamp = Stamp(1.1, 2);
  input.prediction = {Stamp(1.1, 2), 2};
  PlanningGeometry area_geometry;
  ASSERT_TRUE(BuildPlanningGeometry(area_scene, input.odometry, 1.1, 10, 12,
                                    0.1, nullptr, &area_geometry)
                  .ok());
  EXPECT_EQ(world_model::SceneMode::AREA, area_geometry.mode);
  EXPECT_FALSE(area_geometry.reference.has_value());
  EXPECT_FALSE(area_geometry.environment.free_space.empty());
  EXPECT_FALSE(planner
                   .PlanScene(area_scene, input.odometry, input.prediction,
                              input.occupancy, 1.1)
                   .ok());
  ASSERT_NE(nullptr, planner.planning_geometry());
  EXPECT_EQ(world_model::SceneMode::AREA, planner.planning_geometry()->mode);
  EXPECT_EQ(nullptr, planner.reference_geometry());
  EXPECT_FALSE(planner.trajectory().valid);

  lane_scene.source.sequence = 3;
  lane_scene.source.measurement_time = 1.2;
  lane_scene.source.publication_time = 1.2;
  lane_scene.source.valid_until = 1.6;
  lane_scene.environment.source = lane_scene.source;
  input.odometry.stamp = Stamp(1.2, 3);
  input.prediction = {Stamp(1.2, 3), 3};
  ASSERT_TRUE(planner
                  .PlanScene(lane_scene, input.odometry, input.prediction,
                             input.occupancy, 1.2)
                  .ok());
  ASSERT_NE(nullptr, planner.reference_geometry());
  EXPECT_FALSE(planner.reference_geometry()->continuous);
  EXPECT_FALSE(planner.trajectory().stitched);
}

TEST(CoreFrameworkTest, ProducerPublishesUsableAreaAndLaneGeometryModes) {
  const world_model::TemporalLanePolicy policy{0.5, 2.0, 0.2, 0.6, 1.0, 0.22,
                                               0.3, 2.0, 3.0, 6.0, 0.1, 0.3};
  world_model::LocalSceneBuilder builder(policy, "odom", "base_link",
                                         "simulation");
  ASSERT_TRUE(
      builder.BeginEpoch(WorldSource(1, 1, "odom"), {1, 0, 0, 0, 0.01}, 1, true)
          .ok());
  ASSERT_TRUE(builder
                  .ObserveEnvironment(WorldSource(1, 1, "base_link"),
                                      ObservedEnvironment(1, 1), 1)
                  .ok());
  auto ego = Input().odometry;
  world_model::LocalScene scene;
  PlanningGeometry geometry;
  ASSERT_TRUE(builder.Build(1, &scene).ok());
  ASSERT_EQ(world_model::SceneMode::AREA, scene.mode);
  ASSERT_TRUE(
      BuildPlanningGeometry(scene, ego, 1, 10, 12, 0.1, nullptr, &geometry)
          .ok());
  EXPECT_FALSE(geometry.reference.has_value());

  ASSERT_TRUE(
      builder
          .AddOdometry(WorldSource(1.1, 2, "odom"), {1.1, 0, 0, 0, 0.01}, 1.1)
          .ok());
  ASSERT_TRUE(
      builder
          .Observe(WorldSource(1.1, 2, "base_link"), ObservedLane(1.1, 2), 1.1)
          .ok());
  ASSERT_TRUE(builder
                  .ObserveEnvironment(WorldSource(1.1, 2, "base_link"),
                                      ObservedEnvironment(1.1, 2), 1.1)
                  .ok());
  ego.stamp = Stamp(1.1, 2);
  ASSERT_TRUE(builder.Build(1.1, &scene).ok());
  ASSERT_EQ(world_model::SceneMode::LANE, scene.mode);
  ASSERT_TRUE(
      BuildPlanningGeometry(scene, ego, 1.1, 10, 12, 0.1, nullptr, &geometry)
          .ok());
  EXPECT_TRUE(geometry.reference.has_value());

  ASSERT_TRUE(
      builder
          .AddOdometry(WorldSource(1.2, 3, "odom"), {1.2, 0, 0, 0, 0.01}, 1.2)
          .ok());
  ASSERT_TRUE(
      builder.ObserveLaneAbsence(WorldSource(1.2, 3, "base_link"), 1.2).ok());
  ASSERT_TRUE(builder
                  .ObserveEnvironment(WorldSource(1.2, 3, "base_link"),
                                      ObservedEnvironment(1.2, 3), 1.2)
                  .ok());
  ego.stamp = Stamp(1.2, 3);
  ASSERT_TRUE(builder.Build(1.2, &scene).ok());
  ASSERT_EQ(world_model::SceneMode::AREA, scene.mode);
  ASSERT_TRUE(
      BuildPlanningGeometry(scene, ego, 1.2, 10, 12, 0.1, nullptr, &geometry)
          .ok());
  EXPECT_FALSE(geometry.reference.has_value());

  auto invalid_odom = WorldSource(1.3, 4, "odom");
  invalid_odom.health = world_model::SceneHealth::INVALID;
  EXPECT_FALSE(
      builder.AddOdometry(invalid_odom, {1.3, 0, 0, 0, 0.01}, 1.3).ok());
  EXPECT_FALSE(builder.Build(1.3, &scene).ok());
  EXPECT_EQ(world_model::SceneMode::INVALID, scene.mode);
}

TEST(CoreFrameworkTest, RejectsAmbiguousIdentityGeometryAndProvenance) {
  const std::vector<std::function<void(LocalScene*)>> mutations = {
      [](LocalScene* s) { s->source.frame_id = "map"; },
      [](LocalScene* s) { s->source.clock_id = "other"; },
      [](LocalScene* s) { ++s->source.generation; },
      [](LocalScene* s) {
        s->source.health = world_model::SceneHealth::UNKNOWN;
      },
      [](LocalScene* s) {
        s->reference = world_model::VehicleReference::UNKNOWN;
      },
      [](LocalScene* s) { s->capabilities.lane_follow = false; },
      [](LocalScene* s) { s->boundaries[1].id = s->boundaries[0].id; },
      [](LocalScene* s) { s->lanes[0].left_boundary = 999; },
      [](LocalScene* s) { s->boundaries[0].observation_times[0] = 2; },
      [](LocalScene* s) { s->boundaries[0].valid_until[0] = 1.1; },
      [](LocalScene* s) {
        s->boundaries[0].points[0].x = std::numeric_limits<double>::quiet_NaN();
      },
      [](LocalScene* s) { AddLane(2, -8, 45, 0.1, s); }};
  for (size_t i = 0; i < mutations.size(); ++i) {
    SCOPED_TRACE(i);
    auto scene = Scene();
    mutations[i](&scene);
    LocalCorridor corridor;
    EXPECT_FALSE(SelectCorridor(scene, Input().odometry, 1, &corridor).ok());
    EXPECT_TRUE(corridor.centerline.empty());
  }
}

TEST(CoreFrameworkTest,
     RulesConstrainSpeedAndStoppingWithoutAssumedPermission) {
  auto scene = Scene();
  scene.rules = {
      {1, 1, world_model::RuleType::SPEED_LIMIT, EvidenceState::CONFIRMED, 0,
       0.7},
      {2, 1, world_model::RuleType::STOP, EvidenceState::HYPOTHESIS, 25, 0}};
  auto input = Input();
  LaneFollowPlanner planner(SimulationInputPolicy(), SimulationPlannerConfig());
  ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1).ok());
  auto status = planner.PlanScene(scene, input.odometry, input.prediction,
                                  input.occupancy, 1);
  ASSERT_TRUE(status.ok()) << status.ToString();
  EXPECT_EQ(StopReason::OBSERVED_RULE, planner.trajectory().stop_reason);
  for (const auto& p : planner.trajectory().points) {
    EXPECT_LE(p.speed, 0.7 + 1e-6);
    EXPECT_LT(p.x + 3.8, 17.0);
  }
  scene.rules[0].evidence = EvidenceState::UNKNOWN;
  EXPECT_FALSE(planner
                   .PlanScene(scene, input.odometry, input.prediction,
                              input.occupancy, 1.1)
                   .ok());
  EXPECT_EQ(BehaviorState::INVALID, planner.behavior());
  EXPECT_FALSE(planner.trajectory().valid);
  EXPECT_FALSE(planner.invalid_reason().empty());
}

TEST(CoreFrameworkTest, BehaviorLifecycleClearsHistoryAndRecoversExplicitly) {
  auto scene = Scene();
  auto input = Input();
  LaneFollowPlanner planner(SimulationInputPolicy(), SimulationPlannerConfig());
  ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1).ok());
  EXPECT_EQ(BehaviorState::IDLE, planner.behavior());
  ASSERT_TRUE(planner
                  .PlanScene(scene, input.odometry, input.prediction,
                             input.occupancy, 1)
                  .ok());
  EXPECT_EQ(BehaviorState::FOLLOWING, planner.behavior());
  scene.capabilities.lane_follow = false;
  EXPECT_FALSE(planner
                   .PlanScene(scene, input.odometry, input.prediction,
                              input.occupancy, 1.1)
                   .ok());
  EXPECT_EQ(BehaviorState::INVALID, planner.behavior());
  scene.capabilities.lane_follow = true;
  scene.source.sequence = 2;
  scene.source.publication_time = 1.2;
  scene.source.valid_until = 1.6;
  scene.environment.source = scene.source;
  input.odometry.stamp = Stamp(1.2, 2);
  input.prediction = {Stamp(1.2, 2), 2};
  ASSERT_TRUE(planner
                  .PlanScene(scene, input.odometry, input.prediction,
                             input.occupancy, 1.2)
                  .ok());
  EXPECT_FALSE(planner.trajectory().stitched);
  EXPECT_EQ(BehaviorState::FOLLOWING, planner.behavior());
  input.odometry.x = 39.0;
  input.odometry.stamp = Stamp(1.3, 3);
  input.prediction = {Stamp(1.3, 3), 3};
  scene.source.sequence = 3;
  scene.source.publication_time = 1.3;
  scene.environment.source = scene.source;
  scene.rules = {{1, 1, world_model::RuleType::SPEED_LIMIT,
                  EvidenceState::CONFIRMED, 0, 0}};
  LaneFollowPlanner stopped(SimulationInputPolicy(), SimulationPlannerConfig());
  ASSERT_TRUE(stopped.BeginEpoch(input.odometry, 1.3).ok());
  ASSERT_TRUE(stopped
                  .PlanScene(scene, input.odometry, input.prediction,
                             input.occupancy, 1.3)
                  .ok());
  EXPECT_EQ(BehaviorState::STOPPED, stopped.behavior());
}

TEST(CoreFrameworkTest, IndependentValidatorRejectsForgedGeneratorOutput) {
  auto input = Input();
  const auto config = SimulationPlannerConfig();
  LaneFollowPlanner planner(SimulationInputPolicy(), config);
  ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1).ok());
  ASSERT_TRUE(planner.Plan(input).ok());
  const auto good = planner.trajectory();
  ASSERT_TRUE(ValidateTrajectory(input, config, good).ok());
  const std::vector<std::function<void(LocalTrajectory*)>> mutations = {
      [](LocalTrajectory* t) { t->points[1].time = 0; },
      [](LocalTrajectory* t) { t->points[1].speed = -0.1; },
      [](LocalTrajectory* t) { t->points[1].acceleration = 4; },
      [](LocalTrajectory* t) { t->points[1].curvature = 1; },
      [](LocalTrajectory* t) { t->points[1].x += 0.5; },
      [](LocalTrajectory* t) { t->points[1].heading += 0.5; },
      [](LocalTrajectory* t) { t->points.back().time -= 0.1; },
      [](LocalTrajectory* t) { t->stamp.valid_until += 1; },
      [](LocalTrajectory* t) { ++t->stamp.epoch.generation; },
      [](LocalTrajectory* t) { ++t->graph_sequence; },
      [](LocalTrajectory* t) {
        t->points[1].y = std::numeric_limits<double>::quiet_NaN();
      }};
  for (size_t i = 0; i < mutations.size(); ++i) {
    SCOPED_TRACE(i);
    auto bad = good;
    mutations[i](&bad);
    EXPECT_FALSE(ValidateTrajectory(input, config, bad).ok());
  }
  input.occupancy.obstacles.push_back({1, {0, 0}, {}, 1});
  EXPECT_FALSE(ValidateTrajectory(input, config, good).ok());
}

TEST(CoreFrameworkTest, SameGraphVersionCannotMutateUnusedWorldEvidence) {
  auto input = Input();
  auto scene = Scene();
  LaneFollowPlanner planner(SimulationInputPolicy(), SimulationPlannerConfig());
  ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1).ok());
  ASSERT_TRUE(planner
                  .PlanScene(scene, input.odometry, input.prediction,
                             input.occupancy, 1)
                  .ok());
  input.odometry.stamp = Stamp(1.1, 2);
  input.prediction = {Stamp(1.1, 2), 1};
  scene.boundaries[0].crossing = EvidenceState::CONFIRMED;
  const auto status = planner.PlanScene(scene, input.odometry, input.prediction,
                                        input.occupancy, 1.1);
  EXPECT_FALSE(status.ok());
  EXPECT_NE(std::string::npos, status.error_message().find("version mutated"));
  EXPECT_FALSE(planner.trajectory().valid);
  scene = Scene();
  scene.environment.free_space[0].y += 0.1;
  input.odometry.stamp = Stamp(1.2, 3);
  input.prediction = {Stamp(1.2, 3), 1};
  const auto environment_status = planner.PlanScene(
      scene, input.odometry, input.prediction, input.occupancy, 1.2);
  EXPECT_FALSE(environment_status.ok());
  EXPECT_NE(std::string::npos,
            environment_status.error_message().find("version mutated"));
}

TEST(CoreFrameworkTest, RejectedFutureSceneDoesNotReplaceAcceptedVersion) {
  auto input = Input();
  auto scene = Scene();
  LaneFollowPlanner planner(SimulationInputPolicy(), SimulationPlannerConfig());
  ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1).ok());
  ASSERT_TRUE(planner
                  .PlanScene(scene, input.odometry, input.prediction,
                             input.occupancy, 1)
                  .ok());
  auto rejected = scene;
  rejected.source.sequence = 2;
  input.odometry.stamp = Stamp(1.2, 2);
  input.prediction = {Stamp(1.2, 2), 2};
  // The scene itself passes selection, but the cycle has future ODOM.
  EXPECT_FALSE(planner
                   .PlanScene(rejected, input.odometry, input.prediction,
                              input.occupancy, 1.1)
                   .ok());
  input.odometry.stamp = Stamp(1.3, 3);
  input.prediction = {Stamp(1.3, 3), 1};
  scene.boundaries[0].crossing = EvidenceState::CONFIRMED;
  const auto status = planner.PlanScene(scene, input.odometry, input.prediction,
                                        input.occupancy, 1.3);
  EXPECT_FALSE(status.ok());
  EXPECT_NE(std::string::npos, status.error_message().find("version mutated"));
}

TEST(CoreFrameworkTest, IndependentValidatorChecksActualNotOnlyStitchedState) {
  auto input = Input();
  const auto config = SimulationPlannerConfig();
  LaneFollowPlanner planner(SimulationInputPolicy(), config);
  ASSERT_TRUE(planner.BeginEpoch(input.odometry, 1).ok());
  input.odometry.stamp = Stamp(1.1, 2);
  input.planning_time = 1.1;
  input.odometry.x = 39.58;
  ASSERT_TRUE(planner.Plan(input).ok());
  const auto good = planner.trajectory();
  input.odometry.speed_mps = 0.24;
  EXPECT_FALSE(ValidateTrajectory(input, config, good).ok());
}
}  // namespace
}  // namespace local_planning
}  // namespace apollo
