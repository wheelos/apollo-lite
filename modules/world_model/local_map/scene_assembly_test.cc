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

#include "modules/world_model/local_map/scene_assembly.h"

#include "gtest/gtest.h"

namespace apollo {
namespace world_model {
namespace {

SceneSource Source(uint64_t sequence, double measurement_time) {
  return {"odom", "clock", "session",           1, sequence, measurement_time,
          1.0,    2.0,     SceneHealth::HEALTHY};
}

DrivableEnvironment Environment() {
  DrivableEnvironment environment;
  environment.source = Source(3, 0.8);
  environment.free_space = {{-5, -3}, {20, -3}, {20, 3}, {-5, 3}};
  return environment;
}

LaneLayer LaneEvidence() {
  LaneLayer layer;
  layer.source = Source(2, 0.5);
  LaneBoundary left, right;
  left.id = 11;
  right.id = 12;
  for (double x : {-5.0, 0.0, 5.0}) {
    left.points.push_back({x, 2.0});
    right.points.push_back({x, -2.0});
    left.observation_times.push_back(0.5);
    right.observation_times.push_back(0.5);
    left.valid_until.push_back(2.0);
    right.valid_until.push_back(2.0);
  }
  layer.boundaries = {left, right};
  layer.lanes = {{7, left.id, right.id, EvidenceState::CONFIRMED, 0.02}};
  return layer;
}

TEST(SceneAssemblyTest, CombinesIndependentLayersWithoutChangingEvidence) {
  const auto source = Source(4, 0.8);
  const auto environment = Environment();
  const auto lanes = LaneEvidence();
  LocalScene scene;

  ASSERT_TRUE(
      AssembleLocalScene(source, environment, &lanes, "", nullptr, 1.0, &scene)
          .ok());
  EXPECT_EQ(SceneMode::LANE, scene.mode);
  EXPECT_TRUE(scene.capabilities.drivable_region);
  EXPECT_TRUE(scene.capabilities.lane_follow);
  ASSERT_EQ(2U, scene.boundaries.size());
  EXPECT_DOUBLE_EQ(0.5, scene.boundaries.front().observation_times.front());
  EXPECT_EQ(environment.free_space, scene.environment.free_space);
  EXPECT_TRUE(scene.edges.empty());
  EXPECT_TRUE(scene.rules.empty());
}

TEST(SceneAssemblyTest, AreaModeHasNoFabricatedLaneAndRejectsDanglingRefs) {
  const auto source = Source(4, 0.8);
  const auto environment = Environment();
  LocalScene area;
  ASSERT_TRUE(AssembleLocalScene(source, environment, nullptr,
                                 "no observed lane", nullptr, 1.0, &area)
                  .ok());
  EXPECT_EQ(SceneMode::AREA, area.mode);
  EXPECT_EQ("no observed lane", area.mode_reason);
  EXPECT_TRUE(area.boundaries.empty());
  EXPECT_TRUE(area.lanes.empty());
  EXPECT_FALSE(area.capabilities.lane_follow);

  auto invalid_lanes = LaneEvidence();
  invalid_lanes.lanes.front().left_boundary = 99;
  EXPECT_FALSE(AssembleLocalScene(source, environment, &invalid_lanes, "",
                                  nullptr, 1.0, &area)
                   .ok());
  EXPECT_TRUE(area.boundaries.empty());
}

TEST(SceneAssemblyTest, RejectsMalformedIndependentEnvironmentLayer) {
  const auto source = Source(4, 0.8);
  auto environment = Environment();
  LocalScene scene;
  environment.free_space = {{-5, -3}, {20, 3}, {20, -3}, {-5, 3}};
  EXPECT_FALSE(
      AssembleLocalScene(source, environment, nullptr, "", nullptr, 1.0, &scene)
          .ok());
  EXPECT_EQ(SceneMode::INVALID, scene.mode);
  environment = Environment();
  environment.curbs.push_back({5, {{0, 0}, {0, 0}}});
  EXPECT_FALSE(
      AssembleLocalScene(source, environment, nullptr, "", nullptr, 1.0, &scene)
          .ok());
  environment = Environment();
  environment.obstacles.push_back({9, {{0, 0}, {1, 1}, {1, 0}, {0, 1}}});
  EXPECT_FALSE(
      AssembleLocalScene(source, environment, nullptr, "", nullptr, 1.0, &scene)
          .ok());
}

TEST(SceneAssemblyTest, RejectsMalformedIndependentLaneLayer) {
  const auto source = Source(4, 0.8);
  const auto environment = Environment();
  LocalScene scene;
  auto layer = LaneEvidence();
  layer.lanes.clear();
  EXPECT_FALSE(
      AssembleLocalScene(source, environment, &layer, "", nullptr, 1.0, &scene)
          .ok());
  layer = LaneEvidence();
  layer.boundaries[0].points[1].x = layer.boundaries[0].points[0].x;
  layer.boundaries[1].points[1].x = layer.boundaries[1].points[0].x;
  EXPECT_FALSE(
      AssembleLocalScene(source, environment, &layer, "", nullptr, 1.0, &scene)
          .ok());
  layer = LaneEvidence();
  layer.boundaries[0].valid_until[0] = 0.5;
  EXPECT_FALSE(
      AssembleLocalScene(source, environment, &layer, "", nullptr, 1.0, &scene)
          .ok());
  layer = LaneEvidence();
  layer.edges.push_back({7, 7, EdgeType::SUCCESSOR, EvidenceState::CONFIRMED,
                         EvidenceState::CONFIRMED});
  EXPECT_FALSE(
      AssembleLocalScene(source, environment, &layer, "", nullptr, 1.0, &scene)
          .ok());
}

TEST(SceneAssemblyTest, AreaExplainsWhyNavigationIsNotApplied) {
  const auto source = Source(4, 0.8);
  const auto environment = Environment();
  const NavigationPrior navigation{Source(1, 0.8), 7, 8};
  LocalScene scene;
  ASSERT_TRUE(AssembleLocalScene(source, environment, nullptr, "lane absent",
                                 &navigation, 1.0, &scene)
                  .ok());
  EXPECT_EQ(SceneMode::AREA, scene.mode);
  EXPECT_FALSE(scene.navigation.has_value());
  EXPECT_NE(std::string::npos,
            scene.mode_reason.find("navigation prior not applicable"));
}

}  // namespace
}  // namespace world_model
}  // namespace apollo
