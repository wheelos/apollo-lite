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

#include "modules/prediction/evaluator/vehicle/hivt_scene_feature_builder.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "modules/prediction/common/kml_map_based_test.h"

namespace apollo {
namespace prediction {

namespace {

std::unique_ptr<Obstacle> MakeObstacle(int id, double x, double y) {
  constexpr double kTimestamp = 10.0;
  constexpr double kTimeStep = 0.1;
  constexpr double kSpeed = 10.0;
  auto obstacle = std::make_unique<Obstacle>();
  for (int age = kHiVTHistoricalSteps - 1; age >= 0; --age) {
    Feature feature;
    feature.set_id(id);
    feature.set_timestamp(kTimestamp - age * kTimeStep);
    feature.set_type(apollo::perception::PerceptionObstacle::VEHICLE);
    feature.mutable_position()->set_x(x - age * kSpeed * kTimeStep);
    feature.mutable_position()->set_y(y);
    feature.set_velocity_heading(0.0);
    feature.set_speed(kSpeed);
    feature.set_length(4.0);
    feature.set_width(2.0);
    obstacle->InsertFeature(feature);
  }
  return obstacle;
}

}  // namespace

class HiVTSceneFeatureBuilderTest : public KMLMapBasedTest {};

TEST_F(HiVTSceneFeatureBuilderTest, BuildsSceneTensorsInEgoCoordinates) {
  auto ego = MakeObstacle(FLAGS_ego_vehicle_id, -458.941, -159.240);
  ego->mutable_latest_feature()->set_velocity_heading(0.8);
  auto target = MakeObstacle(1, -455.941, -159.240);
  std::vector<Obstacle*> actors{ego.get(), target.get()};

  HiVTSceneInput input;
  HiVTSceneFeatureBuilder builder;
  ASSERT_TRUE(builder.Build(ego.get(), actors, &input));

  EXPECT_EQ(input.actor_count, 2);
  EXPECT_EQ(input.actor_ids.size(), 2);
  EXPECT_EQ(input.actor_edge_count, 2);
  EXPECT_EQ(input.edge_index, (std::vector<int64_t>{0, 1, 1, 0}));
  EXPECT_EQ(input.x.size(), 2 * kHiVTHistoricalSteps * 2);
  EXPECT_EQ(input.positions.size(),
            2 * (kHiVTHistoricalSteps + kHiVTFutureSteps) * 2);
  EXPECT_EQ(input.padding_mask.size(),
            2 * (kHiVTHistoricalSteps + kHiVTFutureSteps));
  EXPECT_EQ(input.bos_mask.size(), 2 * kHiVTHistoricalSteps);
  EXPECT_NEAR(input.origin_x, -458.941, 1e-3);
  EXPECT_NEAR(input.origin_y, -159.240, 1e-3);

  const std::size_t target_latest_position =
      (kHiVTHistoricalSteps + kHiVTHistoricalSteps + kHiVTFutureSteps - 1) * 2;
  EXPECT_NEAR(input.positions[target_latest_position], 3.0, 1e-3);
  EXPECT_NEAR(input.positions[target_latest_position + 1], 0.0, 1e-3);
  EXPECT_NEAR(input.rotate_angles[1], 0.0, 1e-3);
  EXPECT_GT(input.lane_vector_count, 0);
  EXPECT_LE(input.lane_vector_count, kHiVTMaxLaneVectors);
  EXPECT_GT(input.lane_actor_edge_count, 0);
  EXPECT_EQ(input.lane_vectors.size(),
            static_cast<std::size_t>(input.lane_vector_count * 2));
  EXPECT_EQ(input.lane_actor_vectors.size(),
            static_cast<std::size_t>(input.lane_actor_edge_count * 2));
  EXPECT_EQ(input.lane_actor_index.size(),
            static_cast<std::size_t>(input.lane_actor_edge_count * 2));
}

TEST_F(HiVTSceneFeatureBuilderTest, RejectsScenesBeyondActorCapacity) {
  std::vector<Obstacle*> actors(kHiVTMaxActors + 1, nullptr);
  HiVTSceneInput input;
  HiVTSceneFeatureBuilder builder;
  EXPECT_FALSE(builder.Build(nullptr, actors, &input));
}

TEST_F(HiVTSceneFeatureBuilderTest, SelectsNearestTargetsWithinCapacity) {
  auto ego = MakeObstacle(FLAGS_ego_vehicle_id, 0.0, 0.0);
  std::vector<std::unique_ptr<Obstacle>> owned_targets;
  std::vector<Obstacle*> candidates;
  for (int id = 1; id <= kHiVTMaxActors; ++id) {
    owned_targets.push_back(MakeObstacle(id, id, 0.0));
    candidates.push_back(owned_targets.back().get());
  }

  const auto selected =
      HiVTSceneFeatureBuilder::SelectTargets(ego.get(), candidates);

  ASSERT_EQ(selected.size(), kHiVTMaxActors - 1);
  EXPECT_EQ(selected.front()->id(), 1);
  EXPECT_EQ(selected.back()->id(), kHiVTMaxActors - 1);
}

TEST_F(HiVTSceneFeatureBuilderTest, ExcludesTargetsWithoutHiVTHistory) {
  auto target = std::make_unique<Obstacle>();
  Feature feature;
  feature.set_id(1);
  feature.set_timestamp(10.0);
  feature.set_type(apollo::perception::PerceptionObstacle::VEHICLE);
  feature.mutable_position()->set_x(1.0);
  feature.mutable_position()->set_y(0.0);
  target->InsertFeature(feature);
  auto ego = MakeObstacle(FLAGS_ego_vehicle_id, 0.0, 0.0);

  const auto selected =
      HiVTSceneFeatureBuilder::SelectTargets(ego.get(), {target.get()});

  EXPECT_TRUE(selected.empty());
}

TEST_F(HiVTSceneFeatureBuilderTest, ReservesActorCapacityForTargets) {
  auto ego = MakeObstacle(FLAGS_ego_vehicle_id, 0.0, 0.0);
  auto first_target = MakeObstacle(1, 10.0, 0.0);
  auto second_target = MakeObstacle(2, 20.0, 0.0);
  std::vector<Obstacle*> targets{first_target.get(), second_target.get()};
  std::vector<std::unique_ptr<Obstacle>> owned_context;
  std::vector<Obstacle*> candidates;
  for (int id = 3; id <= kHiVTMaxActors + 6; ++id) {
    const double x = id == 3 ? 10.5 : (id == 4 ? 19.5 : 1000.0 + id);
    owned_context.push_back(MakeObstacle(id, x, 0.0));
    candidates.push_back(owned_context.back().get());
  }

  const auto selected =
      HiVTSceneFeatureBuilder::SelectActors(ego.get(), targets, candidates);

  ASSERT_EQ(selected.size(), kHiVTMaxActors);
  EXPECT_NE(std::find_if(selected.begin(), selected.end(),
                         [](const Obstacle* actor) {
                           return actor->id() == FLAGS_ego_vehicle_id;
                         }),
            selected.end());
  EXPECT_NE(
      std::find_if(selected.begin(), selected.end(),
                   [](const Obstacle* actor) { return actor->id() == 1; }),
      selected.end());
  EXPECT_NE(
      std::find_if(selected.begin(), selected.end(),
                   [](const Obstacle* actor) { return actor->id() == 2; }),
      selected.end());
  EXPECT_NE(
      std::find_if(selected.begin(), selected.end(),
                   [](const Obstacle* actor) { return actor->id() == 3; }),
      selected.end());
  EXPECT_NE(
      std::find_if(selected.begin(), selected.end(),
                   [](const Obstacle* actor) { return actor->id() == 4; }),
      selected.end());
  EXPECT_EQ(std::find_if(selected.begin(), selected.end(),
                         [](const Obstacle* actor) {
                           return actor->id() == kHiVTMaxActors + 6;
                         }),
            selected.end());
}

}  // namespace prediction
}  // namespace apollo
