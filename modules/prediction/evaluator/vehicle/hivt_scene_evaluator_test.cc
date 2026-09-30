// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/prediction/evaluator/vehicle/hivt_scene_evaluator.h"

#include <cmath>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace prediction {

TEST(HiVTSceneEvaluatorTest, DecodesActorLocalDisplacementToWorldCoordinates) {
  Obstacle target;
  Feature feature;
  feature.set_id(42);
  feature.set_timestamp(10.0);
  feature.mutable_position()->set_x(100.0);
  feature.mutable_position()->set_y(200.0);
  feature.set_velocity_heading(0.0);
  feature.set_speed(5.0);
  target.InsertFeature(feature);

  HiVTSceneInput input;
  input.actor_count = 1;
  input.actor_ids = {42};
  input.origin_x = 1000.0;
  input.origin_y = 2000.0;
  input.origin_cos = 0.0;
  input.origin_sin = 1.0;
  input.rotate_angles = {static_cast<float>(M_PI_2)};

  HiVTSceneOutput output;
  output.trajectories.assign(kHiVTNumModes * kHiVTFutureSteps * 4, 0.0F);
  output.logits.assign(kHiVTNumModes, 0.0F);
  output.trajectories[0] = 2.0F;
  output.trajectories[1] = 1.0F;

  std::vector<HiVTModePrediction> predictions;
  ASSERT_TRUE(DecodeHiVTSceneOutput(input, output, {&target}, &predictions));
  ASSERT_EQ(predictions.size(), kHiVTNumModes);
  ASSERT_EQ(predictions.front().points.size(), kHiVTFutureSteps);
  const auto& point = predictions.front().points.front();
  EXPECT_NEAR(point.path_point().x(), 98.0, 1e-6);
  EXPECT_NEAR(point.path_point().y(), 199.0, 1e-6);
  EXPECT_NEAR(point.path_point().theta(), std::atan2(-1.0, -2.0), 1e-6);

  double probability_sum = 0.0;
  for (const auto& prediction : predictions) {
    probability_sum += prediction.probability;
  }
  EXPECT_NEAR(probability_sum, 1.0, 1e-6);
}

TEST(HiVTSceneEvaluatorTest, DecodesConsistentSpeedAndAcceleration) {
  Obstacle target;
  Feature feature;
  feature.set_id(42);
  feature.set_timestamp(10.0);
  feature.mutable_position()->set_x(100.0);
  feature.mutable_position()->set_y(200.0);
  feature.set_velocity_heading(0.0);
  feature.set_speed(5.0);
  target.InsertFeature(feature);

  HiVTSceneInput input;
  input.actor_count = 1;
  input.actor_ids = {42};
  input.origin_cos = 1.0;
  input.origin_sin = 0.0;
  input.rotate_angles = {0.0F};

  HiVTSceneOutput output;
  output.trajectories.assign(kHiVTNumModes * kHiVTFutureSteps * 4, 0.0F);
  output.logits.assign(kHiVTNumModes, 0.0F);
  output.trajectories[0] = 0.5F;
  output.trajectories[4] = 1.1F;
  output.trajectories[8] = 1.8F;

  std::vector<HiVTModePrediction> predictions;
  ASSERT_TRUE(DecodeHiVTSceneOutput(input, output, {&target}, &predictions));
  ASSERT_EQ(predictions.size(), kHiVTNumModes);
  ASSERT_EQ(predictions.front().points.size(), kHiVTFutureSteps);
  const auto& points = predictions.front().points;
  EXPECT_NEAR(points[0].v(), 5.0, 1e-5);
  EXPECT_NEAR(points[0].a(), 0.0, 1e-4);
  EXPECT_NEAR(points[1].v(), 6.0, 1e-5);
  EXPECT_NEAR(points[1].a(), 10.0, 1e-4);
  EXPECT_NEAR(points[2].v(), 7.0, 1e-5);
  EXPECT_NEAR(points[2].a(), 10.0, 1e-4);
}

}  // namespace prediction
}  // namespace apollo
