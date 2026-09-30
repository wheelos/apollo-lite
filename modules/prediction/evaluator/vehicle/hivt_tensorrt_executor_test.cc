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

#include "modules/prediction/evaluator/vehicle/hivt_tensorrt_executor.h"

#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "modules/prediction/common/prediction_system_gflags.h"

namespace apollo {
namespace prediction {

TEST(HiVTTensorRtExecutorTest, RunsTheSceneTensorContract) {
  HiVTTensorRtExecutor executor;
  ASSERT_TRUE(
      executor.Init(FLAGS_hivt_tensorrt_engine, FLAGS_hivt_tensorrt_device_id));

  HiVTSceneInput input;
  input.actor_count = 2;
  input.lane_vector_count = 1;
  input.actor_edge_count = 2;
  input.lane_actor_edge_count = 2;
  input.actor_ids = {0, 1};
  input.x.assign(input.actor_count * kHiVTHistoricalSteps * 2, 0.0F);
  input.positions.assign(
      input.actor_count * (kHiVTHistoricalSteps + kHiVTFutureSteps) * 2, 0.0F);
  input.edge_index = {0, 1, 1, 0};
  input.padding_mask.assign(
      input.actor_count * (kHiVTHistoricalSteps + kHiVTFutureSteps), 0);
  input.bos_mask.assign(input.actor_count * kHiVTHistoricalSteps, 0);
  input.rotate_angles.assign(input.actor_count, 0.0F);
  input.lane_vectors = {1.0F, 0.0F};
  input.is_intersections = {0};
  input.turn_directions = {0};
  input.traffic_controls = {0};
  input.lane_actor_index = {0, 0, 0, 1};
  input.lane_actor_vectors = {-1.9F, 0.0F, -2.9F, -1.0F};

  for (int actor = 0; actor < input.actor_count; ++actor) {
    input.bos_mask[actor * kHiVTHistoricalSteps] = 1;
    for (int step = 0; step < kHiVTHistoricalSteps; ++step) {
      const std::size_t position_index =
          (actor * (kHiVTHistoricalSteps + kHiVTFutureSteps) + step) * 2;
      input.positions[position_index] = static_cast<float>(actor + step * 0.1);
      input.positions[position_index + 1] = static_cast<float>(actor);
      if (step > 0) {
        const std::size_t motion_index =
            (actor * kHiVTHistoricalSteps + step) * 2;
        input.x[motion_index] = 0.1F;
      }
    }
  }

  HiVTSceneOutput output;
  ASSERT_TRUE(executor.Run(input, &output));
  ASSERT_EQ(output.trajectories.size(),
            kHiVTNumModes * input.actor_count * kHiVTFutureSteps * 4);
  ASSERT_EQ(output.logits.size(), input.actor_count * kHiVTNumModes);
  for (const float value : output.trajectories) {
    EXPECT_TRUE(std::isfinite(value));
  }
  for (const float value : output.logits) {
    EXPECT_TRUE(std::isfinite(value));
  }
}

}  // namespace prediction
}  // namespace apollo
