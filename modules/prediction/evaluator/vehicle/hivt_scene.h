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

#pragma once

#include <cstdint>
#include <vector>

namespace apollo {
namespace prediction {

constexpr int kHiVTHistoricalSteps = 20;
constexpr int kHiVTFutureSteps = 30;
constexpr int kHiVTNumModes = 6;
constexpr int kHiVTMaxActors = 64;
constexpr int kHiVTMaxLanePolylines = 12;
constexpr int kHiVTMaxLaneVectors = 1024;
constexpr double kHiVTTimeStepSeconds = 0.1;

struct HiVTSceneInput {
  int actor_count = 0;
  int lane_vector_count = 0;
  int actor_edge_count = 0;
  int lane_actor_edge_count = 0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_cos = 1.0;
  double origin_sin = 0.0;
  std::vector<int> actor_ids;
  std::vector<float> x;
  std::vector<float> positions;
  std::vector<int64_t> edge_index;
  std::vector<uint8_t> padding_mask;
  std::vector<uint8_t> bos_mask;
  std::vector<float> rotate_angles;
  std::vector<float> lane_vectors;
  std::vector<uint8_t> is_intersections;
  std::vector<uint8_t> turn_directions;
  std::vector<uint8_t> traffic_controls;
  std::vector<int64_t> lane_actor_index;
  std::vector<float> lane_actor_vectors;
};

struct HiVTSceneOutput {
  std::vector<float> trajectories;
  std::vector<float> logits;
};

}  // namespace prediction
}  // namespace apollo
