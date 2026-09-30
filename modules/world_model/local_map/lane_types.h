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

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace apollo {
namespace world_model {

struct LanePoint {
  double x = 0.0;
  double y = 0.0;
  bool operator==(const LanePoint& p) const { return x == p.x && y == p.y; }
};

struct RelativePose {
  double time = 0.0;
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double position_error = 0.0;
};

// Paired, ordered boundary samples in base_link at measurement_time.
struct LaneObservation {
  std::string session;
  uint64_t generation = 0;
  uint64_t sequence = 0;
  double measurement_time = 0.0;
  double position_error = 0.0;
  bool forward_confirmed = false;
  std::vector<LanePoint> left;
  std::vector<LanePoint> right;
};

struct TrackedLaneSample {
  LanePoint left;
  LanePoint right;
  double last_observed = 0.0;
  double position_error = 0.0;
};

struct TemporalLaneSnapshot {
  std::string session;
  uint64_t generation = 0;
  uint64_t version = 0;
  uint64_t lane_id = 0;
  double measurement_time = 0.0;
  double valid_until = 0.0;
  std::vector<TrackedLaneSample> samples;
};

struct TemporalLanePolicy {
  double grid_spacing = 0.0;
  double history_seconds = 0.0;
  double max_pose_gap = 0.0;
  double max_observation_age = 0.0;
  double sample_lifetime = 0.0;
  double publication_lifetime = 0.0;
  double association_distance = 0.0;
  double min_overlap = 0.0;
  double min_width = 0.0;
  double max_width = 0.0;
  double max_position_error = 0.0;
  double fresh_weight = 0.0;
};

}  // namespace world_model
}  // namespace apollo
