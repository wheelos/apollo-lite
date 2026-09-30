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
#include <tuple>

#include "modules/local_planning/planning_context/scene.h"
#include "modules/world_model/local_map/local_scene.h"

namespace apollo {
namespace local_planning {

enum class InputHealth { UNKNOWN, HEALTHY, DEGRADED, INVALID };

struct OdomEpoch {
  std::string producer_session;
  uint64_t generation = 0;

  bool operator==(const OdomEpoch& other) const {
    return producer_session == other.producer_session &&
           generation == other.generation;
  }
};

// Internal admission metadata, not a wire schema or proof of payload safety.
struct SourceStamp {
  std::string frame_id;
  std::string clock_id;
  OdomEpoch epoch;
  uint64_t sequence = 0;
  double measurement_time = 0.0;
  double publication_time = 0.0;
  double valid_until = 0.0;
  InputHealth health = InputHealth::UNKNOWN;

  bool operator==(const SourceStamp& other) const {
    return std::tie(frame_id, clock_id, epoch, sequence, measurement_time,
                    publication_time, valid_until, health) ==
           std::tie(other.frame_id, other.clock_id, other.epoch, other.sequence,
                    other.measurement_time, other.publication_time,
                    other.valid_until, other.health);
  }
};

struct OdometryInput {
  SourceStamp stamp;
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double speed_mps = 0.0;

  bool operator==(const OdometryInput& other) const {
    return stamp == other.stamp &&
           std::tie(x, y, heading, speed_mps) ==
               std::tie(other.x, other.y, other.heading, other.speed_mps);
  }
};

struct PredictionInput {
  SourceStamp stamp;
  uint64_t graph_sequence = 0;
};

struct CycleInput {
  double planning_time = 0.0;
  OdometryInput odometry;
  SourceStamp graph;
  PredictionInput prediction;
  LocalCorridor corridor;
  world_model::DrivableEnvironment environment;
  LocalOccupancy occupancy;
};

// Budgets must come from the caller's declared operating domain.
struct InputPolicy {
  std::string odom_frame;
  std::string clock_id;
  double max_odom_age = 0.0;
  double max_graph_age = 0.0;
  double max_prediction_age = 0.0;
  double stationary_speed_limit = 0.0;
};

}  // namespace local_planning
}  // namespace apollo
