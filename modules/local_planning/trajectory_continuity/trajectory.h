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

#include <vector>

#include "modules/local_planning/planning_context/input_contract.h"

namespace apollo {
namespace local_planning {

struct LaneFollowConfig {
  double wheelbase = 0.0;
  double front_extent = 0.0;  // Rear axle to front bumper.
  double rear_extent = 0.0;
  double half_width = 0.0;
  double cruise_speed = 0.0;
  double acceleration = 0.0;
  double braking = 0.0;
  double max_curvature = 0.0;
  double max_lateral_acceleration = 0.0;
  double lookahead = 0.0;
  double horizon = 0.0;
  double step = 0.0;
  double execution_lifetime = 0.0;
  double clearance = 0.0;
  double max_position_error = 0.0;
  double max_jerk = 0.0;
  double max_curvature_rate = 0.0;
  double stitch_position_tolerance = 0.0;
  double stitch_heading_tolerance = 0.0;
  double stitch_speed_tolerance = 0.0;
};

struct TrajectoryPoint {
  double time = 0.0;
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double speed = 0.0;
  double acceleration = 0.0;
  double curvature = 0.0;
};

enum class BehaviorState { IDLE, FOLLOWING, STOPPING, STOPPED, INVALID };
enum class StopReason { CORRIDOR_END, OBSTACLE, OBSERVED_RULE };

struct LocalTrajectory {
  bool valid = false;
  SourceStamp stamp;
  uint64_t graph_sequence = 0;
  uint64_t prediction_sequence = 0;
  uint64_t lane_id = 0;
  bool stitched = false;
  BehaviorState behavior = BehaviorState::INVALID;
  StopReason stop_reason = StopReason::CORRIDOR_END;
  std::vector<TrajectoryPoint> points;
};

}  // namespace local_planning
}  // namespace apollo
