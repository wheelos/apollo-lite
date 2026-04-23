// Copyright 2025 WheelOS All Rights Reserved.
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

//  Created Date: 2025-01-03
//  Author: daohu527

#include "modules/planning/common/obstacle_decider.h"

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace planning {

SemanticSafetyConfig ObstacleDecider::GetSafetyMargin(const Obstacle& obs) {
  SemanticSafetyConfig config;
  switch (obs.Perception().sub_type()) {
    case perception::PerceptionObstacle::PEDESTRIAN:
      // Pedestrians: Large lateral cushioning (to prevent falls/sudden changes
      // of direction), large longitudinal cushioning
      config.lat_buffer = 0.6;
      config.lon_buffer = 1.0;
      break;

    case perception::PerceptionObstacle::BICYCLE:
      // The cyclist swings with a large amplitude
      config.lat_buffer = 0.8;
      config.lon_buffer = 2.0;
      break;

    case perception::PerceptionObstacle::VEHICLE:
      if (obs.IsStatic()) {
        // Stationary vehicle: It may be illegally parked; drive around it
        // closely.
        config.lat_buffer = 0.3;
        config.lon_buffer = 0.5;
      } else {
        // Dynamic vehicle: Normal following distance
        config.lat_buffer = 0.4;
        config.lon_buffer = 2.0;  // Corresponding time interval 2s
      }
      break;

    case perception::PerceptionObstacle::UNKNOWN_MOVABLE:
      // High risk of ghost peeking out
      config.lat_buffer = 0.5;
      config.lon_buffer = 1.0;
      break;

    default:  // STATIC, WALL, ETC.
      // Inanimate objects: can be placed close to the edge
      config.lat_buffer = 0.15;
      config.lon_buffer = 0.2;
      break;
  }
  return config;
}

bool ObstacleDecider::IsIgnorable(const SLBoundary& obs_sl, const Obstacle& obs,
                                  const ReferenceLine& ref_line,
                                  const SLBoundary& adc_sl,
                                  bool is_change_lane_path) {
  // High-caution obstacles are never ignorable
  if (obs.IsCautionLevelObstacle()) return false;

  // 1. Check if beyond road length
  if (obs_sl.start_s() > ref_line.Length()) {
    AERROR << "Ignored: Beyond road length. S: " << obs_sl.start_s()
           << " Length: " << ref_line.Length();
    return true;
  }

  // 2. Check if behind ego vehicle
  double back_buffer = 2.0;
  if (obs_sl.end_s() < adc_sl.start_s() - back_buffer) {
    if (is_change_lane_path && ref_line.IsOnLane(obs_sl)) {
      return false;
    }
    AERROR << "Ignored: Behind ego. ObsEndS: " << obs_sl.end_s()
           << " ADCStartS: " << adc_sl.start_s();
    return true;
  }

  // 3. Check lateral position
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!ref_line.GetLaneWidth(obs_sl.start_s(), &lane_left_width,
                             &lane_right_width)) {
    AERROR << "Ignored: Failed to get lane width at S: " << obs_sl.start_s();
    return true;
  }

  // Debug print
  ADEBUG << "GetLaneWidth: s=" << obs_sl.start_s()
         << " lane_left=" << lane_left_width
         << " lane_right=" << lane_right_width << " obs_l=[" << obs_sl.start_l()
         << "," << obs_sl.end_l() << "]";

  const double lane_min_l = -lane_right_width;
  const double lane_max_l = lane_left_width;

  if (obs_sl.start_l() > lane_max_l || obs_sl.end_l() < lane_min_l) {
    AERROR << "Ignored: Out of lane. L: [" << obs_sl.start_l() << ", "
           << obs_sl.end_l() << "] Lane: [" << lane_min_l << ", " << lane_max_l
           << "]";
    return true;
  }

  return false;
}

bool ObstacleDecider::IsStaticBlocking(const SLBoundary& obs_sl,
                                       const Obstacle& obs, double ego_width,
                                       const ReferenceLine& ref_line) {
  auto safety = GetSafetyMargin(obs);
  double pass_width = ego_width + safety.lat_buffer;

  double lane_left = 0.0, lane_right = 0.0;
  if (!ref_line.GetLaneWidth(obs_sl.start_s(), &lane_left, &lane_right)) {
    AERROR << "Failed to get lane width at s=" << obs_sl.start_s();
    // Consider it blocking if lane info not available
    return true;
  }

  const double lane_min_l = -lane_right;
  const double lane_max_l = lane_left;

  // Debug print
  ADEBUG << "IsStaticBlocking: s=" << obs_sl.start_s()
         << " lane_left=" << lane_left << " lane_right=" << lane_right
         << " obs_l=[" << obs_sl.start_l() << "," << obs_sl.end_l() << "]"
         << " pass_width=" << pass_width;

  double left_gap = lane_max_l - obs_sl.end_l();
  double right_gap = obs_sl.start_l() - lane_min_l;

  // If either side has enough gap to pass, it is not blocking
  if (left_gap >= pass_width || right_gap >= pass_width) {
    return false;
  }

  // Otherwise, obstacle blocks the lane
  return true;
}

InteractionType ObstacleDecider::ComputeInteractionType(
    const SLBoundary& obs_sl, const Obstacle& obs, double ego_width,
    const ReferenceLine& ref_line, const SLBoundary& adc_sl,
    bool is_change_lane_path) {
  // 1. Global Filtering
  if (IsIgnorable(obs_sl, obs, ref_line, adc_sl, is_change_lane_path)) {
    return InteractionType::IGNORE;
  }

  // 2. Static logic: only distinguishes between Blocking and Nudgeable.
  if (obs.IsStatic()) {
    if (IsStaticBlocking(obs_sl, obs, ego_width, ref_line)) {
      return InteractionType::BLOCKING;
    } else {
      // Static but not blocking -> can be nudged -> don't build ST
      return InteractionType::NUDGEABLE;
    }
  }

  // 3. Dynamic logic: as long as it's not ignored, it needs to enter ST graph
  // by default Even if it appears to be nigable, dynamic objects still need to
  // have their trajectory occupancy reflected in the ST graph.

  // Whether a collision occurs is handled by the overlap calculation within
  // BuildTrajectoryStBoundary.
  return InteractionType::YIELDING;
}

}  // namespace planning
}  // namespace apollo
