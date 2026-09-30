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

#include "modules/local_planning/trajectory_continuity/trajectory_continuity.h"

#include <algorithm>
#include <cmath>

namespace apollo {
namespace local_planning {
namespace {

common::Status Error(const char* reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR, reason);
}

double Angle(double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

}  // namespace

common::Status AlignTrajectoryStart(const CycleInput& input,
                                    const LocalTrajectory* previous,
                                    const TrajectoryStitchPolicy& policy,
                                    TrajectoryStart* start) {
  if (start == nullptr) return Error("trajectory start output required");
  *start = {};
  if (!std::isfinite(input.planning_time) || !std::isfinite(input.odometry.x) ||
      !std::isfinite(input.odometry.y) ||
      !std::isfinite(input.odometry.heading) ||
      !std::isfinite(input.odometry.speed_mps) ||
      !std::isfinite(policy.position_tolerance) ||
      !std::isfinite(policy.heading_tolerance) ||
      !std::isfinite(policy.speed_tolerance) ||
      policy.position_tolerance < 0.0 || policy.heading_tolerance < 0.0 ||
      policy.speed_tolerance < 0.0) {
    return Error("invalid ODOM start or continuity tolerances");
  }
  start->point = {0.0,
                  input.odometry.x,
                  input.odometry.y,
                  input.odometry.heading,
                  std::max(0.0, input.odometry.speed_mps),
                  0.0,
                  0.0};
  if (previous == nullptr) return common::Status::OK();
  if (!previous->valid ||
      !(previous->stamp.epoch == input.odometry.stamp.epoch) ||
      previous->stamp.frame_id != input.odometry.stamp.frame_id ||
      previous->stamp.clock_id != input.odometry.stamp.clock_id ||
      previous->lane_id != input.corridor.lane_id) {
    start->continuity = TrajectoryContinuityState::SOURCE_CHANGED;
    return common::Status::OK();
  }
  if (input.planning_time >= previous->stamp.valid_until) {
    start->continuity = TrajectoryContinuityState::EXPIRED;
    return common::Status::OK();
  }
  const double elapsed = input.planning_time - previous->stamp.measurement_time;
  if (!std::isfinite(elapsed) || previous->points.size() < 2 ||
      elapsed < previous->points.front().time ||
      elapsed > previous->points.back().time) {
    start->continuity = TrajectoryContinuityState::OUTSIDE_HORIZON;
    return common::Status::OK();
  }
  for (size_t i = 1; i < previous->points.size(); ++i) {
    const auto& a = previous->points[i - 1];
    const auto& b = previous->points[i];
    if (a.time <= elapsed && elapsed <= b.time) {
      const double duration = b.time - a.time;
      if (!std::isfinite(duration) || duration <= 0.0)
        return Error("invalid previous trajectory point times");
      const double weight = (elapsed - a.time) / duration;
      TrajectoryPoint seed;
      seed.x = a.x + weight * (b.x - a.x);
      seed.y = a.y + weight * (b.y - a.y);
      seed.heading = a.heading + weight * Angle(b.heading - a.heading);
      seed.speed = a.speed + weight * (b.speed - a.speed);
      seed.acceleration =
          a.acceleration + weight * (b.acceleration - a.acceleration);
      seed.curvature = a.curvature + weight * (b.curvature - a.curvature);
      if (!std::isfinite(seed.x) || !std::isfinite(seed.y) ||
          !std::isfinite(seed.heading) || !std::isfinite(seed.speed) ||
          !std::isfinite(seed.acceleration) || !std::isfinite(seed.curvature)) {
        return Error("nonfinite previous trajectory seed");
      }
      start->point.acceleration = seed.acceleration;
      start->point.curvature = seed.curvature;
      if (std::hypot(seed.x - start->point.x, seed.y - start->point.y) <=
              policy.position_tolerance &&
          std::abs(Angle(seed.heading - start->point.heading)) <=
              policy.heading_tolerance &&
          std::abs(seed.speed - start->point.speed) <= policy.speed_tolerance) {
        start->point = seed;
        start->continuity = TrajectoryContinuityState::STITCHED;
      } else {
        start->continuity = TrajectoryContinuityState::REANCHORED;
      }
      return common::Status::OK();
    }
  }
  start->continuity = TrajectoryContinuityState::OUTSIDE_HORIZON;
  return common::Status::OK();
}

void TrajectoryContinuity::Reset() { previous_.reset(); }

common::Status TrajectoryContinuity::Start(const CycleInput& input,
                                           const TrajectoryStitchPolicy& policy,
                                           TrajectoryStart* start) const {
  return AlignTrajectoryStart(input, previous_ ? &*previous_ : nullptr, policy,
                              start);
}

common::Status TrajectoryContinuity::Accept(const CycleInput& input,
                                            const LaneFollowConfig& config,
                                            const LocalTrajectory& trajectory) {
  auto status = ValidateTrajectory(input, config, trajectory);
  if (!status.ok()) {
    Reset();
    return status;
  }
  previous_ = trajectory;
  previous_->valid = true;
  return common::Status::OK();
}

}  // namespace local_planning
}  // namespace apollo
