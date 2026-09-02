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

#include "modules/open_space_planning/safety/default_fallback_planner.h"

#include <algorithm>
#include <cmath>

namespace apollo {
namespace open_space_planning {

DefaultFallbackPlanner::DefaultFallbackPlanner(FallbackPlannerConfig config)
    : config_(config) {}

Status DefaultFallbackPlanner::Plan(const FallbackPlanningRequest& request,
                                    PhysicalTrajectory* trajectory) {
  if (trajectory == nullptr) {
    return {StatusCode::kInvalidInput, "trajectory pointer is null"};
  }
  trajectory->points.clear();

  const auto& problem = request.problem;
  trajectory->source_candidate_id = 0;
  trajectory->map_revision =
      problem.grid_map != nullptr ? problem.grid_map->revision : 0;
  trajectory->goal_revision = problem.goal.revision;

  const double init_v = problem.start.longitudinal_velocity;
  const Gear init_gear = problem.start.gear;
  const double abs_v = std::abs(init_v);
  const double decel = std::max(0.5, config_.deceleration);
  const double dt = std::max(0.02, config_.time_step);

  const double stop_time =
      std::max(config_.minimum_stop_time, (abs_v > 1e-3 ? abs_v / decel : dt));
  const std::size_t num_points =
      static_cast<std::size_t>(std::ceil(stop_time / dt)) + 1;

  double current_x = problem.start.pose.x;
  double current_y = problem.start.pose.y;
  const double heading = problem.start.pose.heading;
  const double dir = (init_gear == Gear::kReverse) ? -1.0 : 1.0;
  double current_s = 0.0;
  double current_v = abs_v;

  for (std::size_t i = 0; i < num_points; ++i) {
    const double t = i * dt;
    if (i > 0) {
      current_v = std::max(0.0, abs_v - decel * t);
      const double ds = current_v * dt;
      current_s += ds;
      current_x += dir * ds * std::cos(heading);
      current_y += dir * ds * std::sin(heading);
    }

    PhysicalTrajectoryPoint pt;
    pt.pose.x = current_x;
    pt.pose.y = current_y;
    pt.pose.heading = heading;
    pt.curvature = 0.0;
    pt.curvature_derivative = 0.0;
    pt.velocity = (current_v > 1e-4) ? dir * current_v : 0.0;
    pt.acceleration = (current_v > 1e-4) ? -dir * decel : 0.0;
    pt.relative_time = t;
    pt.s = current_s;
    pt.gear = init_gear;

    trajectory->points.push_back(pt);
  }

  trajectory->cost = 100.0;
  return Status::Ok();
}

}  // namespace open_space_planning
}  // namespace apollo
