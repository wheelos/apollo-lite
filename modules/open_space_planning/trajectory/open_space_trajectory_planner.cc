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

#include "modules/open_space_planning/trajectory/open_space_trajectory_planner.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace apollo {
namespace open_space_planning {

OpenSpaceTrajectoryPlanner::OpenSpaceTrajectoryPlanner() = default;

OpenSpaceTrajectoryPlanner::OpenSpaceTrajectoryPlanner(
    const planning::PlannerOpenSpaceConfig& config)
    : config_(config) {}

bool OpenSpaceTrajectoryPlanner::PlanKinematicFallback(
    const PlanningProblem& problem, const RouteCandidate& route,
    PhysicalTrajectory* trajectory) {
  if (route.skeleton.empty()) {
    return false;
  }

  trajectory->source_candidate_id = route.id;
  trajectory->map_revision =
      problem.grid_map != nullptr ? problem.grid_map->revision : 0;
  trajectory->goal_revision = problem.goal.revision;
  trajectory->points.clear();

  double accumulated_s = 0.0;
  double current_time = 0.0;
  constexpr double dt = 0.1;
  const double nominal_v = (problem.goal.target_speed > 0.0)
                               ? problem.goal.target_speed
                               : 1.0;

  for (std::size_t i = 0; i < route.skeleton.size(); ++i) {
    const auto& sp = route.skeleton[i];
    PhysicalTrajectoryPoint pt;
    pt.pose = sp.pose;
    pt.curvature = sp.curvature;
    pt.gear = sp.gear;
    pt.velocity = (sp.gear == Gear::kReverse) ? -nominal_v : nominal_v;
    pt.acceleration = 0.0;

    if (i > 0) {
      const double dx = pt.pose.x - trajectory->points.back().pose.x;
      const double dy = pt.pose.y - trajectory->points.back().pose.y;
      const double ds = std::hypot(dx, dy);
      accumulated_s += ds;
      current_time += (nominal_v > 1e-3) ? (ds / nominal_v) : dt;
    }
    pt.s = accumulated_s;
    pt.relative_time = current_time;
    trajectory->points.push_back(pt);
  }

  trajectory->cost = 2.0;
  return !trajectory->points.empty();
}

bool OpenSpaceTrajectoryPlanner::PlanQpPathAndSpeed(
    const PlanningProblem& problem, const RouteCandidate& route,
    PhysicalTrajectory* trajectory) {
  if (route.skeleton.empty()) {
    return false;
  }

  // Set vehicle turning radius / curvature limit
  const double max_curvature = (problem.vehicle.maximum_curvature > 0.0)
                                   ? problem.vehicle.maximum_curvature
                                   : 0.5;
  path_optimizer_.set_curvature_constraint(max_curvature);

  // 1. Partition route into contiguous single-gear segments
  struct GearSubSegment {
    Gear gear = Gear::kDrive;
    std::vector<GeometricPathPoint> skeleton;
    std::vector<CorridorSample> corridor;
  };

  std::vector<GearSubSegment> segments;
  for (std::size_t i = 0; i < route.skeleton.size(); ++i) {
    const auto& pt = route.skeleton[i];
    const auto& cs = (i < route.corridor.size())
                         ? route.corridor[i]
                         : CorridorSample{pt.s, -2.0, 2.0};

    if (segments.empty() || segments.back().gear != pt.gear) {
      GearSubSegment new_seg;
      new_seg.gear = pt.gear;
      if (!segments.empty() && !segments.back().skeleton.empty()) {
        // Boundary transition anchor
        new_seg.skeleton.push_back(segments.back().skeleton.back());
        new_seg.corridor.push_back(segments.back().corridor.back());
      }
      new_seg.skeleton.push_back(pt);
      new_seg.corridor.push_back(cs);
      segments.push_back(std::move(new_seg));
    } else {
      segments.back().skeleton.push_back(pt);
      segments.back().corridor.push_back(cs);
    }
  }

  if (segments.empty()) {
    return false;
  }

  trajectory->source_candidate_id = route.id;
  trajectory->map_revision =
      problem.grid_map != nullptr ? problem.grid_map->revision : 0;
  trajectory->goal_revision = problem.goal.revision;
  trajectory->points.clear();

  double global_time_offset = 0.0;
  double global_s_offset = 0.0;
  constexpr double kTimeHorizon = 4.0;
  constexpr double kDeltaT = 0.1;
  constexpr double kSafetyMargin = 1.0;

  for (std::size_t seg_idx = 0; seg_idx < segments.size(); ++seg_idx) {
    const auto& seg = segments[seg_idx];
    if (seg.skeleton.empty()) {
      continue;
    }

    // A. Smooth path within corridor respecting curvature limits
    std::vector<GeometricPathPoint> smoothed_segment;
    if (seg.skeleton.size() < 3 ||
        !path_optimizer_.Solve(seg.skeleton, seg.corridor, &smoothed_segment) ||
        smoothed_segment.empty()) {
      smoothed_segment = seg.skeleton;
    }

    // Re-accumulate segment s
    double seg_accum_s = 0.0;
    smoothed_segment[0].s = 0.0;
    for (std::size_t i = 1; i < smoothed_segment.size(); ++i) {
      const double dx =
          smoothed_segment[i].pose.x - smoothed_segment[i - 1].pose.x;
      const double dy =
          smoothed_segment[i].pose.y - smoothed_segment[i - 1].pose.y;
      seg_accum_s += std::hypot(dx, dy);
      smoothed_segment[i].s = seg_accum_s;
    }

    const double seg_max_s = smoothed_segment.back().s;
    if (seg_max_s <= 1e-4) {
      continue;
    }

    // B. Compute dynamic obstacle ST boundaries for this segment
    std::vector<STBoundary> st_boundaries;
    PiecewiseJerkSpeedOsqp::ComputeSTBoundaries(
        smoothed_segment, problem.dynamic_obstacles, kTimeHorizon, kDeltaT,
        kSafetyMargin, &st_boundaries);

    // C. Piecewise Jerk Speed QP Optimization
    // Boundary conditions: cusps (gear shifts) require v = 0, a = 0
    const double v_init =
        (seg_idx == 0) ? std::abs(problem.start.longitudinal_velocity) : 0.0;
    const double a_init =
        (seg_idx == 0) ? problem.start.longitudinal_acceleration : 0.0;
    const double v_target = (seg_idx + 1 == segments.size())
                                ? ((problem.goal.target_speed > 0.0)
                                       ? problem.goal.target_speed
                                       : 2.0)
                                : 0.0;

    std::vector<SpeedProfilePoint> speed_profile;
    if (!speed_optimizer_.Solve(0.0, v_init, a_init, v_target, seg_max_s,
                                st_boundaries, &speed_profile) ||
        speed_profile.empty()) {
      return false;
    }

    // D. Interpolate physical trajectory points for this segment
    std::size_t path_idx = 0;
    for (std::size_t sp_i = 0; sp_i < speed_profile.size(); ++sp_i) {
      const auto& sp = speed_profile[sp_i];
      // Avoid duplicate timestamp point at segment boundary if already added
      if (sp_i == 0 && !trajectory->points.empty()) {
        continue;
      }

      while (path_idx + 1 < smoothed_segment.size() &&
             smoothed_segment[path_idx + 1].s < sp.s) {
        ++path_idx;
      }

      PhysicalTrajectoryPoint pt;
      if (path_idx + 1 < smoothed_segment.size()) {
        const auto& p0 = smoothed_segment[path_idx];
        const auto& p1 = smoothed_segment[path_idx + 1];
        const double ds = p1.s - p0.s;
        const double r = (ds > 1e-4) ? ((sp.s - p0.s) / ds) : 0.0;
        const double ratio = std::clamp(r, 0.0, 1.0);

        pt.pose.x = (1.0 - ratio) * p0.pose.x + ratio * p1.pose.x;
        pt.pose.y = (1.0 - ratio) * p0.pose.y + ratio * p1.pose.y;
        pt.pose.heading = p0.pose.heading;
        pt.curvature = (1.0 - ratio) * p0.curvature + ratio * p1.curvature;
        pt.gear = seg.gear;
      } else {
        const auto& p_last = smoothed_segment.back();
        pt.pose = p_last.pose;
        pt.curvature = p_last.curvature;
        pt.gear = seg.gear;
      }

      pt.s = global_s_offset + sp.s;
      pt.relative_time = global_time_offset + sp.t;
      pt.velocity = (seg.gear == Gear::kReverse) ? -sp.v : sp.v;
      pt.acceleration = (seg.gear == Gear::kReverse) ? -sp.a : sp.a;
      trajectory->points.push_back(pt);
    }

    global_time_offset += speed_profile.back().t;
    global_s_offset += speed_profile.back().s;
  }

  trajectory->cost = 0.5;
  return !trajectory->points.empty();
}

Status OpenSpaceTrajectoryPlanner::Plan(
    const TrajectoryPlanningRequest& request, PhysicalTrajectory* trajectory) {
  if (trajectory == nullptr) {
    return {StatusCode::kInvalidInput, "trajectory pointer is null"};
  }
  trajectory->points.clear();

  const auto& problem = request.problem;
  const auto& route = request.route;

  if (route.skeleton.empty()) {
    return {StatusCode::kInvalidInput, "route skeleton is empty"};
  }

  // 1. Fast SFC-QP Path & Piecewise Jerk ST Speed Optimization
  if (PlanQpPathAndSpeed(problem, route, trajectory)) {
    return Status::Ok();
  }

  // 2. Kinematic Fallback
  if (PlanKinematicFallback(problem, route, trajectory)) {
    return Status::Ok();
  }

  return {StatusCode::kTrajectoryOptimizationFailed,
          "failed to generate valid physical trajectory"};
}

}  // namespace open_space_planning
}  // namespace apollo
