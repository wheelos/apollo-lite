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

#include "modules/local_planning/trajectory_continuity/trajectory_validation.h"

#include <algorithm>
#include <cmath>

#include "modules/local_planning/planning_context/environment_geometry.h"
#include "modules/local_planning/planning_context/trajectory_geometry.h"

namespace apollo {
namespace local_planning {
namespace {
common::Status Error(const std::string& reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR,
                        "trajectory validation: " + reason);
}
double Angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
common::Status Footprint(const CycleInput& input, const LaneFollowConfig& c,
                         const CorridorPath& path, const TrajectoryPoint& p) {
  const double sweep = (p.speed + c.acceleration * c.step) * c.step *
                       (1.0 + c.front_extent * c.max_curvature);
  const double radius =
      std::hypot((c.front_extent + c.rear_extent) / 6.0, c.half_width) +
      c.clearance + input.corridor.position_error_bound + sweep;
  const double age =
      input.planning_time - input.prediction.stamp.measurement_time;
  for (int i = 0; i < 3; ++i) {
    const double offset =
        -c.rear_extent + (i + 0.5) * (c.front_extent + c.rear_extent) / 3.0;
    const common::math::Vec2d center(p.x + offset * std::cos(p.heading),
                                     p.y + offset * std::sin(p.heading));
    if (path.Project(center).distance + radius > input.corridor.half_width)
      return Error("swept footprint leaves observed corridor");
    for (const auto& object : input.occupancy.obstacles) {
      const common::math::Vec2d position(
          object.position.x + object.velocity.x * (age + p.time),
          object.position.y + object.velocity.y * (age + p.time));
      const double object_sweep =
          std::hypot(object.velocity.x, object.velocity.y) * c.step;
      if (center.DistanceTo(position) <= radius + object.radius + object_sweep)
        return Error("swept footprint intersects predicted occupancy");
    }
  }
  const VehicleEnvelope vehicle{c.front_extent, c.rear_extent, c.half_width,
                                c.clearance};
  const EnvironmentPose pose{p.x, p.y, p.heading};
  auto environment_status =
      CheckEnvironmentSweep(input.environment, vehicle, pose, pose);
  if (!environment_status.ok()) return environment_status;
  return common::Status::OK();
}
}  // namespace

common::Status CheckLaneFootprint(const CycleInput& input,
                                  const LaneFollowConfig& c,
                                  const TrajectoryPoint& p) {
  if (input.corridor.centerline.size() < 2) return Error("missing corridor");
  return Footprint(input, c, CorridorPath(input.corridor), p);
}

common::Status ValidateTrajectory(const CycleInput& input,
                                  const LaneFollowConfig& c,
                                  const LocalTrajectory& t) {
  for (double v : {c.step, c.horizon, c.braking, c.acceleration, c.max_jerk,
                   c.max_curvature, c.max_curvature_rate, c.cruise_speed,
                   c.max_lateral_acceleration, c.execution_lifetime,
                   c.front_extent, c.rear_extent, c.half_width, c.clearance,
                   c.max_position_error, c.stitch_position_tolerance,
                   c.stitch_heading_tolerance, c.stitch_speed_tolerance}) {
    if (!std::isfinite(v) || v <= 0.0)
      return Error("invalid validation limits");
  }
  for (double v : {input.planning_time, input.odometry.x, input.odometry.y,
                   input.odometry.heading, input.odometry.speed_mps})
    if (!std::isfinite(v)) return Error("nonfinite ego or reference time");
  for (const auto* stamp :
       {&input.odometry.stamp, &input.graph, &input.prediction.stamp}) {
    if (stamp->health != InputHealth::HEALTHY || stamp->frame_id.empty() ||
        stamp->clock_id.empty() || stamp->epoch.producer_session.empty() ||
        stamp->sequence == 0 || stamp->epoch.generation == 0 ||
        stamp->frame_id != input.odometry.stamp.frame_id ||
        stamp->clock_id != input.odometry.stamp.clock_id ||
        !(stamp->epoch == input.odometry.stamp.epoch) ||
        !std::isfinite(stamp->measurement_time) ||
        !std::isfinite(stamp->publication_time) ||
        !std::isfinite(stamp->valid_until) || stamp->measurement_time < 0.0 ||
        stamp->measurement_time > stamp->publication_time ||
        stamp->publication_time > input.planning_time ||
        input.planning_time >= stamp->valid_until)
      return Error("invalid input stamp");
  }
  if (input.odometry.stamp.measurement_time != input.planning_time ||
      input.odometry.speed_mps < -0.05 ||
      input.odometry.speed_mps > c.cruise_speed + 0.5 ||
      input.prediction.graph_sequence != input.graph.sequence ||
      input.corridor.lane_id == 0 ||
      (input.corridor.speed_limit &&
       (!std::isfinite(*input.corridor.speed_limit) ||
        *input.corridor.speed_limit < 0.0)) ||
      (input.corridor.stop_station &&
       (!std::isfinite(*input.corridor.stop_station) ||
        *input.corridor.stop_station < 0.0)))
    return Error("invalid input alignment or road restrictions");
  if (t.points.size() < 2 || t.points.size() > 1001 ||
      input.corridor.centerline.size() < 2 ||
      input.corridor.centerline.size() > 2000 ||
      t.stamp.frame_id != input.odometry.stamp.frame_id ||
      t.stamp.clock_id != input.odometry.stamp.clock_id ||
      !(t.stamp.epoch == input.odometry.stamp.epoch) ||
      t.stamp.sequence != input.odometry.stamp.sequence ||
      t.stamp.health != InputHealth::HEALTHY ||
      t.stamp.measurement_time != input.planning_time ||
      t.stamp.publication_time != input.planning_time ||
      !std::isfinite(t.stamp.valid_until) ||
      t.stamp.valid_until <= input.planning_time ||
      t.stamp.valid_until >
          std::min({input.planning_time + c.execution_lifetime,
                    input.odometry.stamp.valid_until, input.graph.valid_until,
                    input.prediction.stamp.valid_until}) ||
      t.graph_sequence != input.graph.sequence ||
      t.prediction_sequence != input.prediction.stamp.sequence ||
      t.lane_id != input.corridor.lane_id ||
      !input.corridor.confirmed_forward ||
      !input.occupancy.corridor_fully_observed ||
      !std::isfinite(input.corridor.half_width) ||
      input.corridor.half_width <= c.half_width ||
      !std::isfinite(input.corridor.position_error_bound) ||
      input.corridor.position_error_bound < 0.0 ||
      input.corridor.position_error_bound > c.max_position_error ||
      !std::isfinite(input.occupancy.horizon) ||
      input.occupancy.horizon < input.planning_time -
                                    input.prediction.stamp.measurement_time +
                                    c.horizon + c.step) {
    return Error("invalid trajectory provenance, deadline or scene");
  }
  for (size_t i = 0; i < input.corridor.centerline.size(); ++i) {
    const auto& p = input.corridor.centerline[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
      return Error("nonfinite corridor");
    if (i > 0) {
      const auto& a = input.corridor.centerline[i - 1];
      const double distance = std::hypot(p.x - a.x, p.y - a.y);
      if (distance < 0.05 || distance > 2.0)
        return Error("invalid corridor spacing");
    }
  }
  for (const auto& object : input.occupancy.obstacles) {
    for (double v : {object.position.x, object.position.y, object.velocity.x,
                     object.velocity.y, object.radius})
      if (!std::isfinite(v)) return Error("nonfinite predicted obstacle");
    if (object.radius <= 0.0) return Error("invalid predicted obstacle");
  }
  const world_model::SceneSource scene_source{
      input.graph.frame_id,
      input.graph.clock_id,
      input.graph.epoch.producer_session,
      input.graph.epoch.generation,
      input.graph.sequence,
      input.graph.measurement_time,
      input.graph.publication_time,
      input.graph.valid_until,
      world_model::SceneHealth::HEALTHY};
  auto environment_status =
      ValidateEnvironment(input.environment, scene_source, input.planning_time);
  if (!environment_status.ok()) return environment_status;
  const CorridorPath path(input.corridor);
  const double cover =
      std::hypot((c.front_extent + c.rear_extent) / 6.0, c.half_width);
  double stop = path.length() - c.front_extent - c.clearance - cover;
  if (input.corridor.stop_station) {
    stop = std::min(stop, *input.corridor.stop_station - c.front_extent -
                              c.clearance - cover);
  }
  TrajectoryPoint actual;
  actual.x = input.odometry.x;
  actual.y = input.odometry.y;
  actual.heading = input.odometry.heading;
  actual.speed = std::max(0.0, input.odometry.speed_mps);
  const double actual_remaining = stop - path.Project({actual.x, actual.y}).s;
  if (actual_remaining < 0.0 ||
      actual.speed * actual.speed / (2.0 * c.braking) + actual.speed * c.step >
          actual_remaining)
    return Error("actual ODOM stopping state is infeasible");
  auto actual_status = Footprint(input, c, path, actual);
  if (!actual_status.ok()) return actual_status;
  const auto& first = t.points.front();
  if (std::abs(first.time) > 1e-9 ||
      std::abs(t.points.back().time - c.horizon) > 1e-6 ||
      std::hypot(first.x - input.odometry.x, first.y - input.odometry.y) >
          c.stitch_position_tolerance ||
      std::abs(Angle(first.heading - input.odometry.heading)) >
          c.stitch_heading_tolerance ||
      std::abs(first.speed - std::max(0.0, input.odometry.speed_mps)) >
          c.stitch_speed_tolerance) {
    return Error("trajectory horizon or measured-state continuity mismatch");
  }
  for (size_t i = 0; i < t.points.size(); ++i) {
    const auto& p = t.points[i];
    for (double v :
         {p.time, p.x, p.y, p.heading, p.speed, p.acceleration, p.curvature})
      if (!std::isfinite(v)) return Error("nonfinite trajectory point");
    if (p.speed < 0.0 || p.speed > c.cruise_speed + 0.5 ||
        (input.corridor.speed_limit &&
         p.speed > *input.corridor.speed_limit + 1e-6) ||
        p.acceleration < -c.braking - 1e-6 ||
        p.acceleration > c.acceleration + 1e-6 ||
        std::abs(p.curvature) > c.max_curvature + 1e-6 ||
        p.speed * p.speed * std::abs(p.curvature) >
            c.max_lateral_acceleration + 1e-6) {
      return Error("trajectory exceeds dynamic limits");
    }
    const double remaining = stop - path.Project({p.x, p.y}).s;
    if (remaining < -1e-6 ||
        p.speed * p.speed / (2.0 * c.braking) + p.speed * c.step >
            remaining + 1e-6)
      return Error("trajectory cannot stop within supported distance");
    if (i > 0) {
      const auto& a = t.points[i - 1];
      const double dt = p.time - a.time;
      const double v = std::max(0.0, a.speed + a.acceleration * dt);
      const double distance = (a.speed + v) * 0.5 * dt;
      const double heading = a.heading + distance * a.curvature * 0.5;
      if (std::abs(dt - c.step) > 1e-6 ||
          std::abs(p.acceleration - a.acceleration) > c.max_jerk * dt + 1e-6 ||
          std::abs(p.curvature - a.curvature) >
              c.max_curvature_rate * dt + 1e-6 ||
          std::abs(p.speed - v) > 1e-6 ||
          std::hypot(p.x - a.x - distance * std::cos(heading),
                     p.y - a.y - distance * std::sin(heading)) > 1e-6 ||
          std::abs(Angle(p.heading - a.heading - distance * a.curvature)) >
              1e-6)
        return Error("trajectory time, rate or kinematic consistency failure");
    }
    auto status = Footprint(input, c, path, p);
    if (!status.ok()) return status;
    if (i > 0) {
      const VehicleEnvelope vehicle{c.front_extent, c.rear_extent, c.half_width,
                                    c.clearance};
      status = CheckEnvironmentSweep(
          input.environment, vehicle,
          {t.points[i - 1].x, t.points[i - 1].y, t.points[i - 1].heading},
          {p.x, p.y, p.heading});
      if (!status.ok()) return status;
    }
  }
  return common::Status::OK();
}

}  // namespace local_planning
}  // namespace apollo
