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

#include "modules/local_planning/planning_context/environment_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>

#include "modules/local_planning/planning_context/trajectory_geometry.h"

namespace apollo {
namespace local_planning {
namespace {

using world_model::LanePoint;

common::Status Error(const std::string& reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR,
                        "drivable environment: " + reason);
}

bool Finite(const LanePoint& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::abs(p.x) < 1e6 &&
         std::abs(p.y) < 1e6;
}

double Cross(const LanePoint& a, const LanePoint& b, const LanePoint& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

double DistanceToSegment(const LanePoint& p, const LanePoint& a,
                         const LanePoint& b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length_squared = dx * dx + dy * dy;
  const double t =
      length_squared > 0.0
          ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length_squared,
                       0.0, 1.0)
          : 0.0;
  return std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
}

int Orientation(const LanePoint& a, const LanePoint& b, const LanePoint& c) {
  const double value = Cross(a, b, c);
  return value > 1e-9 ? 1 : value < -1e-9 ? -1 : 0;
}

bool SegmentsIntersect(const LanePoint& a, const LanePoint& b,
                       const LanePoint& c, const LanePoint& d) {
  const int ab_c = Orientation(a, b, c);
  const int ab_d = Orientation(a, b, d);
  const int cd_a = Orientation(c, d, a);
  const int cd_b = Orientation(c, d, b);
  if (ab_c != ab_d && cd_a != cd_b) return true;
  return (ab_c == 0 && DistanceToSegment(c, a, b) <= 1e-9) ||
         (ab_d == 0 && DistanceToSegment(d, a, b) <= 1e-9) ||
         (cd_a == 0 && DistanceToSegment(a, c, d) <= 1e-9) ||
         (cd_b == 0 && DistanceToSegment(b, c, d) <= 1e-9);
}

double SegmentDistance(const LanePoint& a, const LanePoint& b,
                       const LanePoint& c, const LanePoint& d) {
  if (SegmentsIntersect(a, b, c, d)) return 0.0;
  return std::min({DistanceToSegment(a, c, d), DistanceToSegment(b, c, d),
                   DistanceToSegment(c, a, b), DistanceToSegment(d, a, b)});
}

bool InsideConvex(const LanePoint& point, const std::vector<LanePoint>& polygon,
                  double margin) {
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto& a = polygon[i];
    const auto& b = polygon[(i + 1) % polygon.size()];
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    if (Cross(a, b, point) < margin * length - 1e-9) return false;
  }
  return true;
}

double SegmentPolygonDistance(const LanePoint& a, const LanePoint& b,
                              const std::vector<LanePoint>& polygon) {
  if (InsideConvex(a, polygon, 0.0) || InsideConvex(b, polygon, 0.0))
    return 0.0;
  double distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < polygon.size(); ++i) {
    distance = std::min(
        distance,
        SegmentDistance(a, b, polygon[i], polygon[(i + 1) % polygon.size()]));
  }
  return distance;
}

bool ValidConvex(const std::vector<LanePoint>& polygon) {
  if (polygon.size() < 3 || polygon.size() > 2000) return false;
  double area = 0.0;
  int sign = 0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto& a = polygon[i];
    const auto& b = polygon[(i + 1) % polygon.size()];
    const auto& c = polygon[(i + 2) % polygon.size()];
    if (!Finite(a) || std::hypot(b.x - a.x, b.y - a.y) < 0.01) return false;
    area += a.x * b.y - a.y * b.x;
    const double cross = Cross(a, b, c);
    if (std::abs(cross) <= 1e-9) continue;
    const int current = cross > 0.0 ? 1 : -1;
    if (sign != 0 && sign != current) return false;
    sign = current;
  }
  return sign > 0 && area > 1e-3;
}

LanePoint DiscCenter(const EnvironmentPose& point, double offset) {
  return {point.x + offset * std::cos(point.heading),
          point.y + offset * std::sin(point.heading)};
}

bool ValidVehicle(const VehicleEnvelope& vehicle) {
  return std::isfinite(vehicle.front) && vehicle.front > 0.0 &&
         std::isfinite(vehicle.rear) && vehicle.rear >= 0.0 &&
         std::isfinite(vehicle.half_width) && vehicle.half_width > 0.0 &&
         std::isfinite(vehicle.clearance) && vehicle.clearance >= 0.0;
}

bool ValidPose(const EnvironmentPose& pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.heading);
}

}  // namespace

common::Status ValidateEnvironment(
    const world_model::DrivableEnvironment& environment,
    const world_model::SceneSource& scene_source, double now) {
  const auto& source = environment.source;
  if (source.frame_id != scene_source.frame_id ||
      source.clock_id != scene_source.clock_id ||
      source.session != scene_source.session ||
      source.generation != scene_source.generation ||
      source.health != world_model::SceneHealth::HEALTHY ||
      source.sequence == 0 || !std::isfinite(source.measurement_time) ||
      !std::isfinite(source.publication_time) ||
      !std::isfinite(source.valid_until) || source.measurement_time < 0.0 ||
      source.measurement_time > source.publication_time ||
      source.publication_time > now || now >= source.valid_until ||
      source.valid_until < scene_source.valid_until ||
      !std::isfinite(environment.position_error) ||
      environment.position_error < 0.0 ||
      !ValidConvex(environment.free_space) || environment.curbs.size() > 200 ||
      environment.obstacles.size() > 200) {
    return Error("invalid source, uncertainty or affirmative free space");
  }
  std::unordered_set<uint64_t> ids;
  for (const auto& curb : environment.curbs) {
    if (curb.id == 0 || !ids.insert(curb.id).second || curb.points.size() < 2 ||
        curb.points.size() > 2000)
      return Error("invalid or duplicate curb");
    for (const auto& point : curb.points)
      if (!Finite(point)) return Error("nonfinite curb");
  }
  ids.clear();
  for (const auto& obstacle : environment.obstacles) {
    if (obstacle.id == 0 || !ids.insert(obstacle.id).second ||
        !ValidConvex(obstacle.points))
      return Error("invalid or duplicate static obstacle");
  }
  return common::Status::OK();
}

common::Status CheckEnvironmentSweep(
    const world_model::DrivableEnvironment& environment,
    const VehicleEnvelope& vehicle, const EnvironmentPose& from,
    const EnvironmentPose& to) {
  if (!ValidVehicle(vehicle) || !ValidPose(from) || !ValidPose(to) ||
      !std::isfinite(environment.position_error) ||
      environment.position_error < 0.0 ||
      !ValidConvex(environment.free_space)) {
    return Error("invalid vehicle, pose or affirmative free space");
  }
  for (const auto& curb : environment.curbs) {
    if (curb.points.size() < 2) return Error("invalid curb geometry");
    for (const auto& point : curb.points)
      if (!Finite(point)) return Error("nonfinite curb geometry");
  }
  for (const auto& obstacle : environment.obstacles)
    if (!ValidConvex(obstacle.points))
      return Error("invalid static obstacle geometry");
  const double cover =
      std::hypot((vehicle.front + vehicle.rear) / 6.0, vehicle.half_width);
  const double heading_delta = std::atan2(std::sin(to.heading - from.heading),
                                          std::cos(to.heading - from.heading));
  const double rotation_sweep =
      std::max(vehicle.front, vehicle.rear) * std::abs(heading_delta) * 0.5;
  const double margin =
      cover + vehicle.clearance + environment.position_error + rotation_sweep;
  if (!std::isfinite(margin)) return Error("nonfinite vehicle sweep expansion");
  for (int i = 0; i < 3; ++i) {
    const double offset =
        -vehicle.rear + (i + 0.5) * (vehicle.front + vehicle.rear) / 3.0;
    const LanePoint a = DiscCenter(from, offset);
    const LanePoint b = DiscCenter(to, offset);
    if (!InsideConvex(a, environment.free_space, margin) ||
        !InsideConvex(b, environment.free_space, margin)) {
      return Error("vehicle sweep leaves measured free space");
    }
    for (const auto& curb : environment.curbs) {
      for (size_t j = 1; j < curb.points.size(); ++j) {
        if (SegmentDistance(a, b, curb.points[j - 1], curb.points[j]) <=
            margin) {
          return Error("vehicle sweep intersects curb restriction");
        }
      }
    }
    for (const auto& obstacle : environment.obstacles) {
      if (SegmentPolygonDistance(a, b, obstacle.points) <= margin)
        return Error("vehicle sweep intersects static obstacle");
    }
  }
  return common::Status::OK();
}

common::Status CheckCorridorFootprint(const LocalCorridor& corridor,
                                      const VehicleEnvelope& vehicle,
                                      const EnvironmentPose& pose) {
  if (corridor.centerline.size() < 2 ||
      corridor.left_boundary.size() != corridor.centerline.size() ||
      corridor.right_boundary.size() != corridor.centerline.size() ||
      !std::isfinite(corridor.half_width) || corridor.half_width <= 0.0 ||
      !std::isfinite(corridor.position_error_bound) ||
      corridor.position_error_bound < 0.0 || !ValidVehicle(vehicle) ||
      !ValidPose(pose)) {
    return Error("invalid vehicle or corridor footprint query");
  }
  const double radius =
      std::hypot((vehicle.front + vehicle.rear) / 6.0, vehicle.half_width);
  if (!std::isfinite(radius))
    return Error("nonfinite lane footprint expansion");
  const CorridorPath path(corridor);
  for (int i = 0; i < 3; ++i) {
    const double offset =
        -vehicle.rear + (i + 0.5) * (vehicle.front + vehicle.rear) / 3.0;
    const common::math::Vec2d center(pose.x + offset * std::cos(pose.heading),
                                     pose.y + offset * std::sin(pose.heading));
    const double distance = path.Project(center).distance;
    if (!std::isfinite(distance) ||
        distance + radius + vehicle.clearance + corridor.position_error_bound >=
            corridor.half_width) {
      return Error("vehicle footprint leaves measured lane corridor");
    }
  }
  return common::Status::OK();
}

}  // namespace local_planning
}  // namespace apollo
