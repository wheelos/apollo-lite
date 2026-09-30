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

#include "modules/world_model/local_map/drivable_environment.h"

#include <cmath>
#include <unordered_set>
#include <utility>

namespace apollo {
namespace world_model {
namespace {

common::Status Error(const std::string& reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR,
                        "drivable environment: " + reason);
}

bool Finite(const LanePoint& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::abs(point.x) < 1e5 && std::abs(point.y) < 1e5;
}

LanePoint Transform(const LanePoint& point, const RelativePose& pose) {
  return {pose.x + std::cos(pose.heading) * point.x -
              std::sin(pose.heading) * point.y,
          pose.y + std::sin(pose.heading) * point.x +
              std::cos(pose.heading) * point.y};
}

double Cross(const LanePoint& a, const LanePoint& b, const LanePoint& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool ConvexCounterClockwise(const std::vector<LanePoint>& polygon) {
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

common::Status TransformPolyline(const ObservedPolyline& input,
                                 const RelativePose& pose,
                                 ObservedPolyline* output) {
  if (input.id == 0 || input.points.size() < 2 || input.points.size() > 2000)
    return Error("invalid curb identity or extent");
  output->id = input.id;
  for (size_t i = 0; i < input.points.size(); ++i) {
    const auto& point = input.points[i];
    if (!Finite(point)) return Error("nonfinite curb geometry");
    if (i > 0 && std::hypot(point.x - input.points[i - 1].x,
                            point.y - input.points[i - 1].y) < 0.01)
      return Error("degenerate curb segment");
    output->points.push_back(Transform(point, pose));
  }
  return common::Status::OK();
}

common::Status TransformPolygon(const ObservedPolygon& input,
                                const RelativePose& pose,
                                ObservedPolygon* output) {
  if (input.id == 0 || !ConvexCounterClockwise(input.points))
    return Error("invalid obstacle identity or unsupported polygon");
  output->id = input.id;
  for (const auto& point : input.points)
    output->points.push_back(Transform(point, pose));
  return common::Status::OK();
}

}  // namespace

common::Status ValidateDrivableEnvironment(
    const DrivableEnvironment& environment) {
  const auto& source = environment.source;
  if (source.frame_id.empty() || source.clock_id.empty() ||
      source.session.empty() || source.generation == 0 ||
      source.sequence == 0 || source.health != SceneHealth::HEALTHY ||
      !std::isfinite(source.measurement_time) ||
      !std::isfinite(source.publication_time) ||
      !std::isfinite(source.valid_until) || source.measurement_time < 0.0 ||
      source.measurement_time > source.publication_time ||
      source.publication_time >= source.valid_until ||
      !std::isfinite(environment.position_error) ||
      environment.position_error < 0.0 ||
      !ConvexCounterClockwise(environment.free_space) ||
      environment.curbs.size() > 200 || environment.obstacles.size() > 200) {
    return Error("invalid source, uncertainty or free-space geometry");
  }
  std::unordered_set<uint64_t> curb_ids;
  for (const auto& curb : environment.curbs) {
    if (curb.id == 0 || !curb_ids.insert(curb.id).second ||
        curb.points.size() < 2 || curb.points.size() > 2000)
      return Error("invalid curb identity or extent");
    for (size_t i = 0; i < curb.points.size(); ++i) {
      if (!Finite(curb.points[i])) return Error("nonfinite curb geometry");
      if (i > 0 && std::hypot(curb.points[i].x - curb.points[i - 1].x,
                              curb.points[i].y - curb.points[i - 1].y) < 0.01)
        return Error("degenerate curb segment");
    }
  }
  std::unordered_set<uint64_t> obstacle_ids;
  for (const auto& obstacle : environment.obstacles) {
    if (obstacle.id == 0 || !obstacle_ids.insert(obstacle.id).second ||
        !ConvexCounterClockwise(obstacle.points))
      return Error("invalid obstacle identity or geometry");
  }
  return common::Status::OK();
}

common::Status BuildDrivableEnvironment(
    const SceneSource& source, const EnvironmentObservation& observation,
    const RelativePose& pose, const std::string& odom_frame,
    double max_position_error, DrivableEnvironment* environment) {
  if (environment == nullptr) return Error("output required");
  *environment = {};
  if (source.frame_id.empty() || odom_frame.empty() ||
      source.clock_id.empty() || source.session.empty() ||
      source.generation == 0 || source.sequence == 0 ||
      source.health != SceneHealth::HEALTHY ||
      !std::isfinite(source.measurement_time) ||
      !std::isfinite(source.publication_time) ||
      !std::isfinite(source.valid_until) || source.measurement_time < 0.0 ||
      source.measurement_time > source.publication_time ||
      source.publication_time >= source.valid_until ||
      source.session != observation.session ||
      source.generation != observation.generation ||
      source.sequence != observation.sequence ||
      source.measurement_time != observation.measurement_time ||
      pose.time != observation.measurement_time || !std::isfinite(pose.x) ||
      !std::isfinite(pose.y) || !std::isfinite(pose.heading) ||
      !std::isfinite(max_position_error) || max_position_error <= 0.0 ||
      !std::isfinite(observation.position_error) ||
      observation.position_error < 0.0 || !std::isfinite(pose.position_error) ||
      pose.position_error < 0.0 ||
      observation.position_error + pose.position_error > max_position_error ||
      !ConvexCounterClockwise(observation.free_space) ||
      observation.curbs.size() > 200 || observation.obstacles.size() > 200) {
    return Error("invalid provenance, uncertainty or free-space coverage");
  }
  environment->source = source;
  environment->source.frame_id = odom_frame;
  environment->position_error =
      observation.position_error + pose.position_error;
  for (const auto& point : observation.free_space)
    environment->free_space.push_back(Transform(point, pose));
  std::unordered_set<uint64_t> curb_ids;
  for (const auto& curb : observation.curbs) {
    if (!curb_ids.insert(curb.id).second)
      return Error("duplicate curb identity");
    ObservedPolyline transformed;
    auto status = TransformPolyline(curb, pose, &transformed);
    if (!status.ok()) return status;
    environment->curbs.push_back(std::move(transformed));
  }
  std::unordered_set<uint64_t> obstacle_ids;
  for (const auto& obstacle : observation.obstacles) {
    if (!obstacle_ids.insert(obstacle.id).second)
      return Error("duplicate obstacle identity");
    ObservedPolygon transformed;
    auto status = TransformPolygon(obstacle, pose, &transformed);
    if (!status.ok()) return status;
    environment->obstacles.push_back(std::move(transformed));
  }
  return ValidateDrivableEnvironment(*environment);
}

}  // namespace world_model
}  // namespace apollo
