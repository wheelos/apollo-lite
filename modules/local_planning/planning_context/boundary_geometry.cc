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

#include "modules/local_planning/planning_context/boundary_geometry.h"

#include <cmath>
#include <string>

namespace apollo {
namespace local_planning {
namespace {

common::Status Error(const std::string& reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR,
                        "measured boundaries: " + reason);
}

bool Finite(const Point2& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::abs(point.x) < 1e6 && std::abs(point.y) < 1e6;
}

Point2 Interpolate(const std::vector<Point2>& points,
                   const std::vector<Point2>& center, double station) {
  double s = 0.0;
  for (size_t i = 1; i < center.size(); ++i) {
    const double length = std::hypot(center[i].x - center[i - 1].x,
                                     center[i].y - center[i - 1].y);
    if (station <= s + length) {
      const double t = (station - s) / length;
      return {points[i - 1].x + t * (points[i].x - points[i - 1].x),
              points[i - 1].y + t * (points[i].y - points[i - 1].y)};
    }
    s += length;
  }
  return points.back();
}

}  // namespace

common::Status BuildMeasuredBoundaryLines(const LocalCorridor& corridor,
                                          double start_s, double end_s,
                                          MeasuredBoundaryLines* output) {
  if (output == nullptr) return Error("output required");
  *output = {};
  if (corridor.centerline.size() < 2 ||
      corridor.left_boundary.size() != corridor.centerline.size() ||
      corridor.right_boundary.size() != corridor.centerline.size() ||
      !std::isfinite(start_s) || !std::isfinite(end_s) || start_s < 0.0 ||
      end_s <= start_s) {
    return Error("invalid corridor or station interval");
  }
  double length = 0.0;
  for (size_t i = 1; i < corridor.centerline.size(); ++i) {
    const double step =
        std::hypot(corridor.centerline[i].x - corridor.centerline[i - 1].x,
                   corridor.centerline[i].y - corridor.centerline[i - 1].y);
    if (!Finite(corridor.centerline[i - 1]) ||
        !Finite(corridor.left_boundary[i - 1]) ||
        !Finite(corridor.right_boundary[i - 1]) || !std::isfinite(step) ||
        step < 0.05 || step > 2.0) {
      return Error("invalid paired centerline samples");
    }
    length += step;
    if (!std::isfinite(length)) return Error("nonfinite centerline length");
  }
  if (!Finite(corridor.centerline.back()) ||
      !Finite(corridor.left_boundary.back()) ||
      !Finite(corridor.right_boundary.back())) {
    return Error("nonfinite terminal boundary sample");
  }
  if (end_s > length + 1e-9) return Error("station interval exceeds coverage");

  auto add = [&](double station) {
    output->stations.push_back(station);
    output->center.push_back(
        Interpolate(corridor.centerline, corridor.centerline, station));
    output->left.push_back(
        Interpolate(corridor.left_boundary, corridor.centerline, station));
    output->right.push_back(
        Interpolate(corridor.right_boundary, corridor.centerline, station));
  };
  add(start_s);
  double station = 0.0;
  for (size_t i = 1; i < corridor.centerline.size(); ++i) {
    station +=
        std::hypot(corridor.centerline[i].x - corridor.centerline[i - 1].x,
                   corridor.centerline[i].y - corridor.centerline[i - 1].y);
    if (station > start_s + 1e-9 && station < end_s - 1e-9) add(station);
  }
  add(end_s);
  if (output->center.size() < 2 ||
      output->center.size() != output->left.size() ||
      output->center.size() != output->right.size()) {
    *output = {};
    return Error("incomplete measured interval");
  }
  return common::Status::OK();
}

}  // namespace local_planning
}  // namespace apollo
