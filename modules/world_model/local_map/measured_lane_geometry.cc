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

#include "modules/world_model/local_map/measured_lane_geometry.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace apollo {
namespace world_model {
namespace {

common::Status Error(const std::string& reason) {
  return common::Status(common::ErrorCode::PLANNING_ERROR,
                        "measured lane geometry: " + reason);
}

bool FiniteBoundary(const MeasuredCubicBoundary& boundary) {
  return std::isfinite(boundary.c0) && std::isfinite(boundary.c1) &&
         std::isfinite(boundary.c2) && std::isfinite(boundary.c3) &&
         std::isfinite(boundary.start_x) && std::isfinite(boundary.end_x) &&
         boundary.start_x < boundary.end_x;
}

double Evaluate(const MeasuredCubicBoundary& boundary, double x) {
  return ((boundary.c3 * x + boundary.c2) * x + boundary.c1) * x + boundary.c0;
}

}  // namespace

common::Status SampleMeasuredCubicBoundary(
    const MeasuredCubicBoundary& boundary, double start_x, double end_x,
    double spacing, std::vector<MeasuredBoundarySample>* samples) {
  if (samples == nullptr) return Error("sample output required");
  samples->clear();
  if (!FiniteBoundary(boundary) || !std::isfinite(start_x) ||
      !std::isfinite(end_x) || !std::isfinite(spacing) || spacing < 0.05 ||
      spacing > 2.0 || start_x < boundary.start_x || end_x > boundary.end_x ||
      start_x >= end_x || (end_x - start_x) / spacing > 2000.0) {
    return Error("invalid measured interval or sampling");
  }
  double station = 0.0;
  for (double x = start_x;; x = std::min(x + spacing, end_x)) {
    const double y = Evaluate(boundary, x);
    const double dy =
        (3.0 * boundary.c3 * x + 2.0 * boundary.c2) * x + boundary.c1;
    const double d2y = 6.0 * boundary.c3 * x + 2.0 * boundary.c2;
    if (!std::isfinite(y) || !std::isfinite(dy) || !std::isfinite(d2y))
      return Error("nonfinite sampled geometry");
    if (!samples->empty()) {
      station +=
          std::hypot(x - samples->back().point.x, y - samples->back().point.y);
    }
    samples->push_back({{x, y},
                        station,
                        std::atan2(dy, 1.0),
                        d2y / std::pow(1.0 + dy * dy, 1.5)});
    if (x >= end_x) break;
  }
  return common::Status::OK();
}

common::Status BuildMeasuredLaneObservation(const MeasuredCubicLane& input,
                                            LaneObservation* observation) {
  if (observation == nullptr) return Error("lane output required");
  *observation = {};
  if (input.session.empty() || input.generation == 0 || input.sequence == 0 ||
      !std::isfinite(input.measurement_time) || input.measurement_time < 0.0 ||
      !std::isfinite(input.position_error) || input.position_error < 0.0 ||
      !input.forward_confirmed || !FiniteBoundary(input.left) ||
      !FiniteBoundary(input.right)) {
    return Error("invalid lane provenance or boundary coefficients");
  }
  const double start_x = std::max(input.left.start_x, input.right.start_x);
  const double end_x = std::min(input.left.end_x, input.right.end_x);
  std::vector<MeasuredBoundarySample> left;
  std::vector<MeasuredBoundarySample> right;
  auto status = SampleMeasuredCubicBoundary(input.left, start_x, end_x,
                                            input.sample_spacing, &left);
  if (status.ok()) {
    status = SampleMeasuredCubicBoundary(input.right, start_x, end_x,
                                         input.sample_spacing, &right);
  }
  if (!status.ok()) return status;
  if (left.size() != right.size() || left.size() < 2)
    return Error("paired measured samples required");
  for (size_t i = 0; i < left.size(); ++i) {
    if (left[i].point.y <= right[i].point.y)
      return Error("left and right measured boundaries cross");
  }
  observation->session = input.session;
  observation->generation = input.generation;
  observation->sequence = input.sequence;
  observation->measurement_time = input.measurement_time;
  observation->position_error = input.position_error;
  observation->forward_confirmed = true;
  for (const auto& sample : left) observation->left.push_back(sample.point);
  for (const auto& sample : right) observation->right.push_back(sample.point);
  return common::Status::OK();
}

}  // namespace world_model
}  // namespace apollo
