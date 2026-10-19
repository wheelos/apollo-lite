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

#include "modules/open_space_planning/parking/parking_slot.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace apollo {
namespace open_space_planning {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGeometryTolerance = 1e-6;

double NormalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double AcuteAngle(double angle) {
  double result = std::fabs(NormalizeAngle(angle));
  return result > kPi / 2.0 ? kPi - result : result;
}

ParkingSlotType InferType(const double slot_heading,
                          const double lane_heading) {
  const double angle = AcuteAngle(slot_heading - lane_heading);
  if (angle < kPi / 6.0) {
    return ParkingSlotType::kParallel;
  }
  if (angle > 5.0 * kPi / 12.0) {
    return ParkingSlotType::kPerpendicular;
  }
  return ParkingSlotType::kAngled;
}

}  // namespace

Status BuildParkingSlot(const ParkingSlotInput& input, ParkingSlot* slot) {
  if (slot == nullptr) {
    return {StatusCode::kInvalidInput, "parking slot output is null"};
  }
  if (input.corners.size() != 4) {
    return {StatusCode::kInvalidInput,
            "parking slot must contain exactly four corner points"};
  }
  if (!input.has_lane_heading) {
    return {StatusCode::kInvalidInput,
            "lane heading is required to classify the parking slot"};
  }
  if (!std::isfinite(input.heading) || !std::isfinite(input.lane_heading)) {
    return {StatusCode::kInvalidInput,
            "parking slot or lane heading is invalid"};
  }
  if (input.opening_edge_index < 0 || input.opening_edge_index >= 4) {
    return {StatusCode::kInvalidInput,
            "parking slot opening edge must be explicitly identified"};
  }
  for (const auto& point : input.corners) {
    if (!std::isfinite(point.x()) || !std::isfinite(point.y())) {
      return {StatusCode::kInvalidInput,
              "parking slot contains invalid corner geometry"};
    }
  }
  double twice_area = 0.0;
  for (std::size_t i = 0; i < input.corners.size(); ++i) {
    const auto& point = input.corners[i];
    const auto& next = input.corners[(i + 1) % input.corners.size()];
    twice_area += point.x() * next.y() - point.y() * next.x();
  }
  if (std::fabs(twice_area) <= kGeometryTolerance) {
    return {StatusCode::kInvalidInput,
            "parking slot corner polygon has zero area"};
  }

  ParkingSlot result;
  result.id = input.id;
  result.heading = NormalizeAngle(input.heading);
  result.lane_heading = NormalizeAngle(input.lane_heading);
  result.type = InferType(result.heading, result.lane_heading);
  result.polygon = input.corners;
  result.opening_edge = {
      input.corners[static_cast<std::size_t>(input.opening_edge_index)],
      input.corners[static_cast<std::size_t>((input.opening_edge_index + 1) %
                                             4)]};
  result.opening_center =
      (result.opening_edge[0] + result.opening_edge[1]) * 0.5;
  result.center = common::math::Vec2d(0.0, 0.0);
  for (const auto& point : input.corners) {
    result.center += point;
  }
  result.center /= static_cast<double>(input.corners.size());
  const std::size_t opening_start =
      static_cast<std::size_t>(input.opening_edge_index);
  const std::size_t rear_start = (opening_start + 2) % input.corners.size();
  result.rear_center =
      (input.corners[rear_start] +
       input.corners[(rear_start + 1) % input.corners.size()]) *
      0.5;
  const double opening_width = result.opening_edge[1] - result.opening_edge[0];
  const double rear_width =
      input.corners[(rear_start + 1) % input.corners.size()] -
      input.corners[rear_start];
  result.width = 0.5 * (opening_width.Length() + rear_width.Length());
  result.depth = (result.rear_center - result.opening_center).Length();
  if (result.width <= kGeometryTolerance ||
      result.depth <= kGeometryTolerance) {
    return {StatusCode::kInvalidInput,
            "parking slot opening or depth has zero length"};
  }
  *slot = std::move(result);
  return Status::Ok();
}

}  // namespace open_space_planning
}  // namespace apollo
