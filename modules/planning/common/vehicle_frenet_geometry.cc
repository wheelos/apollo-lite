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

#include "modules/planning/common/vehicle_frenet_geometry.h"

namespace apollo {
namespace planning {

double VehicleFrenetGeometry::ComputeStopReferenceS(
    const double target_s, const double stop_margin,
    const common::TravelDirection travel_direction,
    const common::ReferencePoint reference_point) const {
  const common::VehicleBounds bounds = geometry_model_.Bounds(reference_point);
  if (travel_direction == common::TravelDirection::TRAVEL_DIRECTION_REVERSE) {
    return target_s + stop_margin + bounds.rear;
  }
  return target_s - stop_margin - bounds.front;
}

std::pair<double, double> VehicleFrenetGeometry::GetOccupancySRange(
    const double reference_s,
    const common::ReferencePoint reference_point) const {
  const common::VehicleBounds bounds = geometry_model_.Bounds(reference_point);
  return {reference_s - bounds.rear, reference_s + bounds.front};
}

std::pair<double, double> VehicleFrenetGeometry::GetOccupancyLRange(
    const double reference_l) const {
  const common::VehicleBounds bounds = geometry_model_.Bounds();
  return {reference_l - bounds.right, reference_l + bounds.left};
}

}  // namespace planning
}  // namespace apollo
