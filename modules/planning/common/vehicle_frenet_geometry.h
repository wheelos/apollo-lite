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

#pragma once

#include <utility>

#include "modules/common/vehicle_state/vehicle_geometry_model.h"

namespace apollo {
namespace planning {

// Planning-only conversion from vehicle footprint bounds to reference-line
// s/l semantics. VehicleGeometryModel remains responsible for vehicle shape;
// this class is the only owner of stop alignment and s/l occupancy meaning.
// Do not add reference-line distance semantics back to VehicleGeometryModel.
class VehicleFrenetGeometry {
 public:
  explicit VehicleFrenetGeometry(
      const common::VehicleGeometryModel& geometry_model)
      : geometry_model_(geometry_model) {}

  double ComputeStopReferenceS(
      double target_s, double stop_margin = 0.0,
      common::TravelDirection travel_direction =
          common::TravelDirection::TRAVEL_DIRECTION_FORWARD,
      common::ReferencePoint reference_point =
          common::ReferencePoint::REAR_AXLE_CENTER) const;

  std::pair<double, double> GetOccupancySRange(
      double reference_s, common::ReferencePoint reference_point =
                              common::ReferencePoint::REAR_AXLE_CENTER) const;

  std::pair<double, double> GetOccupancyLRange(double reference_l) const;

 private:
  const common::VehicleGeometryModel& geometry_model_;
};

}  // namespace planning
}  // namespace apollo
