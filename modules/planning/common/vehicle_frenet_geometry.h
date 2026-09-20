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

#include "modules/planning/common/planning_geometry_adapter.h"

namespace apollo {
namespace planning {

// Planning-only conversion from vehicle footprint bounds to reference-line
// s/l semantics. VehicleGeometryModel remains responsible for vehicle shape;
// this class is the only owner of stop alignment and s/l occupancy meaning.
// It never selects the physical reference point itself -- that binding
// comes from the PlanningGeometryAdapter supplied by the caller (typically
// sourced from the planning state snapshot that generated the trajectory
// being interpreted).
class VehicleFrenetGeometry {
 public:
  explicit VehicleFrenetGeometry(
      const PlanningGeometryAdapter& geometry_adapter)
      : geometry_adapter_(geometry_adapter) {}

  double ComputeStopReferenceS(
      double target_s, double stop_margin = 0.0,
      common::TravelDirection travel_direction =
          common::TravelDirection::TRAVEL_DIRECTION_FORWARD) const;

  std::pair<double, double> GetOccupancySRange(double reference_s) const;

  std::pair<double, double> GetOccupancyLRange(double reference_l) const;

 private:
  PlanningGeometryAdapter geometry_adapter_;
};

}  // namespace planning
}  // namespace apollo
