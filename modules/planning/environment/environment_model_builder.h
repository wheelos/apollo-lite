/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include "modules/planning/common/local_view.h"
#include "modules/planning/environment/environment_model.h"

namespace apollo {
namespace planning {

class EnvironmentModelBuilder {
 public:
  EnvironmentModel Build(const LocalView& local_view) const;

 private:
  VehicleStateSnapshot BuildVehicleStateSnapshot(
      const LocalView& local_view) const;
  MissionContextSnapshot BuildMissionContextSnapshot(
      const LocalView& local_view) const;
  RouteContext BuildRouteContext(const LocalView& local_view) const;
  LocalTopology BuildLocalTopology(const LocalView& local_view,
                                   const RouteContext& route) const;
  DrivableAreaModel BuildDrivableAreaModel(const LocalView& local_view) const;
  ParkingContext BuildParkingContext(const MissionContextSnapshot& mission,
                                     const RouteContext& route,
                                     const LocalTopology& local_topology) const;
  OpenSpaceContext BuildOpenSpaceContext(
      const MissionContextSnapshot& mission,
      const DrivableAreaModel& drivable_area,
      const ParkingContext& parking) const;
  RegulatoryContext BuildRegulatoryContext(const LocalView& local_view) const;
  DynamicObjectContext BuildDynamicObjectContext(
      const LocalView& local_view) const;
  SourceHealth BuildSourceHealth(const LocalView& local_view) const;
};

}  // namespace planning
}  // namespace apollo
