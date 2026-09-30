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

#pragma once

#include "modules/common/status/status.h"
#include "modules/local_planning/planning_context/scene.h"
#include "modules/world_model/local_map/local_scene.h"

namespace apollo {
namespace local_planning {

struct VehicleEnvelope {
  double front = 0.0;
  double rear = 0.0;
  double half_width = 0.0;
  double clearance = 0.0;
};

struct EnvironmentPose {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
};

common::Status ValidateEnvironment(
    const world_model::DrivableEnvironment& environment,
    const world_model::SceneSource& scene_source, double now);

common::Status CheckEnvironmentSweep(
    const world_model::DrivableEnvironment& environment,
    const VehicleEnvelope& vehicle, const EnvironmentPose& from,
    const EnvironmentPose& to);

common::Status CheckCorridorFootprint(const LocalCorridor& corridor,
                                      const VehicleEnvelope& vehicle,
                                      const EnvironmentPose& pose);

}  // namespace local_planning
}  // namespace apollo
