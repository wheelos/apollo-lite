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

#include <optional>
#include <string>

#include "modules/local_planning/planning_context/reference_line.h"

namespace apollo {
namespace local_planning {

// Optimizer-independent geometric output. AREA intentionally has no reference
// line or fabricated longitudinal/lateral coordinates.
struct PlanningGeometry {
  world_model::SceneSource source;
  world_model::SceneMode mode = world_model::SceneMode::INVALID;
  world_model::SceneCapabilities capabilities;
  world_model::DrivableEnvironment environment;
  std::optional<ReferenceGeometry> reference;
  std::string mode_reason;
};

common::Status BuildPlanningGeometry(
    const world_model::LocalScene& scene, const OdometryInput& ego, double now,
    double rear, double front, double continuity_tolerance,
    const ReferenceGeometry* previous_reference, PlanningGeometry* output);

}  // namespace local_planning
}  // namespace apollo
