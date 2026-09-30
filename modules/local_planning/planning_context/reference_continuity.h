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

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "modules/local_planning/planning_context/corridor_selector.h"
#include "modules/world_model/local_map/local_scene.h"

namespace apollo {
namespace local_planning {

enum class ContinuityReason {
  NO_PREVIOUS_REFERENCE,
  MODE_CHANGED,
  FRAME_OR_CLOCK_CHANGED,
  ODOM_EPOCH_CHANGED,
  PREVIOUS_REFERENCE_EXPIRED,
  LANE_IDENTITY_CHANGED,
  RESTRICTION_CHANGED,
  NO_USABLE_OVERLAP,
  REUSED_OVERLAP
};

struct ReferenceContinuityReport {
  ContinuityReason reason = ContinuityReason::NO_PREVIOUS_REFERENCE;
  size_t reused_points = 0;
  double first_reused_station = 0.0;
  double last_reused_station = 0.0;
  double maximum_displacement = 0.0;
  std::string detail;
};

struct ReferenceContinuityConfig {
  double maximum_displacement = 0.0;
};

struct PreviousReferenceGeometry {
  world_model::SceneSource source;
  LocalCorridor corridor;
  world_model::DrivableEnvironment environment;
  double observed_half_width = 0.0;
};

common::Status ApplyReferenceContinuity(
    const world_model::LocalScene& scene, double now,
    const ReferenceContinuityConfig& config,
    const PreviousReferenceGeometry* previous, LocalCorridor* current,
    ReferenceContinuityReport* report);

}  // namespace local_planning
}  // namespace apollo
