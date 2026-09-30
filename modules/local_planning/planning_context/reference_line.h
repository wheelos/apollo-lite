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

#include <vector>

#include "modules/local_planning/planning_context/corridor_selector.h"
#include "modules/local_planning/planning_context/reference_continuity.h"

namespace apollo {
namespace local_planning {

struct ReferenceGeometry {
  world_model::SceneSource source;
  LocalCorridor corridor;
  world_model::DrivableEnvironment environment;
  double ego_s = 0.0;
  double ego_l = 0.0;
  double start_s = 0.0;
  double end_s = 0.0;
  double observed_half_width = 0.0;
  bool continuous = false;
  ReferenceContinuityReport continuity;
  std::vector<double> stations;
  std::vector<Point2> line;
  std::vector<Point2> left;
  std::vector<Point2> right;
};

// Stateless planner-owned geometry construction. The previous *accepted*
// reference is an optional continuity input, not another observation tracker.
common::Status BuildReferenceGeometry(const world_model::LocalScene& scene,
                                      const OdometryInput& ego, double now,
                                      double rear, double front,
                                      double continuity_tolerance,
                                      const ReferenceGeometry* previous,
                                      ReferenceGeometry* output);

}  // namespace local_planning
}  // namespace apollo
