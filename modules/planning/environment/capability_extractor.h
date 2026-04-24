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

#include "modules/planning/environment/environment_model.h"

namespace apollo {
namespace planning {

struct CapabilitySet {
  bool has_lane_graph = false;
  bool has_route_semantics = false;
  bool has_local_corridor = false;
  bool has_drivable_area = false;
  bool has_parking_roi = false;
  bool has_goal_pose = false;
  bool has_stop_target = false;
  bool has_regulatory_context = false;
  double topology_confidence = 0.0;
  double drivable_area_confidence = 0.0;
  double target_geometry_confidence = 0.0;
};

class CapabilityExtractor {
 public:
  CapabilitySet Extract(const EnvironmentModel& model) const;
};

}  // namespace planning
}  // namespace apollo
