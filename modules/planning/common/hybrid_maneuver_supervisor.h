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

#include <string>

#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "modules/common_msgs/planning_msgs/planning_runtime_status.pb.h"
#include "modules/planning/planning_runtime_context.h"
#include "modules/planning/proto/planning_status.pb.h"

namespace apollo {
namespace planning {

struct HybridManeuverSummary {
  HybridManeuverType active_maneuver = HYBRID_MANEUVER_NONE;
  ManeuverSegmentType active_segment = MANEUVER_SEGMENT_NONE;
  HandoffState handoff_state = HANDOFF_STATE_NONE;
  std::string handoff_reason;
};

// HybridManeuverSupervisor is the first explicit runtime owner of the maneuver-
// level handoff summary that sits between command semantics and shell switching.
class HybridManeuverSupervisor {
 public:
  HybridManeuverSummary Evaluate(const PlanningCoordinatorState& coordinator_state,
                                 const PlanningStatus* planning_status,
                                 RuntimeState runtime_state) const;

  void Apply(const HybridManeuverSummary& summary,
             PlanningExecutionContext* execution) const;

  void Apply(const HybridManeuverSummary& summary,
             PlanningRuntimeStatus* runtime_status) const;
};

}  // namespace planning
}  // namespace apollo
