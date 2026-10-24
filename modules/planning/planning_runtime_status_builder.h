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

#include "modules/planning/common/hybrid_maneuver_supervisor.h"
#include "modules/planning/common/planning_semantics.h"
#include "modules/planning/environment/capability_extractor.h"
#include "modules/planning/planning_runtime_context.h"
#include "modules/planning/validation/validation_supervisor.h"
#include "wheelos_msgs/planning_msgs/planning_runtime_status.pb.h"

namespace apollo {
namespace planning {

class PlanningRuntimeStatusBuilder {
 public:
  PlanningRuntimeStatus Build(
      const std::string& node_name,
      const PlanningSemanticSummary& semantic_summary,
      const HybridManeuverSummary& hybrid_summary,
      const ValidationResult& validation_result,
      const PlanningCoordinatorState& coordinator_state,
      const PlanningExecutionContext& execution,
      const MissionCommandIdentity& accepted_directive_identity,
      const CapabilitySet* capability_set, const std::string& reason) const;
};

}  // namespace planning
}  // namespace apollo
