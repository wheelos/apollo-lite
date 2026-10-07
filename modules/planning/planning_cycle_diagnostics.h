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
#include "modules/planning/planning_runtime_context.h"

namespace apollo {
namespace planning {

class PlanningCycleDiagnostics {
 public:
  void LogCycle(const PlanningCoordinatorState& coordinator_state,
                const PlanningSemanticSummary& semantic_summary,
                const HybridManeuverSummary& hybrid_summary,
                const std::string& reason);

 private:
  std::string last_logged_command_id_;
  PlanningMode last_logged_mode_ = MODE_UNKNOWN;
  PlanningShellType last_logged_shell_ = PLANNING_SHELL_UNKNOWN;
};

}  // namespace planning
}  // namespace apollo
