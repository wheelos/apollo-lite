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

#include "modules/planning/planning_cycle_diagnostics.h"

#include <sstream>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

void PlanningCycleDiagnostics::LogCycle(
    const PlanningCoordinatorState& coordinator_state,
    const PlanningSemanticSummary& semantic_summary,
    const HybridManeuverSummary& hybrid_summary, const std::string& reason) {
  const bool should_log_info =
      coordinator_state.command_id != last_logged_command_id_ ||
      coordinator_state.resolved_mode != last_logged_mode_ ||
      coordinator_state.active_shell != last_logged_shell_ ||
      coordinator_state.transition_pending ||
      semantic_summary.runtime_state != RUNTIME_RUNNING ||
      hybrid_summary.handoff_state != HANDOFF_STATE_NONE;

  const auto format_summary = [&]() {
    std::ostringstream stream;
    stream << "planning cycle: cmd=" << coordinator_state.command_id
           << " scene="
           << PlanningSceneType_Name(coordinator_state.active_scene)
           << " mode=" << PlanningMode_Name(coordinator_state.resolved_mode)
           << " shell="
           << PlanningShellType_Name(coordinator_state.active_shell)
           << " runtime="
           << RuntimeState_Name(semantic_summary.runtime_state);
    if (hybrid_summary.active_maneuver != HYBRID_MANEUVER_NONE) {
      stream << " hybrid="
             << HybridManeuverType_Name(hybrid_summary.active_maneuver) << "/"
             << ManeuverSegmentType_Name(hybrid_summary.active_segment) << "/"
             << HandoffState_Name(hybrid_summary.handoff_state);
    }
    if (!reason.empty()) {
      stream << " reason=" << reason;
    }
    return stream.str();
  };

  if (should_log_info) {
    AINFO << format_summary();
    last_logged_command_id_ = coordinator_state.command_id;
    last_logged_mode_ = coordinator_state.resolved_mode;
    last_logged_shell_ = coordinator_state.active_shell;
    return;
  }
  ADEBUG << format_summary();
}

}  // namespace planning
}  // namespace apollo
