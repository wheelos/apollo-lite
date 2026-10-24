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

#include "modules/planning/common/hybrid_maneuver_supervisor.h"

namespace apollo {
namespace planning {

namespace {

HybridManeuverType InferManeuverType(PlanningSceneType scene) {
  switch (scene) {
    case SCENE_PARK_IN:
      return HYBRID_MANEUVER_PARK_IN;
    case SCENE_PULL_OVER:
      return HYBRID_MANEUVER_PULL_OVER;
    case SCENE_PULL_OUT:
      return HYBRID_MANEUVER_PULL_OUT;
    case SCENE_UNKNOWN:
    case SCENE_LANE_CRUISE:
    case SCENE_DOCK:
    case SCENE_SUMMON:
    case SCENE_EMERGENCY_STOP:
    case SCENE_HOLD:
    default:
      return HYBRID_MANEUVER_NONE;
  }
}

bool IsHybridManeuver(HybridManeuverType maneuver) {
  return maneuver != HYBRID_MANEUVER_NONE;
}

bool IsTerminalRuntimeState(RuntimeState runtime_state) {
  return runtime_state == RUNTIME_HOLDING || runtime_state == RUNTIME_COMPLETED;
}

}  // namespace

HybridManeuverSummary HybridManeuverSupervisor::Evaluate(
    const PlanningCoordinatorState& coordinator_state,
    const PlanningStatus* planning_status, RuntimeState runtime_state) const {
  HybridManeuverSummary summary;
  summary.active_maneuver = InferManeuverType(coordinator_state.active_scene);
  if (!IsHybridManeuver(summary.active_maneuver)) {
    return summary;
  }

  const auto active_shell = coordinator_state.active_shell;
  const auto desired_shell = coordinator_state.desired_shell;
  const bool transition_pending = coordinator_state.transition_pending;
  const bool entering_open_space =
      desired_shell == PLANNING_SHELL_OPEN_SPACE &&
      active_shell != PLANNING_SHELL_OPEN_SPACE;
  const bool exiting_open_space =
      active_shell == PLANNING_SHELL_OPEN_SPACE &&
      desired_shell != PLANNING_SHELL_OPEN_SPACE &&
      desired_shell != PLANNING_SHELL_UNKNOWN;
  const bool just_committed_exit =
      coordinator_state.previous_shell == PLANNING_SHELL_OPEN_SPACE &&
      active_shell != PLANNING_SHELL_OPEN_SPACE;

  const auto scenario_status =
      planning_status != nullptr && planning_status->has_scenario()
          ? &planning_status->scenario()
          : nullptr;
  const auto stage_type = scenario_status != nullptr && scenario_status->has_stage_type()
                              ? scenario_status->stage_type()
                              : NO_STAGE;

  switch (summary.active_maneuver) {
    case HYBRID_MANEUVER_PARK_IN:
      if (active_shell == PLANNING_SHELL_OPEN_SPACE) {
        summary.active_segment =
            IsTerminalRuntimeState(runtime_state) ? MANEUVER_SEGMENT_TERMINAL_HOLD
                                                  : MANEUVER_SEGMENT_LOCAL_MANEUVER;
      } else if (transition_pending && entering_open_space) {
        summary.active_segment = MANEUVER_SEGMENT_PREPARE_ENTRY;
        summary.handoff_state = HANDOFF_STATE_PENDING_APPROVAL;
      } else if (entering_open_space) {
        summary.active_segment = MANEUVER_SEGMENT_PREPARE_ENTRY;
        summary.handoff_state = HANDOFF_STATE_PREPARING;
      } else {
        summary.active_segment = MANEUVER_SEGMENT_GUIDED_APPROACH;
      }
      break;
    case HYBRID_MANEUVER_PULL_OVER:
      if (active_shell == PLANNING_SHELL_OPEN_SPACE) {
        summary.active_segment =
            IsTerminalRuntimeState(runtime_state) ? MANEUVER_SEGMENT_TERMINAL_HOLD
                                                  : MANEUVER_SEGMENT_LOCAL_MANEUVER;
      } else if (transition_pending && entering_open_space) {
        summary.active_segment = MANEUVER_SEGMENT_PREPARE_ENTRY;
        summary.handoff_state = HANDOFF_STATE_PENDING_APPROVAL;
      } else if (entering_open_space ||
                 stage_type == PULL_OVER_RETRY_APPROACH_PARKING) {
        summary.active_segment = MANEUVER_SEGMENT_PREPARE_ENTRY;
        summary.handoff_state = HANDOFF_STATE_PREPARING;
      } else if (IsTerminalRuntimeState(runtime_state)) {
        summary.active_segment = MANEUVER_SEGMENT_TERMINAL_HOLD;
      } else {
        summary.active_segment = MANEUVER_SEGMENT_GUIDED_APPROACH;
      }
      break;
    case HYBRID_MANEUVER_PULL_OUT:
      if (just_committed_exit) {
        summary.active_segment = MANEUVER_SEGMENT_GUIDED_RESUME;
        summary.handoff_state = HANDOFF_STATE_COMMITTED;
      } else if (active_shell == PLANNING_SHELL_OPEN_SPACE &&
                 stage_type == PARK_AND_GO_CRUISE) {
        summary.active_segment = MANEUVER_SEGMENT_HANDOFF_EXIT;
        summary.handoff_state = transition_pending
                                    ? HANDOFF_STATE_PENDING_APPROVAL
                                    : HANDOFF_STATE_READY;
      } else if (active_shell == PLANNING_SHELL_OPEN_SPACE &&
                 (exiting_open_space || transition_pending)) {
        summary.active_segment = MANEUVER_SEGMENT_HANDOFF_EXIT;
        summary.handoff_state = HANDOFF_STATE_PENDING_APPROVAL;
      } else if (active_shell == PLANNING_SHELL_OPEN_SPACE) {
        summary.active_segment = MANEUVER_SEGMENT_LOCAL_MANEUVER;
      } else if (runtime_state == RUNTIME_RUNNING ||
                 runtime_state == RUNTIME_DEGRADED) {
        summary.active_segment = MANEUVER_SEGMENT_GUIDED_RESUME;
      }
      break;
    case HYBRID_MANEUVER_NONE:
    default:
      break;
  }

  if (summary.handoff_state != HANDOFF_STATE_NONE) {
    if (!coordinator_state.reason.empty()) {
      summary.handoff_reason = coordinator_state.reason;
    } else if (summary.active_maneuver == HYBRID_MANEUVER_PULL_OUT &&
               summary.handoff_state == HANDOFF_STATE_READY) {
      summary.handoff_reason =
          "open-space local departure ready for guided-route re-entry";
    } else if (summary.active_segment == MANEUVER_SEGMENT_PREPARE_ENTRY) {
      summary.handoff_reason = "preparing hybrid maneuver shell entry";
    } else if (summary.active_segment == MANEUVER_SEGMENT_HANDOFF_EXIT) {
      summary.handoff_reason = "preparing hybrid maneuver shell exit";
    }
  }

  return summary;
}

void HybridManeuverSupervisor::Apply(
    const HybridManeuverSummary& summary,
    PlanningExecutionContext* execution) const {
  if (execution == nullptr || !IsHybridManeuver(summary.active_maneuver)) {
    return;
  }
  auto* status = execution->mutable_hybrid_maneuver();
  status->set_active_maneuver(summary.active_maneuver);
  status->set_active_segment(summary.active_segment);
  status->set_handoff_state(summary.handoff_state);
  if (!summary.handoff_reason.empty()) {
    status->set_handoff_reason(summary.handoff_reason);
  }
}

void HybridManeuverSupervisor::Apply(
    const HybridManeuverSummary& summary,
    PlanningRuntimeStatus* runtime_status) const {
  if (runtime_status == nullptr || !IsHybridManeuver(summary.active_maneuver)) {
    return;
  }
  auto* status = runtime_status->mutable_hybrid_maneuver();
  status->set_active_maneuver(summary.active_maneuver);
  status->set_active_segment(summary.active_segment);
  status->set_handoff_state(summary.handoff_state);
  if (!summary.handoff_reason.empty()) {
    status->set_handoff_reason(summary.handoff_reason);
  }
}

}  // namespace planning
}  // namespace apollo
