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

#include "modules/planning/planning_runtime_status_builder.h"

#include "modules/common/util/message_util.h"

namespace apollo {
namespace planning {

PlanningRuntimeStatus PlanningRuntimeStatusBuilder::Build(
    const std::string& node_name,
    const PlanningSemanticSummary& semantic_summary,
    const HybridManeuverSummary& hybrid_summary,
    const ValidationResult& validation_result,
    const PlanningCoordinatorState& coordinator_state,
    const PlanningExecutionContext& execution,
    const MissionCommandIdentity& accepted_directive_identity,
    const CapabilitySet* capability_set, const std::string& reason) const {
  PlanningRuntimeStatus runtime_status;
  common::util::FillHeader(node_name, &runtime_status);
  runtime_status.set_state(semantic_summary.runtime_state);
  runtime_status.set_active_scene(execution.has_active_scene()
                                      ? execution.active_scene()
                                      : coordinator_state.active_scene);
  runtime_status.set_active_mode(execution.has_active_mode()
                                     ? execution.active_mode()
                                     : coordinator_state.resolved_mode);
  runtime_status.set_requested_mode(execution.has_requested_mode()
                                        ? execution.requested_mode()
                                        : coordinator_state.requested_mode);
  runtime_status.set_active_shell(execution.has_active_shell()
                                      ? execution.active_shell()
                                      : coordinator_state.active_shell);
  runtime_status.set_active_domain(execution.has_active_domain()
                                       ? execution.active_domain()
                                       : coordinator_state.active_domain);
  if (execution.has_execution_channel()) {
    runtime_status.set_execution_channel(execution.execution_channel());
  }
  if (coordinator_state.transition_pending) {
    auto* transition = runtime_status.mutable_transition();
    transition->set_from_mode(coordinator_state.resolved_mode);
    transition->set_to_mode(coordinator_state.desired_mode);
    transition->set_from_shell(coordinator_state.active_shell);
    transition->set_to_shell(coordinator_state.desired_shell);
    transition->set_approved(false);
    transition->set_continuity_hold(coordinator_state.continuity_hold);
    if (!reason.empty()) {
      transition->set_trigger(reason);
    } else if (!coordinator_state.reason.empty()) {
      transition->set_trigger(coordinator_state.reason);
    } else {
      transition->set_trigger("planner shell transition pending");
    }
  } else if (coordinator_state.previous_mode != MODE_UNKNOWN &&
             (coordinator_state.previous_mode !=
                  coordinator_state.resolved_mode ||
              coordinator_state.previous_shell !=
                  coordinator_state.active_shell)) {
    auto* transition = runtime_status.mutable_transition();
    transition->set_from_mode(coordinator_state.previous_mode);
    transition->set_to_mode(coordinator_state.resolved_mode);
    transition->set_from_shell(coordinator_state.previous_shell);
    transition->set_to_shell(coordinator_state.active_shell);
    transition->set_approved(coordinator_state.resolved_mode != MODE_UNKNOWN);
    transition->set_continuity_hold(false);
    if (reason.empty() && !coordinator_state.reason.empty()) {
      transition->set_trigger(coordinator_state.reason);
    } else if (!reason.empty()) {
      transition->set_trigger(reason);
    } else {
      transition->set_trigger("planner shell switched");
    }
  } else if (coordinator_state.requested_mode !=
             coordinator_state.resolved_mode) {
    auto* transition = runtime_status.mutable_transition();
    transition->set_from_mode(coordinator_state.requested_mode);
    transition->set_to_mode(coordinator_state.resolved_mode);
    transition->set_from_shell(
        ResolveShellForMode(coordinator_state.requested_mode));
    transition->set_to_shell(coordinator_state.active_shell);
    if (reason.empty() && !coordinator_state.reason.empty()) {
      transition->set_trigger(coordinator_state.reason);
    }
    transition->set_approved(semantic_summary.runtime_state !=
                                 RUNTIME_REJECTED &&
                             coordinator_state.resolved_mode != MODE_UNKNOWN);
    transition->set_continuity_hold(false);
  }
  if (execution.has_mission_id()) {
    runtime_status.set_mission_id(execution.mission_id());
  } else if (!coordinator_state.mission_id.empty()) {
    runtime_status.set_mission_id(coordinator_state.mission_id);
  }
  if (execution.has_command_id()) {
    runtime_status.set_command_id(execution.command_id());
  } else if (!coordinator_state.command_id.empty()) {
    runtime_status.set_command_id(coordinator_state.command_id);
  }
  if (coordinator_state.mission_identity.has_revision()) {
    runtime_status.mutable_mission_identity()->CopyFrom(
        coordinator_state.mission_identity);
    runtime_status.set_mission_session_state(
        coordinator_state.mission_session_state);
    runtime_status.set_mission_phase(coordinator_state.mission_phase);
    if (coordinator_state.accepted_start.has_snapshot_time_sec()) {
      runtime_status.mutable_accepted_start()->CopyFrom(
          coordinator_state.accepted_start);
    }
    if (accepted_directive_identity.has_revision()) {
      runtime_status.mutable_accepted_directive_identity()->CopyFrom(
          accepted_directive_identity);
    }
    if (coordinator_state.mission_route.has_state()) {
      runtime_status.mutable_mission_route()->CopyFrom(
          coordinator_state.mission_route);
    }
  }
  if (execution.blockers_size() > 0) {
    for (const auto& blocker : execution.blockers()) {
      runtime_status.add_blockers(blocker);
    }
  } else {
    for (const auto& blocker : coordinator_state.blockers) {
      runtime_status.add_blockers(blocker);
    }
  }
  if (reason.empty() && execution.has_reason()) {
    runtime_status.set_reason(execution.reason());
  } else if (reason.empty() && !coordinator_state.reason.empty()) {
    runtime_status.set_reason(coordinator_state.reason);
  }

  if (!reason.empty()) {
    bool has_same_blocker = false;
    for (const auto& blocker : runtime_status.blockers()) {
      if (blocker == reason) {
        has_same_blocker = true;
        break;
      }
    }
    if (!has_same_blocker) {
      runtime_status.add_blockers(reason);
    }
    runtime_status.set_reason(reason);
    if (runtime_status.has_transition() &&
        !runtime_status.transition().has_trigger()) {
      runtime_status.mutable_transition()->set_trigger(reason);
    }
  }

  ApplyPlanningSemanticsToRuntimeStatus(semantic_summary, &runtime_status);
  HybridManeuverSupervisor().Apply(hybrid_summary, &runtime_status);

  if (capability_set != nullptr) {
    auto* capability = runtime_status.mutable_capability();
    capability->set_has_lane_graph(capability_set->has_lane_graph);
    capability->set_has_route_semantics(capability_set->has_route_semantics);
    capability->set_has_local_corridor(capability_set->has_local_corridor);
    capability->set_has_drivable_area(capability_set->has_drivable_area);
    capability->set_has_parking_roi(capability_set->has_parking_roi);
    capability->set_has_goal_pose(capability_set->has_goal_pose);
    capability->set_has_stop_target(capability_set->has_stop_target);
    capability->set_has_regulatory_context(
        capability_set->has_regulatory_context);
    capability->set_can_run_on_lane_shell(
        capability_set->can_run_on_lane_shell);
    capability->set_can_run_corridor_shell(
        capability_set->can_run_corridor_shell);
    capability->set_can_run_safety_hold_shell(
        capability_set->can_run_safety_hold_shell);
    capability->set_has_structured_mapless_context(
        capability_set->has_structured_mapless_context);
    capability->set_can_run_structured_mapless_shell(
        capability_set->can_run_structured_mapless_shell);
    capability->set_can_run_open_space_shell(
        capability_set->can_run_open_space_shell);
    capability->set_has_known_open_space_environment(
        capability_set->has_known_open_space_environment);
    capability->set_supports_open_space_exploration(
        capability_set->supports_open_space_exploration);
    capability->set_topology_confidence(capability_set->topology_confidence);
    capability->set_drivable_area_confidence(
        capability_set->drivable_area_confidence);
    capability->set_target_geometry_confidence(
        capability_set->target_geometry_confidence);
  }

  auto* validation = runtime_status.mutable_validation();
  validation->set_trajectory_valid(validation_result.trajectory_valid);
  validation->set_command_admissible(validation_result.command_admissible);
  validation->set_fallback_active(validation_result.fallback_active);
  if (!validation_result.reason.empty()) {
    validation->set_validation_reason(validation_result.reason);
  } else if (!reason.empty()) {
    validation->set_validation_reason(reason);
  }
  return runtime_status;
}

}  // namespace planning
}  // namespace apollo
