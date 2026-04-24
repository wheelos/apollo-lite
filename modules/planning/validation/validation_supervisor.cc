#include "modules/planning/validation/validation_supervisor.h"

namespace apollo {
namespace planning {

namespace {

ValidationResult HoldResult(bool command_admissible, const std::string& reason) {
  ValidationResult result;
  result.command_admissible = command_admissible;
  result.trajectory_valid = false;
  result.should_publish = false;
  result.should_hold = true;
  result.fallback_active = true;
  result.reason = reason;
  return result;
}

ValidationResult GateResult(const LocalView* local_view,
                            bool command_admissible,
                            const std::string& reason) {
  auto result = HoldResult(command_admissible, reason);
  if (command_admissible && local_view != nullptr &&
      local_view->planning_command != nullptr &&
      local_view->planning_command->has_fallback() &&
      local_view->planning_command->fallback().has_allow_hold() &&
      !local_view->planning_command->fallback().allow_hold()) {
    result.command_admissible = false;
    result.reason = reason + "; hold disallowed by command fallback policy";
  }
  return result;
}

bool IsAllowedDegradedMode(const PlanningCommand& command,
                           PlanningMode resolved_mode) {
  if (!command.has_fallback() ||
      command.fallback().allowed_degraded_modes_size() == 0) {
    return true;
  }
  for (const auto mode : command.fallback().allowed_degraded_modes()) {
    if (mode == resolved_mode) {
      return true;
    }
  }
  return false;
}

}  // namespace

ValidationResult ValidationSupervisor::Validate(
    const ValidationInput& input) const {
  if (input.local_view == nullptr || input.trajectory == nullptr) {
    return HoldResult(false, "validation input incomplete");
  }

  if (input.local_view->planning_command != nullptr &&
      input.planning_state != nullptr) {
    const auto& command = *input.local_view->planning_command;
    const bool mode_degraded =
        input.planning_state->requested_mode != input.planning_state->resolved_mode;
    if (mode_degraded && command.has_fallback() &&
        command.fallback().has_allow_mode_degradation() &&
        !command.fallback().allow_mode_degradation()) {
      return GateResult(
          input.local_view,
          false,
          "requested mode degradation disallowed by command fallback policy");
    }
    if (mode_degraded && !IsAllowedDegradedMode(
                             command, input.planning_state->resolved_mode)) {
      return GateResult(input.local_view, false,
                        "resolved mode is not listed in allowed degraded modes");
    }
  }

  if (input.planning_state != nullptr &&
      input.planning_state->resolved_mode == MODE_UNKNOWN) {
    return GateResult(
        input.local_view, true,
        input.planning_state->reason.empty()
            ? "no admissible planning mode resolved"
            : input.planning_state->reason);
  }

  if (input.local_view->capability_set != nullptr &&
      input.planning_state != nullptr) {
    const auto& capability = *input.local_view->capability_set;
    switch (input.planning_state->resolved_mode) {
      case MODE_LANE_GRAPH:
        if (!capability.has_lane_graph) {
          return GateResult(input.local_view, true,
                            "lane-graph capability unavailable");
        }
        break;
      case MODE_CORRIDOR:
        if (!capability.has_local_corridor) {
          return GateResult(input.local_view, true,
                            "corridor capability unavailable");
        }
        break;
      case MODE_OPEN_SPACE:
        if (!capability.has_parking_roi || !capability.has_goal_pose) {
          return GateResult(input.local_view, true,
                            "open-space capability unavailable");
        }
        break;
      case MODE_FREE_SPACE:
        if (!capability.has_drivable_area || !capability.has_goal_pose) {
          return GateResult(input.local_view, true,
                            "free-space capability unavailable");
        }
        break;
      default:
        break;
    }
  }

  if (input.trajectory->has_decision() &&
      input.trajectory->decision().has_main_decision() &&
      input.trajectory->decision().main_decision().has_not_ready() &&
      input.trajectory->decision().main_decision().not_ready().has_reason()) {
    return GateResult(
        input.local_view, true,
        input.trajectory->decision().main_decision().not_ready().reason());
  }

  return ValidationResult();
}

}  // namespace planning
}  // namespace apollo
