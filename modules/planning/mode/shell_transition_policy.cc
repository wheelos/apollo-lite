#include "modules/planning/mode/shell_transition_policy.h"

#include <utility>

namespace apollo {
namespace planning {

namespace {

constexpr std::size_t kRequiredStableCycles = 2;

PlanningShellType ResolveShellType(PlanningMode mode) {
  switch (mode) {
    case MODE_LANE_GRAPH:
      return PLANNING_SHELL_ON_LANE;
    case MODE_CORRIDOR:
      return PLANNING_SHELL_CORRIDOR;
    case MODE_FREE_SPACE:
      return PLANNING_SHELL_STRUCTURED_MAPLESS;
    case MODE_OPEN_SPACE:
      return PLANNING_SHELL_OPEN_SPACE;
    case MODE_SAFETY_HOLD:
      return PLANNING_SHELL_SAFETY_HOLD;
    case MODE_UNKNOWN:
    default:
      return PLANNING_SHELL_UNKNOWN;
  }
}

const char* ShellName(PlanningShellType shell) {
  switch (shell) {
    case PLANNING_SHELL_ON_LANE:
      return "on_lane";
    case PLANNING_SHELL_CORRIDOR:
      return "corridor";
    case PLANNING_SHELL_STRUCTURED_MAPLESS:
      return "structured_mapless";
    case PLANNING_SHELL_OPEN_SPACE:
      return "open_space";
    case PLANNING_SHELL_SAFETY_HOLD:
      return "safety_hold";
    case PLANNING_SHELL_UNKNOWN:
    default:
      return "unknown";
  }
}

bool IsModeAvailable(PlanningMode mode,
                     const ModeShellAvailability& availability) {
  switch (mode) {
    case MODE_LANE_GRAPH:
      return availability.lane_graph_available;
    case MODE_CORRIDOR:
      return availability.corridor_available;
    case MODE_OPEN_SPACE:
      return availability.open_space_available;
    case MODE_FREE_SPACE:
      return availability.free_space_available;
    case MODE_SAFETY_HOLD:
      return availability.safety_hold_available;
    case MODE_UNKNOWN:
    default:
      return false;
  }
}

bool IsModeExecutable(PlanningMode mode, const CapabilitySet* capability,
                      const ModeShellAvailability& availability) {
  return mode != MODE_UNKNOWN && IsModeAvailable(mode, availability) &&
         HasCapabilityForMode(mode, capability);
}

void ResetPolicyState(ShellTransitionPolicyState* policy_state) {
  if (policy_state == nullptr) {
    return;
  }
  policy_state->pending_mode = MODE_UNKNOWN;
  policy_state->pending_shell = PLANNING_SHELL_UNKNOWN;
  policy_state->stable_cycle_count = 0;
}

void TrackCandidate(const PlanningMode desired_mode,
                    const PlanningShellType desired_shell,
                    ShellTransitionPolicyState* policy_state) {
  if (policy_state == nullptr) {
    return;
  }
  if (policy_state->pending_mode == desired_mode &&
      policy_state->pending_shell == desired_shell) {
    ++policy_state->stable_cycle_count;
    return;
  }
  policy_state->pending_mode = desired_mode;
  policy_state->pending_shell = desired_shell;
  policy_state->stable_cycle_count = 1;
}

}  // namespace

ShellTransitionDecision ShellTransitionPolicy::Apply(
    const ModeResolutionResult& resolution,
    const PlanningCoordinatorState& previous_state,
    const std::string& command_id, const PlanningSceneType active_scene,
    const CapabilitySet* capability, const ModeShellAvailability& availability,
    ShellTransitionPolicyState* policy_state) {
  ShellTransitionDecision decision;
  decision.desired_mode = resolution.resolved_mode;
  decision.desired_shell = ResolveShellType(resolution.resolved_mode);
  decision.active_mode = resolution.resolved_mode;
  decision.active_shell = decision.desired_shell;
  decision.reason = resolution.reason;

  if (decision.desired_mode == MODE_UNKNOWN ||
      decision.desired_shell == PLANNING_SHELL_UNKNOWN) {
    ResetPolicyState(policy_state);
    return decision;
  }

  const bool command_changed = previous_state.command_id != command_id;
  const bool scene_changed = previous_state.active_scene != active_scene;
  if ((command_changed || scene_changed) &&
      previous_state.resolved_mode != MODE_UNKNOWN &&
      previous_state.active_shell != PLANNING_SHELL_UNKNOWN &&
      previous_state.active_shell != decision.desired_shell) {
    ResetPolicyState(policy_state);
    if (decision.reason.empty()) {
      decision.reason = std::string(
                            "switching immediately for new command "
                            "context from ") +
                        ShellName(previous_state.active_shell) + " to " +
                        ShellName(decision.desired_shell);
    }
    return decision;
  }

  if (previous_state.resolved_mode == MODE_UNKNOWN ||
      previous_state.active_shell == PLANNING_SHELL_UNKNOWN ||
      previous_state.active_shell == decision.desired_shell) {
    ResetPolicyState(policy_state);
    return decision;
  }

  if (decision.desired_shell == PLANNING_SHELL_SAFETY_HOLD) {
    ResetPolicyState(policy_state);
    if (decision.reason.empty()) {
      decision.reason = "switching to safety_hold shell";
    }
    return decision;
  }

  if (!IsModeExecutable(previous_state.resolved_mode, capability,
                        availability)) {
    ResetPolicyState(policy_state);
    if (decision.reason.empty()) {
      decision.reason = std::string("switching to ") +
                        ShellName(decision.desired_shell) +
                        " shell because current shell is no longer executable";
    }
    return decision;
  }

  TrackCandidate(decision.desired_mode, decision.desired_shell, policy_state);
  if (policy_state == nullptr ||
      policy_state->stable_cycle_count >= kRequiredStableCycles) {
    ResetPolicyState(policy_state);
    if (decision.reason.empty()) {
      decision.reason = std::string("approved shell transition from ") +
                        ShellName(previous_state.active_shell) + " to " +
                        ShellName(decision.desired_shell);
    }
    return decision;
  }

  decision.active_mode = previous_state.resolved_mode;
  decision.active_shell = previous_state.active_shell;
  decision.transition_pending = true;
  decision.continuity_hold = true;
  decision.reason = std::string("holding ") +
                    ShellName(previous_state.active_shell) +
                    " shell while transition to " +
                    ShellName(decision.desired_shell) + " stabilizes";
  if (!resolution.reason.empty()) {
    decision.reason += ": " + resolution.reason;
  }
  return decision;
}

}  // namespace planning
}  // namespace apollo
