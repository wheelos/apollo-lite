#pragma once

#include <string>

#include "modules/planning/environment/capability_extractor.h"
#include "modules/planning/mode/mode_resolution.h"
#include "modules/planning/planning_runtime_context.h"

namespace apollo {
namespace planning {

struct ShellTransitionPolicyState {
  PlanningMode pending_mode = MODE_UNKNOWN;
  PlanningShellType pending_shell = PLANNING_SHELL_UNKNOWN;
  std::size_t stable_cycle_count = 0;
};

struct ShellTransitionDecision {
  PlanningMode active_mode = MODE_UNKNOWN;
  PlanningShellType active_shell = PLANNING_SHELL_UNKNOWN;
  PlanningMode desired_mode = MODE_UNKNOWN;
  PlanningShellType desired_shell = PLANNING_SHELL_UNKNOWN;
  bool transition_pending = false;
  bool continuity_hold = false;
  std::string reason;
};

class ShellTransitionPolicy {
 public:
  static ShellTransitionDecision Apply(
      const ModeResolutionResult& resolution,
      const PlanningCoordinatorState& previous_state,
      const std::string& command_id, PlanningSceneType active_scene,
      const CapabilitySet* capability,
      const ModeShellAvailability& availability,
      ShellTransitionPolicyState* policy_state);
};

}  // namespace planning
}  // namespace apollo
