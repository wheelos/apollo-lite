#pragma once

#include <string>
#include <vector>

#include "modules/common_msgs/planning_msgs/mission_directive.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"

namespace apollo {
namespace planning {

inline PlanningOperatingDomain ResolveOperatingDomainForMode(
    PlanningMode mode) {
  switch (mode) {
    case MODE_LANE_GRAPH:
    case MODE_CORRIDOR:
      return DOMAIN_HDMAP_ROUTED;
    case MODE_FREE_SPACE:
      return DOMAIN_STRUCTURED_MAPLESS;
    case MODE_OPEN_SPACE:
      return DOMAIN_OPEN_SPACE;
    case MODE_SAFETY_HOLD:
      return DOMAIN_SAFETY_HOLD;
    case MODE_UNKNOWN:
    default:
      return DOMAIN_UNKNOWN;
  }
}

struct PlanningCoordinatorState {
  std::string mission_id;
  std::string command_id;
  MissionCommandIdentity mission_identity;
  MissionTaskType mission_task = MISSION_TASK_UNKNOWN;
  MissionSessionState mission_session_state = MISSION_SESSION_UNKNOWN;
  MissionSessionPhase mission_phase = MISSION_PHASE_UNKNOWN;
  MissionStartSnapshot accepted_start;
  MissionRouteContext mission_route;
  bool mission_cancellation_fenced = false;
  PlanningSceneType active_scene = SCENE_LANE_CRUISE;
  PlanningOperatingDomain active_domain = DOMAIN_UNKNOWN;
  PlanningShellType previous_shell = PLANNING_SHELL_UNKNOWN;
  PlanningMode previous_mode = MODE_UNKNOWN;
  PlanningMode requested_mode = MODE_UNKNOWN;
  PlanningMode desired_mode = MODE_UNKNOWN;
  PlanningMode resolved_mode = MODE_UNKNOWN;
  PlanningShellType desired_shell = PLANNING_SHELL_UNKNOWN;
  PlanningShellType active_shell = PLANNING_SHELL_UNKNOWN;
  bool transition_pending = false;
  bool continuity_hold = false;
  std::string reason;
  std::vector<std::string> blockers;
};

}  // namespace planning
}  // namespace apollo
