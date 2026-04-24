#pragma once

#include <string>
#include <vector>

#include "modules/common_msgs/planning_msgs/planning_command.pb.h"

namespace apollo {
namespace planning {

struct PlanningCoordinatorState {
  std::string mission_id;
  std::string command_id;
  PlanningSceneType active_scene = SCENE_LANE_CRUISE;
  PlanningMode requested_mode = MODE_UNKNOWN;
  PlanningMode resolved_mode = MODE_UNKNOWN;
  std::string reason;
  std::vector<std::string> blockers;
};

}  // namespace planning
}  // namespace apollo
