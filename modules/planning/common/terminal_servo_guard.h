#pragma once

#include <string>

#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/planning/common/planning_semantics.h"

namespace apollo {
namespace planning {

struct TerminalServoSessionState {
  std::string command_id;
  double start_time_sec = -1.0;
};

std::string ApplyTerminalServoGuardrails(
    const std::string& command_id, double now_sec,
    TerminalServoSessionState* session_state,
    PlanningSemanticSummary* semantic_summary, ADCTrajectory* trajectory);

}  // namespace planning
}  // namespace apollo
