#pragma once

#include <string>

#include "modules/control/common/control_command_goal.h"
#include "modules/control/proto/control_conf.pb.h"
#include "wheelos_msgs/planning_msgs/motion_execution.pb.h"

namespace apollo {
namespace control {

struct SemanticControlProfile {
  std::string profile_key = "default-tracking";
  bool enforce_hold_stop = false;
  bool suppress_large_steer = false;
  bool prefer_pose_servo = false;
  bool prefer_trajectory_tracking = true;
  std::string profile_reason;
  ControllerParameterProfile parameter_profile = CONTROL_PROFILE_ROAD_TRACKING;
  bool supported = true;
};

class StrategyOrchestrator {
 public:
  SemanticControlProfile Resolve(const ControlCommandGoal& goal) const;
  SemanticControlProfile Resolve(
      const planning::MotionExecutionCommand& command) const;
  void Apply(const SemanticControlProfile& profile,
             planning::ADCTrajectory* trajectory) const;
};

}  // namespace control
}  // namespace apollo
