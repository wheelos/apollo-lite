#pragma once

#include <string>

#include "modules/common_msgs/planning_msgs/planning.pb.h"

namespace apollo {
namespace control {

enum class GoalSemantic {
  kDefaultTracking = 0,
  kRoadTracking = 1,
  kPoseServo = 2,
  kApproachStop = 3,
  kStandstillHold = 4,
  kEmergencyStop = 5,
};

struct ControlCommandGoal {
  GoalSemantic semantic = GoalSemantic::kDefaultTracking;
  bool has_trajectory = false;
  bool has_target_pose = false;
  bool has_control_intent = false;
  planning::PlanningSceneType active_scene = planning::SCENE_UNKNOWN;
  planning::PlanningMode active_mode = planning::MODE_UNKNOWN;
  planning::ControlIntent control_intent;
  std::string reason;
};

ControlCommandGoal BuildControlCommandGoal(
    const planning::ADCTrajectory& trajectory);

void EnsureControlIntentCompatibility(const ControlCommandGoal& goal,
                                      planning::ADCTrajectory* trajectory);

}  // namespace control
}  // namespace apollo
