#include "modules/control/common/control_command_goal.h"

namespace apollo {
namespace control {

namespace {

GoalSemantic InferSemanticFromIntent(const planning::ControlIntent& intent) {
  if (intent.tracking_mode() == planning::TRACKING_MODE_POSE_SERVO ||
      intent.primitive_type() == planning::CONTROL_PRIMITIVE_POSE_SERVO) {
    return GoalSemantic::kPoseServo;
  }
  if (intent.tracking_mode() == planning::TRACKING_MODE_STANDSTILL_HOLD ||
      intent.longitudinal_intent() == planning::LON_INTENT_HOLD_STOP ||
      intent.longitudinal_intent() == planning::LON_INTENT_MRM_STOP) {
    return GoalSemantic::kStandstillHold;
  }
  if (intent.longitudinal_intent() == planning::LON_INTENT_APPROACH_STOP ||
      intent.longitudinal_intent() == planning::LON_INTENT_PRECISE_STOP ||
      intent.longitudinal_intent() == planning::LON_INTENT_YIELD_STOP) {
    return GoalSemantic::kApproachStop;
  }
  return GoalSemantic::kRoadTracking;
}

}  // namespace

ControlCommandGoal BuildControlCommandGoal(
    const planning::ADCTrajectory& trajectory) {
  ControlCommandGoal goal;
  goal.has_trajectory = trajectory.trajectory_point_size() > 0;
  goal.has_control_intent = trajectory.has_control_intent();

  if (trajectory.has_execution()) {
    const auto& execution = trajectory.execution();
    if (execution.has_active_scene()) {
      goal.active_scene = execution.active_scene();
    }
    if (execution.has_active_mode()) {
      goal.active_mode = execution.active_mode();
    }
    if (execution.has_reason()) {
      goal.reason = execution.reason();
    }
  }

  if (goal.has_control_intent) {
    goal.control_intent = trajectory.control_intent();
    goal.semantic = InferSemanticFromIntent(goal.control_intent);
    goal.has_target_pose = goal.control_intent.has_target_stop_point();
    if (goal.reason.empty() && goal.control_intent.has_reason()) {
      goal.reason = goal.control_intent.reason();
    }
  } else {
    goal.semantic = goal.has_trajectory ? GoalSemantic::kRoadTracking
                                        : GoalSemantic::kStandstillHold;
  }

  if (trajectory.has_estop() && trajectory.estop().is_estop()) {
    goal.semantic = GoalSemantic::kEmergencyStop;
    if (goal.reason.empty() && trajectory.estop().has_reason()) {
      goal.reason = trajectory.estop().reason();
    }
  }

  return goal;
}

void EnsureControlIntentCompatibility(const ControlCommandGoal& goal,
                                      planning::ADCTrajectory* trajectory) {
  if (trajectory == nullptr) {
    return;
  }

  auto* control_intent = trajectory->mutable_control_intent();
  if (!control_intent->has_tracking_mode()) {
    control_intent->set_tracking_mode(goal.has_trajectory
                                          ? planning::TRACKING_MODE_TRAJECTORY
                                          : planning::TRACKING_MODE_STANDSTILL_HOLD);
  }
  if (!control_intent->has_longitudinal_intent()) {
    control_intent->set_longitudinal_intent(
        goal.has_trajectory ? planning::LON_INTENT_CRUISE
                            : planning::LON_INTENT_HOLD_STOP);
  }
  if (!control_intent->has_lateral_intent()) {
    control_intent->set_lateral_intent(goal.has_trajectory
                                           ? planning::LAT_INTENT_TRACK_PATH
                                           : planning::LAT_INTENT_MINIMIZE_STEER);
  }
  if (!control_intent->has_primitive_type()) {
    control_intent->set_primitive_type(
        goal.has_trajectory ? planning::CONTROL_PRIMITIVE_NONE
                            : planning::CONTROL_PRIMITIVE_STANDSTILL_HOLD);
  }
  if (!control_intent->has_reason() && !goal.reason.empty()) {
    control_intent->set_reason(goal.reason);
  }
}

}  // namespace control
}  // namespace apollo
