#include "modules/control/common/strategy_orchestrator.h"

namespace apollo {
namespace control {

namespace {

bool IsDockLikeScene(planning::PlanningSceneType scene) {
  return scene == planning::SCENE_DOCK || scene == planning::SCENE_PARK_IN ||
         scene == planning::SCENE_PULL_OVER;
}

planning::ControlExecutionChannel ResolveExecutionChannelFromIntent(
    const planning::ControlIntent& intent, bool has_trajectory_points) {
  if (intent.primitive_type() != planning::CONTROL_PRIMITIVE_NONE ||
      intent.tracking_mode() == planning::TRACKING_MODE_POSE_SERVO ||
      intent.tracking_mode() == planning::TRACKING_MODE_STANDSTILL_HOLD) {
    return planning::EXECUTION_CHANNEL_PRIMITIVE;
  }
  if (has_trajectory_points) {
    return planning::EXECUTION_CHANNEL_TRAJECTORY;
  }
  return planning::EXECUTION_CHANNEL_UNKNOWN;
}

}  // namespace

SemanticControlProfile StrategyOrchestrator::Resolve(
    const ControlCommandGoal& goal) const {
  SemanticControlProfile profile;
  profile.profile_reason = goal.reason;

  if (goal.semantic == GoalSemantic::kEmergencyStop) {
    profile.profile_key = "emergency-stop";
    profile.enforce_hold_stop = true;
    profile.suppress_large_steer = true;
    profile.prefer_trajectory_tracking = false;
    return profile;
  }

  if (goal.semantic == GoalSemantic::kStandstillHold ||
      goal.active_scene == planning::SCENE_HOLD) {
    profile.profile_key = "standstill-hold";
    profile.enforce_hold_stop = true;
    profile.suppress_large_steer = true;
    profile.prefer_trajectory_tracking = false;
    return profile;
  }

  if (goal.semantic == GoalSemantic::kPoseServo || goal.has_target_pose ||
      IsDockLikeScene(goal.active_scene)) {
    profile.profile_key = "pose-servo";
    profile.suppress_large_steer = true;
    profile.prefer_pose_servo = true;
    profile.prefer_trajectory_tracking = goal.has_trajectory;
    return profile;
  }

  if (goal.semantic == GoalSemantic::kApproachStop) {
    profile.profile_key = "approach-stop";
    profile.suppress_large_steer = true;
    profile.prefer_trajectory_tracking = true;
    return profile;
  }

  profile.profile_key = goal.active_scene == planning::SCENE_LANE_CRUISE
                            ? "lane-cruise"
                            : "default-tracking";
  return profile;
}

void StrategyOrchestrator::Apply(const SemanticControlProfile& profile,
                                 planning::ADCTrajectory* trajectory) const {
  if (trajectory == nullptr) {
    return;
  }
  auto* intent = trajectory->mutable_control_intent();

  if (profile.prefer_trajectory_tracking) {
    if (!intent->has_tracking_mode() ||
        intent->tracking_mode() == planning::TRACKING_MODE_UNKNOWN) {
      intent->set_tracking_mode(planning::TRACKING_MODE_TRAJECTORY);
    }
    if (!intent->has_longitudinal_intent() ||
        intent->longitudinal_intent() == planning::LON_INTENT_UNKNOWN) {
      intent->set_longitudinal_intent(planning::LON_INTENT_CRUISE);
    }
    if (!intent->has_lateral_intent() ||
        intent->lateral_intent() == planning::LAT_INTENT_UNKNOWN) {
      intent->set_lateral_intent(planning::LAT_INTENT_TRACK_PATH);
    }
    if (!intent->has_primitive_type()) {
      intent->set_primitive_type(planning::CONTROL_PRIMITIVE_NONE);
    }
  }

  if (profile.enforce_hold_stop) {
    intent->set_tracking_mode(planning::TRACKING_MODE_STANDSTILL_HOLD);
    intent->set_longitudinal_intent(planning::LON_INTENT_HOLD_STOP);
    if (!intent->has_lateral_intent()) {
      intent->set_lateral_intent(planning::LAT_INTENT_MINIMIZE_STEER);
    }
    intent->set_primitive_type(planning::CONTROL_PRIMITIVE_STANDSTILL_HOLD);
  } else if (profile.prefer_pose_servo && intent->has_target_stop_point()) {
    intent->set_tracking_mode(planning::TRACKING_MODE_POSE_SERVO);
    if (!intent->has_lateral_intent() ||
        intent->lateral_intent() == planning::LAT_INTENT_UNKNOWN) {
      intent->set_lateral_intent(planning::LAT_INTENT_ALIGN_GOAL_HEADING);
    }
    if (!intent->has_primitive_type() ||
        intent->primitive_type() == planning::CONTROL_PRIMITIVE_NONE) {
      intent->set_primitive_type(planning::CONTROL_PRIMITIVE_POSE_SERVO);
    }
  }

  if (profile.suppress_large_steer) {
    intent->set_suppress_large_steer(true);
  }

  if (intent->has_reason()) {
    intent->set_reason(intent->reason() + " | profile=" + profile.profile_key);
  } else if (!profile.profile_reason.empty()) {
    intent->set_reason(profile.profile_reason + " | profile=" +
                       profile.profile_key);
  } else {
    intent->set_reason("profile=" + profile.profile_key);
  }

  intent->set_execution_channel(
      ResolveExecutionChannelFromIntent(*intent,
                                        trajectory->trajectory_point_size() > 0));
}

}  // namespace control
}  // namespace apollo
