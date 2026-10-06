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
  if (intent.tracking_mode() == planning::TRACKING_MODE_PATH_SPEED &&
      intent.execution_channel() == planning::EXECUTION_CHANNEL_PRIMITIVE &&
      has_trajectory_points) {
    return planning::EXECUTION_CHANNEL_PRIMITIVE;
  }
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

SemanticControlProfile UnsupportedProfile(const std::string& reason) {
  SemanticControlProfile profile;
  profile.supported = false;
  profile.profile_reason = reason;
  return profile;
}

}  // namespace

SemanticControlProfile StrategyOrchestrator::Resolve(
    const planning::MotionExecutionCommand& command) const {
  SemanticControlProfile profile;
  if (!command.has_control_intent()) {
    return UnsupportedProfile("authorized motion has no Control semantics");
  }
  const auto& intent = command.control_intent();
  if (command.has_trajectory()) {
    const auto expected_channel = ResolveExecutionChannelFromIntent(
        intent, command.trajectory().point_size() > 0);
    if (intent.execution_channel() != expected_channel) {
      return UnsupportedProfile(
          "trajectory payload conflicts with its declared execution channel");
    }
    profile.suppress_large_steer =
        intent.lateral_intent() == planning::LAT_INTENT_MINIMIZE_STEER ||
        intent.lateral_intent() == planning::LAT_INTENT_STABILIZE_NEAR_STOP ||
        intent.lateral_intent() == planning::LAT_INTENT_ALIGN_GOAL_HEADING;
    profile.prefer_trajectory_tracking =
        intent.tracking_mode() != planning::TRACKING_MODE_POSE_SERVO &&
        intent.tracking_mode() != planning::TRACKING_MODE_STANDSTILL_HOLD;
    if (intent.tracking_mode() ==
        planning::TRACKING_MODE_POSE_SERVO) {
      if (intent.primitive_type() !=
              planning::CONTROL_PRIMITIVE_POSE_SERVO ||
          intent.lateral_intent() !=
              planning::LAT_INTENT_ALIGN_GOAL_HEADING ||
          intent.longitudinal_intent() ==
              planning::LON_INTENT_HOLD_STOP ||
          intent.longitudinal_intent() ==
              planning::LON_INTENT_MRM_STOP) {
        return UnsupportedProfile(
            "pose-servo tracking requires matching pose semantics");
      }
      profile.profile_key = "pose-servo";
      profile.parameter_profile = CONTROL_PROFILE_LOW_SPEED_PRECISION;
      profile.prefer_pose_servo = true;
      profile.suppress_large_steer = true;
      return profile;
    }
    if (intent.tracking_mode() ==
        planning::TRACKING_MODE_STANDSTILL_HOLD) {
      if (intent.longitudinal_intent() !=
              planning::LON_INTENT_HOLD_STOP ||
          intent.primitive_type() !=
              planning::CONTROL_PRIMITIVE_STANDSTILL_HOLD) {
        return UnsupportedProfile(
            "standstill tracking requires matching hold semantics");
      }
      profile.profile_key = "standstill-hold";
      profile.parameter_profile = CONTROL_PROFILE_STANDSTILL_HOLD;
      profile.prefer_trajectory_tracking = false;
      profile.enforce_hold_stop = true;
      return profile;
    }
    if (intent.tracking_mode() ==
            planning::TRACKING_MODE_PATH_SPEED &&
        (intent.primitive_type() != planning::CONTROL_PRIMITIVE_NONE ||
         intent.execution_channel() !=
             planning::EXECUTION_CHANNEL_PRIMITIVE)) {
      return UnsupportedProfile(
          "path-speed tracking requires the primitive execution channel");
    }
    if (intent.primitive_type() ==
        planning::CONTROL_PRIMITIVE_LATERAL_HOLD) {
      return UnsupportedProfile(
          "lateral-hold semantics have no supported controller profile");
    }
    if (intent.primitive_type() ==
        planning::CONTROL_PRIMITIVE_HEADING_HOLD) {
      if (intent.lateral_intent() !=
          planning::LAT_INTENT_ALIGN_GOAL_HEADING) {
        return UnsupportedProfile(
            "heading-hold primitive requires goal-heading alignment");
      }
      profile.profile_key = "heading-hold";
      profile.parameter_profile = CONTROL_PROFILE_LOW_SPEED_PRECISION;
      profile.suppress_large_steer = true;
      return profile;
    }
    const auto longitudinal = intent.longitudinal_intent();
    if (longitudinal == planning::LON_INTENT_MRM_STOP) {
      profile.profile_key = "controlled-stop";
    } else if (longitudinal == planning::LON_INTENT_HOLD_STOP) {
      if (intent.primitive_type() !=
              planning::CONTROL_PRIMITIVE_STANDSTILL_HOLD) {
        return UnsupportedProfile(
            "hold-stop intent requires matching standstill semantics");
      }
      profile.profile_key = "standstill-hold";
      profile.parameter_profile = CONTROL_PROFILE_STANDSTILL_HOLD;
      profile.prefer_trajectory_tracking = false;
      profile.enforce_hold_stop = true;
    } else if (longitudinal == planning::LON_INTENT_PRECISE_STOP) {
      profile.profile_key = "precise-stop";
      profile.parameter_profile = CONTROL_PROFILE_LOW_SPEED_PRECISION;
    } else if (longitudinal == planning::LON_INTENT_APPROACH_STOP) {
      profile.profile_key = "approach-stop";
    } else if (longitudinal == planning::LON_INTENT_YIELD_STOP) {
      profile.profile_key = "yield-stop";
    } else {
      profile.profile_key = "road-tracking";
    }
    if (intent.execution_channel() ==
            planning::EXECUTION_CHANNEL_PRIMITIVE &&
        intent.primitive_type() == planning::CONTROL_PRIMITIVE_NONE) {
      return UnsupportedProfile(
          "primitive execution channel requires a declared primitive");
    }
    return profile;
  }
  if (command.has_primitive()) {
    if (intent.execution_channel() !=
        planning::EXECUTION_CHANNEL_PRIMITIVE) {
      return UnsupportedProfile(
          "primitive payload requires the primitive execution channel");
    }
    switch (command.primitive().type()) {
      case planning::MOTION_PRIMITIVE_STANDSTILL_HOLD:
        if (intent.tracking_mode() !=
                planning::TRACKING_MODE_STANDSTILL_HOLD ||
            intent.longitudinal_intent() != planning::LON_INTENT_HOLD_STOP ||
            intent.primitive_type() !=
                planning::CONTROL_PRIMITIVE_STANDSTILL_HOLD) {
          return UnsupportedProfile(
              "standstill payload conflicts with its declared semantics");
        }
        profile.profile_key = "standstill-hold";
        profile.parameter_profile = CONTROL_PROFILE_STANDSTILL_HOLD;
        profile.prefer_trajectory_tracking = false;
        profile.enforce_hold_stop = true;
        return profile;
      case planning::MOTION_PRIMITIVE_POSE_SERVO:
        if (intent.tracking_mode() !=
                planning::TRACKING_MODE_POSE_SERVO ||
            intent.primitive_type() !=
                planning::CONTROL_PRIMITIVE_POSE_SERVO ||
            intent.longitudinal_intent() ==
                planning::LON_INTENT_HOLD_STOP ||
            intent.longitudinal_intent() ==
                planning::LON_INTENT_MRM_STOP) {
          return UnsupportedProfile(
              "pose-servo payload conflicts with its declared semantics");
        }
        profile.profile_key = "pose-servo";
        profile.parameter_profile = CONTROL_PROFILE_LOW_SPEED_PRECISION;
        profile.prefer_pose_servo = true;
        profile.suppress_large_steer = true;
        return profile;
      case planning::MOTION_PRIMITIVE_CORRIDOR_SERVO:
        if (intent.tracking_mode() !=
                planning::TRACKING_MODE_PATH_SPEED ||
            intent.primitive_type() !=
                planning::CONTROL_PRIMITIVE_NONE ||
            intent.longitudinal_intent() ==
                planning::LON_INTENT_HOLD_STOP ||
            intent.longitudinal_intent() ==
                planning::LON_INTENT_MRM_STOP) {
          return UnsupportedProfile(
              "corridor-servo payload conflicts with its declared semantics");
        }
        profile.profile_key = "spatial-primitive";
        profile.parameter_profile = CONTROL_PROFILE_LOW_SPEED_PRECISION;
        return profile;
      default:
        break;
    }
  }
  return UnsupportedProfile(
      "authorized payload has no supported controller profile");
}

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

  if (goal.has_trajectory && goal.control_intent.require_full_stop() &&
      goal.control_intent.longitudinal_intent() == planning::LON_INTENT_MRM_STOP &&
      goal.control_intent.execution_channel() == planning::EXECUTION_CHANNEL_TRAJECTORY) {
    profile.profile_key = "controlled-stop";
    return profile;
  }

  if (goal.has_trajectory &&
      goal.control_intent.tracking_mode() ==
          planning::TRACKING_MODE_PATH_SPEED &&
      goal.execution_channel == planning::EXECUTION_CHANNEL_PRIMITIVE) {
    profile.profile_key = "spatial-primitive";
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
