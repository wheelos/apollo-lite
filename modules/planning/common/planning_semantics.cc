#include "modules/planning/common/planning_semantics.h"

#include <cmath>
#include <limits>

#include "modules/common/math/math_utils.h"

namespace apollo {
namespace planning {

namespace {

constexpr double kFullStopSpeedMps = 0.2;
constexpr double kNearTerminalDistanceM = 3.0;
constexpr double kPositionToleranceM = 0.5;
constexpr double kHeadingToleranceRad = 10.0 * M_PI / 180.0;
constexpr double kTerminalServoEntrySpeedMps = 0.6;
constexpr double kTerminalServoMaxSpeedMps = 0.2;
constexpr double kTerminalServoTimeoutSec = 8.0;

bool HasTrajectoryPoints(const ADCTrajectory* trajectory) {
  return trajectory != nullptr && trajectory->trajectory_point_size() > 0;
}

bool IsFullStop(const canbus::Chassis* chassis) {
  return chassis != nullptr && std::abs(chassis->speed_mps()) <= kFullStopSpeedMps;
}

bool IsTerminalServoEntrySpeed(const canbus::Chassis* chassis) {
  return chassis != nullptr &&
         std::abs(chassis->speed_mps()) <= kTerminalServoEntrySpeedMps;
}

double DistanceToStopPoint(const localization::LocalizationEstimate* localization,
                          const apollo::common::PointENU& stop_point) {
  if (localization == nullptr || !localization->has_pose()) {
    return std::numeric_limits<double>::infinity();
  }
  const double dx = localization->pose().position().x() - stop_point.x();
  const double dy = localization->pose().position().y() - stop_point.y();
  return std::hypot(dx, dy);
}

double HeadingErrorToStopPoint(
    const localization::LocalizationEstimate* localization,
    double target_stop_heading) {
  if (localization == nullptr || !localization->has_pose()) {
    return std::numeric_limits<double>::infinity();
  }
  return std::abs(common::math::NormalizeAngle(localization->pose().heading() -
                                               target_stop_heading));
}

const MainDecision* GetMainDecision(const ADCTrajectory* trajectory) {
  if (trajectory == nullptr || !trajectory->has_decision() ||
      !trajectory->decision().has_main_decision()) {
    return nullptr;
  }
  return &trajectory->decision().main_decision();
}

std::string DescribeStopReasonCode(StopReasonCode reason_code) {
  switch (reason_code) {
    case STOP_REASON_HEAD_VEHICLE:
      return "stop for leading vehicle";
    case STOP_REASON_DESTINATION:
      return "stop for destination goal";
    case STOP_REASON_PEDESTRIAN:
      return "stop for pedestrian";
    case STOP_REASON_OBSTACLE:
      return "stop for obstacle";
    case STOP_REASON_PREPARKING:
      return "stop for pre-parking preparation";
    case STOP_REASON_SIGNAL:
      return "stop for traffic signal";
    case STOP_REASON_STOP_SIGN:
      return "stop for stop sign";
    case STOP_REASON_YIELD_SIGN:
      return "stop for yield sign";
    case STOP_REASON_CLEAR_ZONE:
      return "stop for keep-clear zone";
    case STOP_REASON_CROSSWALK:
      return "stop for crosswalk";
    case STOP_REASON_CREEPER:
      return "stop for creeper";
    case STOP_REASON_REFERENCE_END:
      return "stop for reference line end";
    case STOP_REASON_YELLOW_SIGNAL:
      return "stop for yellow signal";
    case STOP_REASON_PULL_OVER:
      return "stop for pull-over target";
    case STOP_REASON_SIDEPASS_SAFETY:
      return "stop for side-pass safety";
    case STOP_REASON_PRE_OPEN_SPACE_STOP:
      return "stop for open-space preparation";
    case STOP_REASON_LANE_CHANGE_URGENCY:
      return "stop for lane change urgency";
    case STOP_REASON_EMERGENCY:
      return "stop for emergency";
    default:
      return "";
  }
}

void PopulateStopTargetFromDecision(const MainDecision* main_decision,
                                    PlanningSemanticSummary* summary) {
  if (main_decision == nullptr || summary == nullptr) {
    return;
  }
  if (main_decision->has_mission_complete()) {
    const auto& mission_complete = main_decision->mission_complete();
    if (mission_complete.has_stop_point()) {
      summary->has_target_stop_point = true;
      summary->target_stop_x = mission_complete.stop_point().x();
      summary->target_stop_y = mission_complete.stop_point().y();
      summary->target_stop_z = mission_complete.stop_point().z();
    }
    if (mission_complete.has_stop_heading()) {
      summary->has_target_stop_heading = true;
      summary->target_stop_heading = mission_complete.stop_heading();
    }
    return;
  }
  if (!main_decision->has_stop()) {
    return;
  }
  const auto& stop = main_decision->stop();
  if (stop.has_stop_point()) {
    summary->has_target_stop_point = true;
    summary->target_stop_x = stop.stop_point().x();
    summary->target_stop_y = stop.stop_point().y();
    summary->target_stop_z = stop.stop_point().z();
  }
  if (stop.has_stop_heading()) {
    summary->has_target_stop_heading = true;
    summary->target_stop_heading = stop.stop_heading();
  }
}

void PopulateStopReasonFromDecision(const PlanningSemanticInput& input,
                                    const MainDecision* main_decision,
                                    PlanningSemanticSummary* summary) {
  if (summary == nullptr) {
    return;
  }
  if (input.trajectory != nullptr && input.trajectory->has_estop() &&
      input.trajectory->estop().is_estop()) {
    summary->has_stop_reason_code = true;
    summary->stop_reason_code = STOP_REASON_EMERGENCY;
    if (input.trajectory->estop().has_reason()) {
      summary->control_reason = input.trajectory->estop().reason();
    }
    return;
  }
  if (main_decision == nullptr) {
    if (input.validation_should_hold && !input.validation_reason.empty()) {
      summary->control_reason = input.validation_reason;
    } else if (input.planning_state != nullptr &&
               !input.planning_state->reason.empty()) {
      summary->control_reason = input.planning_state->reason;
    } else if (input.planning_state != nullptr &&
               input.planning_state->resolved_mode == MODE_SAFETY_HOLD) {
      summary->control_reason = "planning safety hold active";
    }
    return;
  }
  if (main_decision->has_mission_complete()) {
    summary->has_stop_reason_code = true;
    summary->stop_reason_code = STOP_REASON_DESTINATION;
    summary->control_reason = "stop for mission destination";
    return;
  }
  if (main_decision->has_estop()) {
    summary->has_stop_reason_code = true;
    summary->stop_reason_code = STOP_REASON_EMERGENCY;
    if (main_decision->estop().has_reason()) {
      summary->control_reason = main_decision->estop().reason();
    }
    return;
  }
  if (main_decision->has_stop()) {
    const auto& stop = main_decision->stop();
    if (stop.has_reason_code()) {
      summary->has_stop_reason_code = true;
      summary->stop_reason_code = stop.reason_code();
      summary->control_reason = DescribeStopReasonCode(stop.reason_code());
    }
    if (stop.has_reason() && !stop.reason().empty()) {
      summary->control_reason = stop.reason();
    }
    return;
  }
  if (main_decision->has_not_ready()) {
    summary->control_reason = main_decision->not_ready().reason();
    if (summary->control_reason.empty() && input.planning_state != nullptr &&
        !input.planning_state->reason.empty()) {
      summary->control_reason = input.planning_state->reason;
    }
    return;
  }
  if (input.validation_should_hold && !input.validation_reason.empty()) {
    summary->control_reason = input.validation_reason;
  } else if (input.planning_state != nullptr &&
             !input.planning_state->reason.empty()) {
    summary->control_reason = input.planning_state->reason;
  }
}

StopClass InferStopClass(const PlanningSemanticInput& input,
                         const MainDecision* main_decision) {
  if (input.trajectory != nullptr && input.trajectory->has_estop() &&
      input.trajectory->estop().is_estop()) {
    return STOP_CLASS_MRM;
  }
  if ((input.planning_state != nullptr &&
       input.planning_state->resolved_mode == MODE_SAFETY_HOLD) ||
      input.validation_should_hold) {
    return STOP_CLASS_SAFETY_HOLD;
  }
  if (main_decision == nullptr) {
    return HasTrajectoryPoints(input.trajectory) ? STOP_CLASS_NONE
                                                 : STOP_CLASS_UNKNOWN;
  }
  if (main_decision->has_not_ready()) {
    return STOP_CLASS_SAFETY_HOLD;
  }
  if (main_decision->has_estop()) {
    return STOP_CLASS_MRM;
  }
  if (main_decision->has_mission_complete()) {
    return STOP_CLASS_TERMINAL_GOAL;
  }
  if (main_decision->has_stop()) {
    return ClassifyStopReason(main_decision->stop().reason_code());
  }
  return HasTrajectoryPoints(input.trajectory) ? STOP_CLASS_NONE
                                               : STOP_CLASS_UNKNOWN;
}

ExecutionPhase InferExecutionPhase(const PlanningSemanticSummary& summary) {
  if (summary.command_completed) {
    return EXECUTION_COMPLETED;
  }
  if (summary.stop_class == STOP_CLASS_SAFETY_HOLD ||
      summary.stop_class == STOP_CLASS_MRM) {
    return summary.full_stop_reached ? EXECUTION_HOLDING : EXECUTION_STOPPING;
  }
  if (summary.stop_class == STOP_CLASS_TERMINAL_GOAL) {
    if (summary.terminal_servo_authorized) {
      return EXECUTION_TERMINAL_ALIGN;
    }
    if (summary.full_stop_reached && summary.near_terminal) {
      return EXECUTION_HOLDING;
    }
    return summary.near_terminal ? EXECUTION_APPROACH : EXECUTION_ENROUTE;
  }
  if (summary.stop_class == STOP_CLASS_PRE_MANEUVER ||
      summary.stop_class == STOP_CLASS_REGULATORY ||
      summary.stop_class == STOP_CLASS_DYNAMIC_OBSTACLE ||
      summary.stop_class == STOP_CLASS_STATIC_OBSTACLE) {
    return summary.full_stop_reached ? EXECUTION_HOLDING : EXECUTION_STOPPING;
  }
  return summary.near_terminal ? EXECUTION_APPROACH : EXECUTION_ENROUTE;
}

TrackingMode InferTrackingMode(const PlanningSemanticSummary& summary,
                               const PlanningSemanticInput& input) {
  if (summary.terminal_servo_authorized) {
    return TRACKING_MODE_POSE_SERVO;
  }
  if (summary.stop_class == STOP_CLASS_SAFETY_HOLD ||
      !HasTrajectoryPoints(input.trajectory)) {
    return TRACKING_MODE_STANDSTILL_HOLD;
  }
  return TRACKING_MODE_TRAJECTORY;
}

LongitudinalIntent InferLongitudinalIntent(
    const PlanningSemanticSummary& summary) {
  switch (summary.stop_class) {
    case STOP_CLASS_TERMINAL_GOAL:
      return summary.near_terminal ? LON_INTENT_PRECISE_STOP
                                   : LON_INTENT_APPROACH_STOP;
    case STOP_CLASS_REGULATORY:
    case STOP_CLASS_DYNAMIC_OBSTACLE:
    case STOP_CLASS_STATIC_OBSTACLE:
      return LON_INTENT_YIELD_STOP;
    case STOP_CLASS_PRE_MANEUVER:
      return LON_INTENT_APPROACH_STOP;
    case STOP_CLASS_SAFETY_HOLD:
      return LON_INTENT_HOLD_STOP;
    case STOP_CLASS_MRM:
      return LON_INTENT_MRM_STOP;
    case STOP_CLASS_NONE:
      return LON_INTENT_CRUISE;
    case STOP_CLASS_UNKNOWN:
    default:
      return LON_INTENT_UNKNOWN;
  }
}

LateralIntent InferLateralIntent(const PlanningSemanticSummary& summary) {
  if (summary.terminal_servo_authorized) {
    return LAT_INTENT_ALIGN_GOAL_HEADING;
  }
  if (summary.stop_class == STOP_CLASS_TERMINAL_GOAL && summary.near_terminal) {
    return summary.has_target_stop_heading ? LAT_INTENT_ALIGN_GOAL_HEADING
                                           : LAT_INTENT_MINIMIZE_STEER;
  }
  if (summary.near_terminal &&
      summary.stop_class != STOP_CLASS_NONE &&
      summary.stop_class != STOP_CLASS_UNKNOWN) {
    return LAT_INTENT_STABILIZE_NEAR_STOP;
  }
  return LAT_INTENT_TRACK_PATH;
}

bool ShouldMarkTerminalCommandCompleted(
    const PlanningSemanticSummary& summary) {
  if (summary.stop_class != STOP_CLASS_TERMINAL_GOAL ||
      !summary.full_stop_reached || !summary.has_position_tolerance ||
      !summary.within_position_tolerance) {
    return false;
  }
  return !summary.has_heading_tolerance || summary.within_heading_tolerance;
}

bool CanAuthorizeTerminalServo(const PlanningSemanticSummary& summary,
                               const PlanningSemanticInput& input) {
  if (summary.stop_class != STOP_CLASS_TERMINAL_GOAL || !summary.near_terminal ||
      !summary.has_target_stop_point || !summary.has_target_stop_heading ||
      !summary.has_heading_tolerance || summary.within_heading_tolerance ||
      input.validation_should_hold) {
    return false;
  }
  if (input.planning_state != nullptr &&
      input.planning_state->resolved_mode == MODE_SAFETY_HOLD) {
    return false;
  }
  return IsTerminalServoEntrySpeed(input.chassis);
}

}  // namespace

StopClass ClassifyStopReason(StopReasonCode reason_code) {
  switch (reason_code) {
    case STOP_REASON_DESTINATION:
    case STOP_REASON_PULL_OVER:
      return STOP_CLASS_TERMINAL_GOAL;
    case STOP_REASON_SIGNAL:
    case STOP_REASON_STOP_SIGN:
    case STOP_REASON_YIELD_SIGN:
    case STOP_REASON_CLEAR_ZONE:
    case STOP_REASON_CROSSWALK:
    case STOP_REASON_CREEPER:
    case STOP_REASON_YELLOW_SIGNAL:
    case STOP_REASON_LANE_CHANGE_URGENCY:
      return STOP_CLASS_REGULATORY;
    case STOP_REASON_HEAD_VEHICLE:
    case STOP_REASON_PEDESTRIAN:
      return STOP_CLASS_DYNAMIC_OBSTACLE;
    case STOP_REASON_OBSTACLE:
    case STOP_REASON_REFERENCE_END:
    case STOP_REASON_SIDEPASS_SAFETY:
      return STOP_CLASS_STATIC_OBSTACLE;
    case STOP_REASON_PREPARKING:
    case STOP_REASON_PRE_OPEN_SPACE_STOP:
      return STOP_CLASS_PRE_MANEUVER;
    case STOP_REASON_EMERGENCY:
      return STOP_CLASS_MRM;
    default:
      return STOP_CLASS_UNKNOWN;
  }
}

PlanningSemanticSummary InferPlanningSemantics(
    const PlanningSemanticInput& input, RuntimeState default_runtime_state) {
  PlanningSemanticSummary summary;
  summary.runtime_state = default_runtime_state;
  summary.full_stop_reached = IsFullStop(input.chassis);
  summary.terminal_position_tolerance_m = kPositionToleranceM;
  summary.terminal_heading_tolerance_rad = kHeadingToleranceRad;
  summary.max_terminal_speed_mps = kTerminalServoMaxSpeedMps;
  summary.terminal_servo_timeout_sec = kTerminalServoTimeoutSec;

  const MainDecision* main_decision = GetMainDecision(input.trajectory);
  summary.stop_class = InferStopClass(input, main_decision);
  PopulateStopTargetFromDecision(main_decision, &summary);
  PopulateStopReasonFromDecision(input, main_decision, &summary);

  if (summary.has_target_stop_point) {
    apollo::common::PointENU stop_point;
    stop_point.set_x(summary.target_stop_x);
    stop_point.set_y(summary.target_stop_y);
    stop_point.set_z(summary.target_stop_z);
    const double distance =
        DistanceToStopPoint(input.localization, stop_point);
    summary.near_terminal = std::isfinite(distance) &&
                            distance <= kNearTerminalDistanceM;
    summary.has_position_tolerance = std::isfinite(distance);
    summary.within_position_tolerance =
        std::isfinite(distance) && distance <= kPositionToleranceM;
  } else if (main_decision != nullptr && main_decision->has_mission_complete()) {
    summary.near_terminal = true;
  }

  if (summary.has_target_stop_heading) {
    const double heading_error =
        HeadingErrorToStopPoint(input.localization, summary.target_stop_heading);
    summary.has_heading_tolerance = std::isfinite(heading_error);
    summary.within_heading_tolerance =
        std::isfinite(heading_error) && heading_error <= kHeadingToleranceRad;
  }

  summary.terminal_servo_authorized = CanAuthorizeTerminalServo(summary, input);

  if (ShouldMarkTerminalCommandCompleted(summary)) {
    summary.command_completed = true;
    summary.runtime_state = RUNTIME_COMPLETED;
    summary.near_terminal = true;
    summary.completion_reason = "terminal goal tolerance satisfied";
  } else if (main_decision != nullptr && main_decision->has_mission_complete()) {
    summary.completion_reason = "legacy mission_complete pending terminal alignment";
  }

  summary.execution_phase = InferExecutionPhase(summary);
  return summary;
}

void ApplyPlanningSemanticsToRuntimeStatus(
    const PlanningSemanticSummary& summary,
    PlanningRuntimeStatus* runtime_status) {
  if (runtime_status == nullptr) {
    return;
  }
  runtime_status->set_state(summary.runtime_state);
  runtime_status->set_execution_phase(summary.execution_phase);
  runtime_status->set_stop_class(summary.stop_class);

  auto* completion = runtime_status->mutable_completion();
  completion->set_command_completed(summary.command_completed);
  completion->set_near_terminal(summary.near_terminal);
  completion->set_full_stop_reached(summary.full_stop_reached);
  if (summary.has_position_tolerance) {
    completion->set_within_position_tolerance(
        summary.within_position_tolerance);
  }
  if (summary.has_heading_tolerance) {
    completion->set_within_heading_tolerance(summary.within_heading_tolerance);
  }
  if (!summary.completion_reason.empty()) {
    completion->set_completion_reason(summary.completion_reason);
  }
}

void ApplyPlanningSemanticsToTrajectory(const PlanningSemanticSummary& summary,
                                       ADCTrajectory* trajectory) {
  if (trajectory == nullptr) {
    return;
  }
  PlanningSemanticInput input;
  input.trajectory = trajectory;
  auto* control_intent = trajectory->mutable_control_intent();
  control_intent->set_tracking_mode(InferTrackingMode(summary, input));
  control_intent->set_longitudinal_intent(
      InferLongitudinalIntent(summary));
  control_intent->set_lateral_intent(InferLateralIntent(summary));
  if (control_intent->tracking_mode() == TRACKING_MODE_POSE_SERVO) {
    control_intent->set_primitive_type(CONTROL_PRIMITIVE_POSE_SERVO);
  } else if (control_intent->tracking_mode() == TRACKING_MODE_STANDSTILL_HOLD ||
             control_intent->longitudinal_intent() == LON_INTENT_HOLD_STOP ||
             control_intent->longitudinal_intent() == LON_INTENT_MRM_STOP) {
    control_intent->set_primitive_type(CONTROL_PRIMITIVE_STANDSTILL_HOLD);
  } else if (control_intent->lateral_intent() == LAT_INTENT_ALIGN_GOAL_HEADING &&
             summary.has_target_stop_heading && summary.near_terminal) {
    control_intent->set_primitive_type(CONTROL_PRIMITIVE_HEADING_HOLD);
  } else {
    control_intent->set_primitive_type(CONTROL_PRIMITIVE_NONE);
  }
  control_intent->set_stop_class(summary.stop_class);
  control_intent->set_near_terminal(summary.near_terminal);
  control_intent->set_suppress_large_steer(
      summary.near_terminal &&
      summary.stop_class != STOP_CLASS_NONE &&
      summary.stop_class != STOP_CLASS_UNKNOWN);
  control_intent->set_require_full_stop(
      summary.stop_class != STOP_CLASS_NONE &&
      summary.stop_class != STOP_CLASS_UNKNOWN);
  control_intent->set_terminal_position_tolerance_m(
      summary.terminal_position_tolerance_m);
  control_intent->set_terminal_heading_tolerance_rad(
      summary.terminal_heading_tolerance_rad);
  control_intent->set_max_terminal_speed_mps(summary.max_terminal_speed_mps);
  control_intent->set_terminal_servo_timeout_sec(
      summary.terminal_servo_timeout_sec);
  if (!summary.control_reason.empty()) {
    control_intent->set_reason(summary.control_reason);
  }
  if (summary.has_stop_reason_code) {
    control_intent->set_stop_reason_code(summary.stop_reason_code);
  }
  if (summary.has_target_stop_point) {
    auto* stop_point = control_intent->mutable_target_stop_point();
    stop_point->set_x(summary.target_stop_x);
    stop_point->set_y(summary.target_stop_y);
    stop_point->set_z(summary.target_stop_z);
  }
  if (summary.has_target_stop_heading) {
    control_intent->set_target_stop_heading(summary.target_stop_heading);
  }
}

}  // namespace planning
}  // namespace apollo
