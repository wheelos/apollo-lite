#include "modules/planning/common/terminal_servo_guard.h"

namespace apollo {
namespace planning {

namespace {

void ClearSessionState(TerminalServoSessionState* session_state) {
  if (session_state == nullptr) {
    return;
  }
  session_state->command_id.clear();
  session_state->start_time_sec = -1.0;
}

}  // namespace

std::string ApplyTerminalServoGuardrails(
    const std::string& command_id, double now_sec,
    TerminalServoSessionState* session_state,
    PlanningSemanticSummary* semantic_summary, ADCTrajectory* trajectory) {
  if (session_state == nullptr || semantic_summary == nullptr ||
      trajectory == nullptr || !trajectory->has_control_intent()) {
    ClearSessionState(session_state);
    return "";
  }

  auto* control_intent = trajectory->mutable_control_intent();
  const bool pose_servo_active =
      control_intent->tracking_mode() == TRACKING_MODE_POSE_SERVO;
  if (!pose_servo_active || command_id.empty() ||
      semantic_summary->command_completed) {
    ClearSessionState(session_state);
    return "";
  }

  if (session_state->command_id != command_id ||
      session_state->start_time_sec < 0.0) {
    session_state->command_id = command_id;
    session_state->start_time_sec = now_sec;
    return "";
  }

  const double timeout_sec =
      control_intent->has_terminal_servo_timeout_sec()
          ? control_intent->terminal_servo_timeout_sec()
          : semantic_summary->terminal_servo_timeout_sec;
  if (timeout_sec <= 0.0 ||
      now_sec - session_state->start_time_sec <= timeout_sec) {
    return "";
  }

  control_intent->set_tracking_mode(TRACKING_MODE_STANDSTILL_HOLD);
  control_intent->set_longitudinal_intent(LON_INTENT_HOLD_STOP);
  control_intent->set_lateral_intent(LAT_INTENT_MINIMIZE_STEER);
  control_intent->set_suppress_large_steer(true);
  semantic_summary->terminal_servo_authorized = false;
  semantic_summary->execution_phase = EXECUTION_HOLDING;
  semantic_summary->runtime_state = RUNTIME_HOLDING;
  semantic_summary->completion_reason.clear();
  ClearSessionState(session_state);
  return "terminal servo timeout, degraded to hold";
}

}  // namespace planning
}  // namespace apollo
