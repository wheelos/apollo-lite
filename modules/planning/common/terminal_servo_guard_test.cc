#include "modules/planning/common/terminal_servo_guard.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(TerminalServoGuardTest, TimeoutDegradesPoseServoToHold) {
  ADCTrajectory trajectory;
  auto* control_intent = trajectory.mutable_control_intent();
  control_intent->set_tracking_mode(TRACKING_MODE_POSE_SERVO);
  control_intent->set_longitudinal_intent(LON_INTENT_PRECISE_STOP);
  control_intent->set_lateral_intent(LAT_INTENT_ALIGN_GOAL_HEADING);
  control_intent->set_terminal_servo_timeout_sec(2.0);

  PlanningSemanticSummary summary;
  summary.runtime_state = RUNTIME_RUNNING;
  summary.execution_phase = EXECUTION_TERMINAL_ALIGN;
  summary.terminal_servo_authorized = true;
  summary.terminal_servo_timeout_sec = 2.0;

  TerminalServoSessionState session_state;
  EXPECT_TRUE(ApplyTerminalServoGuardrails("cmd", 1.0, &session_state, &summary,
                                           &trajectory)
                  .empty());
  const auto reason = ApplyTerminalServoGuardrails("cmd", 4.5, &session_state,
                                                   &summary, &trajectory);
  EXPECT_EQ(reason, "terminal servo timeout, degraded to hold");
  EXPECT_EQ(trajectory.control_intent().tracking_mode(),
            TRACKING_MODE_STANDSTILL_HOLD);
  EXPECT_EQ(summary.runtime_state, RUNTIME_HOLDING);
  EXPECT_EQ(summary.execution_phase, EXECUTION_HOLDING);
}

}  // namespace planning
}  // namespace apollo
