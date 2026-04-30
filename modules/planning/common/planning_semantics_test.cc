#include "modules/planning/common/planning_semantics.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

namespace {

PlanningSemanticInput BaseInput() {
  PlanningSemanticInput input;
  return input;
}

TEST(PlanningSemanticsTest, MissionCompletePublishesCompletedTerminalIntent) {
  ADCTrajectory trajectory;
  auto* mission_complete =
      trajectory.mutable_decision()->mutable_main_decision()->mutable_mission_complete();
  mission_complete->mutable_stop_point()->set_x(10.0);
  mission_complete->mutable_stop_point()->set_y(5.0);
  mission_complete->set_stop_heading(0.1);

  canbus::Chassis chassis;
  chassis.set_speed_mps(0.0);

  localization::LocalizationEstimate localization;
  localization.mutable_pose()->mutable_position()->set_x(10.1);
  localization.mutable_pose()->mutable_position()->set_y(5.1);
  localization.mutable_pose()->set_heading(0.12);

  auto input = BaseInput();
  input.trajectory = &trajectory;
  input.chassis = &chassis;
  input.localization = &localization;

  const auto summary = InferPlanningSemantics(input, RUNTIME_RUNNING);
  EXPECT_EQ(summary.runtime_state, RUNTIME_COMPLETED);
  EXPECT_EQ(summary.execution_phase, EXECUTION_COMPLETED);
  EXPECT_EQ(summary.stop_class, STOP_CLASS_TERMINAL_GOAL);
  EXPECT_TRUE(summary.command_completed);
  EXPECT_TRUE(summary.near_terminal);
  EXPECT_TRUE(summary.full_stop_reached);
  EXPECT_TRUE(summary.has_position_tolerance);
  EXPECT_TRUE(summary.within_position_tolerance);
  EXPECT_TRUE(summary.has_heading_tolerance);
  EXPECT_TRUE(summary.within_heading_tolerance);

  ApplyPlanningSemanticsToTrajectory(summary, &trajectory);
  ASSERT_TRUE(trajectory.has_control_intent());
  EXPECT_TRUE(trajectory.control_intent().has_stop_reason_code());
  EXPECT_EQ(trajectory.control_intent().stop_reason_code(),
            STOP_REASON_DESTINATION);
  EXPECT_EQ(trajectory.control_intent().reason(), "stop for mission destination");
  EXPECT_EQ(trajectory.control_intent().tracking_mode(),
            TRACKING_MODE_STANDSTILL_HOLD);
  EXPECT_EQ(trajectory.control_intent().longitudinal_intent(),
            LON_INTENT_PRECISE_STOP);
  EXPECT_EQ(trajectory.control_intent().lateral_intent(),
            LAT_INTENT_ALIGN_GOAL_HEADING);
  EXPECT_TRUE(trajectory.control_intent().suppress_large_steer());
}

TEST(PlanningSemanticsTest, SafetyHoldPublishesHoldIntent) {
  ADCTrajectory trajectory;
  canbus::Chassis chassis;
  chassis.set_speed_mps(0.0);

  PlanningCoordinatorState planning_state;
  planning_state.resolved_mode = MODE_SAFETY_HOLD;
  planning_state.reason = "stop for traffic signal";

  auto input = BaseInput();
  input.trajectory = &trajectory;
  input.chassis = &chassis;
  input.planning_state = &planning_state;

  const auto summary = InferPlanningSemantics(input, RUNTIME_DEGRADED);
  EXPECT_EQ(summary.runtime_state, RUNTIME_DEGRADED);
  EXPECT_EQ(summary.stop_class, STOP_CLASS_SAFETY_HOLD);
  EXPECT_EQ(summary.execution_phase, EXECUTION_HOLDING);
  EXPECT_TRUE(summary.full_stop_reached);

  ApplyPlanningSemanticsToTrajectory(summary, &trajectory);
  EXPECT_EQ(trajectory.control_intent().reason(), "stop for traffic signal");
  EXPECT_FALSE(trajectory.control_intent().has_stop_reason_code());
  EXPECT_EQ(trajectory.control_intent().tracking_mode(),
            TRACKING_MODE_STANDSTILL_HOLD);
  EXPECT_EQ(trajectory.control_intent().longitudinal_intent(),
            LON_INTENT_HOLD_STOP);
}

TEST(PlanningSemanticsTest, RegulatoryStopMapsToYieldIntent) {
  ADCTrajectory trajectory;
  auto* stop =
      trajectory.mutable_decision()->mutable_main_decision()->mutable_stop();
  stop->set_reason_code(STOP_REASON_SIGNAL);
  stop->mutable_stop_point()->set_x(20.0);
  stop->mutable_stop_point()->set_y(0.0);

  canbus::Chassis chassis;
  chassis.set_speed_mps(2.0);

  localization::LocalizationEstimate localization;
  localization.mutable_pose()->mutable_position()->set_x(0.0);
  localization.mutable_pose()->mutable_position()->set_y(0.0);
  localization.mutable_pose()->set_heading(0.0);

  auto input = BaseInput();
  input.trajectory = &trajectory;
  input.chassis = &chassis;
  input.localization = &localization;

  const auto summary = InferPlanningSemantics(input, RUNTIME_RUNNING);
  EXPECT_EQ(summary.runtime_state, RUNTIME_RUNNING);
  EXPECT_EQ(summary.stop_class, STOP_CLASS_REGULATORY);
  EXPECT_EQ(summary.execution_phase, EXECUTION_STOPPING);
  EXPECT_FALSE(summary.near_terminal);
  EXPECT_FALSE(summary.full_stop_reached);

  ApplyPlanningSemanticsToTrajectory(summary, &trajectory);
  EXPECT_EQ(trajectory.control_intent().tracking_mode(),
            TRACKING_MODE_STANDSTILL_HOLD);
  EXPECT_EQ(trajectory.control_intent().longitudinal_intent(),
            LON_INTENT_YIELD_STOP);
  EXPECT_EQ(trajectory.control_intent().lateral_intent(),
            LAT_INTENT_TRACK_PATH);
}

TEST(PlanningSemanticsTest, NearTerminalHeadingMismatchPublishesTerminalAlign) {
  ADCTrajectory trajectory;
  auto* stop =
      trajectory.mutable_decision()->mutable_main_decision()->mutable_stop();
  stop->set_reason_code(STOP_REASON_DESTINATION);
  stop->mutable_stop_point()->set_x(1.0);
  stop->mutable_stop_point()->set_y(0.0);
  stop->set_stop_heading(0.8);

  canbus::Chassis chassis;
  chassis.set_speed_mps(0.0);

  localization::LocalizationEstimate localization;
  localization.mutable_pose()->mutable_position()->set_x(0.9);
  localization.mutable_pose()->mutable_position()->set_y(0.0);
  localization.mutable_pose()->set_heading(0.0);

  auto input = BaseInput();
  input.trajectory = &trajectory;
  input.chassis = &chassis;
  input.localization = &localization;

  const auto summary = InferPlanningSemantics(input, RUNTIME_RUNNING);
  EXPECT_EQ(summary.execution_phase, EXECUTION_TERMINAL_ALIGN);
  EXPECT_EQ(summary.stop_class, STOP_CLASS_TERMINAL_GOAL);
  EXPECT_TRUE(summary.near_terminal);
  EXPECT_TRUE(summary.full_stop_reached);
  EXPECT_TRUE(summary.has_heading_tolerance);
  EXPECT_FALSE(summary.within_heading_tolerance);
  EXPECT_TRUE(summary.terminal_servo_authorized);

  ApplyPlanningSemanticsToTrajectory(summary, &trajectory);
  EXPECT_EQ(trajectory.control_intent().reason(), "stop for destination goal");
  ASSERT_TRUE(trajectory.control_intent().has_stop_reason_code());
  EXPECT_EQ(trajectory.control_intent().stop_reason_code(),
            STOP_REASON_DESTINATION);
  EXPECT_EQ(trajectory.control_intent().tracking_mode(),
            TRACKING_MODE_POSE_SERVO);
  EXPECT_EQ(trajectory.control_intent().lateral_intent(),
            LAT_INTENT_ALIGN_GOAL_HEADING);
}

TEST(PlanningSemanticsTest, TerminalGoalCompletionWaitsForHeadingTolerance) {
  ADCTrajectory trajectory;
  auto* mission_complete =
      trajectory.mutable_decision()->mutable_main_decision()->mutable_mission_complete();
  mission_complete->mutable_stop_point()->set_x(0.2);
  mission_complete->mutable_stop_point()->set_y(0.0);
  mission_complete->set_stop_heading(0.8);

  canbus::Chassis chassis;
  chassis.set_speed_mps(0.0);

  localization::LocalizationEstimate localization;
  localization.mutable_pose()->mutable_position()->set_x(0.2);
  localization.mutable_pose()->mutable_position()->set_y(0.0);
  localization.mutable_pose()->set_heading(0.0);

  auto input = BaseInput();
  input.trajectory = &trajectory;
  input.chassis = &chassis;
  input.localization = &localization;

  const auto summary = InferPlanningSemantics(input, RUNTIME_RUNNING);
  EXPECT_FALSE(summary.command_completed);
  EXPECT_EQ(summary.execution_phase, EXECUTION_TERMINAL_ALIGN);
  EXPECT_TRUE(summary.terminal_servo_authorized);
}

}  // namespace

}  // namespace planning
}  // namespace apollo
