#include "modules/control/common/terminal_control_helper.h"

#include "gtest/gtest.h"

namespace apollo {
namespace control {

TEST(TerminalControlHelperTest, BuildsTerminalAlignCorrection) {
  planning::ControlIntent intent;
  intent.set_tracking_mode(planning::TRACKING_MODE_POSE_SERVO);
  intent.set_lateral_intent(planning::LAT_INTENT_ALIGN_GOAL_HEADING);
  intent.set_target_stop_heading(0.4);

  localization::LocalizationEstimate localization;
  localization.mutable_pose()->set_heading(0.0);

  const auto adjustment =
      BuildTerminalLateralControlAdjustment(intent, &localization, 0.0, 16.0,
                                            470.0);
  EXPECT_TRUE(adjustment.terminal_align_active);
  EXPECT_FALSE(adjustment.suppress_large_steer);
  EXPECT_LE(adjustment.max_abs_steer_pct, 20.0);
  EXPECT_LE(adjustment.max_steer_rate_pct, 6.0);
  EXPECT_GT(adjustment.heading_correction_pct, 0.0);
}

TEST(TerminalControlHelperTest, PreservesSuppressLargeSteerForNonAlignIntent) {
  planning::ControlIntent intent;
  intent.set_suppress_large_steer(true);
  intent.set_lateral_intent(planning::LAT_INTENT_MINIMIZE_STEER);

  localization::LocalizationEstimate localization;
  localization.mutable_pose()->set_heading(0.0);

  const auto adjustment =
      BuildTerminalLateralControlAdjustment(intent, &localization, 0.0, 16.0,
                                            470.0);
  EXPECT_FALSE(adjustment.terminal_align_active);
  EXPECT_TRUE(adjustment.suppress_large_steer);
  EXPECT_EQ(adjustment.max_abs_steer_pct, 35.0);
  EXPECT_EQ(adjustment.max_steer_rate_pct, 10.0);
  EXPECT_EQ(adjustment.heading_correction_pct, 0.0);
}

TEST(TerminalControlHelperTest, BuildsTrajectorylessTerminalServoCommand) {
  planning::ADCTrajectory trajectory;
  auto* intent = trajectory.mutable_control_intent();
  intent->set_tracking_mode(planning::TRACKING_MODE_POSE_SERVO);
  intent->mutable_target_stop_point()->set_x(1.0);
  intent->mutable_target_stop_point()->set_y(0.0);
  intent->set_terminal_position_tolerance_m(0.2);
  intent->set_max_terminal_speed_mps(0.15);
  intent->set_primitive_type(planning::CONTROL_PRIMITIVE_POSE_SERVO);
  EXPECT_TRUE(IsTrajectorylessPoseServo(trajectory));
  EXPECT_TRUE(IsTrajectorylessControlPrimitive(trajectory));

  localization::LocalizationEstimate localization;
  localization.mutable_pose()->mutable_position()->set_x(0.7);
  localization.mutable_pose()->mutable_position()->set_y(0.0);
  localization.mutable_pose()->set_heading(0.0);

  canbus::Chassis chassis;
  chassis.set_speed_mps(0.0);

  const auto adjustment = BuildTerminalLongitudinalControlAdjustment(
      trajectory.control_intent(), &localization, &chassis);
  EXPECT_TRUE(adjustment.primitive_active);
  EXPECT_TRUE(adjustment.pose_servo_active);
  EXPECT_TRUE(adjustment.trajectory_optional);
  EXPECT_FALSE(adjustment.full_stop);
  EXPECT_GT(adjustment.desired_speed_mps, 0.0);
  EXPECT_LE(adjustment.desired_speed_mps, 0.15);
}

TEST(TerminalControlHelperTest, DetectsTerminalServoFullStop) {
  planning::ControlIntent intent;
  intent.set_tracking_mode(planning::TRACKING_MODE_POSE_SERVO);
  intent.mutable_target_stop_point()->set_x(0.1);
  intent.mutable_target_stop_point()->set_y(0.0);
  intent.set_terminal_position_tolerance_m(0.2);
  intent.set_primitive_type(planning::CONTROL_PRIMITIVE_POSE_SERVO);

  localization::LocalizationEstimate localization;
  localization.mutable_pose()->mutable_position()->set_x(0.0);
  localization.mutable_pose()->mutable_position()->set_y(0.0);
  localization.mutable_pose()->set_heading(0.0);

  canbus::Chassis chassis;
  chassis.set_speed_mps(0.03);

  const auto adjustment = BuildTerminalLongitudinalControlAdjustment(
      intent, &localization, &chassis);
  EXPECT_TRUE(adjustment.full_stop);
  EXPECT_EQ(adjustment.desired_speed_mps, 0.0);
  EXPECT_EQ(adjustment.desired_acceleration_mps2, 0.0);
}

TEST(TerminalControlHelperTest, BuildsHeadingHoldPrimitive) {
  planning::ADCTrajectory trajectory;
  auto* intent = trajectory.mutable_control_intent();
  intent->set_primitive_type(planning::CONTROL_PRIMITIVE_HEADING_HOLD);
  intent->set_target_stop_heading(0.2);

  localization::LocalizationEstimate localization;
  localization.mutable_pose()->set_heading(0.0);

  const auto adjustment = BuildTerminalLateralControlAdjustment(
      trajectory.control_intent(), &localization, 0.0, 16.0, 470.0);
  EXPECT_TRUE(IsTrajectorylessControlPrimitive(trajectory));
  EXPECT_TRUE(adjustment.primitive_active);
  EXPECT_TRUE(adjustment.terminal_align_active);
  EXPECT_GT(adjustment.heading_correction_pct, 0.0);
}

TEST(TerminalControlHelperTest, BuildsLateralHoldPrimitive) {
  planning::ADCTrajectory trajectory;
  auto* intent = trajectory.mutable_control_intent();
  intent->set_primitive_type(planning::CONTROL_PRIMITIVE_LATERAL_HOLD);
  intent->mutable_reference_line_point()->set_x(0.0);
  intent->mutable_reference_line_point()->set_y(0.0);
  intent->set_reference_line_heading(0.0);
  intent->set_target_lateral_offset_m(0.0);

  localization::LocalizationEstimate localization;
  localization.mutable_pose()->mutable_position()->set_x(0.0);
  localization.mutable_pose()->mutable_position()->set_y(1.0);
  localization.mutable_pose()->set_heading(0.0);

  const auto adjustment = BuildTerminalLateralControlAdjustment(
      trajectory.control_intent(), &localization, 0.0, 16.0, 470.0);
  EXPECT_TRUE(IsTrajectorylessControlPrimitive(trajectory));
  EXPECT_TRUE(adjustment.lateral_hold_active);
  EXPECT_NE(adjustment.lateral_error_m, 0.0);
  EXPECT_NE(adjustment.heading_correction_pct, 0.0);
}

TEST(TerminalControlHelperTest, BuildsStandstillHoldPrimitive) {
  planning::ControlIntent intent;
  intent.set_primitive_type(planning::CONTROL_PRIMITIVE_STANDSTILL_HOLD);

  canbus::Chassis chassis;
  chassis.set_speed_mps(0.2);

  const auto adjustment = BuildTerminalLongitudinalControlAdjustment(
      intent, nullptr, &chassis);
  EXPECT_TRUE(adjustment.primitive_active);
  EXPECT_TRUE(adjustment.trajectory_optional);
  EXPECT_TRUE(adjustment.full_stop);
  EXPECT_EQ(adjustment.desired_speed_mps, 0.0);
}

}  // namespace control
}  // namespace apollo
