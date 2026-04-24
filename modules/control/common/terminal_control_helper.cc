#include "modules/control/common/terminal_control_helper.h"

#include <algorithm>
#include <cmath>

#include "modules/common/math/math_utils.h"

namespace apollo {
namespace control {

namespace {

constexpr double kTerminalAlignSteerLimitPct = 20.0;
constexpr double kTerminalAlignSteerRateLimitPct = 6.0;
constexpr double kSuppressLargeSteerLimitPct = 35.0;
constexpr double kSuppressLargeSteerRateLimitPct = 10.0;
constexpr double kTerminalHeadingCorrectionGain = 0.3;
constexpr double kTerminalHeadingCorrectionLimitPct = 10.0;
constexpr double kTerminalServoSpeedGain = 0.8;
constexpr double kTerminalServoAccelGain = 1.5;
constexpr double kTerminalServoAccelLimit = 1.0;
constexpr double kTerminalServoDecelLimit = 1.5;

double CurrentHeading(
    const apollo::localization::LocalizationEstimate* localization) {
  if (localization == nullptr || !localization->has_pose()) {
    return 0.0;
  }
  if (localization->pose().has_heading()) {
    return localization->pose().heading();
  }
  return common::math::QuaternionToHeading(localization->pose().orientation().qw(),
                                           localization->pose().orientation().qx(),
                                           localization->pose().orientation().qy(),
                                           localization->pose().orientation().qz());
}

double HeadingErrorToSteerPct(double heading_error, double steer_ratio,
                              double steer_single_direction_max_degree) {
  if (steer_single_direction_max_degree <= 0.0) {
    return 0.0;
  }
  return heading_error * 180.0 / M_PI * steer_ratio /
         steer_single_direction_max_degree * 100.0;
}

}  // namespace

bool IsTrajectorylessPoseServo(
    const apollo::planning::ADCTrajectory& trajectory) {
  return trajectory.trajectory_point().empty() &&
         trajectory.has_control_intent() &&
         trajectory.control_intent().tracking_mode() ==
             apollo::planning::TRACKING_MODE_POSE_SERVO &&
         trajectory.control_intent().has_target_stop_point();
}

TerminalLateralControlAdjustment BuildTerminalLateralControlAdjustment(
    const apollo::planning::ControlIntent& control_intent,
    double current_heading, double steer_ratio,
    double steer_single_direction_max_degree) {
  TerminalLateralControlAdjustment adjustment;
  adjustment.suppress_large_steer = control_intent.suppress_large_steer();

  if (adjustment.suppress_large_steer) {
    adjustment.max_abs_steer_pct = kSuppressLargeSteerLimitPct;
    adjustment.max_steer_rate_pct = kSuppressLargeSteerRateLimitPct;
  }

  const bool wants_terminal_align =
      control_intent.tracking_mode() ==
          apollo::planning::TRACKING_MODE_POSE_SERVO ||
      control_intent.lateral_intent() ==
          apollo::planning::LAT_INTENT_ALIGN_GOAL_HEADING;
  if (!wants_terminal_align || !control_intent.has_target_stop_heading()) {
    return adjustment;
  }

  adjustment.terminal_align_active = true;
  adjustment.max_abs_steer_pct =
      std::min(adjustment.max_abs_steer_pct, kTerminalAlignSteerLimitPct);
  adjustment.max_steer_rate_pct =
      std::min(adjustment.max_steer_rate_pct, kTerminalAlignSteerRateLimitPct);

  const double heading_error = common::math::NormalizeAngle(
      control_intent.target_stop_heading() - current_heading);
  adjustment.heading_correction_pct = common::math::Clamp(
      HeadingErrorToSteerPct(heading_error, steer_ratio,
                             steer_single_direction_max_degree) *
          kTerminalHeadingCorrectionGain,
      -kTerminalHeadingCorrectionLimitPct,
      kTerminalHeadingCorrectionLimitPct);
  return adjustment;
}

TerminalLongitudinalControlAdjustment BuildTerminalLongitudinalControlAdjustment(
    const apollo::planning::ControlIntent& control_intent,
    const apollo::localization::LocalizationEstimate* localization,
    const apollo::canbus::Chassis* chassis) {
  TerminalLongitudinalControlAdjustment adjustment;
  adjustment.pose_servo_active =
      control_intent.tracking_mode() == apollo::planning::TRACKING_MODE_POSE_SERVO;
  adjustment.trajectory_optional =
      adjustment.pose_servo_active && control_intent.has_target_stop_point();
  if (!adjustment.trajectory_optional || localization == nullptr ||
      chassis == nullptr || !localization->has_pose()) {
    return adjustment;
  }

  const double heading = CurrentHeading(localization);
  const double dx =
      control_intent.target_stop_point().x() - localization->pose().position().x();
  const double dy =
      control_intent.target_stop_point().y() - localization->pose().position().y();
  adjustment.signed_distance_m = dx * std::cos(heading) + dy * std::sin(heading);

  const double position_tolerance =
      control_intent.has_terminal_position_tolerance_m()
          ? control_intent.terminal_position_tolerance_m()
          : 0.5;
  const double max_terminal_speed =
      control_intent.has_max_terminal_speed_mps()
          ? control_intent.max_terminal_speed_mps()
          : 0.2;
  const double current_speed = chassis->speed_mps();

  adjustment.full_stop =
      std::abs(adjustment.signed_distance_m) <= position_tolerance;
  if (adjustment.full_stop) {
    adjustment.desired_speed_mps = 0.0;
    adjustment.desired_acceleration_mps2 = 0.0;
    return adjustment;
  }

  const double target_speed = common::math::Clamp(
      adjustment.signed_distance_m * kTerminalServoSpeedGain,
      -max_terminal_speed, max_terminal_speed);
  adjustment.desired_speed_mps = target_speed;
  adjustment.desired_acceleration_mps2 = common::math::Clamp(
      (target_speed - current_speed) * kTerminalServoAccelGain,
      -kTerminalServoDecelLimit, kTerminalServoAccelLimit);
  return adjustment;
}

}  // namespace control
}  // namespace apollo
