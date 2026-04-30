#include "modules/control/common/terminal_control_helper.h"

#include <algorithm>
#include <cmath>

#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"

namespace apollo {
namespace control {

namespace {

constexpr double kTerminalAlignSteerLimitPct = 20.0;
constexpr double kTerminalAlignSteerRateLimitPct = 6.0;
constexpr double kSuppressLargeSteerLimitPct = 35.0;
constexpr double kSuppressLargeSteerRateLimitPct = 10.0;
constexpr double kTerminalHeadingCorrectionGain = 0.3;
constexpr double kTerminalHeadingCorrectionLimitPct = 10.0;
constexpr double kLateralHoldCorrectionGainPctPerM = 18.0;
constexpr double kLateralHoldCorrectionLimitPct = 12.0;
constexpr double kTerminalServoSpeedGain = 0.8;
constexpr double kTerminalServoAccelGain = 1.5;
constexpr double kTerminalServoAccelLimit = 1.0;
constexpr double kTerminalServoDecelLimit = 1.5;
constexpr double kPrimitiveSpeedAccelGain = 1.2;
constexpr double kPrimitiveSpeedAccelLimit = 1.0;
constexpr double kPrimitiveSpeedDecelLimit = 1.5;

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

bool HasReferenceLineTarget(const apollo::planning::ControlIntent& control_intent) {
  return control_intent.has_reference_line_point() &&
         control_intent.has_reference_line_heading();
}

double SignedLateralErrorToReferenceLine(
    const apollo::planning::ControlIntent& control_intent,
    const apollo::localization::LocalizationEstimate* localization) {
  if (localization == nullptr || !localization->has_pose() ||
      !HasReferenceLineTarget(control_intent)) {
    return 0.0;
  }
  const double dx = localization->pose().position().x() -
                    control_intent.reference_line_point().x();
  const double dy = localization->pose().position().y() -
                    control_intent.reference_line_point().y();
  const double signed_lateral_position =
      -std::sin(control_intent.reference_line_heading()) * dx +
      std::cos(control_intent.reference_line_heading()) * dy;
  const double target_offset =
      control_intent.has_target_lateral_offset_m()
          ? control_intent.target_lateral_offset_m()
          : 0.0;
  return signed_lateral_position - target_offset;
}

}  // namespace

bool IsTrajectorylessControlPrimitive(
    const apollo::planning::ADCTrajectory& trajectory) {
  if (!trajectory.trajectory_point().empty() || !trajectory.has_control_intent()) {
    return false;
  }
  const auto& control_intent = trajectory.control_intent();
  if (control_intent.primitive_type() ==
      apollo::planning::CONTROL_PRIMITIVE_POSE_SERVO) {
    return control_intent.has_target_stop_point();
  }
  if (control_intent.tracking_mode() ==
          apollo::planning::TRACKING_MODE_POSE_SERVO &&
      control_intent.has_target_stop_point()) {
    return true;
  }
  switch (control_intent.primitive_type()) {
    case apollo::planning::CONTROL_PRIMITIVE_STANDSTILL_HOLD:
      return true;
    case apollo::planning::CONTROL_PRIMITIVE_HEADING_HOLD:
      return control_intent.has_target_stop_heading();
    case apollo::planning::CONTROL_PRIMITIVE_LATERAL_HOLD:
      return HasReferenceLineTarget(control_intent);
    case apollo::planning::CONTROL_PRIMITIVE_POSE_SERVO:
      return control_intent.has_target_stop_point();
    case apollo::planning::CONTROL_PRIMITIVE_NONE:
    default:
      return false;
  }
}

bool IsTrajectorylessPoseServo(
    const apollo::planning::ADCTrajectory& trajectory) {
  return IsTrajectorylessControlPrimitive(trajectory) &&
         trajectory.has_control_intent() &&
         ((trajectory.control_intent().tracking_mode() ==
               apollo::planning::TRACKING_MODE_POSE_SERVO &&
           trajectory.control_intent().has_target_stop_point()) ||
          trajectory.control_intent().primitive_type() ==
              apollo::planning::CONTROL_PRIMITIVE_POSE_SERVO);
}

TerminalLateralControlAdjustment BuildTerminalLateralControlAdjustment(
    const apollo::planning::ControlIntent& control_intent,
    const apollo::localization::LocalizationEstimate* localization,
    double current_heading, double steer_ratio,
    double steer_single_direction_max_degree) {
  TerminalLateralControlAdjustment adjustment;
  adjustment.primitive_active =
      control_intent.primitive_type() != apollo::planning::CONTROL_PRIMITIVE_NONE;
  adjustment.suppress_large_steer = control_intent.suppress_large_steer();

  if (adjustment.suppress_large_steer) {
    adjustment.max_abs_steer_pct = kSuppressLargeSteerLimitPct;
    adjustment.max_steer_rate_pct = kSuppressLargeSteerRateLimitPct;
  }

  const bool wants_terminal_align =
      control_intent.primitive_type() ==
          apollo::planning::CONTROL_PRIMITIVE_HEADING_HOLD ||
      control_intent.primitive_type() ==
          apollo::planning::CONTROL_PRIMITIVE_POSE_SERVO ||
      control_intent.tracking_mode() ==
          apollo::planning::TRACKING_MODE_POSE_SERVO ||
      control_intent.lateral_intent() ==
          apollo::planning::LAT_INTENT_ALIGN_GOAL_HEADING;
  const bool wants_lateral_hold =
      control_intent.primitive_type() ==
          apollo::planning::CONTROL_PRIMITIVE_LATERAL_HOLD &&
      control_intent.has_reference_line_heading();
  if (!wants_terminal_align && !wants_lateral_hold) {
    return adjustment;
  }

  adjustment.terminal_align_active = true;
  adjustment.max_abs_steer_pct =
      std::min(adjustment.max_abs_steer_pct, kTerminalAlignSteerLimitPct);
  adjustment.max_steer_rate_pct =
      std::min(adjustment.max_steer_rate_pct, kTerminalAlignSteerRateLimitPct);

  double target_heading = current_heading;
  if (wants_lateral_hold) {
    target_heading = control_intent.reference_line_heading();
    adjustment.lateral_hold_active = true;
    adjustment.lateral_error_m =
        SignedLateralErrorToReferenceLine(control_intent, localization);
  } else if (control_intent.has_target_stop_heading()) {
    target_heading = control_intent.target_stop_heading();
  } else {
    return adjustment;
  }

  const double heading_error =
      common::math::NormalizeAngle(target_heading - current_heading);
  adjustment.heading_correction_pct =
      HeadingErrorToSteerPct(heading_error, steer_ratio,
                             steer_single_direction_max_degree) *
      kTerminalHeadingCorrectionGain;
  if (adjustment.lateral_hold_active) {
    adjustment.heading_correction_pct += common::math::Clamp(
        -adjustment.lateral_error_m * kLateralHoldCorrectionGainPctPerM,
        -kLateralHoldCorrectionLimitPct, kLateralHoldCorrectionLimitPct);
  }
  adjustment.heading_correction_pct = common::math::Clamp(
      adjustment.heading_correction_pct, -kTerminalHeadingCorrectionLimitPct,
      kTerminalHeadingCorrectionLimitPct);
  return adjustment;
}

TerminalLongitudinalControlAdjustment BuildTerminalLongitudinalControlAdjustment(
    const apollo::planning::ControlIntent& control_intent,
    const apollo::localization::LocalizationEstimate* localization,
    const apollo::canbus::Chassis* chassis) {
  TerminalLongitudinalControlAdjustment adjustment;
  adjustment.primitive_active =
      control_intent.primitive_type() != apollo::planning::CONTROL_PRIMITIVE_NONE;
  adjustment.pose_servo_active =
      control_intent.tracking_mode() == apollo::planning::TRACKING_MODE_POSE_SERVO ||
      control_intent.primitive_type() ==
          apollo::planning::CONTROL_PRIMITIVE_POSE_SERVO;
  adjustment.trajectory_optional =
      adjustment.pose_servo_active && control_intent.has_target_stop_point();

  if (control_intent.primitive_type() ==
      apollo::planning::CONTROL_PRIMITIVE_STANDSTILL_HOLD) {
    adjustment.trajectory_optional = true;
    adjustment.full_stop = true;
    return adjustment;
  }

  if (!adjustment.trajectory_optional &&
      (control_intent.primitive_type() ==
           apollo::planning::CONTROL_PRIMITIVE_HEADING_HOLD ||
       control_intent.primitive_type() ==
           apollo::planning::CONTROL_PRIMITIVE_LATERAL_HOLD)) {
    adjustment.trajectory_optional = true;
    if (chassis == nullptr) {
      return adjustment;
    }
    const double desired_speed =
        control_intent.has_primitive_speed_mps()
            ? control_intent.primitive_speed_mps()
            : 0.0;
    adjustment.desired_speed_mps = desired_speed;
    adjustment.desired_acceleration_mps2 = common::math::Clamp(
        (desired_speed - chassis->speed_mps()) * kPrimitiveSpeedAccelGain,
        -kPrimitiveSpeedDecelLimit, kPrimitiveSpeedAccelLimit);
    adjustment.full_stop = std::abs(desired_speed) <= 1e-3;
    return adjustment;
  }

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
