#pragma once

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"

namespace apollo {
namespace control {

struct TerminalLateralControlAdjustment {
  bool terminal_align_active = false;
  bool suppress_large_steer = false;
  double max_abs_steer_pct = 100.0;
  double max_steer_rate_pct = 100.0;
  double heading_correction_pct = 0.0;
};

struct TerminalLongitudinalControlAdjustment {
  bool pose_servo_active = false;
  bool trajectory_optional = false;
  bool full_stop = false;
  double signed_distance_m = 0.0;
  double desired_speed_mps = 0.0;
  double desired_acceleration_mps2 = 0.0;
};

bool IsTrajectorylessPoseServo(
    const apollo::planning::ADCTrajectory& trajectory);

TerminalLateralControlAdjustment BuildTerminalLateralControlAdjustment(
    const apollo::planning::ControlIntent& control_intent,
    double current_heading, double steer_ratio,
    double steer_single_direction_max_degree);

TerminalLongitudinalControlAdjustment BuildTerminalLongitudinalControlAdjustment(
    const apollo::planning::ControlIntent& control_intent,
    const apollo::localization::LocalizationEstimate* localization,
    const apollo::canbus::Chassis* chassis);

}  // namespace control
}  // namespace apollo
