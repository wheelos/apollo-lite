#pragma once

#include <string>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/common_msgs/planning_msgs/planning_runtime_status.pb.h"
#include "modules/planning/planning_runtime_context.h"

namespace apollo {
namespace planning {

struct PlanningSemanticInput {
  const PlanningCoordinatorState* planning_state = nullptr;
  const canbus::Chassis* chassis = nullptr;
  const localization::LocalizationEstimate* localization = nullptr;
  const ADCTrajectory* trajectory = nullptr;
  bool validation_should_hold = false;
};

struct PlanningSemanticSummary {
  RuntimeState runtime_state = RUNTIME_UNKNOWN;
  ExecutionPhase execution_phase = EXECUTION_UNKNOWN;
  StopClass stop_class = STOP_CLASS_UNKNOWN;
  bool command_completed = false;
  bool near_terminal = false;
  bool has_position_tolerance = false;
  bool within_position_tolerance = false;
  bool has_heading_tolerance = false;
  bool within_heading_tolerance = false;
  bool full_stop_reached = false;
  bool has_target_stop_point = false;
  double target_stop_x = 0.0;
  double target_stop_y = 0.0;
  double target_stop_z = 0.0;
  bool has_target_stop_heading = false;
  double target_stop_heading = 0.0;
  bool terminal_servo_authorized = false;
  double terminal_position_tolerance_m = 0.0;
  double terminal_heading_tolerance_rad = 0.0;
  double max_terminal_speed_mps = 0.0;
  double terminal_servo_timeout_sec = 0.0;
  std::string completion_reason;
};

StopClass ClassifyStopReason(StopReasonCode reason_code);

PlanningSemanticSummary InferPlanningSemantics(
    const PlanningSemanticInput& input, RuntimeState default_runtime_state);

void ApplyPlanningSemanticsToRuntimeStatus(
    const PlanningSemanticSummary& summary,
    PlanningRuntimeStatus* runtime_status);

void ApplyPlanningSemanticsToTrajectory(const PlanningSemanticSummary& summary,
                                       ADCTrajectory* trajectory);

}  // namespace planning
}  // namespace apollo
