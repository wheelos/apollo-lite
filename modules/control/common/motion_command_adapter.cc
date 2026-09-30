#include "modules/control/common/motion_command_adapter.h"

namespace apollo {
namespace control {

bool MotionCommandAdapter::ToLegacyControllerInput(
    const planning::MotionExecutionCommand& command,
    planning::ADCTrajectory* trajectory, std::string* reason) const {
  if (trajectory == nullptr) {
    if (reason != nullptr) {
      *reason = "legacy controller input is null";
    }
    return false;
  }
  trajectory->Clear();
  trajectory->mutable_header()->CopyFrom(command.header());

  if (command.payload_case() ==
      planning::MotionExecutionCommand::kTrajectory) {
    trajectory->set_gear(command.trajectory().gear());
    for (const auto& point : command.trajectory().point()) {
      auto* output = trajectory->add_trajectory_point();
      output->mutable_path_point()->CopyFrom(point.path_point());
      output->set_v(point.speed_mps());
      output->set_a(point.acceleration_mps2());
      output->set_relative_time(point.relative_time_sec());
      if (point.has_jerk_mps3()) {
        output->set_da(point.jerk_mps3());
      }
    }
    trajectory->mutable_control_intent()->set_execution_channel(
        planning::EXECUTION_CHANNEL_TRAJECTORY);
    return true;
  }

  if (command.payload_case() !=
          planning::MotionExecutionCommand::kPrimitive ||
      command.primitive().type() !=
          planning::MOTION_PRIMITIVE_STANDSTILL_HOLD) {
    if (reason != nullptr) {
      *reason = "primitive executor backend is not implemented";
    }
    return false;
  }

  trajectory->set_gear(command.start_condition().expected_gear());
  for (double relative_time : {0.0, 0.1}) {
    auto* point = trajectory->add_trajectory_point();
    point->mutable_path_point()->set_x(
        command.start_condition().expected_position().x());
    point->mutable_path_point()->set_y(
        command.start_condition().expected_position().y());
    point->mutable_path_point()->set_z(
        command.start_condition().expected_position().z());
    point->mutable_path_point()->set_theta(
        command.start_condition().expected_heading());
    point->mutable_path_point()->set_s(0.0);
    point->mutable_path_point()->set_kappa(0.0);
    point->mutable_path_point()->set_dkappa(0.0);
    point->set_v(0.0);
    point->set_a(0.0);
    point->set_relative_time(relative_time);
  }
  auto* intent = trajectory->mutable_control_intent();
  intent->set_tracking_mode(planning::TRACKING_MODE_STANDSTILL_HOLD);
  intent->set_longitudinal_intent(planning::LON_INTENT_HOLD_STOP);
  intent->set_lateral_intent(planning::LAT_INTENT_MINIMIZE_STEER);
  intent->set_require_full_stop(true);
  intent->set_execution_channel(planning::EXECUTION_CHANNEL_PRIMITIVE);
  intent->set_primitive_type(planning::CONTROL_PRIMITIVE_STANDSTILL_HOLD);
  return true;
}

}  // namespace control
}  // namespace apollo
