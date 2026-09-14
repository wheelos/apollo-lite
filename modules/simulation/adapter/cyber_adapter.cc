// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2026-09-10
//  Author: daohu527

#include "modules/simulation/adapter/cyber_adapter.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "cyber/time/time.h"

namespace apollo {
namespace simulation {

bool CyberAdapter::Init(const std::shared_ptr<cyber::Node>& node) {
  if (!node) {
    AERROR << "Cyber node is null!";
    return false;
  }
  node_ = node;

  // Initialize Writer for Chassis and Localization
  chassis_writer_ =
      node_->CreateWriter<apollo::canbus::Chassis>(FLAGS_chassis_topic);
  chassis_detail_writer_ =
      node_->CreateWriter<apollo::canbus::ChassisDetail>(
          FLAGS_chassis_detail_topic);
  localization_writer_ =
      node_->CreateWriter<apollo::localization::LocalizationEstimate>(
          FLAGS_localization_topic);

  // Initialize Reader for ControlCommand
  control_reader_ = node_->CreateReader<apollo::control::ControlCommand>(
      FLAGS_control_command_topic,
      [this](const std::shared_ptr<apollo::control::ControlCommand>& msg) {
        OnControlCommand(msg);
      });

  AINFO << "CyberAdapter initialized. Subscribing to: "
        << FLAGS_control_command_topic
        << ", publishing to: " << FLAGS_chassis_topic << " and "
        << FLAGS_localization_topic << " and "
        << FLAGS_chassis_detail_topic;
  return true;
}

void CyberAdapter::OnControlCommand(
    const std::shared_ptr<apollo::control::ControlCommand>& msg) {
  if (!msg) return;

  std::lock_guard<std::mutex> lock(mutex_);
  FromControlCommand(*msg, &latest_command_, max_steer_angle_rad_);
  has_received_cmd_ = true;
}

void CyberAdapter::FromControlCommand(
    const apollo::control::ControlCommand& msg, VehicleCommand* cmd,
    double max_steer_angle_rad) {
  if (!cmd) return;

  *cmd = VehicleCommand{};
  cmd->timestamp_sec = msg.has_header() ? msg.header().timestamp_sec()
                                        : cyber::Time::Now().ToSecond();
  cmd->sequence_num = msg.has_header() ? msg.header().sequence_num() : 0;

  // Apollo ControlCommand uses percentage units [0, 100].
  double throttle = msg.throttle();
  double brake = msg.brake();
  if (!std::isfinite(throttle) || !std::isfinite(brake) ||
      throttle < 0.0 || throttle > 100.0 || brake < 0.0 || brake > 100.0 ||
      !std::isfinite(msg.steering_target()) || msg.steering_target() < -100.0 ||
      msg.steering_target() > 100.0 || !std::isfinite(msg.steering_rate()) ||
      msg.steering_rate() < -100.0 || msg.steering_rate() > 100.0 ||
      (msg.has_acceleration() &&
       (!std::isfinite(msg.acceleration()) || msg.acceleration() < -50.0 ||
        msg.acceleration() > 50.0)) ||
      (msg.has_speed() && (!std::isfinite(msg.speed()) ||
                           msg.speed() < -100.0 || msg.speed() > 100.0))) {
    AERROR << "Ignoring invalid ControlCommand input";
    cmd->emergency_stop = true;
    cmd->brake = 1.0;
    return;
  }
  cmd->throttle = throttle / 100.0;
  cmd->brake = brake / 100.0;

  // Lateral steering: percentage [-100, 100] -> radians
  cmd->front_steering_rad =
      (msg.steering_target() / 100.0) * max_steer_angle_rad;
  cmd->steering_rate_radps =
      (msg.steering_rate() / 100.0) * max_steer_angle_rad;

  if (msg.has_acceleration()) {
    cmd->target_acceleration_mps2 = msg.acceleration();
  }
  if (msg.has_speed()) {
    cmd->target_speed_mps = msg.speed();
  }

  // Gear mapping
  switch (msg.gear_location()) {
    case apollo::canbus::Chassis::GEAR_REVERSE:
      cmd->gear = VehicleCommand::Gear::GEAR_REVERSE;
      break;
    case apollo::canbus::Chassis::GEAR_PARKING:
      cmd->gear = VehicleCommand::Gear::GEAR_PARKING;
      break;
    case apollo::canbus::Chassis::GEAR_LOW:
      cmd->gear = VehicleCommand::Gear::GEAR_LOW;
      break;
    case apollo::canbus::Chassis::GEAR_NEUTRAL:
      cmd->gear = (cmd->throttle > 0.0) ? VehicleCommand::Gear::GEAR_DRIVE
                                        : VehicleCommand::Gear::GEAR_NEUTRAL;
      break;
    case apollo::canbus::Chassis::GEAR_DRIVE:
    default:
      cmd->gear = VehicleCommand::Gear::GEAR_DRIVE;
      break;
  }
}

bool CyberAdapter::PollCommand(VehicleCommand* cmd) {
  if (!cmd) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  *cmd = latest_command_;
  const bool command_received = has_received_cmd_;
  has_received_cmd_ = false;
  return command_received;
}

void CyberAdapter::ToChassis(const VehicleState& state,
                             apollo::canbus::Chassis* chassis,
                             double timestamp_sec) {
  if (!chassis) return;

  auto* header = chassis->mutable_header();
  header->set_timestamp_sec(timestamp_sec > 0.0 ? timestamp_sec
                                                : state.timestamp_sec);
  header->set_module_name("simulation");

  chassis->set_speed_mps(static_cast<float>(state.linear_velocity_mps));
  chassis->set_odometer_m(static_cast<float>(state.odometer_m));
  chassis->set_steering_percentage(
      static_cast<float>(state.steering_percentage));
  chassis->set_steering_percentage_cmd(
      static_cast<float>(state.steering_percentage_cmd));
  chassis->set_throttle_percentage(
      static_cast<float>(state.throttle_percentage));
  chassis->set_throttle_percentage_cmd(
      static_cast<float>(state.throttle_percentage_cmd));
  chassis->set_brake_percentage(static_cast<float>(state.brake_percentage));
  chassis->set_brake_percentage_cmd(
      static_cast<float>(state.brake_percentage_cmd));
  chassis->set_driving_mode(apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE);
  chassis->set_error_code(
      state.is_collision ? apollo::canbus::Chassis::CHASSIS_ERROR
                         : apollo::canbus::Chassis::NO_ERROR);
  chassis->set_chassis_error_mask(state.is_collision ? 1 : 0);
  chassis->set_engine_started(state.current_gear !=
                              VehicleCommand::Gear::GEAR_PARKING);
  chassis->set_parking_brake(
      state.current_gear == VehicleCommand::Gear::GEAR_PARKING);
  chassis->set_steering_timestamp(
      timestamp_sec > 0.0 ? timestamp_sec : state.timestamp_sec);
  const auto wheel_direction = [](double speed_mps) {
    return speed_mps > 0.01
          ? apollo::canbus::WheelSpeed::FORWARD
      : speed_mps < -0.01
          ? apollo::canbus::WheelSpeed::BACKWARD
          : apollo::canbus::WheelSpeed::STANDSTILL;
  };
  auto* wheel_speed = chassis->mutable_wheel_speed();
  wheel_speed->set_is_wheel_spd_fl_valid(true);
  wheel_speed->set_wheel_spd_fl(state.wheel_speed_mps[0]);
  wheel_speed->set_wheel_direction_fl(wheel_direction(state.wheel_speed_mps[0]));
  wheel_speed->set_is_wheel_spd_fr_valid(true);
  wheel_speed->set_wheel_spd_fr(state.wheel_speed_mps[1]);
  wheel_speed->set_wheel_direction_fr(wheel_direction(state.wheel_speed_mps[1]));
  wheel_speed->set_is_wheel_spd_rl_valid(true);
  wheel_speed->set_wheel_spd_rl(state.wheel_speed_mps[2]);
  wheel_speed->set_wheel_direction_rl(wheel_direction(state.wheel_speed_mps[2]));
  wheel_speed->set_is_wheel_spd_rr_valid(true);
  wheel_speed->set_wheel_spd_rr(state.wheel_speed_mps[3]);
  wheel_speed->set_wheel_direction_rr(wheel_direction(state.wheel_speed_mps[3]));

  switch (state.current_gear) {
    case VehicleCommand::Gear::GEAR_REVERSE:
      chassis->set_gear_location(apollo::canbus::Chassis::GEAR_REVERSE);
      break;
    case VehicleCommand::Gear::GEAR_PARKING:
      chassis->set_gear_location(apollo::canbus::Chassis::GEAR_PARKING);
      break;
    case VehicleCommand::Gear::GEAR_NEUTRAL:
      chassis->set_gear_location(apollo::canbus::Chassis::GEAR_NEUTRAL);
      break;
    case VehicleCommand::Gear::GEAR_LOW:
      chassis->set_gear_location(apollo::canbus::Chassis::GEAR_LOW);
      break;
    case VehicleCommand::Gear::GEAR_DRIVE:
    default:
      chassis->set_gear_location(apollo::canbus::Chassis::GEAR_DRIVE);
      break;
  }
}

void CyberAdapter::ToLocalization(
    const VehicleState& state, apollo::localization::LocalizationEstimate* loc,
    double timestamp_sec) {
  if (!loc) return;

  double t = timestamp_sec > 0.0 ? timestamp_sec : state.timestamp_sec;
  auto* header = loc->mutable_header();
  header->set_timestamp_sec(t);
  header->set_module_name("simulation");

  loc->set_measurement_time(t);
  auto* pose = loc->mutable_pose();

  // Position
  pose->mutable_position()->set_x(state.x);
  pose->mutable_position()->set_y(state.y);
  pose->mutable_position()->set_z(state.z);

  // Orientation
  pose->mutable_orientation()->set_qx(state.qx);
  pose->mutable_orientation()->set_qy(state.qy);
  pose->mutable_orientation()->set_qz(state.qz);
  pose->mutable_orientation()->set_qw(state.qw);
  pose->set_heading(state.yaw);
  pose->mutable_euler_angles()->set_x(state.pitch);
  pose->mutable_euler_angles()->set_y(state.roll);
  pose->mutable_euler_angles()->set_z(state.yaw);

  pose->mutable_linear_velocity()->set_x(
      state.linear_velocity_world_mps[0]);
  pose->mutable_linear_velocity()->set_y(
      state.linear_velocity_world_mps[1]);
  pose->mutable_linear_velocity()->set_z(
      state.linear_velocity_world_mps[2]);

  pose->mutable_linear_acceleration()->set_x(
      state.linear_acceleration_world_mps2[0]);
  pose->mutable_linear_acceleration()->set_y(
      state.linear_acceleration_world_mps2[1]);
  pose->mutable_linear_acceleration()->set_z(
      state.linear_acceleration_world_mps2[2]);

  // Linear acceleration in vehicle reference frame (VRF)
  // Apollo VRF uses x=right, y=forward. The simulator state uses
  // x=forward, y=left, so lateral acceleration changes sign on conversion.
  pose->mutable_linear_acceleration_vrf()->set_x(
      -state.linear_acceleration_body_mps2[1]);
  pose->mutable_linear_acceleration_vrf()->set_y(
      state.linear_acceleration_body_mps2[0]);
  pose->mutable_linear_acceleration_vrf()->set_z(
      state.linear_acceleration_body_mps2[2]);

  // Angular velocity in world and vehicle frame
  pose->mutable_angular_velocity()->set_x(
      state.angular_velocity_world_radps[0]);
  pose->mutable_angular_velocity()->set_y(
      state.angular_velocity_world_radps[1]);
  pose->mutable_angular_velocity()->set_z(
      state.angular_velocity_world_radps[2]);

  // Apollo VRF is right/forward/up, while the simulator body frame is
  // forward/left/up.
  pose->mutable_angular_velocity_vrf()->set_x(
      -state.angular_velocity_body_radps[1]);
  pose->mutable_angular_velocity_vrf()->set_y(
      state.angular_velocity_body_radps[0]);
  pose->mutable_angular_velocity_vrf()->set_z(
      state.angular_velocity_body_radps[2]);
}

void CyberAdapter::PublishFeedback(const VehicleState& state) {
  if (!timestamp_offset_initialized_) {
    timestamp_offset_sec_ = cyber::Time::Now().ToSecond() - state.timestamp_sec;
    timestamp_offset_initialized_ = true;
  }
  const double timestamp_sec = timestamp_offset_sec_ + state.timestamp_sec;
  uint64_t seq = ++msg_seq_num_;

  // 1. Publish Chassis
  if (chassis_writer_) {
    auto chassis = std::make_shared<apollo::canbus::Chassis>();
    ToChassis(state, chassis.get(), timestamp_sec);
    chassis->mutable_header()->set_sequence_num(seq);
    chassis_writer_->Write(chassis);
  }

  // ChassisDetail.Any is reserved for a vehicle-specific protocol extension.
  if (chassis_detail_writer_) {
    auto chassis_detail = std::make_shared<apollo::canbus::ChassisDetail>();
    if (chassis_detail_filler_) {
      chassis_detail_filler_(state, chassis_detail.get());
    }
    chassis_detail_writer_->Write(chassis_detail);
  }

  // 2. Publish LocalizationEstimate
  if (localization_writer_) {
    auto loc = std::make_shared<apollo::localization::LocalizationEstimate>();
    ToLocalization(state, loc.get(), timestamp_sec);
    loc->mutable_header()->set_sequence_num(seq);
    localization_writer_->Write(loc);
  }
}

}  // namespace simulation
}  // namespace apollo
