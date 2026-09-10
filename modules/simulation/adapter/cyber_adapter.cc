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
        << FLAGS_localization_topic;
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

  // Longitudinal throttle and brake [0, 1]
  double throttle = msg.throttle();
  double brake = msg.brake();
  const bool throttle_percent = throttle > 1.0;
  const bool brake_percent = brake > 1.0;
  if (!std::isfinite(throttle) || !std::isfinite(brake) ||
      (throttle_percent && throttle > 100.0) ||
      (!throttle_percent && throttle < 0.0) ||
      (brake_percent && brake > 100.0) || (!brake_percent && brake < 0.0) ||
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
  cmd->throttle = throttle > 1.0 ? throttle / 100.0 : throttle;

  cmd->brake = brake > 1.0 ? brake / 100.0 : brake;

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
  chassis->set_steering_percentage(
      static_cast<float>(state.steering_percentage));
  chassis->set_throttle_percentage(
      static_cast<float>(state.throttle_percentage));
  chassis->set_brake_percentage(static_cast<float>(state.brake_percentage));
  chassis->set_driving_mode(apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE);
  chassis->set_error_code(apollo::canbus::Chassis::NO_ERROR);
  chassis->set_engine_started(true);

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

  // Body-frame velocity/acceleration rotated into the world frame.
  double vx = state.linear_velocity_mps * std::cos(state.yaw) -
              state.lateral_velocity_mps * std::sin(state.yaw);
  double vy = state.linear_velocity_mps * std::sin(state.yaw) +
              state.lateral_velocity_mps * std::cos(state.yaw);
  pose->mutable_linear_velocity()->set_x(vx);
  pose->mutable_linear_velocity()->set_y(vy);
  pose->mutable_linear_velocity()->set_z(0.0);

  double ax = state.linear_acceleration_mps2 * std::cos(state.yaw) -
              state.lateral_acceleration_mps2 * std::sin(state.yaw);
  double ay = state.linear_acceleration_mps2 * std::sin(state.yaw) +
              state.lateral_acceleration_mps2 * std::cos(state.yaw);
  pose->mutable_linear_acceleration()->set_x(ax);
  pose->mutable_linear_acceleration()->set_y(ay);
  pose->mutable_linear_acceleration()->set_z(0.0);

  // Linear acceleration in vehicle reference frame (VRF)
  pose->mutable_linear_acceleration_vrf()->set_x(
      state.linear_acceleration_mps2);
  pose->mutable_linear_acceleration_vrf()->set_y(
      state.lateral_acceleration_mps2);
  pose->mutable_linear_acceleration_vrf()->set_z(0.0);

  // Angular velocity in world and vehicle frame
  pose->mutable_angular_velocity()->set_x(0.0);
  pose->mutable_angular_velocity()->set_y(0.0);
  pose->mutable_angular_velocity()->set_z(state.angular_velocity_yaw_radps);

  pose->mutable_angular_velocity_vrf()->set_x(0.0);
  pose->mutable_angular_velocity_vrf()->set_y(0.0);
  pose->mutable_angular_velocity_vrf()->set_z(state.angular_velocity_yaw_radps);
}

void CyberAdapter::PublishFeedback(const VehicleState& state) {
  double now_sec = cyber::Time::Now().ToSecond();
  uint64_t seq = ++msg_seq_num_;

  // 1. Publish Chassis
  if (chassis_writer_) {
    auto chassis = std::make_shared<apollo::canbus::Chassis>();
    ToChassis(state, chassis.get(), now_sec);
    chassis->mutable_header()->set_sequence_num(seq);
    chassis_writer_->Write(chassis);
  }

  // 2. Publish LocalizationEstimate
  if (localization_writer_) {
    auto loc = std::make_shared<apollo::localization::LocalizationEstimate>();
    ToLocalization(state, loc.get(), now_sec);
    loc->mutable_header()->set_sequence_num(seq);
    localization_writer_->Write(loc);
  }
}

}  // namespace simulation
}  // namespace apollo
