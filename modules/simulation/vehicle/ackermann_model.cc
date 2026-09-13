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

#include "modules/simulation/vehicle/ackermann_model.h"

#include <algorithm>
#include <cmath>

namespace apollo {
namespace simulation {

VehicleActuation AckermannModel::ComputeActuation(
    const VehicleCommand& cmd, const VehicleState& /*current_state*/,
    double /*dt_sec*/) {
  VehicleActuation actuation{};

  // 1. Steering computation via front- or four-wheel Ackermann geometry.
  const double front_steer =
      std::max(-max_steer_angle_rad_,
               std::min(max_steer_angle_rad_, cmd.front_steering_rad));
  const double rear_steer =
      max_rear_steer_angle_rad_ > 0.0
          ? std::max(-max_rear_steer_angle_rad_,
                     std::min(max_rear_steer_angle_rad_,
                              -front_steer * max_rear_steer_angle_rad_ /
                                  max_steer_angle_rad_))
          : 0.0;

  if (std::abs(front_steer - rear_steer) < 1e-5) {
    actuation.wheel_steer_rad[0] = front_steer;  // FL
    actuation.wheel_steer_rad[1] = front_steer;  // FR
    actuation.wheel_steer_rad[2] = rear_steer;   // RL
    actuation.wheel_steer_rad[3] = rear_steer;   // RR
  } else {
    const double turning_radius =
        wheelbase_m_ / (std::tan(front_steer) - std::tan(rear_steer));
    const double icr_x = -turning_radius * std::tan(rear_steer);
    actuation.wheel_steer_rad[0] =
        std::atan((wheelbase_m_ - icr_x) /
                  (turning_radius - track_width_m_ * 0.5));
    actuation.wheel_steer_rad[1] =
        std::atan((wheelbase_m_ - icr_x) /
                  (turning_radius + track_width_m_ * 0.5));
    actuation.wheel_steer_rad[2] =
        std::atan(-icr_x / (turning_radius - track_width_m_ * 0.5));
    actuation.wheel_steer_rad[3] =
        std::atan(-icr_x / (turning_radius + track_width_m_ * 0.5));
  }
  actuation.wheel_steer_rad[0] =
      std::max(-max_steer_angle_rad_,
               std::min(max_steer_angle_rad_, actuation.wheel_steer_rad[0]));
  actuation.wheel_steer_rad[1] =
      std::max(-max_steer_angle_rad_,
               std::min(max_steer_angle_rad_, actuation.wheel_steer_rad[1]));
  actuation.wheel_steer_rad[2] =
      std::max(-max_rear_steer_angle_rad_,
               std::min(max_rear_steer_angle_rad_,
                        actuation.wheel_steer_rad[2]));
  actuation.wheel_steer_rad[3] =
      std::max(-max_rear_steer_angle_rad_,
               std::min(max_rear_steer_angle_rad_,
                        actuation.wheel_steer_rad[3]));

  // 2. Drive & Brake Torques
  if (cmd.emergency_stop || cmd.gear == VehicleCommand::Gear::GEAR_PARKING) {
    // Apply full parking / emergency brake
    double per_wheel_brake = max_brake_torque_nm_ * 0.25;
    for (int i = 0; i < 4; ++i) {
      actuation.drive_torque_nm[i] = 0.0;
      actuation.brake_torque_nm[i] = per_wheel_brake;
    }
    return actuation;
  }

  double total_drive_torque = 0.0;
  double total_brake_torque = 0.0;

  if (cmd.throttle > 0.0) {
    total_drive_torque = cmd.throttle * max_drive_torque_nm_;
  } else if (cmd.brake > 0.0) {
    total_brake_torque = cmd.brake * max_brake_torque_nm_;
  } else if (std::abs(cmd.target_acceleration_mps2) > 1e-3) {
    // Convert target acceleration to torque
    double req_force = mass_kg_ * cmd.target_acceleration_mps2;
    double req_torque = req_force * wheel_radius_m_;
    if (req_torque > 0.0) {
      total_drive_torque = std::min(req_torque, max_drive_torque_nm_);
    } else {
      total_brake_torque = std::min(-req_torque, max_brake_torque_nm_);
    }
  }

  // Adjust drive torque direction based on gear
  if (cmd.gear == VehicleCommand::Gear::GEAR_REVERSE) {
    total_drive_torque = -total_drive_torque;
  } else if (cmd.gear == VehicleCommand::Gear::GEAR_NEUTRAL) {
    total_drive_torque = 0.0;
  }

  // Distribute drive torque equally across 4 wheels
  double per_wheel_drive = total_drive_torque * 0.25;
  // Distribute brake torque 60% front, 40% rear for realistic braking bias
  double front_wheel_brake = total_brake_torque * 0.30;
  double rear_wheel_brake = total_brake_torque * 0.20;

  actuation.drive_torque_nm[0] = per_wheel_drive;
  actuation.drive_torque_nm[1] = per_wheel_drive;
  actuation.drive_torque_nm[2] = per_wheel_drive;
  actuation.drive_torque_nm[3] = per_wheel_drive;

  actuation.brake_torque_nm[0] = front_wheel_brake;
  actuation.brake_torque_nm[1] = front_wheel_brake;
  actuation.brake_torque_nm[2] = rear_wheel_brake;
  actuation.brake_torque_nm[3] = rear_wheel_brake;

  return actuation;
}

}  // namespace simulation
}  // namespace apollo
