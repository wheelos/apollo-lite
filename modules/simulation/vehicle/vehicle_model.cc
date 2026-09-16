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

//  Created Date: 2026-09-16
//  Author: daohu527

#include "modules/simulation/vehicle/vehicle_model.h"

#include <algorithm>
#include <cmath>

namespace apollo {
namespace simulation {

const char* VehicleModelTypeName(VehicleModelType type) {
  switch (type) {
    case VehicleModelType::kAckermann:
      return "ackermann";
    case VehicleModelType::kFourWheelSteering:
      return "four_wheel_steering";
  }
  return "unknown";
}

bool ParseVehicleModelType(const std::string& value, VehicleModelType* type) {
  if (type == nullptr) return false;
  if (value == "ackermann") {
    *type = VehicleModelType::kAckermann;
    return true;
  }
  if (value == "four_wheel_steering" || value == "4ws") {
    *type = VehicleModelType::kFourWheelSteering;
    return true;
  }
  return false;
}

void VehicleModelBase::SetGeometry(double wheelbase_m, double track_width_m,
                                   double wheel_radius_m) {
  config_.wheelbase_m = wheelbase_m;
  config_.track_width_m = track_width_m;
  config_.wheel_radius_m = wheel_radius_m;
}

void VehicleModelBase::SetTorqueLimits(double max_drive_torque_nm,
                                       double max_brake_torque_nm) {
  config_.max_drive_torque_nm = max_drive_torque_nm;
  config_.max_brake_torque_nm = max_brake_torque_nm;
}

void VehicleModelBase::SetMaxFrontSteerAngle(double max_steer_angle_rad) {
  config_.max_front_steer_rad = max_steer_angle_rad;
}

bool VehicleModelBase::ConfigureBase(const VehicleModelConfig& config) {
  if (!std::isfinite(config.wheelbase_m) || config.wheelbase_m <= 0.0 ||
      !std::isfinite(config.track_width_m) || config.track_width_m <= 0.0 ||
      !std::isfinite(config.wheel_radius_m) || config.wheel_radius_m <= 0.0 ||
      !std::isfinite(config.max_front_steer_rad) ||
      config.max_front_steer_rad <= 0.0 ||
      !std::isfinite(config.max_rear_steer_rad) ||
      config.max_rear_steer_rad < 0.0 ||
      !std::isfinite(config.max_drive_torque_nm) ||
      config.max_drive_torque_nm < 0.0 ||
      !std::isfinite(config.max_brake_torque_nm) ||
      config.max_brake_torque_nm < 0.0) {
    return false;
  }
  config_ = config;
  return true;
}

VehicleActuation VehicleModelBase::BuildActuation(
    const VehicleCommand& command, double front_axle_steer_rad,
    double rear_axle_steer_rad) const {
  VehicleActuation actuation{};
  const double front =
      std::clamp(front_axle_steer_rad, -config_.max_front_steer_rad,
                 config_.max_front_steer_rad);
  const double rear =
      std::clamp(rear_axle_steer_rad, -config_.max_rear_steer_rad,
                 config_.max_rear_steer_rad);
  if (std::abs(front - rear) < 1e-5) {
    actuation.wheel_steer_rad = {front, front, rear, rear};
  } else {
    const double turning_radius =
        config_.wheelbase_m / (std::tan(front) - std::tan(rear));
    const double icr_x = -turning_radius * std::tan(rear);
    const double half_track = config_.track_width_m * 0.5;
    actuation.wheel_steer_rad[0] = std::atan((config_.wheelbase_m - icr_x) /
                                             (turning_radius - half_track));
    actuation.wheel_steer_rad[1] = std::atan((config_.wheelbase_m - icr_x) /
                                             (turning_radius + half_track));
    actuation.wheel_steer_rad[2] =
        std::atan(-icr_x / (turning_radius - half_track));
    actuation.wheel_steer_rad[3] =
        std::atan(-icr_x / (turning_radius + half_track));
  }
  actuation.wheel_steer_rad[0] =
      std::clamp(actuation.wheel_steer_rad[0], -config_.max_front_steer_rad,
                 config_.max_front_steer_rad);
  actuation.wheel_steer_rad[1] =
      std::clamp(actuation.wheel_steer_rad[1], -config_.max_front_steer_rad,
                 config_.max_front_steer_rad);
  actuation.wheel_steer_rad[2] =
      std::clamp(actuation.wheel_steer_rad[2], -config_.max_rear_steer_rad,
                 config_.max_rear_steer_rad);
  actuation.wheel_steer_rad[3] =
      std::clamp(actuation.wheel_steer_rad[3], -config_.max_rear_steer_rad,
                 config_.max_rear_steer_rad);

  if (command.emergency_stop ||
      command.gear == VehicleCommand::Gear::GEAR_PARKING) {
    const double brake_per_wheel = config_.max_brake_torque_nm * 0.25;
    for (int i = 0; i < 4; ++i) {
      actuation.brake_torque_nm[i] = brake_per_wheel;
    }
    return actuation;
  }

  double drive_torque = 0.0;
  double brake_torque = 0.0;
  if (command.throttle > 0.0) {
    drive_torque = command.throttle * config_.max_drive_torque_nm;
  } else if (command.brake > 0.0) {
    brake_torque = command.brake * config_.max_brake_torque_nm;
  } else if (std::abs(command.target_acceleration_mps2) > 1e-3) {
    constexpr double kVehicleMassKg = 1710.0;
    const double requested_torque = kVehicleMassKg *
                                    command.target_acceleration_mps2 *
                                    config_.wheel_radius_m;
    if (requested_torque > 0.0) {
      drive_torque = std::min(requested_torque, config_.max_drive_torque_nm);
    } else {
      brake_torque = std::min(-requested_torque, config_.max_brake_torque_nm);
    }
  }
  if (command.gear == VehicleCommand::Gear::GEAR_REVERSE) {
    drive_torque = -drive_torque;
  } else if (command.gear == VehicleCommand::Gear::GEAR_NEUTRAL) {
    drive_torque = 0.0;
  }
  const double drive_per_wheel = drive_torque * 0.25;
  const double front_brake_per_wheel = brake_torque * 0.30;
  const double rear_brake_per_wheel = brake_torque * 0.20;
  actuation.drive_torque_nm = {drive_per_wheel, drive_per_wheel,
                               drive_per_wheel, drive_per_wheel};
  actuation.brake_torque_nm = {front_brake_per_wheel, front_brake_per_wheel,
                               rear_brake_per_wheel, rear_brake_per_wheel};
  return actuation;
}

}  // namespace simulation
}  // namespace apollo
