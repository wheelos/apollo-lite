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

#include "modules/simulation/backend/kinematic_backend.h"

#include <algorithm>
#include <cmath>

namespace apollo {
namespace simulation {

KinematicBackend::KinematicBackend() { Reset(0.0, 0.0, 0.0); }

bool KinematicBackend::Init(const std::string& /*model_path*/) {
  Reset(0.0, 0.0, 0.0);
  return true;
}

void KinematicBackend::Reset(double x, double y, double yaw) {
  sim_time_sec_ = 0.0;
  x_ = x;
  y_ = y;
  z_ = 0.0;
  yaw_ = yaw;
  speed_mps_ = 0.0;
  acceleration_mps2_ = 0.0;
  front_steering_rad_ = 0.0;
  current_actuation_ = VehicleActuation{};
}

bool KinematicBackend::ApplyActuation(const VehicleActuation& actuation) {
  current_actuation_ = actuation;
  front_steering_rad_ =
      (actuation.wheel_steer_rad[0] + actuation.wheel_steer_rad[1]) * 0.5;
  return true;
}

bool KinematicBackend::Step(double dt_sec) {
  if (dt_sec <= 0.0) {
    return false;
  }

  // Calculate total drive and brake torque
  double total_drive_torque = current_actuation_.drive_torque_nm[0] +
                              current_actuation_.drive_torque_nm[1] +
                              current_actuation_.drive_torque_nm[2] +
                              current_actuation_.drive_torque_nm[3];

  double total_brake_torque = current_actuation_.brake_torque_nm[0] +
                              current_actuation_.brake_torque_nm[1] +
                              current_actuation_.brake_torque_nm[2] +
                              current_actuation_.brake_torque_nm[3];

  // Effective net longitudinal force
  double drive_force = total_drive_torque / wheel_radius_m_;
  double brake_force = total_brake_torque / wheel_radius_m_;

  // Simple rolling and aerodynamic resistance
  const double rolling_friction_coeff = 0.015;
  const double g = 9.81;
  const double aero_drag_coeff = 0.35;
  double resistance =
      mass_kg_ * g * rolling_friction_coeff +
      0.5 * aero_drag_coeff * 1.225 * speed_mps_ * std::abs(speed_mps_);

  double net_force = drive_force;
  if (speed_mps_ > 0.0) {
    net_force -= (brake_force + resistance);
  } else if (speed_mps_ < 0.0) {
    net_force += (brake_force + resistance);
  } else {
    // Stationary: brake and static friction counteract drive force
    if (std::abs(net_force) <=
        brake_force + mass_kg_ * g * rolling_friction_coeff) {
      net_force = 0.0;
    } else if (net_force > 0.0) {
      net_force -= brake_force;
    } else {
      net_force += brake_force;
    }
  }

  acceleration_mps2_ = net_force / mass_kg_;
  speed_mps_ += acceleration_mps2_ * dt_sec;

  // Prevent small numerical oscillation around zero speed when braking
  if (drive_force == 0.0 && total_brake_torque > 0.0) {
    if (std::abs(speed_mps_) < 0.05) {
      speed_mps_ = 0.0;
      acceleration_mps2_ = 0.0;
    }
  }

  // Kinematic bicycle update
  double curvature = std::tan(front_steering_rad_) / wheelbase_m_;
  double yaw_rate = speed_mps_ * curvature;

  yaw_ += yaw_rate * dt_sec;
  // Normalize yaw to [-pi, pi]
  while (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;
  while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;

  x_ += speed_mps_ * std::cos(yaw_) * dt_sec;
  y_ += speed_mps_ * std::sin(yaw_) * dt_sec;

  sim_time_sec_ += dt_sec;
  return true;
}

bool KinematicBackend::GetVehicleState(VehicleState* state) const {
  if (!state) {
    return false;
  }

  state->timestamp_sec = sim_time_sec_;
  state->x = x_;
  state->y = y_;
  state->z = z_;

  // ENU Euler to quaternion (roll = 0, pitch = 0, yaw around Z)
  state->roll = 0.0;
  state->pitch = 0.0;
  state->yaw = yaw_;
  state->qx = 0.0;
  state->qy = 0.0;
  state->qz = std::sin(yaw_ * 0.5);
  state->qw = std::cos(yaw_ * 0.5);

  state->linear_velocity_mps = speed_mps_;
  state->lateral_velocity_mps = 0.0;
  state->angular_velocity_yaw_radps =
      speed_mps_ * std::tan(front_steering_rad_) / wheelbase_m_;
  state->linear_acceleration_mps2 = acceleration_mps2_;

  state->front_steering_rad = front_steering_rad_;
  const double max_steer_rad = 0.50;  // ~28.6 degrees
  state->steering_percentage = (front_steering_rad_ / max_steer_rad) * 100.0;
  state->steering_percentage =
      std::max(-100.0, std::min(100.0, state->steering_percentage));

  return true;
}

}  // namespace simulation
}  // namespace apollo
