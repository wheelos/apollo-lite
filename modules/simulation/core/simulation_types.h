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

#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace apollo {
namespace simulation {

/**
 * @brief Normalized vehicle control command.
 * Units: SI, radian, m/s, m/s^2. Steering: positive = left turn.
 */
struct VehicleCommand {
  double timestamp_sec{0.0};
  uint64_t sequence_num{0};

  // Longitudinal
  double throttle{0.0};                  // [0.0, 1.0]
  double brake{0.0};                     // [0.0, 1.0]
  double target_acceleration_mps2{0.0};  // Target acceleration (m/s^2)
  double target_speed_mps{0.0};          // Target speed (m/s)

  // Lateral (front axle equivalent steering angle, rad)
  double front_steering_rad{0.0};
  double steering_rate_radps{0.0};

  enum class Gear {
    GEAR_NEUTRAL = 0,
    GEAR_DRIVE = 1,
    GEAR_REVERSE = 2,
    GEAR_PARKING = 3,
    GEAR_LOW = 4
  };
  Gear gear{Gear::GEAR_DRIVE};

  bool emergency_stop{false};
};

/**
 * @brief Wheel actuation target applied to simulator backend.
 * Wheels: [0] Front-Left, [1] Front-Right, [2] Rear-Left, [3] Rear-Right.
 */
struct VehicleActuation {
  std::array<double, 4> wheel_steer_rad{0.0, 0.0, 0.0, 0.0};
  std::array<double, 4> drive_torque_nm{0.0, 0.0, 0.0, 0.0};
  std::array<double, 4> brake_torque_nm{0.0, 0.0, 0.0, 0.0};
};

/**
 * @brief Vehicle state ground truth produced by physics backend.
 * Position: ENU world frame (meters).
 * Velocities: Body frame (x forward, y left, z up).
 */
struct VehicleState {
  double timestamp_sec{0.0};
  uint64_t sequence_num{0};
  double odometer_m{0.0};

  // ENU world pose at the rear axle center.
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};  // Heading angle (rad, CCW from East)

  // Body frame dynamics
  double linear_velocity_mps{0.0};         // Longitudinal speed
  double lateral_velocity_mps{0.0};        // Lateral speed
  double angular_velocity_yaw_radps{0.0};  // Yaw rate
  double linear_acceleration_mps2{0.0};    // Longitudinal acceleration
  double lateral_acceleration_mps2{0.0};   // Lateral acceleration
  std::array<double, 3> linear_velocity_world_mps{0.0, 0.0, 0.0};
  std::array<double, 3> linear_acceleration_body_mps2{0.0, 0.0, 0.0};
  std::array<double, 3> linear_acceleration_world_mps2{0.0, 0.0, 0.0};
  std::array<double, 3> angular_velocity_body_radps{0.0, 0.0, 0.0};
  std::array<double, 3> angular_velocity_world_radps{0.0, 0.0, 0.0};
  std::array<double, 4> wheel_speed_mps{0.0, 0.0, 0.0, 0.0};

  // Feedback to Chassis
  double front_steering_rad{0.0};
  double rear_steering_rad{0.0};
  double steering_percentage{0.0};  // [-100.0, 100.0]
  double steering_percentage_cmd{0.0};
  VehicleCommand::Gear current_gear{VehicleCommand::Gear::GEAR_DRIVE};
  double throttle_percentage{0.0};  // [0.0, 100.0]
  double throttle_percentage_cmd{0.0};
  double brake_percentage{0.0};     // [0.0, 100.0]
  double brake_percentage_cmd{0.0};

  bool is_collision{false};
};

}  // namespace simulation
}  // namespace apollo
