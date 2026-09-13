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

#include <cmath>

#include "modules/simulation/core/simulation_types.h"

namespace apollo {
namespace simulation {

/**
 * @class AckermannModel
 * @brief Converts VehicleCommand to four-wheel steering angles and drive/brake
 * torques.
 */
class AckermannModel {
 public:
  AckermannModel() = default;
  ~AckermannModel() = default;

  /**
   * @brief Compute physical wheel actuation targets from vehicle command.
   */
  VehicleActuation ComputeActuation(const VehicleCommand& cmd,
                                    const VehicleState& current_state,
                                    double dt_sec);

  // Parameter configuration
  void SetGeometry(double wheelbase_m, double track_width_m,
                   double wheel_radius_m) {
    wheelbase_m_ = wheelbase_m;
    track_width_m_ = track_width_m;
    wheel_radius_m_ = wheel_radius_m;
  }

  void SetTorqueLimits(double max_drive_torque_nm, double max_brake_torque_nm) {
    max_drive_torque_nm_ = max_drive_torque_nm;
    max_brake_torque_nm_ = max_brake_torque_nm;
  }

  void SetMaxSteerAngle(double max_steer_angle_rad) {
    max_steer_angle_rad_ = max_steer_angle_rad;
  }
  // A positive rear limit enables counter-phase four-wheel steering. A zero
  // limit retains the front-wheel Ackermann model.
  void SetMaxRearSteerAngle(double max_rear_steer_angle_rad) {
    max_rear_steer_angle_rad_ = max_rear_steer_angle_rad;
  }

 private:
  double wheelbase_m_{2.8448};
  double track_width_m_{1.58};
  double wheel_radius_m_{0.33};
  double mass_kg_{1710.0};

  double max_steer_angle_rad_{0.50};
  double max_rear_steer_angle_rad_{0.0};
  double max_drive_torque_nm_{2400.0};
  double max_brake_torque_nm_{4800.0};
};

}  // namespace simulation
}  // namespace apollo
