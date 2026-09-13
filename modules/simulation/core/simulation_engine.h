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
#include <memory>
#include <string>

#include "modules/simulation/backend/simulator_backend.h"
#include "modules/simulation/core/simulation_types.h"
#include "modules/simulation/vehicle/ackermann_model.h"

namespace apollo {
namespace simulation {

/**
 * @class SimulationEngine
 * @brief Coordinates vehicle model and physics backend with substepping and
 * timeout safety.
 */
class SimulationEngine {
 public:
  SimulationEngine();
  ~SimulationEngine() = default;

  bool Init(const std::string& backend_type, const std::string& model_path);
  bool Step(const VehicleCommand& cmd, double control_dt_sec,
            bool command_received = true);
  bool GetVehicleState(VehicleState* state) const;
  void Reset(double x, double y, double yaw);
  void SetControlMode(const std::string& control_mode) {
    control_mode_ = control_mode;
  }
  void SetSpeedKp(double speed_kp) { speed_kp_ = speed_kp; }
  void SetMaxSteerAngle(double max_steer_angle_rad) {
    max_steer_angle_rad_ = max_steer_angle_rad;
  }
  void SetMaxRearSteerAngle(double max_rear_steer_angle_rad) {
    max_rear_steer_angle_rad_ = max_rear_steer_angle_rad;
  }
  void SetVehicleGeometry(double wheelbase_m, double track_width_m,
                          double wheel_radius_m) {
    wheelbase_m_ = wheelbase_m;
    track_width_m_ = track_width_m;
    wheel_radius_m_ = wheel_radius_m;
  }

  bool SetPhysicsDt(double dt_sec) {
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0) {
      return false;
    }
    physics_dt_sec_ = dt_sec;
    return true;
  }
  bool SetCommandTimeout(double timeout_sec) {
    if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0) {
      return false;
    }
    command_timeout_sec_ = timeout_sec;
    return true;
  }

  ISimulatorBackend* Backend() { return backend_.get(); }

 private:
  std::unique_ptr<ISimulatorBackend> backend_;
  AckermannModel vehicle_model_;
  double max_steer_angle_rad_{0.50};
  double max_rear_steer_angle_rad_{0.0};
  double wheelbase_m_{2.8448};
  double track_width_m_{1.58};
  double wheel_radius_m_{0.33};

  double physics_dt_sec_{0.002};      // 500 Hz physics rate
  double command_timeout_sec_{0.20};  // 200 ms timeout
  double speed_kp_{1.5};
  double odometer_m_{0.0};
  double last_position_x_{0.0};
  double last_position_y_{0.0};
  double last_cmd_time_sec_{0.0};
  bool has_received_command_{false};
  uint64_t step_count_{0};
  std::string control_mode_{"throttle"};

  VehicleState current_state_{};
};

}  // namespace simulation
}  // namespace apollo
