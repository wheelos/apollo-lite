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
#include <string>

#include "modules/simulation/backend/simulator_backend.h"

namespace apollo {
namespace simulation {

/**
 * @class KinematicBackend
 * @brief Lightweight 2-DOF kinematic bicycle simulation backend.
 */
class KinematicBackend : public ISimulatorBackend {
 public:
  KinematicBackend();
  ~KinematicBackend() override = default;

  bool Init(const std::string& model_path) override;
  bool ApplyActuation(const VehicleActuation& actuation) override;
  bool Step(double dt_sec) override;
  bool GetVehicleState(VehicleState* state) const override;
  void Reset(double x, double y, double yaw) override;
  bool SetVehicleGeometry(double wheelbase_m, double track_width_m,
                          double wheel_radius_m) override;
  bool SetMaxSteerAngle(double max_steer_angle_rad) override;
  double SimulationTime() const override { return sim_time_sec_; }
  const std::string& Name() const override { return name_; }

 private:
  std::string name_{"Kinematic"};

  // Vehicle geometric and physical parameters
  double wheelbase_m_{2.85};
  double mass_kg_{1800.0};
  double wheel_radius_m_{0.33};
  double max_steer_angle_rad_{0.50};

  // State variables
  double sim_time_sec_{0.0};
  double x_{0.0};
  double y_{0.0};
  double z_{0.0};
  double yaw_{0.0};
  double speed_mps_{0.0};
  double acceleration_mps2_{0.0};
  double front_steering_rad_{0.0};
  double rear_steering_rad_{0.0};

  // Current applied actuation
  VehicleActuation current_actuation_{};
};

}  // namespace simulation
}  // namespace apollo
