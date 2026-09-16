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

#pragma once

#include <string>

#include "modules/simulation/core/simulation_types.h"

namespace apollo {
namespace simulation {

// Describes the control-to-actuation behaviour. It deliberately does not
// describe a MuJoCo model: physical assets are owned by the backend.
enum class VehicleModelType {
  kAckermann,
  kFourWheelSteering,
};

const char* VehicleModelTypeName(VehicleModelType type);
bool ParseVehicleModelType(const std::string& value, VehicleModelType* type);

struct VehicleModelConfig {
  VehicleModelType type{VehicleModelType::kAckermann};
  double wheelbase_m{2.8448};
  double track_width_m{1.58};
  double wheel_radius_m{0.335};
  double max_front_steer_rad{0.6108652382};
  double max_rear_steer_rad{0.0};
  double max_drive_torque_nm{2400.0};
  double max_brake_torque_nm{4800.0};
};

// Converts a normalized vehicle command into physical wheel targets. Every
// model uses the same FL/FR/RL/RR actuation ordering and rear-axle convention.
class VehicleModel {
 public:
  virtual ~VehicleModel() = default;

  virtual bool Configure(const VehicleModelConfig& config) = 0;
  virtual VehicleActuation ComputeActuation(const VehicleCommand& command,
                                            const VehicleState& state,
                                            double dt_sec) const = 0;
  virtual const char* Name() const = 0;
};

// Shared mechanics for steering models. Derived classes choose axle targets;
// this class converts them to per-wheel Ackermann geometry and applies common
// longitudinal, brake, and parking semantics.
class VehicleModelBase : public VehicleModel {
 public:
  void SetGeometry(double wheelbase_m, double track_width_m,
                   double wheel_radius_m);
  void SetTorqueLimits(double max_drive_torque_nm, double max_brake_torque_nm);
  void SetMaxFrontSteerAngle(double max_steer_angle_rad);

 protected:
  bool ConfigureBase(const VehicleModelConfig& config);
  VehicleActuation BuildActuation(const VehicleCommand& command,
                                  double front_axle_steer_rad,
                                  double rear_axle_steer_rad) const;

  VehicleModelConfig config_;
};

}  // namespace simulation
}  // namespace apollo
