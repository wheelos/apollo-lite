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

#include <string>

#include "modules/simulation/core/simulation_types.h"

namespace apollo {
namespace simulation {

/**
 * @class ISimulatorBackend
 * @brief Abstract interface for vehicle physics backend (MuJoCo, Kinematic,
 * etc.).
 */
class ISimulatorBackend {
 public:
  virtual ~ISimulatorBackend() = default;

  virtual bool Init(const std::string& model_path) = 0;
  virtual bool ApplyActuation(const VehicleActuation& actuation) = 0;
  virtual bool Step(double dt_sec) = 0;
  virtual bool GetVehicleState(VehicleState* state) const = 0;
  virtual void Reset(double x, double y, double yaw) = 0;
  // Returns false when the backend cannot honor the configured vehicle model.
  virtual bool SetVehicleGeometry(double wheelbase_m, double track_width_m,
                                  double wheel_radius_m) = 0;
  virtual bool SetMaxSteerAngle(double max_steer_angle_rad) = 0;
  virtual bool SetMaxRearSteerAngle(double max_rear_steer_angle_rad) = 0;
  virtual double SimulationTime() const = 0;
  virtual const std::string& Name() const = 0;
};

}  // namespace simulation
}  // namespace apollo
