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

#include "modules/simulation/vehicle/four_wheel_steering_model.h"

#include <algorithm>

namespace apollo {
namespace simulation {

bool FourWheelSteeringModel::Configure(const VehicleModelConfig& config) {
  if (config.type != VehicleModelType::kFourWheelSteering ||
      config.max_rear_steer_rad <= 0.0 ||
      config.max_rear_steer_rad > config.max_front_steer_rad) {
    return false;
  }
  return ConfigureBase(config);
}

VehicleActuation FourWheelSteeringModel::ComputeActuation(
    const VehicleCommand& command, const VehicleState& /*state*/,
    double /*dt_sec*/) const {
  const double front =
      std::clamp(command.front_steering_rad, -config_.max_front_steer_rad,
                 config_.max_front_steer_rad);
  // At maximum front lock, the rear axle reaches its own configured limit.
  const double rear =
      -front * config_.max_rear_steer_rad / config_.max_front_steer_rad;
  return BuildActuation(command, front, rear);
}

}  // namespace simulation
}  // namespace apollo
