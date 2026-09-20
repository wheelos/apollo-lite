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

#include "modules/simulation/vehicle/vehicle_model.h"

namespace apollo {
namespace simulation {

/**
 * @class AckermannModel
 * @brief Front-wheel-only vehicle model.
 */
class AckermannModel final : public VehicleModelBase {
 public:
  bool Configure(const VehicleModelConfig& config) override;
  VehicleActuation ComputeActuation(const VehicleCommand& command,
                                    const VehicleState& state,
                                    double dt_sec) const override;
  const char* Name() const override { return "Ackermann"; }
};

}  // namespace simulation
}  // namespace apollo
