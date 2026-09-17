/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include "modules/common/vehicle_state/vehicle_state_provider.h"

namespace apollo {
namespace control {

class DependencyInjector {
 public:
  DependencyInjector() = default;

  ~DependencyInjector() = default;

  apollo::common::Status UpdateVehicleState(
      const apollo::localization::LocalizationEstimate& localization,
      const apollo::canbus::Chassis& chassis) {
    return vehicle_state_.Update(localization, chassis);
  }
  // Control consumes one read-only VehicleState snapshot for the current
  // control cycle. Controllers must not reconstruct it from raw localization
  // or chassis messages, or change its reference point locally. The provider,
  // VehicleOperatingState, raw localization, and chassis messages are not
  // exposed through this interface.
  const apollo::common::VehicleState& vehicle_state() const {
    return vehicle_state_.state();
  }

 private:
  apollo::common::VehicleStateProvider vehicle_state_;
};

}  // namespace control
}  // namespace apollo
