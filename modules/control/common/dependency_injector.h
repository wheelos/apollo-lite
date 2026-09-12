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
#include "modules/common/vehicle_state/reference_point_resolver.h"
#include "modules/common/configs/config_gflags.h"

namespace apollo {
namespace control {

class DependencyInjector {
 public:
  DependencyInjector() = default;

  ~DependencyInjector() = default;

  apollo::common::VehicleStateProvider* vehicle_state() {
    return &vehicle_state_;
  }
  apollo::common::ReferencePointResolver* reference_point_resolver() {
    return &reference_point_resolver_;
  }
  const apollo::common::VehicleState& canonical_vehicle_state() const {
    return vehicle_state_.canonical_state();
  }
  const apollo::common::VehicleOperatingState& operating_state() const {
    return vehicle_state_.operating_state();
  }
  apollo::common::Status ResolveCurrentState(
      const apollo::common::ReferencePoint reference_point,
      apollo::common::ReferenceState* reference_state,
      const double center_of_mass_offset = 0.0) const {
    return reference_point_resolver_.Resolve(
        canonical_vehicle_state(), reference_point, reference_state,
        center_of_mass_offset);
  }
  apollo::common::Status ResolveControlState(
      apollo::common::ReferenceState* reference_state,
      const double center_of_mass_offset) const {
    const auto gear = operating_state().gear();
    const bool use_center_of_mass =
        (gear == apollo::canbus::Chassis::GEAR_DRIVE &&
         FLAGS_state_transform_to_com_drive) ||
        (gear == apollo::canbus::Chassis::GEAR_REVERSE &&
         FLAGS_state_transform_to_com_reverse);
    return ResolveCurrentState(
        use_center_of_mass ? apollo::common::CENTER_OF_MASS
                           : apollo::common::REAR_AXLE_CENTER,
        reference_state, center_of_mass_offset);
  }

 private:
  apollo::common::VehicleStateProvider vehicle_state_;
  apollo::common::ReferencePointResolver reference_point_resolver_;
};

}  // namespace control
}  // namespace apollo
