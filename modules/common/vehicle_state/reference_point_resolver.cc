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

#include "modules/common/vehicle_state/reference_point_resolver.h"

#include <cmath>

#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace common {

ReferencePointResolver::ReferencePointResolver()
    : description_(VehicleConfigHelper::GetConfig()) {}

ReferencePointResolver::ReferencePointResolver(
    const VehicleDescription& description)
    : description_(description) {}

Status ReferencePointResolver::Resolve(
    const VehicleState& vehicle_state, const ReferencePoint reference_point,
    ReferenceState* reference_state, const double center_of_mass_offset) const {
  if (reference_state == nullptr) {
    return Status(ErrorCode::LOCALIZATION_ERROR, "reference_state is null");
  }

  const double target_offset =
      description_.LongitudinalOffset(reference_point, center_of_mass_offset);
  const double source_offset = description_.LongitudinalOffset(
      vehicle_state.reference_point(), center_of_mass_offset);
  const double longitudinal_offset = target_offset - source_offset;
  double offset_x = 0.0;
  double offset_y = longitudinal_offset;

  if (vehicle_state.has_pose() && vehicle_state.pose().has_orientation()) {
    const auto& orientation = vehicle_state.pose().orientation();
    const double qw = orientation.qw();
    const double qx = orientation.qx();
    const double qy = orientation.qy();
    const double qz = orientation.qz();
    const double rotation_x = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double rotation_y = 2.0 * (qx * qy - qw * qz);
    const double translation_x = rotation_x * offset_x + rotation_y * offset_y;
    const double rotation_x_y = 2.0 * (qx * qy + qw * qz);
    const double rotation_y_y = 1.0 - 2.0 * (qx * qx + qz * qz);
    const double translation_y =
        rotation_x_y * offset_x + rotation_y_y * offset_y;
    reference_state->set_x(vehicle_state.x() + translation_x);
    reference_state->set_y(vehicle_state.y() + translation_y);
  } else {
    const double heading = vehicle_state.heading();
    reference_state->set_x(vehicle_state.x() +
                           std::cos(heading) * offset_y -
                           std::sin(heading) * offset_x);
    reference_state->set_y(vehicle_state.y() +
                           std::sin(heading) * offset_y +
                           std::cos(heading) * offset_x);
  }

  reference_state->set_z(vehicle_state.z());
  reference_state->set_reference_point(reference_point);
  reference_state->set_heading(vehicle_state.heading());
  reference_state->set_linear_velocity(vehicle_state.linear_velocity());
  reference_state->set_angular_velocity(vehicle_state.angular_velocity());
  reference_state->set_linear_acceleration(vehicle_state.linear_acceleration());
  reference_state->set_kappa(vehicle_state.kappa());
  reference_state->set_timestamp(vehicle_state.timestamp());
  return Status::OK();
}

}  // namespace common
}  // namespace apollo
