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

#include "modules/common/vehicle_state/vehicle_description.h"

#include "modules/common/configs/config_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace common {

VehicleDescription::VehicleDescription()
    : VehicleDescription(VehicleConfigHelper::GetConfig()) {}

VehicleDescription::VehicleDescription(const VehicleConfig& vehicle_config)
    : VehicleDescription(vehicle_config,
                         FLAGS_vehicle_state_center_of_mass_offset) {}

VehicleDescription::VehicleDescription(const VehicleConfig& vehicle_config,
                                       const double center_of_mass_offset)
    : wheel_base_(vehicle_config.vehicle_param().wheel_base()),
      length_(vehicle_config.vehicle_param().length()),
      width_(vehicle_config.vehicle_param().width()),
      height_(vehicle_config.vehicle_param().height()),
      front_edge_to_center_(
          vehicle_config.vehicle_param().front_edge_to_center()),
      back_edge_to_center_(
          vehicle_config.vehicle_param().back_edge_to_center()),
      left_edge_to_center_(
          vehicle_config.vehicle_param().left_edge_to_center()),
      right_edge_to_center_(
          vehicle_config.vehicle_param().right_edge_to_center()),
      max_road_wheel_angle_(
          vehicle_config.vehicle_param().steer_ratio() > 0.0
              ? vehicle_config.vehicle_param().max_steer_angle() /
                    vehicle_config.vehicle_param().steer_ratio()
              : 0.0),
      max_acceleration_(vehicle_config.vehicle_param().max_acceleration()),
      max_deceleration_(vehicle_config.vehicle_param().max_deceleration()),
      center_of_mass_offset_(center_of_mass_offset) {}

double VehicleDescription::LongitudinalOffset(
    const ReferencePoint reference_point) const {
  switch (reference_point) {
    case REAR_AXLE_CENTER:
      return 0.0;
    case FRONT_AXLE_CENTER:
      return wheel_base_;
    case CENTER_OF_MASS:
      return center_of_mass_offset_;
    default:
      return 0.0;
  }
}

math::Vec2d VehicleDescription::CenterOffset(
    const ReferencePoint reference_point) const {
  const double longitudinal_to_center =
      (front_edge_to_center_ - back_edge_to_center_) / 2.0 -
      LongitudinalOffset(reference_point);
  const double lateral_to_center =
      (left_edge_to_center_ - right_edge_to_center_) / 2.0;
  return math::Vec2d(longitudinal_to_center, lateral_to_center);
}

}  // namespace common
}  // namespace apollo
