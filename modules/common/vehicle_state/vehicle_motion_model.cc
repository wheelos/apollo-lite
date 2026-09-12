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

#include "modules/common/vehicle_state/vehicle_motion_model.h"

#include <cmath>

namespace apollo {
namespace common {

math::Vec2d VehicleMotionModel::EstimateFuturePosition(
    const VehicleState& vehicle_state, const double time_horizon) {
  double local_x = 0.0;
  double local_y = vehicle_state.linear_velocity() * time_horizon;
  if (std::fabs(vehicle_state.angular_velocity()) >= 0.0001) {
    local_x = -vehicle_state.linear_velocity() /
              vehicle_state.angular_velocity() *
              (1.0 - std::cos(vehicle_state.angular_velocity() * time_horizon));
    local_y = std::sin(vehicle_state.angular_velocity() * time_horizon) *
              vehicle_state.linear_velocity() /
              vehicle_state.angular_velocity();
  }

  if (vehicle_state.has_pose() && vehicle_state.pose().has_orientation()) {
    const auto& orientation = vehicle_state.pose().orientation();
    const double qw = orientation.qw();
    const double qx = orientation.qx();
    const double qy = orientation.qy();
    const double qz = orientation.qz();
    const double rotation_x = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double rotation_y = 2.0 * (qx * qy - qw * qz);
    const double rotation_x_y = 2.0 * (qx * qy + qw * qz);
    const double rotation_y_y = 1.0 - 2.0 * (qx * qx + qz * qz);
    return math::Vec2d(
        vehicle_state.x() + rotation_x * local_x + rotation_y * local_y,
        vehicle_state.y() + rotation_x_y * local_x + rotation_y_y * local_y);
  }

  const double heading = vehicle_state.heading();
  return math::Vec2d(vehicle_state.x() + std::sin(heading) * local_x +
                         std::cos(heading) * local_y,
                     vehicle_state.y() - std::cos(heading) * local_x +
                         std::sin(heading) * local_y);
}

}  // namespace common
}  // namespace apollo
