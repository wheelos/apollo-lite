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

#pragma once

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "wheelos_msgs/config_msgs/vehicle_config.pb.h"

#include "modules/common/math/vec2d.h"
#include "modules/common/status/status.h"

namespace apollo {
namespace common {

bool IsSupportedReferencePoint(VehicleReferencePoint reference_point);

// Immutable vehicle geometry used by reference-point conversion and spatial
// occupancy calculations. It does not contain motion, steering, or prediction
// logic.
class VehicleDescription {
 public:
  // Prefer explicit VehicleConfig injection in new code.
  [[deprecated("Use an explicit VehicleConfig or VehicleDescription")]]
  VehicleDescription();
  explicit VehicleDescription(const VehicleConfig& vehicle_config);
  VehicleDescription(const VehicleConfig& vehicle_config,
                     double center_of_mass_offset);

  // Returns the signed longitudinal coordinate measured from
  // REAR_AXLE_CENTER. CENTER_OF_MASS uses the configured vehicle offset.
  Status LongitudinalOffset(VehicleReferencePoint reference_point,
                            double* offset) const;

  // Returns the geometric center relative to reference_point in vehicle-frame
  // coordinates as (longitudinal, lateral).
  Status FootprintCenterOffset(VehicleReferencePoint reference_point,
                               math::Vec2d* offset) const;

  double wheel_base() const { return wheel_base_; }
  double length() const { return length_; }
  double width() const { return width_; }
  double height() const { return height_; }
  double front_edge_to_center() const { return front_edge_to_center_; }
  double back_edge_to_center() const { return back_edge_to_center_; }
  double left_edge_to_center() const { return left_edge_to_center_; }
  double right_edge_to_center() const { return right_edge_to_center_; }
  double max_road_wheel_angle() const { return max_road_wheel_angle_; }
  double max_acceleration() const { return max_acceleration_; }
  double max_deceleration() const { return max_deceleration_; }
  double center_of_mass_offset() const { return center_of_mass_offset_; }

 private:
  double wheel_base_ = 0.0;
  double length_ = 0.0;
  double width_ = 0.0;
  double height_ = 0.0;
  double front_edge_to_center_ = 0.0;
  double back_edge_to_center_ = 0.0;
  double left_edge_to_center_ = 0.0;
  double right_edge_to_center_ = 0.0;
  double max_road_wheel_angle_ = 0.0;
  double max_acceleration_ = 0.0;
  double max_deceleration_ = 0.0;
  double center_of_mass_offset_ = 0.0;
};

}  // namespace common
}  // namespace apollo
