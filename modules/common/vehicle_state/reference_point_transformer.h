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

#include "modules/common/status/status.h"
#include "modules/common/vehicle_state/vehicle_description.h"

namespace apollo {
namespace common {

// Converts a VehicleState between longitudinal vehicle reference points, such
// as REAR_AXLE_CENTER, FRONT_AXLE_CENTER, and CENTER_OF_MASS.
//
// Responsibilities:
//   * transform planar position according to vehicle geometry;
//   * convert planar reference-point velocity, acceleration, and curvature;
//   * preserve operating metadata when the VehicleState overload is used.
//
// This class does not predict motion, select a vehicle model, or apply
// steering constraints. This is intentionally a strict 2D transform: heading
// is authoritative and z/orientation quaternion fields are preserved as
// metadata rather than transformed. VehicleState.linear_velocity is a
// body-frame longitudinal component and is invariant for longitudinal
// reference-point offsets. This is not a full six-degree-of-freedom rigid-body
// transform.
class ReferencePointTransformer {
 public:
  [[deprecated("Use an explicit VehicleConfig or VehicleDescription")]]
  ReferencePointTransformer();
  explicit ReferencePointTransformer(const VehicleConfig& vehicle_config);
  explicit ReferencePointTransformer(const VehicleDescription& description);

  // Transforms the complete state to target_point, including pose and
  // reference-point-dependent planar kinematics. Operating metadata such as
  // gear and driving mode is preserved. The center-of-mass offset always
  // comes from description_.
  Status TransformState(const VehicleState& source_state,
                        ReferencePoint target_point,
                        VehicleState* target_state) const;

  const VehicleDescription& description() const { return description_; }

 private:
  VehicleDescription description_;
};

}  // namespace common
}  // namespace apollo
