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

//  Created Date: 2026-08-28
//  Author: daohu527

#include "modules/lane/component/lane_transform_adapter.h"

#include <utility>

#include "Eigen/Geometry"

#include "cyber/time/time.h"

namespace apollo {
namespace lane {

bool LaneTransformAdapter::HasCameraToVehicleTransform(
    const std::string& camera_frame, const std::string& vehicle_frame,
    double timestamp_sec, float timeout_sec, std::string* error) const {
  if (camera_frame.empty() || vehicle_frame.empty()) {
    if (error != nullptr) {
      *error = "camera or vehicle frame is empty";
    }
    return false;
  }
  Eigen::Affine3d camera_to_vehicle = Eigen::Affine3d::Identity();
  return query_.LookupTransformToAffine(vehicle_frame, camera_frame,
                                        cyber::Time(timestamp_sec),
                                        &camera_to_vehicle, timeout_sec, error);
}

}  // namespace lane
}  // namespace apollo
