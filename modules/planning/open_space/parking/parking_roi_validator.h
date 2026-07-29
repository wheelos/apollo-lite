/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
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

#include <string>
#include <vector>

#include "wheelos_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/open_space/parking/parking_roi_geometry.h"
#include "modules/planning/proto/open_space_task_config.pb.h"

namespace apollo {
namespace planning {
namespace parking {

struct ParkingRoiValidationResult {
  bool valid = false;
  std::string reason;
  double area = 0.0;
  double goal_clearance = 0.0;
  bool vehicle_inside = false;
  bool goal_inside = false;
  bool goal_inside_roi = false;
  bool goal_inside_envelope = false;
};

class ParkingRoiValidator {
 public:
  explicit ParkingRoiValidator(const OpenSpaceRoiDeciderConfig& config);

  // Geometry-only validation checks whether the ROI itself is coherent enough
  // to continue: non-empty union polygon, positive area, ego already inside ROI,
  // and a valid aisle width estimate.
  ParkingRoiValidationResult ValidateGeometryOnly(
      const ParkingRoiGeometry& geometry,
      const common::math::Vec2d& vehicle_position) const;

  ParkingRoiValidationResult ValidateGeometryOnly(
      const ParkingRoiGeometry& geometry,
      const common::math::Vec2d& vehicle_position, double vehicle_heading,
      const apollo::common::VehicleParam& vehicle_param) const;

  // Goal validation reports both free-space ROI and slot-envelope membership.
  ParkingRoiValidationResult Validate(
      const ParkingRoiGeometry& geometry,
      const common::math::Vec2d& vehicle_position,
      const std::vector<double>& goal_pose,
      const apollo::common::VehicleParam& vehicle_param) const;

 private:
  OpenSpaceRoiDeciderConfig config_;
};

}  // namespace parking
}  // namespace planning
}  // namespace apollo
