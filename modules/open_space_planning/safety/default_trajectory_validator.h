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

#include "modules/open_space_planning/common/types.h"
#include "modules/open_space_planning/safety/trajectory_validator.h"

namespace apollo {
namespace open_space_planning {

struct TrajectoryValidatorConfig {
  double maximum_speed = 5.0;               // m/s
  double maximum_acceleration = 3.0;        // m/s^2
  double maximum_deceleration = 4.0;        // m/s^2
  double maximum_curvature = 0.5;           // 1/m
  double maximum_time_horizon = 60.0;       // s
  double minimum_obstacle_clearance = 0.2;  // m
};

class DefaultTrajectoryValidator : public TrajectoryValidator {
 public:
  DefaultTrajectoryValidator() = default;
  explicit DefaultTrajectoryValidator(TrajectoryValidatorConfig config);
  virtual ~DefaultTrajectoryValidator() = default;

  ValidationReport Validate(
      const TrajectoryValidationRequest& request) const override;

 private:
  TrajectoryValidatorConfig config_;
};

}  // namespace open_space_planning
}  // namespace apollo
