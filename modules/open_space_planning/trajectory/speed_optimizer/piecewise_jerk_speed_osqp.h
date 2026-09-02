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

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <osqp.h>

#include "modules/open_space_planning/common/types.h"

namespace apollo {
namespace open_space_planning {

struct PiecewiseJerkSpeedOsqpConfig {
  double delta_t = 0.1;
  double total_time = 4.0;
  double max_velocity = 3.0;
  double min_velocity = 0.0;
  double max_acceleration = 1.5;
  double max_deceleration = -2.5;
  double max_jerk = 2.0;
  double weight_s = 0.1;
  double weight_v = 1.0;
  double weight_a = 10.0;
  double weight_jerk = 100.0;
  double weight_v_ref = 20.0;
  int max_iter = 4000;
  bool verbose = false;
};

struct SpeedProfilePoint {
  double t = 0.0;
  double s = 0.0;
  double v = 0.0;
  double a = 0.0;
};

class PiecewiseJerkSpeedOsqp {
 public:
  PiecewiseJerkSpeedOsqp();
  explicit PiecewiseJerkSpeedOsqp(PiecewiseJerkSpeedOsqpConfig config);
  ~PiecewiseJerkSpeedOsqp() = default;

  bool Solve(double s_init, double v_init, double a_init, double v_target,
             double max_path_s, const std::vector<STBoundary>& st_boundaries,
             std::vector<SpeedProfilePoint>* speed_profile);

  static void ComputeSTBoundaries(const std::vector<GeometricPathPoint>& path,
                                  const std::vector<DynamicObstacle>& obstacles,
                                  double time_horizon, double delta_t,
                                  double vehicle_safety_margin,
                                  std::vector<STBoundary>* st_boundaries);

 private:
  PiecewiseJerkSpeedOsqpConfig config_;
};

}  // namespace open_space_planning
}  // namespace apollo
