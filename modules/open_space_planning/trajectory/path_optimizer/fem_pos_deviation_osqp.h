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
#include <utility>
#include <vector>

#include <osqp.h>

#include "modules/open_space_planning/common/types.h"

namespace apollo {
namespace open_space_planning {

struct FemPosDeviationOsqpConfig {
  double weight_fem_pos_deviation = 1.0e5;
  double weight_path_length = 1.0;
  double weight_ref_deviation = 1.0e2;
  double weight_curvature_constraint_slack_var = 1.0e5;
  double curvature_constraint = 0.5;  // kappa_max = 1 / R_min
  double default_box_bound = 1.5;
  int max_iter = 4000;
  int sqp_max_iter = 5;
  double sqp_ftol = 1e-4;
  double time_limit = 0.0;
  bool verbose = false;
};

class FemPosDeviationOsqp {
 public:
  FemPosDeviationOsqp();
  explicit FemPosDeviationOsqp(FemPosDeviationOsqpConfig config);
  ~FemPosDeviationOsqp() = default;

  bool Solve(const std::vector<GeometricPathPoint>& raw_path,
             const std::vector<CorridorSample>& corridor,
             std::vector<GeometricPathPoint>* smoothed_path);

  void set_curvature_constraint(double kappa) {
    config_.curvature_constraint = kappa;
  }

 private:
  void CalculateKernel(std::size_t num_points, std::size_t num_slack,
                       std::vector<OSQPFloat>* P_data,
                       std::vector<OSQPInt>* P_indices,
                       std::vector<OSQPInt>* P_indptr);

  void CalculateOffset(const std::vector<GeometricPathPoint>& raw_path,
                       std::size_t num_slack, std::vector<OSQPFloat>* q);

  void CalculateAffineConstraints(
      const std::vector<std::pair<double, double>>& current_points,
      const std::vector<GeometricPathPoint>& raw_path,
      const std::vector<CorridorSample>& corridor,
      std::vector<OSQPFloat>* A_data, std::vector<OSQPInt>* A_indices,
      std::vector<OSQPInt>* A_indptr, std::vector<OSQPFloat>* lower_bounds,
      std::vector<OSQPFloat>* upper_bounds);

  std::vector<double> CalculateLinearizedFemPosParams(
      const std::vector<std::pair<double, double>>& points,
      std::size_t index);

  FemPosDeviationOsqpConfig config_;
};

}  // namespace open_space_planning
}  // namespace apollo
