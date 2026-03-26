/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "Eigen/Eigen"
#include "osqp.h"

#include "cyber/common/log.h"

namespace apollo {
namespace common {
namespace math {

struct MpcOsqpData {
  OSQPInt n;
  OSQPInt m;
  OSQPCscMatrix* P;
  OSQPFloat* q;
  OSQPCscMatrix* A;
  OSQPFloat* l;
  OSQPFloat* u;
};

class MpcOsqp {
 public:
  /**
   * @brief Solver for discrete-time model predictive control problem.
   * @param matrix_a The system dynamic matrix
   * @param matrix_b The control matrix
   * @param matrix_q The cost matrix for control state
   * @param matrix_lower The lower bound control constrain matrix
   * @param matrix_upper The upper bound control constrain matrix
   * @param matrix_initial_state The initial state matrix
   * @param max_iter The maximum iterations
   */
  MpcOsqp(const Eigen::MatrixXd &matrix_a, const Eigen::MatrixXd &matrix_b,
          const Eigen::MatrixXd &matrix_q, const Eigen::MatrixXd &matrix_r,
          const Eigen::MatrixXd &matrix_initial_x,
          const Eigen::MatrixXd &matrix_u_lower,
          const Eigen::MatrixXd &matrix_u_upper,
          const Eigen::MatrixXd &matrix_x_lower,
          const Eigen::MatrixXd &matrix_x_upper,
          const Eigen::MatrixXd &matrix_x_ref, const int max_iter,
          const int horizon, const double eps_abs);

  // control vector
  bool Solve(std::vector<double> *control_cmd);

 private:
  void CalculateKernel(std::vector<OSQPFloat> *P_data,
                       std::vector<OSQPInt> *P_indices,
                       std::vector<OSQPInt> *P_indptr);
  void CalculateEqualityConstraint(std::vector<OSQPFloat> *A_data,
                                   std::vector<OSQPInt> *A_indices,
                                   std::vector<OSQPInt> *A_indptr);
  void CalculateGradient();
  void CalculateConstraintVectors();
  OSQPSettings *Settings();
  MpcOsqpData *Data();
  void FreeData(MpcOsqpData *data);

  template <typename T>
  T *CopyData(const std::vector<T> &vec) {
    if (vec.empty()) {
      return nullptr;
    }
    T *data =
        reinterpret_cast<T *>(std::malloc(sizeof(T) * vec.size()));
    memcpy(data, vec.data(), sizeof(T) * vec.size());
    return data;
  }

 private:
  Eigen::MatrixXd matrix_a_;
  Eigen::MatrixXd matrix_b_;
  Eigen::MatrixXd matrix_q_;
  Eigen::MatrixXd matrix_r_;
  Eigen::MatrixXd matrix_initial_x_;
  const Eigen::MatrixXd matrix_u_lower_;
  const Eigen::MatrixXd matrix_u_upper_;
  const Eigen::MatrixXd matrix_x_lower_;
  const Eigen::MatrixXd matrix_x_upper_;
  const Eigen::MatrixXd matrix_x_ref_;
  int max_iteration_;
  size_t horizon_;
  double eps_abs_;
  size_t state_dim_;
  size_t control_dim_;
  size_t num_param_;
  int num_constraint_;
  Eigen::VectorXd gradient_;
  Eigen::VectorXd lowerBound_;
  Eigen::VectorXd upperBound_;
};
}  // namespace math
}  // namespace common
}  // namespace apollo
