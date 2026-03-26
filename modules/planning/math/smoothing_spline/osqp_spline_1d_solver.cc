/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "modules/planning/math/smoothing_spline/osqp_spline_1d_solver.h"

#include <cmath>

#include "cyber/common/log.h"
#include "modules/common/math/matrix_operations.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace {

bool HasUsableSolutionStatus(const OSQPInt status) {
  return status != OSQP_PRIMAL_INFEASIBLE &&
         status != OSQP_PRIMAL_INFEASIBLE_INACCURATE &&
         status != OSQP_DUAL_INFEASIBLE &&
         status != OSQP_DUAL_INFEASIBLE_INACCURATE &&
         status != OSQP_NON_CVX;
}

bool HasFiniteSolution(const OSQPSolver* solver, const OSQPInt n) {
  if (solver == nullptr || solver->solution == nullptr ||
      solver->solution->x == nullptr) {
    return false;
  }
  for (OSQPInt i = 0; i < n; ++i) {
    if (!std::isfinite(solver->solution->x[i])) {
      return false;
    }
  }
  return true;
}

OSQPCscMatrix* CscMatrix(OSQPInt m, OSQPInt n,
                         std::vector<OSQPFloat>* values,
                         std::vector<OSQPInt>* indices,
                         std::vector<OSQPInt>* indptr) {
  return OSQPCscMatrix_new(m, n, values->size(),
                           values->empty() ? nullptr : values->data(),
                           indices->empty() ? nullptr : indices->data(),
                           indptr->data());
}

void DenseToUpperCSCMatrix(const Eigen::MatrixXd& dense_matrix,
                           std::vector<OSQPFloat>* data,
                           std::vector<OSQPInt>* indices,
                           std::vector<OSQPInt>* indptr) {
  static constexpr double kEpsilon = 1e-9;
  int data_count = 0;
  for (int c = 0; c < dense_matrix.cols(); ++c) {
    indptr->emplace_back(data_count);
    for (int r = 0; r <= c && r < dense_matrix.rows(); ++r) {
      if (std::fabs(dense_matrix(r, c)) < kEpsilon) {
        continue;
      }
      data->emplace_back(static_cast<OSQPFloat>(dense_matrix(r, c)));
      ++data_count;
      indices->emplace_back(r);
    }
  }
  indptr->emplace_back(data_count);
}

}  // namespace

using apollo::common::math::DenseToCSCMatrix;
using Eigen::MatrixXd;

OsqpSpline1dSolver::OsqpSpline1dSolver(const std::vector<double>& x_knots,
                                       const uint32_t order)
    : Spline1dSolver(x_knots, order) {}

OsqpSpline1dSolver::~OsqpSpline1dSolver() = default;

bool OsqpSpline1dSolver::Solve() {
  // Namings here are following osqp convention.
  // For details, visit: https://osqp.org/docs/examples/demo.html

  // change P to csc format
  const MatrixXd P =
      0.5 * (kernel_.kernel_matrix() + kernel_.kernel_matrix().transpose());
  ADEBUG << "P: " << P.rows() << ", " << P.cols();
  if (P.rows() == 0) {
    return false;
  }

  std::vector<OSQPFloat> P_data;
  std::vector<OSQPInt> P_indices;
  std::vector<OSQPInt> P_indptr;
  DenseToUpperCSCMatrix(P, &P_data, &P_indices, &P_indptr);

  // change A to csc format
  const MatrixXd& inequality_constraint_matrix =
      constraint_.inequality_constraint().constraint_matrix();
  const MatrixXd& equality_constraint_matrix =
      constraint_.equality_constraint().constraint_matrix();
  MatrixXd A(
      inequality_constraint_matrix.rows() + equality_constraint_matrix.rows(),
      inequality_constraint_matrix.cols());
  A << inequality_constraint_matrix, equality_constraint_matrix;
  ADEBUG << "A: " << A.rows() << ", " << A.cols();
  if (A.rows() == 0) {
    return false;
  }

  std::vector<OSQPFloat> A_data;
  std::vector<OSQPInt> A_indices;
  std::vector<OSQPInt> A_indptr;
  DenseToCSCMatrix(A, &A_data, &A_indices, &A_indptr);

  // set q, l, u: l < A < u
  const MatrixXd& q_eigen = kernel_.offset();
  std::vector<OSQPFloat> q(q_eigen.rows());
  for (int i = 0; i < q_eigen.size(); ++i) {
    q[i] = static_cast<OSQPFloat>(q_eigen(i));
  }

  const MatrixXd& inequality_constraint_boundary =
      constraint_.inequality_constraint().constraint_boundary();
  const MatrixXd& equality_constraint_boundary =
      constraint_.equality_constraint().constraint_boundary();

  OSQPSettings* settings = OSQPSettings_new();
  if (settings == nullptr) {
    return false;
  }
  settings->alpha = 1.0;
  settings->eps_abs = 1.0e-03;
  settings->eps_rel = 1.0e-03;
  settings->max_iter = 5000;
  settings->polishing = true;
  settings->verbose = FLAGS_enable_osqp_debug;
  settings->warm_starting = true;

  OSQPInt constraint_num = static_cast<OSQPInt>(
      inequality_constraint_boundary.rows() + equality_constraint_boundary.rows());

  static constexpr OSQPFloat kUpperLimit = 1e9;
  const OSQPFloat equality_tolerance =
      std::max<OSQPFloat>(settings->eps_abs, 1.0e-6);
  std::vector<OSQPFloat> l(constraint_num);
  std::vector<OSQPFloat> u(constraint_num);
  for (OSQPInt i = 0; i < constraint_num; ++i) {
    if (i < inequality_constraint_boundary.rows()) {
      l[i] = static_cast<OSQPFloat>(inequality_constraint_boundary(i, 0));
      u[i] = kUpperLimit;
    } else {
      const int idx =
          i - static_cast<int>(inequality_constraint_boundary.rows());
      l[i] = static_cast<OSQPFloat>(equality_constraint_boundary(idx, 0) -
                                    equality_tolerance);
      u[i] = static_cast<OSQPFloat>(equality_constraint_boundary(idx, 0) +
                                    equality_tolerance);
    }
  }

  OSQPCscMatrix* P_matrix = CscMatrix(static_cast<OSQPInt>(P.rows()),
                                      static_cast<OSQPInt>(P.rows()), &P_data,
                                      &P_indices, &P_indptr);
  OSQPCscMatrix* A_matrix = CscMatrix(
      constraint_num, static_cast<OSQPInt>(P.rows()), &A_data, &A_indices,
      &A_indptr);
  if (P_matrix == nullptr || A_matrix == nullptr) {
    AERROR << "Failed to allocate OSQP matrices for 1d spline solver";
    OSQPCscMatrix_free(A_matrix);
    OSQPCscMatrix_free(P_matrix);
    OSQPSettings_free(settings);
    return false;
  }

  OSQPSolver* solver = nullptr;
  const OSQPInt setup_status =
      osqp_setup(&solver, P_matrix, q.data(), A_matrix, l.data(), u.data(),
                 constraint_num, static_cast<OSQPInt>(P.rows()), settings);
  if (setup_status != 0 ||
      solver == nullptr) {
    AERROR << "Failed to setup OSQP 1d spline solver, status "
           << setup_status;
    if (solver != nullptr) {
      osqp_cleanup(solver);
    }
    OSQPCscMatrix_free(A_matrix);
    OSQPCscMatrix_free(P_matrix);
    OSQPSettings_free(settings);
    return false;
  }

  // Solve Problem
  osqp_solve(solver);
  if (!HasFiniteSolution(solver, static_cast<OSQPInt>(P.rows()))) {
    AERROR << "1d spline OSQP returned invalid solution, status "
           << solver->info->status;
    osqp_cleanup(solver);
    OSQPCscMatrix_free(A_matrix);
    OSQPCscMatrix_free(P_matrix);
    OSQPSettings_free(settings);
    return false;
  }

  if (!HasUsableSolutionStatus(solver->info->status_val)) {
    AWARN << "1d spline OSQP returned status " << solver->info->status
          << ", using finite iterate for backward compatibility";
  }

  MatrixXd solved_params = MatrixXd::Zero(P.rows(), 1);
  for (int i = 0; i < P.rows(); ++i) {
    solved_params(i, 0) = solver->solution->x[i];
  }

  last_num_param_ = static_cast<int>(P.rows());
  last_num_constraint_ = static_cast<int>(constraint_num);

  osqp_cleanup(solver);
  OSQPCscMatrix_free(A_matrix);
  OSQPCscMatrix_free(P_matrix);
  OSQPSettings_free(settings);

  return spline_.SetSplineSegs(solved_params, spline_.spline_order());
}

}  // namespace planning
}  // namespace apollo
