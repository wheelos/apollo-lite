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

/**
 * @file
 **/

#include "modules/planning/math/discretized_points_smoothing/fem_pos_deviation_osqp_interface.h"

#include <limits>
#include <vector>

#include "cyber/common/log.h"

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

OSQPCscMatrix* CscMatrix(OSQPInt m, OSQPInt n,
                         std::vector<OSQPFloat>* values,
                         std::vector<OSQPInt>* indices,
                         std::vector<OSQPInt>* indptr) {
  return OSQPCscMatrix_new(m, n, values->size(),
                           values->empty() ? nullptr : values->data(),
                           indices->empty() ? nullptr : indices->data(),
                           indptr->data());
}

}  // namespace

bool FemPosDeviationOsqpInterface::Solve() {
  // Sanity Check
  if (ref_points_.empty()) {
    AERROR << "reference points empty, solver early terminates";
    return false;
  }

  if (ref_points_.size() != bounds_around_refs_.size()) {
    AERROR << "ref_points and bounds size not equal, solver early terminates";
    return false;
  }

  if (ref_points_.size() < 3) {
    AERROR << "ref_points size smaller than 3, solver early terminates";
    return false;
  }

  if (ref_points_.size() >
      static_cast<size_t>(std::numeric_limits<int>::max())) {
    AERROR << "ref_points size too large, solver early terminates";
    return false;
  }

  // Calculate optimization states definitions
  num_of_points_ = static_cast<int>(ref_points_.size());
  num_of_variables_ = num_of_points_ * 2;
  num_of_constraints_ = num_of_variables_;

  // Calculate kernel
  std::vector<OSQPFloat> P_data;
  std::vector<OSQPInt> P_indices;
  std::vector<OSQPInt> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  // Calculate affine constraints
  std::vector<OSQPFloat> A_data;
  std::vector<OSQPInt> A_indices;
  std::vector<OSQPInt> A_indptr;
  std::vector<OSQPFloat> lower_bounds;
  std::vector<OSQPFloat> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  // Calculate offset
  std::vector<OSQPFloat> q;
  CalculateOffset(&q);

  // Set primal warm start
  std::vector<OSQPFloat> primal_warm_start;
  SetPrimalWarmStart(&primal_warm_start);

  OSQPSettings* settings = OSQPSettings_new();
  if (settings == nullptr) {
    return false;
  }
  settings->max_iter = max_iter_;
  settings->time_limit = time_limit_;
  settings->verbose = verbose_;
  settings->scaled_termination = scaled_termination_;
  settings->warm_starting = warm_start_;

  OSQPSolver* solver = nullptr;

  bool res = OptimizeWithOsqp(num_of_variables_, lower_bounds.size(), &P_data,
                              &P_indices, &P_indptr, &A_data, &A_indices,
                              &A_indptr, &lower_bounds, &upper_bounds, &q,
                              &primal_warm_start, &solver, settings);
  if (res == false || solver == nullptr || solver->solution == nullptr) {
    AERROR << "Failed to find solution.";
    osqp_cleanup(solver);
    OSQPSettings_free(settings);

    return false;
  }

  // Extract primal results
  x_.resize(num_of_points_);
  y_.resize(num_of_points_);
  for (int i = 0; i < num_of_points_; ++i) {
    int index = i * 2;
    x_.at(i) = solver->solution->x[index];
    y_.at(i) = solver->solution->x[index + 1];
  }

  // Cleanup
  osqp_cleanup(solver);
  OSQPSettings_free(settings);

  return true;
}

void FemPosDeviationOsqpInterface::CalculateKernel(
    std::vector<OSQPFloat>* P_data, std::vector<OSQPInt>* P_indices,
    std::vector<OSQPInt>* P_indptr) {
  CHECK_GT(num_of_variables_, 4);

  // Three quadratic penalties are involved:
  // 1. Penalty x on distance between middle point and point by finite element
  // estimate;
  // 2. Penalty y on path length;
  // 3. Penalty z on difference between points and reference points

  // General formulation of P matrix is as below(with 6 points as an example):
  // I is a two by two identity matrix, X, Y, Z represents x * I, y * I, z * I
  // 0 is a two by two zero matrix
  // |X+Y+Z, -2X-Y,   X,       0,       0,       0    |
  // |0,     5X+2Y+Z, -4X-Y,   X,       0,       0    |
  // |0,     0,       6X+2Y+Z, -4X-Y,   X,       0    |
  // |0,     0,       0,       6X+2Y+Z, -4X-Y,   X    |
  // |0,     0,       0,       0,       5X+2Y+Z, -2X-Y|
  // |0,     0,       0,       0,       0,       X+Y+Z|

  // Only upper triangle needs to be filled
  std::vector<std::vector<std::pair<OSQPInt, OSQPFloat>>> columns;
  columns.resize(num_of_variables_);
  int col_num = 0;

  for (int col = 0; col < 2; ++col) {
    columns[col].emplace_back(col, weight_fem_pos_deviation_ +
                                       weight_path_length_ +
                                       weight_ref_deviation_);
    ++col_num;
  }

  for (int col = 2; col < 4; ++col) {
    columns[col].emplace_back(
        col - 2, -2.0 * weight_fem_pos_deviation_ - weight_path_length_);
    columns[col].emplace_back(col, 5.0 * weight_fem_pos_deviation_ +
                                       2.0 * weight_path_length_ +
                                       weight_ref_deviation_);
    ++col_num;
  }

  int second_point_from_last_index = num_of_points_ - 2;
  for (int point_index = 2; point_index < second_point_from_last_index;
       ++point_index) {
    int col_index = point_index * 2;
    for (int col = 0; col < 2; ++col) {
      col_index += col;
      columns[col_index].emplace_back(col_index - 4, weight_fem_pos_deviation_);
      columns[col_index].emplace_back(
          col_index - 2,
          -4.0 * weight_fem_pos_deviation_ - weight_path_length_);
      columns[col_index].emplace_back(
          col_index, 6.0 * weight_fem_pos_deviation_ +
                         2.0 * weight_path_length_ + weight_ref_deviation_);
      ++col_num;
    }
  }

  int second_point_col_from_last_col = num_of_variables_ - 4;
  int last_point_col_from_last_col = num_of_variables_ - 2;
  for (int col = second_point_col_from_last_col;
       col < last_point_col_from_last_col; ++col) {
    columns[col].emplace_back(col - 4, weight_fem_pos_deviation_);
    columns[col].emplace_back(
        col - 2, -4.0 * weight_fem_pos_deviation_ - weight_path_length_);
    columns[col].emplace_back(col, 5.0 * weight_fem_pos_deviation_ +
                                       2.0 * weight_path_length_ +
                                       weight_ref_deviation_);
    ++col_num;
  }

  for (int col = last_point_col_from_last_col; col < num_of_variables_; ++col) {
    columns[col].emplace_back(col - 4, weight_fem_pos_deviation_);
    columns[col].emplace_back(
        col - 2, -2.0 * weight_fem_pos_deviation_ - weight_path_length_);
    columns[col].emplace_back(col, weight_fem_pos_deviation_ +
                                       weight_path_length_ +
                                       weight_ref_deviation_);
    ++col_num;
  }

  CHECK_EQ(col_num, num_of_variables_);

  int ind_p = 0;
  for (int i = 0; i < col_num; ++i) {
    P_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      // Rescale by 2.0 as the quadratic term in osqp default qp problem setup
      // is set as (1/2) * x' * P * x
      P_data->push_back(row_data_pair.second * 2.0);
      P_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}

void FemPosDeviationOsqpInterface::CalculateOffset(
    std::vector<OSQPFloat>* q) {
  for (int i = 0; i < num_of_points_; ++i) {
    const auto& ref_point_xy = ref_points_[i];
    q->push_back(-2.0 * weight_ref_deviation_ * ref_point_xy.first);
    q->push_back(-2.0 * weight_ref_deviation_ * ref_point_xy.second);
  }
}

void FemPosDeviationOsqpInterface::CalculateAffineConstraint(
    std::vector<OSQPFloat>* A_data, std::vector<OSQPInt>* A_indices,
    std::vector<OSQPInt>* A_indptr, std::vector<OSQPFloat>* lower_bounds,
    std::vector<OSQPFloat>* upper_bounds) {
  int ind_A = 0;
  for (int i = 0; i < num_of_variables_; ++i) {
    A_data->push_back(1.0);
    A_indices->push_back(i);
    A_indptr->push_back(ind_A);
    ++ind_A;
  }
  A_indptr->push_back(ind_A);

  for (int i = 0; i < num_of_points_; ++i) {
    const auto& ref_point_xy = ref_points_[i];
    upper_bounds->push_back(ref_point_xy.first + bounds_around_refs_[i]);
    upper_bounds->push_back(ref_point_xy.second + bounds_around_refs_[i]);
    lower_bounds->push_back(ref_point_xy.first - bounds_around_refs_[i]);
    lower_bounds->push_back(ref_point_xy.second - bounds_around_refs_[i]);
  }
}

void FemPosDeviationOsqpInterface::SetPrimalWarmStart(
    std::vector<OSQPFloat>* primal_warm_start) {
  CHECK_EQ(ref_points_.size(), static_cast<size_t>(num_of_points_));
  for (const auto& ref_point_xy : ref_points_) {
    primal_warm_start->push_back(ref_point_xy.first);
    primal_warm_start->push_back(ref_point_xy.second);
  }
}

bool FemPosDeviationOsqpInterface::OptimizeWithOsqp(
    const size_t kernel_dim, const size_t num_affine_constraint,
    std::vector<OSQPFloat>* P_data, std::vector<OSQPInt>* P_indices,
    std::vector<OSQPInt>* P_indptr, std::vector<OSQPFloat>* A_data,
    std::vector<OSQPInt>* A_indices, std::vector<OSQPInt>* A_indptr,
    std::vector<OSQPFloat>* lower_bounds,
    std::vector<OSQPFloat>* upper_bounds, std::vector<OSQPFloat>* q,
    std::vector<OSQPFloat>* primal_warm_start, OSQPSolver** solver,
    OSQPSettings* settings) {
  CHECK_EQ(lower_bounds->size(), upper_bounds->size());

  OSQPCscMatrix* P_matrix = CscMatrix(static_cast<OSQPInt>(kernel_dim),
                                      static_cast<OSQPInt>(kernel_dim), P_data,
                                      P_indices, P_indptr);
  OSQPCscMatrix* A_matrix = CscMatrix(
      static_cast<OSQPInt>(num_affine_constraint),
      static_cast<OSQPInt>(kernel_dim), A_data, A_indices, A_indptr);
  if (P_matrix == nullptr || A_matrix == nullptr) {
    OSQPCscMatrix_free(A_matrix);
    OSQPCscMatrix_free(P_matrix);
    return false;
  }

  if (osqp_setup(solver, P_matrix, q->data(), A_matrix, lower_bounds->data(),
                 upper_bounds->data(),
                 static_cast<OSQPInt>(num_affine_constraint),
                 static_cast<OSQPInt>(kernel_dim), settings) != 0 ||
      *solver == nullptr) {
    if (*solver != nullptr) {
      osqp_cleanup(*solver);
      *solver = nullptr;
    }
    OSQPCscMatrix_free(A_matrix);
    OSQPCscMatrix_free(P_matrix);
    return false;
  }

  osqp_warm_start(*solver, primal_warm_start->data(), nullptr);

  // Solve Problem
  osqp_solve(*solver);

  OSQPCscMatrix_free(A_matrix);
  OSQPCscMatrix_free(P_matrix);

  auto status = (*solver)->info->status_val;

  if (!HasUsableSolutionStatus(status)) {
    AERROR << "failed optimization status:\t" << (*solver)->info->status;
    return false;
  }

  return true;
}

}  // namespace planning
}  // namespace apollo
