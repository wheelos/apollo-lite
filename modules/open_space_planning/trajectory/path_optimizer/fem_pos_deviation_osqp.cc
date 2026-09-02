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

#include "modules/open_space_planning/trajectory/path_optimizer/fem_pos_deviation_osqp.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <utility>
#include <vector>

namespace apollo {
namespace open_space_planning {
namespace {

template <typename T>
T* CopyData(const std::vector<T>& vec) {
  if (vec.empty()) {
    return nullptr;
  }
  T* data = reinterpret_cast<T*>(std::malloc(sizeof(T) * vec.size()));
  if (data == nullptr) {
    return nullptr;
  }
  std::memcpy(data, vec.data(), sizeof(T) * vec.size());
  return data;
}

OSQPCscMatrix* OwnedCscMatrix(OSQPInt m, OSQPInt n,
                              const std::vector<OSQPFloat>& values,
                              const std::vector<OSQPInt>& indices,
                              const std::vector<OSQPInt>& indptr) {
  OSQPCscMatrix* matrix =
      OSQPCscMatrix_new(m, n, values.size(), CopyData(values),
                        CopyData(indices), CopyData(indptr));
  if (matrix != nullptr) {
    matrix->owned = 1;
  }
  return matrix;
}

double NormalizeAngle(double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += 2.0 * M_PI;
  }
  return a - M_PI;
}

}  // namespace

FemPosDeviationOsqp::FemPosDeviationOsqp() {}

FemPosDeviationOsqp::FemPosDeviationOsqp(FemPosDeviationOsqpConfig config)
    : config_(config) {}

std::vector<double> FemPosDeviationOsqp::CalculateLinearizedFemPosParams(
    const std::vector<std::pair<double, double>>& points, std::size_t index) {
  const double x_f = points[index - 1].first;
  const double x_m = points[index].first;
  const double x_l = points[index + 1].first;
  const double y_f = points[index - 1].second;
  const double y_m = points[index].second;
  const double y_l = points[index + 1].second;

  const double linear_term_x_f = 2.0 * x_f - 4.0 * x_m + 2.0 * x_l;
  const double linear_term_x_m = 8.0 * x_m - 4.0 * x_f - 4.0 * x_l;
  const double linear_term_x_l = 2.0 * x_l - 4.0 * x_m + 2.0 * x_f;
  const double linear_term_y_f = 2.0 * y_f - 4.0 * y_m + 2.0 * y_l;
  const double linear_term_y_m = 8.0 * y_m - 4.0 * y_f - 4.0 * y_l;
  const double linear_term_y_l = 2.0 * y_l - 4.0 * y_m + 2.0 * y_f;

  const double linear_approx =
      (x_f - 2.0 * x_m + x_l) * (x_f - 2.0 * x_m + x_l) +
      (y_f - 2.0 * y_m + y_l) * (y_f - 2.0 * y_m + y_l) -
      (x_f * linear_term_x_f + x_m * linear_term_x_m + x_l * linear_term_x_l +
       y_f * linear_term_y_f + y_m * linear_term_y_m + y_l * linear_term_y_l);

  return {linear_term_x_f, linear_term_y_f, linear_term_x_m, linear_term_y_m,
          linear_term_x_l, linear_term_y_l, linear_approx};
}

bool FemPosDeviationOsqp::Solve(
    const std::vector<GeometricPathPoint>& raw_path,
    const std::vector<CorridorSample>& corridor,
    std::vector<GeometricPathPoint>* smoothed_path) {
  if (smoothed_path == nullptr || raw_path.size() < 3) {
    return false;
  }
  const std::size_t num_points = raw_path.size();
  const std::size_t num_slack = num_points - 2;
  const OSQPInt num_variables =
      static_cast<OSQPInt>(num_points * 2 + num_slack);

  std::vector<std::pair<double, double>> current_points(num_points);
  for (std::size_t i = 0; i < num_points; ++i) {
    current_points[i] = {raw_path[i].pose.x, raw_path[i].pose.y};
  }

  std::vector<OSQPFloat> P_data;
  std::vector<OSQPInt> P_indices;
  std::vector<OSQPInt> P_indptr;
  CalculateKernel(num_points, num_slack, &P_data, &P_indices, &P_indptr);

  std::vector<OSQPFloat> q;
  CalculateOffset(raw_path, num_slack, &q);

  std::vector<OSQPFloat> A_data;
  std::vector<OSQPInt> A_indices;
  std::vector<OSQPInt> A_indptr;
  std::vector<OSQPFloat> lower_bounds;
  std::vector<OSQPFloat> upper_bounds;

  bool solved = false;
  double last_obj_val = 1e20;

  for (int iter = 0; iter < config_.sqp_max_iter; ++iter) {
    CalculateAffineConstraints(current_points, raw_path, corridor, &A_data,
                               &A_indices, &A_indptr, &lower_bounds,
                               &upper_bounds);

    OSQPCscMatrix* P = OwnedCscMatrix(num_variables, num_variables, P_data,
                                      P_indices, P_indptr);
    OSQPCscMatrix* A =
        OwnedCscMatrix(static_cast<OSQPInt>(lower_bounds.size()), num_variables,
                       A_data, A_indices, A_indptr);

    if (P == nullptr || A == nullptr) {
      if (P != nullptr) OSQPCscMatrix_free(P);
      if (A != nullptr) OSQPCscMatrix_free(A);
      return false;
    }

    OSQPSettings* settings = OSQPSettings_new();
    if (settings != nullptr) {
      settings->max_iter = config_.max_iter;
      if (config_.time_limit > 0.0) {
        settings->time_limit = config_.time_limit;
      }
      settings->verbose = config_.verbose;
      settings->warm_starting = 1;
    }

    OSQPSolver* solver = nullptr;
    const OSQPInt setup_status = osqp_setup(
        &solver, P, q.data(), A, lower_bounds.data(), upper_bounds.data(),
        static_cast<OSQPInt>(lower_bounds.size()), num_variables, settings);

    if (setup_status == 0 && solver != nullptr) {
      osqp_solve(solver);
      if (solver->info != nullptr &&
          (solver->info->status_val == OSQP_SOLVED ||
           solver->info->status_val == OSQP_SOLVED_INACCURATE) &&
          solver->solution != nullptr) {
        solved = true;

        for (std::size_t i = 0; i < num_points; ++i) {
          current_points[i].first = solver->solution->x[2 * i];
          current_points[i].second = solver->solution->x[2 * i + 1];
        }

        const double cur_obj_val = solver->info->obj_val;
        const double diff = std::abs(cur_obj_val - last_obj_val);
        last_obj_val = cur_obj_val;

        if (solver != nullptr) osqp_cleanup(solver);
        if (settings != nullptr) OSQPSettings_free(settings);
        OSQPCscMatrix_free(P);
        OSQPCscMatrix_free(A);

        if (diff < config_.sqp_ftol) {
          break;
        }
        continue;
      }
    }

    if (solver != nullptr) osqp_cleanup(solver);
    if (settings != nullptr) OSQPSettings_free(settings);
    OSQPCscMatrix_free(P);
    OSQPCscMatrix_free(A);
    break;
  }

  if (!solved) {
    return false;
  }

  smoothed_path->clear();
  smoothed_path->resize(num_points);

  for (std::size_t i = 0; i < num_points; ++i) {
    (*smoothed_path)[i].pose.x = current_points[i].first;
    (*smoothed_path)[i].pose.y = current_points[i].second;
    (*smoothed_path)[i].gear = raw_path[i].gear;
  }

  // Compute heading, curvature, and s
  double accumulated_s = 0.0;
  (*smoothed_path)[0].s = 0.0;

  for (std::size_t i = 0; i < num_points; ++i) {
    if (i > 0) {
      const double dx =
          (*smoothed_path)[i].pose.x - (*smoothed_path)[i - 1].pose.x;
      const double dy =
          (*smoothed_path)[i].pose.y - (*smoothed_path)[i - 1].pose.y;
      accumulated_s += std::hypot(dx, dy);
      (*smoothed_path)[i].s = accumulated_s;
    }

    if (i + 1 < num_points) {
      const double dx =
          (*smoothed_path)[i + 1].pose.x - (*smoothed_path)[i].pose.x;
      const double dy =
          (*smoothed_path)[i + 1].pose.y - (*smoothed_path)[i].pose.y;
      (*smoothed_path)[i].pose.heading = std::atan2(dy, dx);
    } else if (i > 0) {
      (*smoothed_path)[i].pose.heading = (*smoothed_path)[i - 1].pose.heading;
    }

    // Curvature calculation via 3 consecutive points
    if (i > 0 && i + 1 < num_points) {
      const double dtheta =
          NormalizeAngle((*smoothed_path)[i + 1].pose.heading -
                         (*smoothed_path)[i - 1].pose.heading);
      const double ds = (*smoothed_path)[i + 1].s - (*smoothed_path)[i - 1].s;
      (*smoothed_path)[i].curvature = (ds > 1e-4) ? (dtheta / ds) : 0.0;
    } else {
      (*smoothed_path)[i].curvature = 0.0;
    }
  }

  return true;
}

void FemPosDeviationOsqp::CalculateKernel(std::size_t num_points,
                                          std::size_t num_slack,
                                          std::vector<OSQPFloat>* P_data,
                                          std::vector<OSQPInt>* P_indices,
                                          std::vector<OSQPInt>* P_indptr) {
  const std::size_t num_variables = num_points * 2 + num_slack;
  const double w_fem = config_.weight_fem_pos_deviation;
  const double w_len = config_.weight_path_length;
  const double w_ref = config_.weight_ref_deviation;

  // cols[col][row] = value (row <= col for upper triangular P)
  std::vector<std::vector<std::pair<OSQPInt, OSQPFloat>>> cols(num_variables);
  auto add_to_P = [&cols](std::size_t r, std::size_t c, double val) {
    if (r > c) std::swap(r, c);
    cols[c].emplace_back(static_cast<OSQPInt>(r), static_cast<OSQPFloat>(val));
  };

  // 1. Reference point deviation cost: w_ref * (x_i^2 + y_i^2)
  for (std::size_t i = 0; i < num_points; ++i) {
    add_to_P(2 * i, 2 * i, 2.0 * w_ref);
    add_to_P(2 * i + 1, 2 * i + 1, 2.0 * w_ref);
  }

  // 2. Length cost: w_len * ((x_{i+1} - x_i)^2 + (y_{i+1} - y_i)^2)
  for (std::size_t i = 0; i + 1 < num_points; ++i) {
    for (std::size_t c = 0; c < 2; ++c) {
      const std::size_t idx0 = 2 * i + c;
      const std::size_t idx1 = 2 * (i + 1) + c;
      add_to_P(idx0, idx0, 2.0 * w_len);
      add_to_P(idx1, idx1, 2.0 * w_len);
      add_to_P(idx0, idx1, -2.0 * w_len);
    }
  }

  // 3. FEM curvature cost: w_fem * ((x_{i+2} - 2*x_{i+1} + x_i)^2 + ...)
  for (std::size_t i = 0; i + 2 < num_points; ++i) {
    for (std::size_t c = 0; c < 2; ++c) {
      const std::size_t idx0 = 2 * i + c;
      const std::size_t idx1 = 2 * (i + 1) + c;
      const std::size_t idx2 = 2 * (i + 2) + c;
      add_to_P(idx0, idx0, 2.0 * w_fem);
      add_to_P(idx1, idx1, 8.0 * w_fem);
      add_to_P(idx2, idx2, 2.0 * w_fem);
      add_to_P(idx0, idx1, -4.0 * w_fem);
      add_to_P(idx0, idx2, 2.0 * w_fem);
      add_to_P(idx1, idx2, -4.0 * w_fem);
    }
  }

  P_indptr->clear();
  P_indices->clear();
  P_data->clear();
  P_indptr->push_back(0);

  for (std::size_t col = 0; col < num_variables; ++col) {
    auto& col_entries = cols[col];
    std::sort(
        col_entries.begin(), col_entries.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    for (std::size_t k = 0; k < col_entries.size();) {
      const OSQPInt row = col_entries[k].first;
      OSQPFloat sum_val = 0.0;
      while (k < col_entries.size() && col_entries[k].first == row) {
        sum_val += col_entries[k].second;
        ++k;
      }
      if (std::abs(sum_val) > 1e-12) {
        P_indices->push_back(row);
        P_data->push_back(sum_val);
      }
    }
    P_indptr->push_back(static_cast<OSQPInt>(P_indices->size()));
  }
}

void FemPosDeviationOsqp::CalculateOffset(
    const std::vector<GeometricPathPoint>& raw_path, std::size_t num_slack,
    std::vector<OSQPFloat>* q) {
  const double w_ref = config_.weight_ref_deviation;
  const double w_slack = config_.weight_curvature_constraint_slack_var;
  const std::size_t num_points = raw_path.size();
  q->clear();
  q->reserve(num_points * 2 + num_slack);

  for (const auto& pt : raw_path) {
    q->push_back(-2.0 * w_ref * pt.pose.x);
    q->push_back(-2.0 * w_ref * pt.pose.y);
  }

  for (std::size_t k = 0; k < num_slack; ++k) {
    q->push_back(w_slack);
  }
}

void FemPosDeviationOsqp::CalculateAffineConstraints(
    const std::vector<std::pair<double, double>>& current_points,
    const std::vector<GeometricPathPoint>& raw_path,
    const std::vector<CorridorSample>& corridor, std::vector<OSQPFloat>* A_data,
    std::vector<OSQPInt>* A_indices, std::vector<OSQPInt>* A_indptr,
    std::vector<OSQPFloat>* lower_bounds,
    std::vector<OSQPFloat>* upper_bounds) {
  const std::size_t num_points = raw_path.size();
  const std::size_t num_slack = num_points - 2;
  const std::size_t num_pos_variables = num_points * 2;
  const std::size_t num_variables = num_pos_variables + num_slack;

  std::vector<std::vector<double>> lin_params;
  lin_params.reserve(num_slack);
  for (std::size_t i = 1; i + 1 < num_points; ++i) {
    lin_params.push_back(CalculateLinearizedFemPosParams(current_points, i));
  }

  const std::size_t num_constraints = num_variables + num_slack;
  std::vector<std::vector<std::pair<OSQPInt, OSQPFloat>>> cols(num_variables);

  // 1. Direct variable bounds (Identity on diagonal)
  for (std::size_t i = 0; i < num_variables; ++i) {
    cols[i].emplace_back(static_cast<OSQPInt>(i), 1.0);
  }

  // 2. Curvature constraint rows
  for (std::size_t k = 0; k < num_slack; ++k) {
    const OSQPInt row = static_cast<OSQPInt>(num_variables + k);
    const auto& lp = lin_params[k];

    // Point k (first)
    cols[2 * k].emplace_back(row, lp[0]);
    cols[2 * k + 1].emplace_back(row, lp[1]);

    // Point k+1 (middle)
    cols[2 * (k + 1)].emplace_back(row, lp[2]);
    cols[2 * (k + 1) + 1].emplace_back(row, lp[3]);

    // Point k+2 (last)
    cols[2 * (k + 2)].emplace_back(row, lp[4]);
    cols[2 * (k + 2) + 1].emplace_back(row, lp[5]);

    // Slack variable
    cols[num_pos_variables + k].emplace_back(row, -1.0);
  }

  A_indptr->clear();
  A_indices->clear();
  A_data->clear();
  A_indptr->push_back(0);

  for (std::size_t col = 0; col < num_variables; ++col) {
    auto& col_entries = cols[col];
    std::sort(
        col_entries.begin(), col_entries.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    for (const auto& entry : col_entries) {
      if (std::abs(entry.second) > 1e-12) {
        A_indices->push_back(entry.first);
        A_data->push_back(entry.second);
      }
    }
    A_indptr->push_back(static_cast<OSQPInt>(A_indices->size()));
  }

  lower_bounds->resize(num_constraints);
  upper_bounds->resize(num_constraints);

  // Position Bounds
  for (std::size_t i = 0; i < num_points; ++i) {
    double box_bound = config_.default_box_bound;
    if (i < corridor.size()) {
      const double width = corridor[i].maximum_lateral_offset -
                           corridor[i].minimum_lateral_offset;
      box_bound = std::max(0.2, width * 0.5);
    }

    // Anchor first and last point for continuity
    if (i == 0 || i + 1 == num_points) {
      box_bound = 0.0;
    }

    (*lower_bounds)[2 * i] = raw_path[i].pose.x - box_bound;
    (*upper_bounds)[2 * i] = raw_path[i].pose.x + box_bound;
    (*lower_bounds)[2 * i + 1] = raw_path[i].pose.y - box_bound;
    (*upper_bounds)[2 * i + 1] = raw_path[i].pose.y + box_bound;
  }

  // Slack Bounds (s_k in [0, 1e20])
  for (std::size_t k = 0; k < num_slack; ++k) {
    (*lower_bounds)[num_pos_variables + k] = 0.0;
    (*upper_bounds)[num_pos_variables + k] = 1e20;
  }

  // Curvature Constraint Bounds
  for (std::size_t k = 0; k < num_slack; ++k) {
    const std::size_t idx = k + 1;
    const double dx = raw_path[idx + 1].pose.x - raw_path[idx - 1].pose.x;
    const double dy = raw_path[idx + 1].pose.y - raw_path[idx - 1].pose.y;
    const double ds = std::max(0.1, std::hypot(dx, dy) * 0.5);

    const double curvature_limit = config_.curvature_constraint;
    const double max_second_diff = curvature_limit * ds * ds;
    const double curvature_sqr = max_second_diff * max_second_diff;

    const OSQPInt row = static_cast<OSQPInt>(num_variables + k);
    (*lower_bounds)[row] = -1e20;
    (*upper_bounds)[row] = curvature_sqr - lin_params[k][6];
  }
}

}  // namespace open_space_planning
}  // namespace apollo
