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

#include "modules/open_space_planning/trajectory/speed_optimizer/piecewise_jerk_speed_osqp.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
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

}  // namespace

PiecewiseJerkSpeedOsqp::PiecewiseJerkSpeedOsqp() {
}

PiecewiseJerkSpeedOsqp::PiecewiseJerkSpeedOsqp(
    PiecewiseJerkSpeedOsqpConfig config)
    : config_(config) {}

void PiecewiseJerkSpeedOsqp::ComputeSTBoundaries(
    const std::vector<GeometricPathPoint>& path,
    const std::vector<DynamicObstacle>& obstacles,
    double time_horizon, double delta_t, double vehicle_safety_margin,
    std::vector<STBoundary>* st_boundaries) {
  if (st_boundaries == nullptr || path.empty() || obstacles.empty()) {
    return;
  }
  st_boundaries->clear();

  const std::size_t num_steps =
      static_cast<std::size_t>(std::ceil(time_horizon / delta_t)) + 1;

  for (const auto& obs : obstacles) {
    STBoundary boundary;
    boundary.obstacle_id = obs.id;

    for (std::size_t step = 0; step < num_steps; ++step) {
      const double t = step * delta_t;
      // Find obstacle pose at time t by linear interpolation
      Pose2d obs_pose;
      bool found_state = false;

      if (!obs.prediction.empty()) {
        if (t <= obs.prediction.front().relative_time) {
          obs_pose = obs.prediction.front().pose;
          found_state = true;
        } else if (t >= obs.prediction.back().relative_time) {
          obs_pose = obs.prediction.back().pose;
          found_state = true;
        } else {
          for (std::size_t k = 0; k + 1 < obs.prediction.size(); ++k) {
            if (t >= obs.prediction[k].relative_time &&
                t <= obs.prediction[k + 1].relative_time) {
              const double dt = obs.prediction[k + 1].relative_time -
                                obs.prediction[k].relative_time;
              const double r =
                  (dt > 1e-6) ? (t - obs.prediction[k].relative_time) / dt
                              : 0.0;
              obs_pose.x = (1.0 - r) * obs.prediction[k].pose.x +
                           r * obs.prediction[k + 1].pose.x;
              obs_pose.y = (1.0 - r) * obs.prediction[k].pose.y +
                           r * obs.prediction[k + 1].pose.y;
              obs_pose.heading = obs.prediction[k].pose.heading;
              found_state = true;
              break;
            }
          }
        }
      }

      if (!found_state) {
        continue;
      }

      // Project obstacle pose onto path points
      double min_s = std::numeric_limits<double>::infinity();
      double max_s = -std::numeric_limits<double>::infinity();
      bool overlaps = false;

      for (const auto& path_pt : path) {
        const double dist = std::hypot(path_pt.pose.x - obs_pose.x,
                                       path_pt.pose.y - obs_pose.y);
        if (dist <= vehicle_safety_margin) {
          overlaps = true;
          min_s = std::min(min_s, path_pt.s);
          max_s = std::max(max_s, path_pt.s);
        }
      }

      if (overlaps) {
        STPoint st_pt;
        st_pt.t = t;
        st_pt.s_lower = std::max(0.0, min_s - vehicle_safety_margin);
        st_pt.s_upper = max_s + vehicle_safety_margin;
        boundary.points.push_back(st_pt);
      }
    }

    if (!boundary.points.empty()) {
      st_boundaries->push_back(std::move(boundary));
    }
  }
}

bool PiecewiseJerkSpeedOsqp::Solve(
    double s_init, double v_init, double a_init, double v_target,
    double max_path_s, const std::vector<STBoundary>& st_boundaries,
    std::vector<SpeedProfilePoint>* speed_profile) {
  if (speed_profile == nullptr) {
    return false;
  }
  speed_profile->clear();

  const double dt = config_.delta_t;
  const std::size_t num_knots =
      static_cast<std::size_t>(std::ceil(config_.total_time / dt)) + 1;
  if (num_knots < 2) {
    return false;
  }

  const std::size_t n = num_knots;
  const OSQPInt num_variables = static_cast<OSQPInt>(3 * n);

  // States: x = [s_0..s_{n-1}, v_0..v_{n-1}, a_0..a_{n-1}]
  // Kernel matrix P (upper triangular: row <= col)
  std::vector<std::vector<std::pair<OSQPInt, OSQPFloat>>> P_cols(num_variables);
  auto add_to_P = [&P_cols](std::size_t r, std::size_t c, double val) {
    if (r > c) std::swap(r, c);
    P_cols[c].emplace_back(static_cast<OSQPInt>(r),
                           static_cast<OSQPFloat>(val));
  };

  // s cost
  for (std::size_t i = 0; i < n; ++i) {
    add_to_P(i, i, 2.0 * config_.weight_s);
  }

  // v cost & v_ref cost
  const double w_v_total = config_.weight_v + config_.weight_v_ref;
  for (std::size_t i = 0; i < n; ++i) {
    add_to_P(n + i, n + i, 2.0 * w_v_total);
  }

  // a cost
  for (std::size_t i = 0; i < n; ++i) {
    add_to_P(2 * n + i, 2 * n + i, 2.0 * config_.weight_a);
  }

  // jerk cost: w_jerk * (a_{i+1} - a_i)^2 / dt^2
  const double w_jerk_factor = (2.0 * config_.weight_jerk) / (dt * dt);
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const std::size_t idx0 = 2 * n + i;
    const std::size_t idx1 = 2 * n + i + 1;
    add_to_P(idx0, idx0, w_jerk_factor);
    add_to_P(idx1, idx1, w_jerk_factor);
    add_to_P(idx0, idx1, -w_jerk_factor);
  }

  std::vector<OSQPFloat> P_data;
  std::vector<OSQPInt> P_indices;
  std::vector<OSQPInt> P_indptr;
  P_indptr.push_back(0);

  for (std::size_t col = 0; col < static_cast<std::size_t>(num_variables);
       ++col) {
    auto& col_entries = P_cols[col];
    std::sort(col_entries.begin(), col_entries.end(),
              [](const std::pair<OSQPInt, OSQPFloat>& a,
                 const std::pair<OSQPInt, OSQPFloat>& b) {
                return a.first < b.first;
              });

    for (std::size_t k = 0; k < col_entries.size();) {
      const OSQPInt row = col_entries[k].first;
      OSQPFloat sum_val = 0.0;
      while (k < col_entries.size() && col_entries[k].first == row) {
        sum_val += col_entries[k].second;
        ++k;
      }
      if (std::abs(sum_val) > 1e-12) {
        P_indices.push_back(row);
        P_data.push_back(sum_val);
      }
    }
    P_indptr.push_back(static_cast<OSQPInt>(P_indices.size()));
  }

  // Linear cost q: -2 * w_v_ref * v_target on velocity states
  std::vector<OSQPFloat> q(num_variables, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    q[n + i] = -2.0 * config_.weight_v_ref * v_target;
  }

  // Constraints:
  // 1. Box constraints on s (0 to max_path_s, modified by ST yield/stop)
  // 2. Box constraints on v (min_velocity to max_velocity)
  // 3. Box constraints on a (max_deceleration to max_acceleration)
  // 4. Initial conditions on s_0, v_0, a_0
  // 5. Kinematic continuity:
  //    s_{i+1} - s_i - v_i * dt - (1/3)*a_i*dt^2 - (1/6)*a_{i+1}*dt^2 = 0
  //    v_{i+1} - v_i - 0.5*a_i*dt - 0.5*a_{i+1}*dt = 0
  // 6. Jerk limits: -j_max * dt <= a_{i+1} - a_i <= j_max * dt

  std::vector<std::pair<double, double>> s_bounds(n, {0.0, max_path_s});

  // Apply ST boundaries (Yield / Follow by constraining s_upper)
  for (std::size_t i = 0; i < n; ++i) {
    const double t = i * dt;
    for (const auto& boundary : st_boundaries) {
      if (boundary.points.empty()) {
        continue;
      }
      const double t_start = boundary.points.front().t;
      const double t_end = boundary.points.back().t;
      if (t < t_start - 1e-3 || t > t_end + 1e-3) {
        continue;
      }

      double s_lower = boundary.points.front().s_lower;
      for (std::size_t j = 0; j + 1 < boundary.points.size(); ++j) {
        const auto& p0 = boundary.points[j];
        const auto& p1 = boundary.points[j + 1];
        if (t >= p0.t - 1e-4 && t <= p1.t + 1e-4) {
          const double ratio =
              (p1.t > p0.t + 1e-6) ? (t - p0.t) / (p1.t - p0.t) : 0.0;
          s_lower = p0.s_lower + ratio * (p1.s_lower - p0.s_lower);
          break;
        }
      }

      if (s_lower > s_init) {
        s_bounds[i].second = std::min(s_bounds[i].second, s_lower);
      }
    }
  }

  std::vector<std::vector<std::pair<OSQPInt, OSQPFloat>>> A_cols(num_variables);
  std::vector<OSQPFloat> lower_bounds;
  std::vector<OSQPFloat> upper_bounds;
  OSQPInt row_idx = 0;

  // 1. Bounds on s_i
  for (std::size_t i = 0; i < n; ++i) {
    A_cols[i].emplace_back(row_idx, 1.0);
    lower_bounds.push_back(s_bounds[i].first);
    upper_bounds.push_back(s_bounds[i].second);
    ++row_idx;
  }

  // 2. Bounds on v_i
  for (std::size_t i = 0; i < n; ++i) {
    A_cols[n + i].emplace_back(row_idx, 1.0);
    lower_bounds.push_back(config_.min_velocity);
    upper_bounds.push_back(config_.max_velocity);
    ++row_idx;
  }

  // 3. Bounds on a_i
  for (std::size_t i = 0; i < n; ++i) {
    A_cols[2 * n + i].emplace_back(row_idx, 1.0);
    lower_bounds.push_back(config_.max_deceleration);
    upper_bounds.push_back(config_.max_acceleration);
    ++row_idx;
  }

  // 4. Initial conditions s_0 = s_init, v_0 = v_init, a_0 = a_init
  A_cols[0].emplace_back(row_idx, 1.0);
  lower_bounds.push_back(s_init);
  upper_bounds.push_back(s_init);
  ++row_idx;

  A_cols[n].emplace_back(row_idx, 1.0);
  lower_bounds.push_back(v_init);
  upper_bounds.push_back(v_init);
  ++row_idx;

  A_cols[2 * n].emplace_back(row_idx, 1.0);
  lower_bounds.push_back(a_init);
  upper_bounds.push_back(a_init);
  ++row_idx;

  // 5. Kinematic equality constraints (s and v continuity)
  for (std::size_t i = 0; i + 1 < n; ++i) {
    // s_{i+1} - s_i - v_i*dt - (1/3)*a_i*dt^2 - (1/6)*a_{i+1}*dt^2 = 0
    A_cols[i].emplace_back(row_idx, -1.0);
    A_cols[i + 1].emplace_back(row_idx, 1.0);
    A_cols[n + i].emplace_back(row_idx, -dt);
    A_cols[2 * n + i].emplace_back(row_idx, -1.0 / 3.0 * dt * dt);
    A_cols[2 * n + i + 1].emplace_back(row_idx, -1.0 / 6.0 * dt * dt);
    lower_bounds.push_back(0.0);
    upper_bounds.push_back(0.0);
    ++row_idx;

    // v_{i+1} - v_i - 0.5*a_i*dt - 0.5*a_{i+1}*dt = 0
    A_cols[n + i].emplace_back(row_idx, -1.0);
    A_cols[n + i + 1].emplace_back(row_idx, 1.0);
    A_cols[2 * n + i].emplace_back(row_idx, -0.5 * dt);
    A_cols[2 * n + i + 1].emplace_back(row_idx, -0.5 * dt);
    lower_bounds.push_back(0.0);
    upper_bounds.push_back(0.0);
    ++row_idx;
  }

  // 6. Jerk constraints
  for (std::size_t i = 0; i + 1 < n; ++i) {
    A_cols[2 * n + i].emplace_back(row_idx, -1.0);
    A_cols[2 * n + i + 1].emplace_back(row_idx, 1.0);
    lower_bounds.push_back(-config_.max_jerk * dt);
    upper_bounds.push_back(config_.max_jerk * dt);
    ++row_idx;
  }

  std::vector<OSQPFloat> A_data;
  std::vector<OSQPInt> A_indices;
  std::vector<OSQPInt> A_indptr;
  A_indptr.push_back(0);

  for (const auto& col : A_cols) {
    for (const auto& elem : col) {
      A_indices.push_back(elem.first);
      A_data.push_back(elem.second);
    }
    A_indptr.push_back(static_cast<OSQPInt>(A_indices.size()));
  }

  OSQPCscMatrix* P = OwnedCscMatrix(num_variables, num_variables, P_data,
                                    P_indices, P_indptr);
  OSQPCscMatrix* A = OwnedCscMatrix(static_cast<OSQPInt>(lower_bounds.size()),
                                    num_variables, A_data, A_indices, A_indptr);

  if (P == nullptr || A == nullptr) {
    if (P != nullptr) OSQPCscMatrix_free(P);
    if (A != nullptr) OSQPCscMatrix_free(A);
    return false;
  }

  OSQPSettings* settings = OSQPSettings_new();
  if (settings != nullptr) {
    settings->max_iter = config_.max_iter;
    settings->verbose = config_.verbose;
    settings->warm_starting = 1;
  }

  OSQPSolver* solver = nullptr;
  const OSQPInt setup_status =
      osqp_setup(&solver, P, q.data(), A, lower_bounds.data(),
                 upper_bounds.data(), static_cast<OSQPInt>(lower_bounds.size()),
                 num_variables, settings);

  bool solved = false;
  if (setup_status == 0 && solver != nullptr) {
    osqp_solve(solver);
    if (solver->info != nullptr &&
        (solver->info->status_val == OSQP_SOLVED ||
         solver->info->status_val == OSQP_SOLVED_INACCURATE) &&
        solver->solution != nullptr) {
      solved = true;
      speed_profile->resize(n);
      for (std::size_t i = 0; i < n; ++i) {
        (*speed_profile)[i].t = i * dt;
        (*speed_profile)[i].s = solver->solution->x[i];
        (*speed_profile)[i].v = solver->solution->x[n + i];
        (*speed_profile)[i].a = solver->solution->x[2 * n + i];
      }
    }
  }

  if (solver != nullptr) osqp_cleanup(solver);
  if (settings != nullptr) OSQPSettings_free(settings);
  OSQPCscMatrix_free(P);
  OSQPCscMatrix_free(A);

  return solved;
}

}  // namespace open_space_planning
}  // namespace apollo
