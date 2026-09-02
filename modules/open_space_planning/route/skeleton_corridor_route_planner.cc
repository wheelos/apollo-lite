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

#include "modules/open_space_planning/route/skeleton_corridor_route_planner.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <utility>
#include <vector>

namespace apollo {
namespace open_space_planning {

namespace {

struct GridNode {
  int x = 0;
  int y = 0;
  double g = 0.0;
  double f = 0.0;
  bool operator>(const GridNode& other) const { return f > other.f; }
};

inline double EuclideanDistance(double x1, double y1, double x2, double y2) {
  return std::hypot(x1 - x2, y1 - y2);
}

}  // namespace

SkeletonCorridorRoutePlanner::SkeletonCorridorRoutePlanner(
    const SkeletonCorridorConfig& config)
    : config_(config) {}

Status SkeletonCorridorRoutePlanner::Plan(
    const RoutePlanningRequest& request,
    std::vector<RouteCandidate>* candidates) {
  if (candidates == nullptr) {
    return {StatusCode::kInvalidInput, "candidates pointer is null"};
  }
  candidates->clear();

  const auto& problem = request.problem;
  const auto& map_ptr = problem.grid_map;

  RouteCandidate candidate;
  candidate.id = 1;
  candidate.topology_id = 1;
  candidate.map_revision = map_ptr != nullptr ? map_ptr->revision : 0;
  candidate.goal_revision = problem.goal.revision;

  if (map_ptr == nullptr || map_ptr->cell_state.empty() ||
      map_ptr->resolution <= 0.0) {
    // Fallback: straight line from start to goal
    const double dx = problem.goal.pose.x - problem.start.pose.x;
    const double dy = problem.goal.pose.y - problem.start.pose.y;
    const double dist = std::hypot(dx, dy);
    const std::size_t num_pts =
        std::max<std::size_t>(3, static_cast<std::size_t>(dist / 0.5) + 1);

    for (std::size_t i = 0; i < num_pts; ++i) {
      const double ratio = static_cast<double>(i) / (num_pts - 1);
      GeometricPathPoint pt;
      pt.pose.x = problem.start.pose.x + ratio * dx;
      pt.pose.y = problem.start.pose.y + ratio * dy;
      pt.pose.heading = std::atan2(dy, dx);
      pt.s = ratio * dist;
      pt.curvature = 0.0;
      pt.gear = Gear::kDrive;
      candidate.skeleton.push_back(pt);

      CorridorSample sample;
      sample.s = pt.s;
      sample.minimum_lateral_offset = -config_.default_corridor_half_width;
      sample.maximum_lateral_offset = config_.default_corridor_half_width;
      candidate.corridor.push_back(sample);
    }
  } else {
    // 2D Grid A* Search
    const auto& map = *map_ptr;
    const int w = static_cast<int>(map.width);
    const int h = static_cast<int>(map.height);
    const double res = map.resolution;
    const double ox = map.origin.x;
    const double oy = map.origin.y;

    auto to_grid_x = [&](double x) {
      return std::clamp(static_cast<int>((x - ox) / res), 0, w - 1);
    };
    auto to_grid_y = [&](double y) {
      return std::clamp(static_cast<int>((y - oy) / res), 0, h - 1);
    };
    auto to_world_x = [&](int gx) { return ox + (gx + 0.5) * res; };
    auto to_world_y = [&](int gy) { return oy + (gy + 0.5) * res; };

    const int start_gx = to_grid_x(problem.start.pose.x);
    const int start_gy = to_grid_y(problem.start.pose.y);
    const int goal_gx = to_grid_x(problem.goal.pose.x);
    const int goal_gy = to_grid_y(problem.goal.pose.y);

    const int margin_cells =
        std::max(1, static_cast<int>(config_.obstacle_inflation_margin / res));

    // Obstacle map check with safety margin
    auto is_occupied = [&](int gx, int gy) {
      for (int dy = -margin_cells; dy <= margin_cells; ++dy) {
        for (int dx = -margin_cells; dx <= margin_cells; ++dx) {
          const int nx = gx + dx;
          const int ny = gy + dy;
          if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
            const std::size_t idx = ny * w + nx;
            if (map.cell_state[idx] == CellState::kOccupied ||
                map.cell_state[idx] == CellState::kNoDrive) {
              return true;
            }
          }
        }
      }
      return false;
    };

    std::vector<double> g_score(w * h, 1e20);
    std::vector<int> parent(w * h, -1);
    std::priority_queue<GridNode, std::vector<GridNode>, std::greater<GridNode>>
        open_set;

    const int start_idx = start_gy * w + start_gx;
    g_score[start_idx] = 0.0;
    open_set.push({start_gx, start_gy, 0.0,
                   EuclideanDistance(start_gx, start_gy, goal_gx, goal_gy)});

    const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const double cost8[8] = {1.0,        1.0,        1.0,        1.0,
                             std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0),
                             std::sqrt(2.0)};

    bool reached = false;
    while (!open_set.empty()) {
      const auto curr = open_set.top();
      open_set.pop();

      if (curr.x == goal_gx && curr.y == goal_gy) {
        reached = true;
        break;
      }

      const int curr_idx = curr.y * w + curr.x;
      if (curr.g > g_score[curr_idx] + 1e-4) {
        continue;
      }

      for (int i = 0; i < 8; ++i) {
        const int nx = curr.x + dx8[i];
        const int ny = curr.y + dy8[i];
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;

        if (is_occupied(nx, ny)) continue;

        const double tentative_g = curr.g + cost8[i];
        const int n_idx = ny * w + nx;
        if (tentative_g < g_score[n_idx]) {
          g_score[n_idx] = tentative_g;
          parent[n_idx] = curr_idx;
          const double h_cost = EuclideanDistance(nx, ny, goal_gx, goal_gy);
          open_set.push({nx, ny, tentative_g, tentative_g + h_cost});
        }
      }
    }

    if (!reached) {
      return {StatusCode::kRouteSearchFailed,
              "SkeletonCorridorRoutePlanner: 2D search could not reach goal"};
    }

    // Reconstruct grid path
    std::vector<std::pair<double, double>> path_points;
    int curr_idx = goal_gy * w + goal_gx;
    while (curr_idx != -1) {
      const int gy = curr_idx / w;
      const int gx = curr_idx % w;
      path_points.emplace_back(to_world_x(gx), to_world_y(gy));
      curr_idx = parent[curr_idx];
    }
    std::reverse(path_points.begin(), path_points.end());

    // Replace endpoints with exact start and goal
    path_points.front() = {problem.start.pose.x, problem.start.pose.y};
    path_points.back() = {problem.goal.pose.x, problem.goal.pose.y};

    // Subsample / Resample to step_size
    double total_s = 0.0;
    std::vector<GeometricPathPoint> raw_pts;
    for (std::size_t i = 0; i < path_points.size(); ++i) {
      GeometricPathPoint pt;
      pt.pose.x = path_points[i].first;
      pt.pose.y = path_points[i].second;
      if (i > 0) {
        total_s += EuclideanDistance(
            path_points[i].first, path_points[i].second,
            path_points[i - 1].first, path_points[i - 1].second);
      }
      pt.s = total_s;
      pt.gear = Gear::kDrive;
      raw_pts.push_back(pt);
    }

    // Compute heading and corridor
    const std::size_t n = raw_pts.size();
    candidate.skeleton.resize(n);
    candidate.corridor.resize(n);

    for (std::size_t i = 0; i < n; ++i) {
      candidate.skeleton[i] = raw_pts[i];
      if (i + 1 < n) {
        const double dx = raw_pts[i + 1].pose.x - raw_pts[i].pose.x;
        const double dy = raw_pts[i + 1].pose.y - raw_pts[i].pose.y;
        candidate.skeleton[i].pose.heading = std::atan2(dy, dx);
      } else if (i > 0) {
        candidate.skeleton[i].pose.heading =
            candidate.skeleton[i - 1].pose.heading;
      }

      // Raycast lateral bounds for Safe Flight Corridor (SFC)
      const double heading = candidate.skeleton[i].pose.heading;
      const double left_nx = -std::sin(heading);
      const double left_ny = std::cos(heading);
      const double right_nx = std::sin(heading);
      const double right_ny = -std::cos(heading);

      double left_bound = config_.default_corridor_half_width;
      double right_bound = config_.default_corridor_half_width;

      for (double d = 0.2; d <= config_.default_corridor_half_width; d += 0.2) {
        const double test_lx = raw_pts[i].pose.x + d * left_nx;
        const double test_ly = raw_pts[i].pose.y + d * left_ny;
        if (is_occupied(to_grid_x(test_lx), to_grid_y(test_ly))) {
          left_bound = std::max(0.2, d - 0.2);
          break;
        }
      }

      for (double d = 0.2; d <= config_.default_corridor_half_width; d += 0.2) {
        const double test_rx = raw_pts[i].pose.x + d * right_nx;
        const double test_ry = raw_pts[i].pose.y + d * right_ny;
        if (is_occupied(to_grid_x(test_rx), to_grid_y(test_ry))) {
          right_bound = std::max(0.2, d - 0.2);
          break;
        }
      }

      CorridorSample cs;
      cs.s = raw_pts[i].s;
      cs.minimum_lateral_offset = -right_bound;
      cs.maximum_lateral_offset = left_bound;
      candidate.corridor[i] = cs;
    }
  }

  GearSegment seg;
  seg.begin_index = 0;
  seg.end_index =
      candidate.skeleton.empty() ? 0 : candidate.skeleton.size() - 1;
  seg.gear = Gear::kDrive;
  candidate.gear_segments.push_back(seg);
  candidate.search_cost = static_cast<double>(candidate.skeleton.size());
  candidate.minimum_clearance = 0.5;

  candidates->push_back(std::move(candidate));
  return Status::Ok();
}

}  // namespace open_space_planning
}  // namespace apollo
