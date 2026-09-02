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

#include "modules/open_space_planning/route/hybrid_a_star_route_planner.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "modules/open_space_planning/route/skeleton_corridor_route_planner.h"

namespace apollo {
namespace open_space_planning {

using apollo::common::math::Vec2d;

HybridAStarRoutePlanner::HybridAStarRoutePlanner() {
  hybrid_a_star_.reset(new planning::HybridAStar(config_));
}

HybridAStarRoutePlanner::HybridAStarRoutePlanner(
    const planning::PlannerOpenSpaceConfig& config)
    : config_(config) {
  hybrid_a_star_.reset(new planning::HybridAStar(config_));
}

void HybridAStarRoutePlanner::ExtractXYBounds(const PlanningProblem& problem,
                                             std::vector<double>* xy_bounds) {
  if (xy_bounds == nullptr) {
    return;
  }
  xy_bounds->clear();

  if (problem.grid_map != nullptr && problem.grid_map->width > 0 &&
      problem.grid_map->height > 0 && problem.grid_map->resolution > 0.0) {
    const double min_x = problem.grid_map->origin.x;
    const double max_x = problem.grid_map->origin.x +
                         problem.grid_map->width * problem.grid_map->resolution;
    const double min_y = problem.grid_map->origin.y;
    const double max_y =
        problem.grid_map->origin.y +
        problem.grid_map->height * problem.grid_map->resolution;
    *xy_bounds = {min_x, max_x, min_y, max_y};
    return;
  }

  // Fallback: build enclosing bounds with margin around start and goal
  const double min_x =
      std::min(problem.start.pose.x, problem.goal.pose.x) - 25.0;
  const double max_x =
      std::max(problem.start.pose.x, problem.goal.pose.x) + 25.0;
  const double min_y =
      std::min(problem.start.pose.y, problem.goal.pose.y) - 25.0;
  const double max_y =
      std::max(problem.start.pose.y, problem.goal.pose.y) + 25.0;
  *xy_bounds = {min_x, max_x, min_y, max_y};
}

void HybridAStarRoutePlanner::ExtractObstacles(
    const PlanningProblem& problem,
    std::vector<std::vector<Vec2d>>* obstacles_vertices_vec) {
  if (obstacles_vertices_vec == nullptr) {
    return;
  }
  obstacles_vertices_vec->clear();

  // Extract from dynamic obstacles footprint
  for (const auto& obstacle : problem.dynamic_obstacles) {
    if (obstacle.footprint.size() >= 3) {
      std::vector<Vec2d> vertices;
      vertices.reserve(obstacle.footprint.size() + 1);
      for (const auto& pt : obstacle.footprint) {
        vertices.emplace_back(pt.x, pt.y);
      }
      // Close the polygon if needed
      if (std::abs(vertices.front().x() - vertices.back().x()) > 1e-4 ||
          std::abs(vertices.front().y() - vertices.back().y()) > 1e-4) {
        vertices.push_back(vertices.front());
      }
      obstacles_vertices_vec->push_back(std::move(vertices));
    }
  }

  // Extract from GridMap occupied / no-drive cells.
  // Merge contiguous horizontal runs of obstacle cells into rectangular strips
  // to avoid per-cell polygon explosion while preserving concave geometry.
  if (problem.grid_map != nullptr && !problem.grid_map->cell_state.empty()) {
    const auto& map = *(problem.grid_map);
    const double res = map.resolution;
    const std::size_t h = map.height;
    const std::size_t w = map.width;

    struct Span {
      std::size_t start_col;
      std::size_t end_col;
      std::size_t row_start;
      std::size_t row_end;
    };
    std::vector<Span> active_spans;

    for (std::size_t r = 0; r < h; ++r) {
      std::vector<Span> row_spans;
      std::size_t c = 0;
      while (c < w) {
        const std::size_t idx = r * w + c;
        if (idx < map.cell_state.size() &&
            (map.cell_state[idx] == CellState::kOccupied ||
             map.cell_state[idx] == CellState::kNoDrive)) {
          std::size_t start_c = c;
          while (c < w) {
            const std::size_t cur_idx = r * w + c;
            if (cur_idx < map.cell_state.size() &&
                (map.cell_state[cur_idx] == CellState::kOccupied ||
                 map.cell_state[cur_idx] == CellState::kNoDrive)) {
              ++c;
            } else {
              break;
            }
          }
          row_spans.push_back({start_c, c - 1, r, r});
        } else {
          ++c;
        }
      }

      std::vector<Span> next_active_spans;
      for (auto& r_span : row_spans) {
        bool merged = false;
        for (auto& a_span : active_spans) {
          if (a_span.start_col == r_span.start_col &&
              a_span.end_col == r_span.end_col &&
              a_span.row_end + 1 == r_span.row_start) {
            a_span.row_end = r_span.row_end;
            next_active_spans.push_back(a_span);
            merged = true;
            break;
          }
        }
        if (!merged) {
          next_active_spans.push_back(r_span);
        }
      }

      for (const auto& a_span : active_spans) {
        bool found = false;
        for (const auto& n_span : next_active_spans) {
          if (n_span.start_col == a_span.start_col &&
              n_span.end_col == a_span.end_col &&
              n_span.row_start == a_span.row_start) {
            found = true;
            break;
          }
        }
        if (!found) {
          const double cell_min_x = map.origin.x + a_span.start_col * res;
          const double cell_max_x = map.origin.x + (a_span.end_col + 1) * res;
          const double cell_min_y = map.origin.y + a_span.row_start * res;
          const double cell_max_y = map.origin.y + (a_span.row_end + 1) * res;
          std::vector<Vec2d> cell_box = {
              Vec2d(cell_min_x, cell_min_y), Vec2d(cell_max_x, cell_min_y),
              Vec2d(cell_max_x, cell_max_y), Vec2d(cell_min_x, cell_max_y),
              Vec2d(cell_min_x, cell_min_y)};
          obstacles_vertices_vec->push_back(std::move(cell_box));
        }
      }
      active_spans = std::move(next_active_spans);
    }

    for (const auto& a_span : active_spans) {
      const double cell_min_x = map.origin.x + a_span.start_col * res;
      const double cell_max_x = map.origin.x + (a_span.end_col + 1) * res;
      const double cell_min_y = map.origin.y + a_span.row_start * res;
      const double cell_max_y = map.origin.y + (a_span.row_end + 1) * res;
      std::vector<Vec2d> cell_box = {
          Vec2d(cell_min_x, cell_min_y), Vec2d(cell_max_x, cell_min_y),
          Vec2d(cell_max_x, cell_max_y), Vec2d(cell_min_x, cell_max_y),
          Vec2d(cell_min_x, cell_min_y)};
      obstacles_vertices_vec->push_back(std::move(cell_box));
    }
  }
}

Status HybridAStarRoutePlanner::Plan(const RoutePlanningRequest& request,
                                     std::vector<RouteCandidate>* candidates) {
  if (candidates == nullptr) {
    return {StatusCode::kInvalidInput, "candidates pointer is null"};
  }
  candidates->clear();

  if (hybrid_a_star_ == nullptr) {
    return {StatusCode::kInternalError, "hybrid a star pointer is null"};
  }

  const auto& problem = request.problem;

  RouteSearchParadigm paradigm = request.paradigm;
  if (paradigm == RouteSearchParadigm::kAuto) {
    paradigm = problem.goal.allow_reverse
                   ? RouteSearchParadigm::kParkingMultiGear
                   : RouteSearchParadigm::kCruisingForward;
  }

  if (paradigm == RouteSearchParadigm::kSkeletonCorridor) {
    SkeletonCorridorRoutePlanner corridor_planner;
    return corridor_planner.Plan(request, candidates);
  }

  std::vector<double> xy_bounds;
  ExtractXYBounds(problem, &xy_bounds);

  std::vector<std::vector<Vec2d>> obstacles_vertices_vec;
  ExtractObstacles(problem, &obstacles_vertices_vec);

  const double sx = problem.start.pose.x;
  const double sy = problem.start.pose.y;
  const double sphi = problem.start.pose.heading;

  const double ex = problem.goal.pose.x;
  const double ey = problem.goal.pose.y;
  const double ephi = problem.goal.pose.heading;

  planning::PlannerOpenSpaceConfig current_config = config_;
  if (paradigm == RouteSearchParadigm::kCruisingForward) {
    // Forward cruising paradigm: suppress reverse and gear switch oscillation
    current_config.mutable_warm_start_config()->set_traj_back_penalty(1.0e8);
    current_config.mutable_warm_start_config()->set_traj_gear_switch_penalty(
        1.0e8);
    current_config.mutable_warm_start_config()->set_traj_steer_penalty(1.5);
    current_config.mutable_warm_start_config()->set_traj_steer_change_penalty(
        3.0);
  } else if (paradigm == RouteSearchParadigm::kParkingMultiGear) {
    // Multi-gear parking paradigm: allow reverse with robust switch penalty
    current_config.mutable_warm_start_config()->set_traj_back_penalty(2.0);
    current_config.mutable_warm_start_config()->set_traj_gear_switch_penalty(
        50.0);
    current_config.mutable_warm_start_config()->set_traj_steer_penalty(1.0);
    current_config.mutable_warm_start_config()->set_traj_steer_change_penalty(
        2.0);
  }
  planning::HybridAStar solver(current_config);

  planning::HybridAStartResult search_result;
  const bool search_success = solver.Plan(
      sx, sy, sphi, ex, ey, ephi, xy_bounds, obstacles_vertices_vec,
      &search_result);

  if (!search_success || search_result.x.empty()) {
    return {StatusCode::kRouteSearchFailed,
            "Hybrid A* failed to find a valid path to destination"};
  }

  // Partition trajectory by gear shifts
  std::vector<planning::HybridAStartResult> partitioned_results;
  if (!solver.TrajectoryPartition(search_result, &partitioned_results) ||
      partitioned_results.empty()) {
    partitioned_results.push_back(search_result);
  }

  RouteCandidate candidate;
  candidate.id = 1;
  candidate.topology_id = 1;
  candidate.map_revision =
      problem.grid_map != nullptr ? problem.grid_map->revision : 0;
  candidate.goal_revision = problem.goal.revision;

  std::size_t current_index = 0;
  for (const auto& segment_result : partitioned_results) {
    GearSegment gear_segment;
    gear_segment.begin_index = current_index;
    gear_segment.gear =
        (!segment_result.v.empty() && segment_result.v.front() < -1e-3)
            ? Gear::kReverse
            : Gear::kDrive;

    const std::size_t seg_size = segment_result.x.size();
    for (std::size_t i = 0; i < seg_size; ++i) {
      GeometricPathPoint pt;
      pt.pose.x = segment_result.x[i];
      pt.pose.y = segment_result.y[i];
      pt.pose.heading = segment_result.phi[i];
      pt.s = (i < segment_result.accumulated_s.size())
                 ? segment_result.accumulated_s[i]
                 : static_cast<double>(current_index + i);
      pt.curvature =
          (i < segment_result.steer.size() && problem.vehicle.wheel_base > 0.0)
              ? std::tan(segment_result.steer[i]) / problem.vehicle.wheel_base
              : 0.0;
      pt.gear = gear_segment.gear;
      candidate.skeleton.push_back(pt);

      CorridorSample corridor_sample;
      corridor_sample.s = pt.s;
      corridor_sample.minimum_lateral_offset = -2.0;
      corridor_sample.maximum_lateral_offset = 2.0;
      candidate.corridor.push_back(corridor_sample);
    }

    current_index += seg_size;
    gear_segment.end_index = (current_index > 0) ? current_index - 1 : 0;
    candidate.gear_segments.push_back(gear_segment);
  }

  candidate.search_cost = static_cast<double>(candidate.skeleton.size());
  candidate.minimum_clearance = 0.5;

  candidates->push_back(std::move(candidate));
  return Status::Ok();
}

}  // namespace open_space_planning
}  // namespace apollo
