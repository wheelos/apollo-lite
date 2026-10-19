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

#include "modules/open_space_planning/map/reachable_region.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

namespace apollo {
namespace open_space_planning {
namespace {

struct QueueNode {
  int index = 0;
  double cost = 0.0;
  bool operator>(const QueueNode& other) const { return cost > other.cost; }
};

double DistanceToSegment(double px, double py, const Pose2d& a,
                         const Pose2d& b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= 1e-12) {
    return std::hypot(px - a.x, py - a.y);
  }
  const double projection =
      std::clamp(((px - a.x) * dx + (py - a.y) * dy) / length_squared, 0.0,
                 1.0);
  return std::hypot(px - (a.x + projection * dx),
                    py - (a.y + projection * dy));
}

bool IsPointInPolygon(const std::vector<Pose2d>& polygon, double x, double y) {
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto& a = polygon[i];
    const auto& b = polygon[j];
    if (DistanceToSegment(x, y, a, b) <= 1e-8) {
      return true;
    }
    const bool intersects =
        ((a.y > y) != (b.y > y)) &&
        (x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x);
    if (intersects) {
      inside = !inside;
    }
  }
  return inside;
}

bool IsBlocked(const GridMap& map, int x, int y, double clearance) {
  const int radius_cells =
      static_cast<int>(std::ceil(clearance / map.resolution));
  const double clearance_squared = clearance * clearance;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      if (static_cast<double>(dx * dx + dy * dy) * map.resolution *
                  map.resolution >
              clearance_squared &&
          (dx != 0 || dy != 0)) {
        continue;
      }
      const int nx = x + dx;
      const int ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(map.width) ||
          ny >= static_cast<int>(map.height)) {
        return true;
      }
      const std::size_t index =
          static_cast<std::size_t>(ny) * map.width +
          static_cast<std::size_t>(nx);
      if (map.cell_state[index] != CellState::kFree) {
        return true;
      }
      const auto center = GridCellCenter(map, nx, ny);
      if (!IsWithinDrivableRegions(map, center.x, center.y, clearance)) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

bool WorldToGrid(const GridMap& map, const double x, const double y,
                 int* grid_x, int* grid_y) {
  if (grid_x == nullptr || grid_y == nullptr || map.resolution <= 0.0 ||
      map.width > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      map.height > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return false;
  }
  const double dx = x - map.origin.x;
  const double dy = y - map.origin.y;
  const double cos_heading = std::cos(map.origin.heading);
  const double sin_heading = std::sin(map.origin.heading);
  const double local_x = cos_heading * dx + sin_heading * dy;
  const double local_y = -sin_heading * dx + cos_heading * dy;
  *grid_x = static_cast<int>(std::floor(local_x / map.resolution));
  *grid_y = static_cast<int>(std::floor(local_y / map.resolution));
  return *grid_x >= 0 && *grid_y >= 0 &&
         *grid_x < static_cast<int>(map.width) &&
         *grid_y < static_cast<int>(map.height);
}

Pose2d GridCellCenter(const GridMap& map, const int grid_x,
                      const int grid_y) {
  const double local_x = (static_cast<double>(grid_x) + 0.5) * map.resolution;
  const double local_y = (static_cast<double>(grid_y) + 0.5) * map.resolution;
  const double cos_heading = std::cos(map.origin.heading);
  const double sin_heading = std::sin(map.origin.heading);
  return {map.origin.x + cos_heading * local_x - sin_heading * local_y,
          map.origin.y + sin_heading * local_x + cos_heading * local_y, 0.0};
}

bool IsWithinDrivableRegions(const GridMap& map, const double x,
                             const double y,
                             const double boundary_clearance) {
  if (map.drivable_regions.empty()) {
    return true;
  }
  for (const auto& region : map.drivable_regions) {
    if (region.boundary.size() < 3 ||
        !IsPointInPolygon(region.boundary, x, y)) {
      continue;
    }
    double minimum_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < region.boundary.size(); ++i) {
      minimum_distance =
          std::min(minimum_distance,
                   DistanceToSegment(x, y, region.boundary[i],
                                     region.boundary[(i + 1) %
                                                     region.boundary.size()]));
    }
    if (minimum_distance + 1e-8 >= boundary_clearance) {
      return true;
    }
  }
  return false;
}

PoseTraversability CheckVehiclePose(const GridMap& map,
                                    const VehicleModel& vehicle,
                                    const Pose2d& pose) {
  if (map.resolution <= 0.0 || map.cell_state.size() != map.cell_count() ||
      vehicle.front_edge_to_center < 0.0 ||
      vehicle.back_edge_to_center < 0.0 ||
      vehicle.left_edge_to_center < 0.0 ||
      vehicle.right_edge_to_center < 0.0) {
    return PoseTraversability::kBlocked;
  }
  const double cos_heading = std::cos(pose.heading);
  const double sin_heading = std::sin(pose.heading);
  const double half_diagonal = map.resolution * std::sqrt(2.0) * 0.5;
  const double x_limits[] = {-vehicle.back_edge_to_center,
                             vehicle.front_edge_to_center};
  const double y_limits[] = {-vehicle.right_edge_to_center,
                             vehicle.left_edge_to_center};
  int minimum_grid_x = static_cast<int>(map.width) - 1;
  int maximum_grid_x = 0;
  int minimum_grid_y = static_cast<int>(map.height) - 1;
  int maximum_grid_y = 0;
  for (const double x_body : x_limits) {
    for (const double y_body : y_limits) {
      const double x = pose.x + x_body * cos_heading - y_body * sin_heading;
      const double y = pose.y + x_body * sin_heading + y_body * cos_heading;
      int grid_x = 0;
      int grid_y = 0;
      if (!WorldToGrid(map, x, y, &grid_x, &grid_y)) {
        return PoseTraversability::kOutsideMap;
      }
      if (!IsWithinDrivableRegions(map, x, y)) {
        return PoseTraversability::kOutsideDrivableRegion;
      }
      minimum_grid_x = std::min(minimum_grid_x, grid_x);
      maximum_grid_x = std::max(maximum_grid_x, grid_x);
      minimum_grid_y = std::min(minimum_grid_y, grid_y);
      maximum_grid_y = std::max(maximum_grid_y, grid_y);
    }
  }

  const int cell_margin =
      static_cast<int>(std::ceil(half_diagonal / map.resolution));
  minimum_grid_x = std::max(0, minimum_grid_x - cell_margin);
  maximum_grid_x =
      std::min(static_cast<int>(map.width) - 1, maximum_grid_x + cell_margin);
  minimum_grid_y = std::max(0, minimum_grid_y - cell_margin);
  maximum_grid_y =
      std::min(static_cast<int>(map.height) - 1, maximum_grid_y + cell_margin);
  for (int y = minimum_grid_y; y <= maximum_grid_y; ++y) {
    for (int x = minimum_grid_x; x <= maximum_grid_x; ++x) {
      const auto cell_center = GridCellCenter(map, x, y);
      const double dx = cell_center.x - pose.x;
      const double dy = cell_center.y - pose.y;
      const double longitudinal = dx * cos_heading + dy * sin_heading;
      const double lateral = -dx * sin_heading + dy * cos_heading;
      if (longitudinal < -vehicle.back_edge_to_center - half_diagonal ||
          longitudinal > vehicle.front_edge_to_center + half_diagonal ||
          lateral < -vehicle.right_edge_to_center - half_diagonal ||
          lateral > vehicle.left_edge_to_center + half_diagonal) {
        continue;
      }
      const std::size_t index =
          static_cast<std::size_t>(y) * map.width +
          static_cast<std::size_t>(x);
      if (map.cell_state[index] != CellState::kFree) {
        return PoseTraversability::kBlocked;
      }
      if (!IsWithinDrivableRegions(map, cell_center.x, cell_center.y)) {
        return PoseTraversability::kOutsideDrivableRegion;
      }
    }
  }
  return PoseTraversability::kFree;
}

Status ComputeReachableRegion(const GridMap& map, const Pose2d& seed,
                              const double obstacle_clearance,
                              ReachableRegion* region) {
  if (region == nullptr) {
    return {StatusCode::kInvalidInput, "reachable region output is null"};
  }
  if (map.width == 0 || map.height == 0 || map.resolution <= 0.0 ||
      map.width > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      map.height > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      map.width > std::numeric_limits<std::size_t>::max() / map.height ||
      map.cell_state.size() != map.cell_count()) {
    return {StatusCode::kInvalidInput, "grid map is invalid"};
  }
  if (!map.traversal_cost.empty() &&
      map.traversal_cost.size() != map.cell_count()) {
    return {StatusCode::kInvalidInput, "grid map cost size mismatch"};
  }
  int seed_x = 0;
  int seed_y = 0;
  if (!WorldToGrid(map, seed.x, seed.y, &seed_x, &seed_y)) {
    return {StatusCode::kInvalidInput, "reachable region seed is outside map"};
  }
  if (!std::isfinite(obstacle_clearance) || obstacle_clearance < 0.0) {
    return {StatusCode::kInvalidInput, "obstacle clearance is invalid"};
  }

  region->reachable.assign(map.cell_count(), false);
  region->accumulated_cost.assign(
      map.cell_count(), std::numeric_limits<double>::infinity());
  region->parent_index.assign(map.cell_count(), -1);
  region->seed_index =
      static_cast<std::size_t>(seed_y) * map.width +
      static_cast<std::size_t>(seed_x);
  if (IsBlocked(map, seed_x, seed_y, obstacle_clearance)) {
    return {StatusCode::kRouteSearchFailed,
            "reachable region seed is blocked"};
  }

  std::priority_queue<QueueNode, std::vector<QueueNode>,
                      std::greater<QueueNode>>
      queue;
  region->reachable[region->seed_index] = true;
  region->accumulated_cost[region->seed_index] = 0.0;
  queue.push({static_cast<int>(region->seed_index), 0.0});

  constexpr int kDx[] = {1, -1, 0, 0, 1, 1, -1, -1};
  constexpr int kDy[] = {0, 0, 1, -1, 1, -1, 1, -1};
  constexpr double kStepCost[] = {1.0, 1.0, 1.0, 1.0, 1.41421356237,
                                  1.41421356237, 1.41421356237,
                                  1.41421356237};
  while (!queue.empty()) {
    const auto current = queue.top();
    queue.pop();
    if (current.cost > region->accumulated_cost[current.index] + 1e-9) {
      continue;
    }
    const int x = current.index % static_cast<int>(map.width);
    const int y = current.index / static_cast<int>(map.width);
    for (std::size_t direction = 0; direction < 8; ++direction) {
      const int next_x = x + kDx[direction];
      const int next_y = y + kDy[direction];
      if (next_x < 0 || next_y < 0 ||
          next_x >= static_cast<int>(map.width) ||
          next_y >= static_cast<int>(map.height) ||
          IsBlocked(map, next_x, next_y, obstacle_clearance)) {
        continue;
      }
      if (kDx[direction] != 0 && kDy[direction] != 0 &&
          (IsBlocked(map, x + kDx[direction], y, obstacle_clearance) ||
           IsBlocked(map, x, y + kDy[direction], obstacle_clearance))) {
        continue;
      }
      const std::size_t next_index =
          static_cast<std::size_t>(next_y) * map.width +
          static_cast<std::size_t>(next_x);
      const double cell_cost =
          map.traversal_cost.empty()
              ? 0.0
              : static_cast<double>(map.traversal_cost[next_index]) / 255.0;
      const double next_cost =
          current.cost + kStepCost[direction] * (1.0 + cell_cost);
      if (next_cost + 1e-9 < region->accumulated_cost[next_index]) {
        region->reachable[next_index] = true;
        region->accumulated_cost[next_index] = next_cost;
        region->parent_index[next_index] = current.index;
        queue.push({static_cast<int>(next_index), next_cost});
      }
    }
  }
  return Status::Ok();
}

}  // namespace open_space_planning
}  // namespace apollo
