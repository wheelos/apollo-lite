// Copyright 2026 WheelOS. All Rights Reserved.

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "modules/lidar_fusion/core/terrain/plane_fit.h"
#include "modules/lidar_fusion/core/terrain/surface_internal.h"

namespace apollo {
namespace lidar_fusion {
namespace terrain {
namespace detail {
namespace {

bool IsFinitePoint(const Point& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

std::size_t CellFrom(const TerrainEstimate& estimate, std::size_t x,
                     std::size_t y) {
  return estimate.CellId(x, y);
}

bool SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

}  // namespace

bool BuildObservedLayers(const std::vector<Point>& points,
                         TerrainEstimate* estimate, SurfaceWorkspace* workspace,
                         std::string* error) {
  auto& estimate_ref = *estimate;
  const auto& config = estimate_ref.config;
  const std::size_t nx = estimate_ref.nx;
  const std::size_t ny = estimate_ref.ny;
  const auto fail = [error](const std::string& message) {
    return SetError(message, error);
  };
  std::unordered_set<uint32_t> ids;
  ids.reserve(points.size());
  std::vector<std::vector<std::size_t>> buckets(nx * ny);
  workspace->point_layer.assign(points.size(), -1);
  auto& point_layer = workspace->point_layer;
  workspace->selected_layer.assign(nx * ny, -1);
  workspace->layer_rows.resize(nx * ny);
  auto& layer_rows = workspace->layer_rows;
  workspace->layer_heights.resize(nx * ny);
  auto& layer_heights = workspace->layer_heights;

  for (std::size_t row = 0; row < points.size(); ++row) {
    const auto& point = points[row];
    if (!ids.insert(point.id).second) {
      return fail("point IDs must be unique");
    }
    auto& output = estimate_ref.points[row];
    output.id = point.id;
    output.risk = point.risk_bits != 0;
    if (!IsFinitePoint(point) || point.x < config.x_min ||
        point.x >= config.x_max || point.y < config.y_min ||
        point.y >= config.y_max) {
      continue;
    }
    const auto ix = static_cast<std::size_t>(
        std::floor((point.x - config.x_min) / config.cell_size));
    const auto iy = static_cast<std::size_t>(
        std::floor((point.y - config.y_min) / config.cell_size));
    if (ix >= nx || iy >= ny) {
      return fail("finite in-ROI point mapped outside the configured grid");
    }
    const std::size_t cell_id = CellFrom(estimate_ref, ix, iy);
    output.cell_id = static_cast<int64_t>(cell_id);
    buckets[cell_id].push_back(row);
  }

  for (std::size_t cell_id = 0; cell_id < buckets.size(); ++cell_id) {
    const auto& bucket = buckets[cell_id];
    auto& cell = estimate_ref.cells[cell_id];
    if (bucket.empty()) {
      continue;
    }
    cell.raw_return_count = bucket.size();
    std::vector<std::size_t> ordered = bucket;
    std::stable_sort(ordered.begin(), ordered.end(),
                     [&points](std::size_t lhs, std::size_t rhs) {
                       return points[lhs].z < points[rhs].z;
                     });
    std::vector<std::vector<std::size_t>> groups;
    groups.emplace_back();
    for (std::size_t row : ordered) {
      if (!groups.back().empty() &&
          points[row].z - points[groups.back().back()].z > config.layer_gap) {
        groups.emplace_back();
      }
      groups.back().push_back(row);
    }
    double low = std::numeric_limits<double>::infinity();
    double high = -std::numeric_limits<double>::infinity();
    for (std::size_t layer = 0; layer < groups.size(); ++layer) {
      auto rows = groups[layer];
      std::vector<double> heights;
      heights.reserve(rows.size());
      for (std::size_t row : rows) {
        heights.push_back(points[row].z);
        point_layer[row] = static_cast<int>(layer);
      }
      std::sort(heights.begin(), heights.end());
      const std::size_t middle = heights.size() / 2;
      const double elevation =
          heights.size() % 2 == 0
              ? heights[middle - 1] * 0.5 + heights[middle] * 0.5
              : heights[middle];
      LayerObservation observation;
      observation.layer_id = static_cast<int>(layer);
      observation.elevation_z = elevation;
      observation.point_count = rows.size();
      observation.source_point_ids.reserve(rows.size());
      for (std::size_t row : rows) {
        observation.source_point_ids.push_back(points[row].id);
      }
      cell.observed_layers.push_back(std::move(observation));
      layer_rows[cell_id].push_back(std::move(rows));
      layer_heights[cell_id].push_back(elevation);
      low = std::min(low, elevation);
      high = std::max(high, elevation);
    }
    cell.observed_layer_span_m = high - low;
    cell.has_layer_span = true;
    cell.status = CellStatus::kUnknownNoAnchor;
  }

  return true;
}

bool SelectSurfaceLayers(const std::vector<Point>& points,
                         std::optional<uint32_t> requested_root_id,
                         TerrainEstimate* estimate, SurfaceWorkspace* workspace,
                         std::string* error) {
  auto& estimate_ref = *estimate;
  const auto& config = estimate_ref.config;
  const std::size_t nx = estimate_ref.nx;
  const std::size_t ny = estimate_ref.ny;
  auto& selected_layer = workspace->selected_layer;
  auto& point_layer = workspace->point_layer;
  auto& layer_rows = workspace->layer_rows;
  auto& layer_heights = workspace->layer_heights;
  auto& anchor = workspace->anchor;
  const auto fail = [error](const std::string& message) {
    return SetError(message, error);
  };
  auto row_layer_key = [&](std::size_t row) {
    const std::size_t cell_id =
        static_cast<std::size_t>(estimate_ref.points[row].cell_id);
    return LayerKey{cell_id, point_layer[row]};
  };
  auto anchor_has_support = [&](std::size_t anchor_cell, int anchor_layer) {
    const auto ax = anchor_cell / ny;
    const auto ay = anchor_cell % ny;
    const double anchor_z = layer_heights[anchor_cell][anchor_layer];
    std::vector<GridPoint> support_xy;
    std::vector<double> support_z;
    const std::size_t x0 = ax == 0 ? 0 : ax - 1;
    const std::size_t y0 = ay == 0 ? 0 : ay - 1;
    const std::size_t x1 = std::min(nx, ax + 2);
    const std::size_t y1 = std::min(ny, ay + 2);
    for (std::size_t ix = x0; ix < x1; ++ix) {
      for (std::size_t iy = y0; iy < y1; ++iy) {
        const std::size_t neighbor = CellFrom(estimate_ref, ix, iy);
        if (layer_heights[neighbor].empty()) {
          continue;
        }
        const double x = config.x_min + (ix + 0.5) * config.cell_size;
        const double y = config.y_min + (iy + 0.5) * config.cell_size;
        const double anchor_x = config.x_min + (ax + 0.5) * config.cell_size;
        const double anchor_y = config.y_min + (ay + 0.5) * config.cell_size;
        const double distance = std::hypot(x - anchor_x, y - anchor_y);
        int best_layer = -1;
        double best_delta = std::numeric_limits<double>::infinity();
        for (std::size_t layer = 0; layer < layer_heights[neighbor].size();
             ++layer) {
          const double delta =
              std::abs(layer_heights[neighbor][layer] - anchor_z);
          if (delta <= config.max_slope_gradient * distance +
                           config.ground_tolerance &&
              delta < best_delta) {
            best_delta = delta;
            best_layer = static_cast<int>(layer);
          }
        }
        if (best_layer >= 0) {
          support_xy.push_back({x, y});
          support_z.push_back(layer_heights[neighbor][best_layer]);
        }
      }
    }
    Plane fit;
    if (!FitPlane(support_xy, support_z, &fit, 3)) {
      return false;
    }
    return std::hypot(fit.a, fit.b) <= config.max_slope_gradient &&
           fit.max_abs_residual <= config.ground_tolerance;
  };

  if (requested_root_id.has_value()) {
    auto it = std::find_if(
        points.begin(), points.end(),
        [&](const Point& point) { return point.id == *requested_root_id; });
    if (it == points.end()) {
      return fail("root_anchor_id must identify one input source point");
    }
    const std::size_t row = static_cast<std::size_t>(it - points.begin());
    if (estimate_ref.points[row].cell_id < 0) {
      return fail("root_anchor_id must identify a finite in-ROI point");
    }
    const LayerKey key = row_layer_key(row);
    const auto local_layer = static_cast<std::size_t>(key.layer);
    if (layer_rows[key.cell][local_layer].size() >= config.min_anchor_points &&
        anchor_has_support(key.cell, key.layer)) {
      anchor = key;
    }
  } else {
    struct Candidate {
      double radius;
      double elevation;
      std::size_t cell;
      int layer;
    };
    std::vector<Candidate> candidates;
    for (std::size_t ix = 0; ix < nx; ++ix) {
      for (std::size_t iy = 0; iy < ny; ++iy) {
        const std::size_t cell_id = CellFrom(estimate_ref, ix, iy);
        const double x = config.x_min + (ix + 0.5) * config.cell_size;
        const double y = config.y_min + (iy + 0.5) * config.cell_size;
        const double radius = std::hypot(x, y);
        if (radius > config.anchor_radius) {
          continue;
        }
        for (std::size_t layer = 0; layer < layer_rows[cell_id].size();
             ++layer) {
          const double elevation = layer_heights[cell_id][layer];
          if (layer_rows[cell_id][layer].size() < config.min_anchor_points ||
              (config.expected_ground_z.has_value() &&
               std::abs(elevation - *config.expected_ground_z) >
                   config.seed_height_band)) {
            continue;
          }
          candidates.push_back(
              {radius, elevation, cell_id, static_cast<int>(layer)});
        }
      }
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& lhs, const Candidate& rhs) {
                if (lhs.radius != rhs.radius) {
                  return lhs.radius < rhs.radius;
                }
                if (lhs.elevation != rhs.elevation) {
                  return lhs.elevation < rhs.elevation;
                }
                if (lhs.cell != rhs.cell) {
                  return lhs.cell < rhs.cell;
                }
                return lhs.layer < rhs.layer;
              });
    for (const auto& candidate : candidates) {
      if (anchor_has_support(candidate.cell, candidate.layer)) {
        anchor = LayerKey{candidate.cell, candidate.layer};
        break;
      }
    }
  }

  workspace->selected.assign(nx * ny, 0);
  auto& selected = workspace->selected;
  if (anchor.has_value()) {
    std::deque<std::size_t> queue;
    selected[anchor->cell] = 1;
    selected_layer[anchor->cell] = anchor->layer;
    auto& root_cell = estimate_ref.cells[anchor->cell];
    root_cell.status = CellStatus::kSupported;
    root_cell.elevation_z = layer_heights[anchor->cell][anchor->layer];
    root_cell.has_elevation = true;
    queue.push_back(anchor->cell);
    while (!queue.empty()) {
      const std::size_t current = queue.front();
      queue.pop_front();
      const std::size_t ix = current / ny;
      const std::size_t iy = current % ny;
      const double source_z = estimate_ref.cells[current].elevation_z;
      const int dxs[] = {1, -1, 0, 0};
      const int dys[] = {0, 0, 1, -1};
      for (int direction = 0; direction < 4; ++direction) {
        const int xx = static_cast<int>(ix) + dxs[direction];
        const int yy = static_cast<int>(iy) + dys[direction];
        if (xx < 0 || yy < 0 || xx >= static_cast<int>(nx) ||
            yy >= static_cast<int>(ny)) {
          continue;
        }
        const std::size_t neighbor =
            CellFrom(estimate_ref, static_cast<std::size_t>(xx),
                     static_cast<std::size_t>(yy));
        if (selected[neighbor] || layer_heights[neighbor].empty()) {
          continue;
        }
        int best_layer = -1;
        double best_delta = std::numeric_limits<double>::infinity();
        for (std::size_t layer = 0; layer < layer_heights[neighbor].size();
             ++layer) {
          const double delta =
              std::abs(layer_heights[neighbor][layer] - source_z);
          if (delta < best_delta) {
            best_delta = delta;
            best_layer = static_cast<int>(layer);
          }
        }
        if (best_layer < 0 || best_delta > config.max_neighbor_height_change ||
            best_delta / config.cell_size > config.max_slope_gradient) {
          continue;
        }
        selected[neighbor] = 1;
        selected_layer[neighbor] = best_layer;
        auto& cell = estimate_ref.cells[neighbor];
        cell.status = CellStatus::kSupported;
        cell.elevation_z = layer_heights[neighbor][best_layer];
        cell.has_elevation = true;
        queue.push_back(neighbor);
      }
    }
    for (std::size_t cell_id = 0; cell_id < selected.size(); ++cell_id) {
      auto& cell = estimate_ref.cells[cell_id];
      if (cell.raw_return_count > 0 && !selected[cell_id]) {
        cell.status = CellStatus::kUnknownGap;
      }
      if (selected[cell_id]) {
        for (auto& layer : cell.observed_layers) {
          layer.selected = layer.layer_id == selected_layer[cell_id];
        }
      }
    }
  } else {
    for (auto& cell : estimate_ref.cells) {
      if (cell.raw_return_count > 0) {
        cell.status = CellStatus::kUnknownNoAnchor;
      }
    }
  }

  return true;
}

void ClassifySourcePoints(const std::vector<Point>& points,
                          TerrainEstimate& estimate,
                          const SurfaceWorkspace& workspace) {
  const auto& config = estimate.config;
  const auto& selected = workspace.selected;
  const auto& selected_layer = workspace.selected_layer;
  const auto& point_layer = workspace.point_layer;
  for (std::size_t row = 0; row < points.size(); ++row) {
    auto& output = estimate.points[row];
    output.layer_id = point_layer[row];
    if (output.cell_id < 0) {
      continue;
    }
    const std::size_t cell_id = static_cast<std::size_t>(output.cell_id);
    if (!selected[cell_id]) {
      continue;
    }
    const auto& cell = estimate.cells[cell_id];
    output.surface_z = cell.elevation_z;
    output.residual_z = points[row].z - cell.elevation_z;
    const std::size_t layer = static_cast<std::size_t>(point_layer[row]);
    output.ground_candidate =
        static_cast<int>(layer) == selected_layer[cell_id] &&
        std::abs(points[row].z - cell.elevation_z) <= config.ground_tolerance;
    output.ground = output.ground_candidate && !output.risk;
  }
}

}  // namespace detail
}  // namespace terrain
}  // namespace lidar_fusion
}  // namespace apollo
