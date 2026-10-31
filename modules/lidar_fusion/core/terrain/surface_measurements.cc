// Copyright 2026 WheelOS. All Rights Reserved.

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>

#include "modules/lidar_fusion/core/terrain/plane_fit.h"
#include "modules/lidar_fusion/core/terrain/surface_internal.h"

namespace apollo {
namespace lidar_fusion {
namespace terrain {
namespace detail {
namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

bool HasObservedLayers(const CellResult& cell) {
  return !cell.observed_layers.empty();
}
}  // namespace

void MeasureSurface(const SurfaceConfig& config, TerrainEstimate& estimate,
                    const std::vector<uint8_t>& selected,
                    const std::vector<std::vector<double>>& layer_heights) {
  const std::size_t nx = estimate.nx;
  const std::size_t ny = estimate.ny;
  using detail::FitPlane;
  using detail::GridPoint;
  using detail::Plane;
  for (std::size_t ix = 0; ix < nx; ++ix) {
    for (std::size_t iy = 0; iy < ny; ++iy) {
      const std::size_t cell_id = estimate.CellId(ix, iy);
      if (!selected[cell_id]) {
        continue;
      }
      auto& cell = estimate.cells[cell_id];
      std::vector<GridPoint> local_xy;
      std::vector<double> local_z;
      for (std::size_t xx = ix == 0 ? 0 : ix - 1; xx < std::min(nx, ix + 2);
           ++xx) {
        for (std::size_t yy = iy == 0 ? 0 : iy - 1; yy < std::min(ny, iy + 2);
             ++yy) {
          const std::size_t neighbor = estimate.CellId(xx, yy);
          if (!selected[neighbor]) {
            continue;
          }
          local_xy.push_back({config.x_min + (xx + 0.5) * config.cell_size,
                              config.y_min + (yy + 0.5) * config.cell_size});
          local_z.push_back(estimate.cells[neighbor].elevation_z);
        }
      }
      Plane local_plane;
      const bool has_plane = FitPlane(local_xy, local_z, &local_plane);
      if (has_plane) {
        cell.slope_gradient = std::hypot(local_plane.a, local_plane.b);
        cell.roughness_m = local_plane.residual_std;
        cell.has_slope = true;
        cell.has_roughness = true;
      }
      std::vector<double> discontinuities;
      const bool layer_span =
          cell.has_layer_span &&
          cell.observed_layer_span_m > config.max_neighbor_height_change;
      if (layer_span) {
        discontinuities.push_back(cell.observed_layer_span_m);
      }
      bool boundary_discontinuity = false;
      const int dxs[] = {1, -1, 0, 0};
      const int dys[] = {0, 0, 1, -1};
      for (int direction = 0; direction < 4; ++direction) {
        const int xx = static_cast<int>(ix) + dxs[direction];
        const int yy = static_cast<int>(iy) + dys[direction];
        if (xx < 0 || yy < 0 || xx >= static_cast<int>(nx) ||
            yy >= static_cast<int>(ny)) {
          continue;
        }
        const std::size_t neighbor = estimate.CellId(
            static_cast<std::size_t>(xx), static_cast<std::size_t>(yy));
        if (estimate.cells[neighbor].observed_layers.empty()) {
          continue;
        }
        double closest = std::numeric_limits<double>::infinity();
        for (double height : layer_heights[neighbor]) {
          if (std::abs(height - cell.elevation_z) <
              std::abs(closest - cell.elevation_z)) {
            closest = height;
          }
        }
        double expected_delta = 0.0;
        if (has_plane) {
          expected_delta = local_plane.a * dxs[direction] * config.cell_size +
                           local_plane.b * dys[direction] * config.cell_size;
        }
        const double difference =
            std::abs(closest - cell.elevation_z - expected_delta);
        if (difference > config.max_neighbor_height_change) {
          discontinuities.push_back(difference);
          boundary_discontinuity = true;
        }
      }
      if (discontinuities.empty()) {
        cell.step_status = StepStatus::kNoDiscontinuityObserved;
      } else {
        cell.step_m =
            *std::max_element(discontinuities.begin(), discontinuities.end());
        cell.has_step = true;
        if (layer_span && boundary_discontinuity) {
          cell.step_status = StepStatus::kLayerSpanAndBoundaryDiscontinuity;
        } else if (layer_span) {
          cell.step_status = StepStatus::kObservedWithinCellLayerSpan;
        } else {
          cell.step_status = StepStatus::kObservedBoundaryDiscontinuity;
        }
      }
    }
  }
}

void MeasureDepressions(const SurfaceConfig& config, TerrainEstimate& estimate,
                        const std::vector<uint8_t>& selected) {
  const std::size_t nx = estimate.nx;
  const std::size_t ny = estimate.ny;
  using detail::ContainsOriginStrict;
  using detail::FitPlane;
  using detail::GridPoint;
  using detail::Plane;
  for (std::size_t cell_id = 0; cell_id < estimate.cells.size(); ++cell_id) {
    auto& cell = estimate.cells[cell_id];
    if (selected[cell_id]) {
      cell.pit_status = cell.observed_layers.size() > 1
                            ? PitStatus::kUnknownLayerAmbiguity
                            : PitStatus::kNoDepressionObserved;
    } else if (HasObservedLayers(cell)) {
      cell.pit_status = PitStatus::kUnknownNoRim;
    } else {
      cell.pit_status = PitStatus::kUnknownNoFloor;
    }
  }

  std::vector<uint8_t> low_cells(estimate.cells.size(), 0);
  std::vector<double> pit_depth(estimate.cells.size(), kNaN);
  for (std::size_t ix = 0; ix < nx; ++ix) {
    for (std::size_t iy = 0; iy < ny; ++iy) {
      const std::size_t cell_id = estimate.CellId(ix, iy);
      const auto& cell = estimate.cells[cell_id];
      if (selected[cell_id] || cell.observed_layers.empty()) {
        continue;
      }
      const auto floor = std::min_element(
          cell.observed_layers.begin(), cell.observed_layers.end(),
          [](const LayerObservation& lhs, const LayerObservation& rhs) {
            return lhs.elevation_z < rhs.elevation_z;
          });
      const double cx = config.x_min + (ix + 0.5) * config.cell_size;
      const double cy = config.y_min + (iy + 0.5) * config.cell_size;
      const double radius_cells_value =
          std::ceil(config.pit_rim_search_radius / config.cell_size);
      const std::size_t radius_cells =
          radius_cells_value >= static_cast<double>(std::max(nx, ny))
              ? std::max(nx, ny)
              : static_cast<std::size_t>(radius_cells_value);
      std::vector<std::size_t> ring_cells;
      std::vector<GridPoint> ring_xy;
      for (std::size_t xx = ix > radius_cells ? ix - radius_cells : 0;
           xx < std::min(nx, ix + radius_cells + 1); ++xx) {
        for (std::size_t yy = iy > radius_cells ? iy - radius_cells : 0;
             yy < std::min(ny, iy + radius_cells + 1); ++yy) {
          const std::size_t rim_id = estimate.CellId(xx, yy);
          if (!selected[rim_id]) {
            continue;
          }
          const double x = config.x_min + (xx + 0.5) * config.cell_size;
          const double y = config.y_min + (yy + 0.5) * config.cell_size;
          if (std::hypot(x - cx, y - cy) <= config.pit_rim_search_radius) {
            ring_cells.push_back(rim_id);
            ring_xy.push_back({x, y});
          }
        }
      }
      if (ring_xy.size() < 4 || !ContainsOriginStrict([&]() {
            std::vector<GridPoint> offsets = ring_xy;
            for (auto& point : offsets) {
              point.x -= cx;
              point.y -= cy;
            }
            return offsets;
          }())) {
        continue;
      }
      std::vector<double> ring_z;
      ring_z.reserve(ring_cells.size());
      for (std::size_t rim_id : ring_cells) {
        ring_z.push_back(estimate.cells[rim_id].elevation_z);
      }
      Plane rim_plane;
      if (!FitPlane(ring_xy, ring_z, &rim_plane)) {
        continue;
      }
      const double reference = rim_plane.a * (cx - rim_plane.center_x) +
                               rim_plane.b * (cy - rim_plane.center_y) +
                               rim_plane.c;
      const double depth = reference - floor->elevation_z;
      if (depth <= config.max_neighbor_height_change) {
        estimate.cells[cell_id].pit_status = PitStatus::kNoDepressionObserved;
        continue;
      }
      low_cells[cell_id] = 1;
      pit_depth[cell_id] = depth;
    }
  }

  std::vector<uint8_t> visited(low_cells.size(), 0);
  for (std::size_t seed = 0; seed < low_cells.size(); ++seed) {
    if (!low_cells[seed] || visited[seed]) {
      continue;
    }
    std::vector<std::size_t> component;
    std::deque<std::size_t> queue{seed};
    visited[seed] = 1;
    std::size_t support = 0;
    std::size_t min_x = nx;
    std::size_t min_y = ny;
    std::size_t max_x = 0;
    std::size_t max_y = 0;
    while (!queue.empty()) {
      const std::size_t cell_id = queue.front();
      queue.pop_front();
      component.push_back(cell_id);
      const std::size_t ix = cell_id / ny;
      const std::size_t iy = cell_id % ny;
      support += estimate.cells[cell_id].observed_layers.front().point_count;
      min_x = std::min(min_x, ix);
      min_y = std::min(min_y, iy);
      max_x = std::max(max_x, ix);
      max_y = std::max(max_y, iy);
      const int dxs[] = {1, -1, 0, 0};
      const int dys[] = {0, 0, 1, -1};
      for (int direction = 0; direction < 4; ++direction) {
        const int xx = static_cast<int>(ix) + dxs[direction];
        const int yy = static_cast<int>(iy) + dys[direction];
        if (xx < 0 || yy < 0 || xx >= static_cast<int>(nx) ||
            yy >= static_cast<int>(ny)) {
          continue;
        }
        const std::size_t neighbor = estimate.CellId(
            static_cast<std::size_t>(xx), static_cast<std::size_t>(yy));
        if (low_cells[neighbor] && !visited[neighbor]) {
          visited[neighbor] = 1;
          queue.push_back(neighbor);
        }
      }
    }
    if (support < config.min_pit_floor_points) {
      for (std::size_t cell_id : component) {
        estimate.cells[cell_id].pit_status =
            PitStatus::kUnknownInsufficientFloor;
      }
      continue;
    }
    const double width_x = (max_x - min_x + 1) * config.cell_size;
    const double width_y = (max_y - min_y + 1) * config.cell_size;
    for (std::size_t cell_id : component) {
      auto& cell = estimate.cells[cell_id];
      cell.pit_status = PitStatus::kMeasured;
      cell.pit_depth_m = pit_depth[cell_id];
      cell.pit_width_x_m = width_x;
      cell.pit_width_y_m = width_y;
      cell.has_pit_depth = true;
      cell.has_pit_width = true;
    }
  }
}

}  // namespace detail
}  // namespace terrain
}  // namespace lidar_fusion
}  // namespace apollo
