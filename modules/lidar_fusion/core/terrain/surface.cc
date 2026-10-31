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

#include "modules/lidar_fusion/core/terrain/surface.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "modules/lidar_fusion/core/terrain/surface_internal.h"

namespace apollo {
namespace lidar_fusion {
namespace terrain {

bool EstimateSurface(const std::vector<Point>& points,
                     const SurfaceConfig& config,
                     std::optional<uint32_t> requested_root_id,
                     TerrainEstimate* result, std::string* error) {
  const auto fail = [error](const std::string& message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };
  if (result == nullptr) {
    return fail("result must not be null");
  }
  if (!ValidateSurfaceConfig(config, error)) {
    return false;
  }
  if (points.size() > kMaxInputPoints) {
    return fail("point count exceeds the twenty-million-point limit");
  }
  const std::size_t nx = static_cast<std::size_t>(
      std::llround((config.x_max - config.x_min) / config.cell_size));
  const std::size_t ny = static_cast<std::size_t>(
      std::llround((config.y_max - config.y_min) / config.cell_size));
  TerrainEstimate estimate;
  estimate.config = config;
  estimate.nx = nx;
  estimate.ny = ny;
  estimate.cells.resize(nx * ny);
  estimate.points.resize(points.size());

  detail::SurfaceWorkspace workspace;
  if (!detail::BuildObservedLayers(points, &estimate, &workspace, error) ||
      !detail::SelectSurfaceLayers(points, requested_root_id, &estimate,
                                   &workspace, error)) {
    return false;
  }

  detail::ClassifySourcePoints(points, estimate, workspace);
  detail::MeasureSurface(config, estimate, workspace.selected,
                         workspace.layer_heights);
  detail::MeasureDepressions(config, estimate, workspace.selected);

  if (workspace.anchor.has_value()) {
    const auto& rows =
        workspace.layer_rows[workspace.anchor->cell][workspace.anchor->layer];
    std::size_t root_row = rows.front();
    if (requested_root_id.has_value()) {
      auto it = std::find_if(rows.begin(), rows.end(), [&](std::size_t row) {
        return points[row].id == *requested_root_id;
      });
      if (it == rows.end()) {
        return fail("selected root point is not in the selected anchor layer");
      }
      root_row = *it;
    } else {
      root_row = *std::min_element(
          rows.begin(), rows.end(), [&](std::size_t lhs, std::size_t rhs) {
            const double lhs_radius = std::hypot(points[lhs].x, points[lhs].y);
            const double rhs_radius = std::hypot(points[rhs].x, points[rhs].y);
            return lhs_radius == rhs_radius ? points[lhs].id < points[rhs].id
                                            : lhs_radius < rhs_radius;
          });
    }
    estimate.root_anchor_id = points[root_row].id;
    estimate.root_cell =
        CellIndex{workspace.anchor->cell / ny, workspace.anchor->cell % ny};
    estimate.status = TerrainEstimate::Status::kSupported;
  }
  *result = std::move(estimate);
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

}  // namespace terrain
}  // namespace lidar_fusion
}  // namespace apollo
