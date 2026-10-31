// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "modules/lidar_fusion/core/terrain/surface.h"

namespace apollo {
namespace lidar_fusion {
namespace terrain {
namespace detail {

struct LayerKey {
  std::size_t cell;
  int layer;
};

struct SurfaceWorkspace {
  std::vector<int> point_layer;
  std::vector<int> selected_layer;
  std::vector<std::vector<std::vector<std::size_t>>> layer_rows;
  std::vector<std::vector<double>> layer_heights;
  std::vector<uint8_t> selected;
  std::optional<LayerKey> anchor;
};

bool BuildObservedLayers(const std::vector<Point>& points,
                         TerrainEstimate* estimate, SurfaceWorkspace* workspace,
                         std::string* error);
bool SelectSurfaceLayers(const std::vector<Point>& points,
                         std::optional<uint32_t> requested_root_id,
                         TerrainEstimate* estimate, SurfaceWorkspace* workspace,
                         std::string* error);
void ClassifySourcePoints(const std::vector<Point>& points,
                          TerrainEstimate& estimate,
                          const SurfaceWorkspace& workspace);
void MeasureSurface(const SurfaceConfig& config, TerrainEstimate& estimate,
                    const std::vector<uint8_t>& selected,
                    const std::vector<std::vector<double>>& layer_heights);
void MeasureDepressions(const SurfaceConfig& config, TerrainEstimate& estimate,
                        const std::vector<uint8_t>& selected);

}  // namespace detail
}  // namespace terrain
}  // namespace lidar_fusion
}  // namespace apollo
