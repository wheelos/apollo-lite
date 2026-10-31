// Copyright 2026 WheelOS. All Rights Reserved.

#include <cmath>
#include <string>

#include "modules/lidar_fusion/core/terrain/surface.h"

namespace apollo {
namespace lidar_fusion {
namespace terrain {

bool ValidateSurfaceConfig(const SurfaceConfig& config, std::string* error) {
  const auto fail = [error](const std::string& message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };
  const double values[] = {
      config.x_min,
      config.x_max,
      config.y_min,
      config.y_max,
      config.cell_size,
      config.seed_height_band,
      config.anchor_radius,
      config.layer_gap,
      config.ground_tolerance,
      config.max_slope_gradient,
      config.max_neighbor_height_change,
      config.pit_rim_search_radius,
  };
  for (double value : values) {
    if (!std::isfinite(value)) {
      return fail("surface config values must be finite");
    }
  }
  if (config.expected_ground_z.has_value() &&
      !std::isfinite(*config.expected_ground_z)) {
    return fail("expected_ground_z must be finite when supplied");
  }
  if (config.x_max <= config.x_min || config.y_max <= config.y_min ||
      config.cell_size <= 0.0) {
    return fail("invalid terrain surface bounds or cell size");
  }
  const double nx_value = (config.x_max - config.x_min) / config.cell_size;
  const double ny_value = (config.y_max - config.y_min) / config.cell_size;
  if (!std::isfinite(nx_value) || !std::isfinite(ny_value) || nx_value < 1.0 ||
      ny_value < 1.0 || nx_value > static_cast<double>(kMaxGridCells) ||
      ny_value > static_cast<double>(kMaxGridCells)) {
    return fail("terrain grid dimensions are out of range");
  }
  const double nx_rounded = std::round(nx_value);
  const double ny_rounded = std::round(ny_value);
  if (std::abs(nx_value - nx_rounded) > 1e-9 ||
      std::abs(ny_value - ny_rounded) > 1e-9) {
    return fail("terrain surface bounds must align to cell size");
  }
  const auto nx = static_cast<std::size_t>(nx_rounded);
  const auto ny = static_cast<std::size_t>(ny_rounded);
  if (nx > kMaxGridCells / ny || nx * ny > kMaxGridCells) {
    return fail("terrain grid exceeds the one-million-cell limit");
  }
  if (config.seed_height_band <= 0.0 || config.anchor_radius <= 0.0 ||
      config.layer_gap <= 0.0 || config.ground_tolerance < 0.0 ||
      config.max_slope_gradient < 0.0 ||
      config.max_neighbor_height_change <= 0.0 ||
      config.pit_rim_search_radius <= 0.0) {
    return fail("invalid terrain surface thresholds");
  }
  if (config.min_anchor_points == 0 || config.min_pit_floor_points < 3) {
    return fail("invalid terrain point support counts");
  }
  if (config.coordinate_frame.empty()) {
    return fail("coordinate_frame must be non-empty");
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

}  // namespace terrain
}  // namespace lidar_fusion
}  // namespace apollo
