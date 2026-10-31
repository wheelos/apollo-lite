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

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace apollo {
namespace lidar_fusion {
namespace terrain {

constexpr std::size_t kMaxGridCells = 1000000;
constexpr std::size_t kMaxInputPoints = 20000000;

struct Point {
  uint32_t id = 0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  uint32_t risk_bits = 0;
};

struct SurfaceConfig {
  double x_min = -60.0;
  double x_max = 60.0;
  double y_min = -40.0;
  double y_max = 40.0;
  double cell_size = 0.5;
  std::optional<double> expected_ground_z;
  double seed_height_band = 0.30;
  double anchor_radius = 8.0;
  double layer_gap = 0.20;
  double ground_tolerance = 0.10;
  double max_slope_gradient = 0.30;
  double max_neighbor_height_change = 0.20;
  double pit_rim_search_radius = 3.0;
  std::size_t min_anchor_points = 3;
  std::size_t min_pit_floor_points = 3;
  std::string coordinate_frame = "gravity-aligned frame-local XYZ; z up";
};

enum class CellStatus {
  kUnknownEmpty,
  kUnknownNoAnchor,
  kUnknownGap,
  kSupported,
};

enum class StepStatus {
  kUnknownUnsupported,
  kNoDiscontinuityObserved,
  kObservedBoundaryDiscontinuity,
  kObservedWithinCellLayerSpan,
  kLayerSpanAndBoundaryDiscontinuity,
};

enum class PitStatus {
  kUnknownNoFloor,
  kUnknownNoRim,
  kUnknownInsufficientFloor,
  kUnknownLayerAmbiguity,
  kNoDepressionObserved,
  kMeasured,
};

struct CellIndex {
  std::size_t x = 0;
  std::size_t y = 0;
};

struct LayerObservation {
  int layer_id = -1;
  double elevation_z = 0.0;
  std::size_t point_count = 0;
  std::vector<uint32_t> source_point_ids;
  bool selected = false;
};

struct PointResult {
  uint32_t id = 0;
  bool ground_candidate = false;
  bool ground = false;
  bool risk = false;
  int64_t cell_id = -1;
  int layer_id = -1;
  double surface_z = 0.0;
  double residual_z = 0.0;
};

struct CellResult {
  CellStatus status = CellStatus::kUnknownEmpty;
  double elevation_z = 0.0;
  std::vector<LayerObservation> observed_layers;
  std::size_t raw_return_count = 0;
  double observed_layer_span_m = 0.0;
  double roughness_m = 0.0;
  double slope_gradient = 0.0;
  double step_m = 0.0;
  StepStatus step_status = StepStatus::kUnknownUnsupported;
  PitStatus pit_status = PitStatus::kUnknownNoFloor;
  double pit_depth_m = 0.0;
  double pit_width_x_m = 0.0;
  double pit_width_y_m = 0.0;
  bool has_elevation = false;
  bool has_layer_span = false;
  bool has_roughness = false;
  bool has_slope = false;
  bool has_step = false;
  bool has_pit_depth = false;
  bool has_pit_width = false;
};

struct TerrainEstimate {
  SurfaceConfig config;
  std::size_t nx = 0;
  std::size_t ny = 0;
  std::vector<PointResult> points;
  std::vector<CellResult> cells;
  std::optional<uint32_t> root_anchor_id;
  std::optional<CellIndex> root_cell;
  enum class Status {
    kUnknownNoAnchor,
    kSupported
  } status = Status::kUnknownNoAnchor;

  std::size_t CellId(std::size_t x, std::size_t y) const { return x * ny + y; }
};

bool ValidateSurfaceConfig(const SurfaceConfig& config, std::string* error);
bool EstimateSurface(const std::vector<Point>& points,
                     const SurfaceConfig& config,
                     std::optional<uint32_t> root_anchor_id,
                     TerrainEstimate* result, std::string* error);

const char* ToString(CellStatus status);
const char* ToString(StepStatus status);
const char* ToString(PitStatus status);
const char* ToString(TerrainEstimate::Status status);

}  // namespace terrain
}  // namespace lidar_fusion
}  // namespace apollo
