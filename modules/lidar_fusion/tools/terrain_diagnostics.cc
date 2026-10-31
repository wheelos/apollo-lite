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

#include <cmath>
#include <cstdint>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "modules/lidar_fusion/core/terrain/surface.h"
#include "modules/lidar_fusion/core/terrain/traversability.h"

namespace fs = std::filesystem;
namespace terrain = apollo::lidar_fusion::terrain;
namespace {

class Tokens {
 public:
  explicit Tokens(std::istream* input) : input_(input) {}

  std::string Next(const std::string& field) {
    std::string token;
    if (!(*input_ >> token)) {
      throw std::runtime_error("missing token for " + field);
    }
    return token;
  }

  void Expect(const std::string& expected) {
    const std::string token = Next(expected);
    if (token != expected) {
      throw std::runtime_error("expected " + expected + ", got " + token);
    }
  }

  double Double(const std::string& field) {
    const std::string token = Next(field);
    std::size_t parsed = 0;
    double value = 0.0;
    try {
      value = std::stod(token, &parsed);
    } catch (const std::exception&) {
      throw std::runtime_error("invalid floating-point " + field + ": " +
                               token);
    }
    if (parsed != token.size()) {
      throw std::runtime_error("invalid floating-point " + field + ": " +
                               token);
    }
    return value;
  }

  uint64_t Unsigned(const std::string& field) {
    const std::string token = Next(field);
    if (token.empty() || token[0] == '-') {
      throw std::runtime_error("invalid unsigned integer " + field + ": " +
                               token);
    }
    std::size_t parsed = 0;
    uint64_t value = 0;
    try {
      value = std::stoull(token, &parsed);
    } catch (const std::exception&) {
      throw std::runtime_error("invalid unsigned integer " + field + ": " +
                               token);
    }
    if (parsed != token.size()) {
      throw std::runtime_error("invalid unsigned integer " + field + ": " +
                               token);
    }
    return value;
  }

  bool HasExtra() {
    std::string token;
    return static_cast<bool>(*input_ >> token);
  }

 private:
  std::istream* input_;
};

uint32_t ReadUint32(Tokens* tokens, const std::string& field) {
  const uint64_t value = tokens->Unsigned(field);
  if (value > std::numeric_limits<uint32_t>::max()) {
    throw std::runtime_error(field + " is outside uint32");
  }
  return static_cast<uint32_t>(value);
}

std::size_t ReadCount(Tokens* tokens, const std::string& field,
                      std::size_t maximum) {
  const uint64_t value = tokens->Unsigned(field);
  if (value > maximum) {
    throw std::runtime_error(field + " exceeds the supported size");
  }
  return static_cast<std::size_t>(value);
}

void WriteDouble(std::ostream* output, bool available, double value) {
  if (!available || !std::isfinite(value)) {
    *output << "nan";
  } else {
    *output << std::setprecision(std::numeric_limits<double>::max_digits10)
            << value;
  }
}

void WriteBool(std::ostream* output, bool value) { *output << (value ? 1 : 0); }

void Usage(std::ostream* output) {
  *output
      << "terrain_diagnostics --input PATH --output PATH\n"
         "Input whitespace-token format (one value per token):\n"
         "  TERRAIN_V1\n"
         "  CONFIG x_min x_max y_min y_max cell_size expected_ground_z "
         "seed_height_band anchor_radius layer_gap ground_tolerance "
         "max_slope_gradient max_neighbor_height_change "
         "pit_rim_search_radius min_anchor_points min_pit_floor_points "
         "root_anchor_id\n"
         "    use nan for absent expected_ground_z; use -1 for no root ID\n"
         "  LIMITS max_slope_gradient max_roughness_m max_step_m "
         "max_depression_depth_m\n"
         "  POINTS count\n"
         "  count rows: point_id:uint32 x y z point_risk_bits:uint32\n"
         "  RISK_GRID nx ny\n"
         "  nx*ny x-major rows: ix iy risk_bits coverage(0|1)\n"
         "Output records:\n"
         "  META nx ny root_id|none root_ix|none root_iy|none "
         "surface_status overall_traversability_status quoted_frame\n"
         "  CELL_COLUMNS ix iy cell_status elevation_z slope_gradient "
         "roughness_m step_m step_status pit_depth_m pit_width_x_m "
         "pit_width_y_m pit_status terrain_state decision_state reason_bits "
         "terrain_candidate risk_candidate terrain_connected "
         "risk_connected risk_bits risk_observed raw_return_count "
         "observed_layer_span_m\n"
         "  LAYER_COLUMNS ix iy layer_id elevation_z point_count selected "
         "source_point_ids(comma-separated)\n"
         "  POINT_COLUMNS id cell_id layer_id ground_candidate ground risk "
         "surface_z residual_z\n"
         "All cell rows are x-major; point rows preserve source order. "
         "Non-finite metrics are `nan`.\n"
         "Enums are serialized by their stable names; reason bits are the "
         "Python diagnostic bit assignments. This is diagnostic only and "
         "never emits FREE.\n";
}

struct Input {
  terrain::SurfaceConfig config;
  std::optional<uint32_t> root_id;
  terrain::TraversabilityLimits limits;
  std::vector<terrain::Point> points;
  terrain::RiskGrid risk;
};

Input ReadInput(const fs::path& path) {
  std::ifstream input(path);
  if (!input.good()) {
    throw std::runtime_error("cannot read input " + path.string());
  }
  Tokens tokens(&input);
  tokens.Expect("TERRAIN_V1");
  tokens.Expect("CONFIG");
  Input data;
  auto& config = data.config;
  config.x_min = tokens.Double("x_min");
  config.x_max = tokens.Double("x_max");
  config.y_min = tokens.Double("y_min");
  config.y_max = tokens.Double("y_max");
  config.cell_size = tokens.Double("cell_size");
  const double expected_ground_z = tokens.Double("expected_ground_z");
  if (!std::isnan(expected_ground_z)) {
    config.expected_ground_z = expected_ground_z;
  }
  config.seed_height_band = tokens.Double("seed_height_band");
  config.anchor_radius = tokens.Double("anchor_radius");
  config.layer_gap = tokens.Double("layer_gap");
  config.ground_tolerance = tokens.Double("ground_tolerance");
  config.max_slope_gradient = tokens.Double("max_slope_gradient");
  config.max_neighbor_height_change =
      tokens.Double("max_neighbor_height_change");
  config.pit_rim_search_radius = tokens.Double("pit_rim_search_radius");
  config.min_anchor_points =
      ReadCount(&tokens, "min_anchor_points", terrain::kMaxInputPoints);
  config.min_pit_floor_points =
      ReadCount(&tokens, "min_pit_floor_points", terrain::kMaxInputPoints);
  const std::string root = tokens.Next("root_anchor_id");
  if (root != "-1") {
    if (root.empty() || root[0] == '-') {
      throw std::runtime_error("invalid root_anchor_id: " + root);
    }
    std::size_t parsed = 0;
    uint64_t value = 0;
    try {
      value = std::stoull(root, &parsed);
    } catch (const std::exception&) {
      throw std::runtime_error("invalid root_anchor_id: " + root);
    }
    if (parsed != root.size() || value > std::numeric_limits<uint32_t>::max()) {
      throw std::runtime_error("root_anchor_id is outside uint32");
    }
    data.root_id = static_cast<uint32_t>(value);
  }

  tokens.Expect("LIMITS");
  data.limits.max_slope_gradient = tokens.Double("limit_max_slope_gradient");
  data.limits.max_roughness_m = tokens.Double("limit_max_roughness_m");
  data.limits.max_step_m = tokens.Double("limit_max_step_m");
  data.limits.max_depression_depth_m =
      tokens.Double("limit_max_depression_depth_m");
  tokens.Expect("POINTS");
  const std::size_t point_count =
      ReadCount(&tokens, "point_count", terrain::kMaxInputPoints);
  data.points.reserve(point_count);
  for (std::size_t i = 0; i < point_count; ++i) {
    terrain::Point point;
    point.id = ReadUint32(&tokens, "point_id");
    point.x = tokens.Double("point_x");
    point.y = tokens.Double("point_y");
    point.z = tokens.Double("point_z");
    point.risk_bits = ReadUint32(&tokens, "point_risk_bits");
    data.points.push_back(point);
  }

  tokens.Expect("RISK_GRID");
  const std::size_t nx =
      ReadCount(&tokens, "risk_grid_nx", terrain::kMaxGridCells);
  const std::size_t ny =
      ReadCount(&tokens, "risk_grid_ny", terrain::kMaxGridCells);
  if (nx == 0 || ny == 0 || nx > terrain::kMaxGridCells / ny ||
      nx * ny > terrain::kMaxGridCells) {
    throw std::runtime_error("risk grid dimensions are invalid or too large");
  }
  data.risk.nx = nx;
  data.risk.ny = ny;
  data.risk.layers.resize(nx * ny);
  data.risk.observed.resize(nx * ny);
  for (std::size_t ix = 0; ix < nx; ++ix) {
    for (std::size_t iy = 0; iy < ny; ++iy) {
      const uint64_t row_x = tokens.Unsigned("risk_grid_ix");
      const uint64_t row_y = tokens.Unsigned("risk_grid_iy");
      if (row_x != ix || row_y != iy) {
        throw std::runtime_error("risk grid rows must be x-major ordered");
      }
      const std::size_t cell_id = ix * ny + iy;
      data.risk.layers[cell_id] = ReadUint32(&tokens, "risk_bits");
      const uint64_t covered = tokens.Unsigned("risk_coverage");
      if (covered > 1) {
        throw std::runtime_error("risk coverage must be 0 or 1");
      }
      data.risk.observed[cell_id] = static_cast<uint8_t>(covered);
    }
  }
  if (tokens.HasExtra()) {
    throw std::runtime_error("unexpected trailing tokens after risk grid");
  }
  return data;
}

void WriteOutput(const fs::path& path, const terrain::TerrainEstimate& surface,
                 const terrain::TraversabilityEstimate& traversability) {
  std::ofstream output(path);
  if (!output.good()) {
    throw std::runtime_error("cannot write output " + path.string());
  }
  output << "TERRAIN_V1_OUTPUT\n";
  output << "META " << surface.nx << ' ' << surface.ny << ' ';
  if (surface.root_anchor_id.has_value()) {
    output << *surface.root_anchor_id;
  } else {
    output << "none";
  }
  output << ' ';
  if (surface.root_cell.has_value()) {
    output << surface.root_cell->x << ' ' << surface.root_cell->y;
  } else {
    output << "none none";
  }
  output << ' ' << terrain::ToString(surface.status) << ' '
         << terrain::ToString(traversability.overall_status) << ' '
         << std::quoted(surface.config.coordinate_frame) << '\n';
  output << "CELL_COLUMNS ix iy cell_status elevation_z slope_gradient "
            "roughness_m step_m step_status pit_depth_m pit_width_x_m "
            "pit_width_y_m pit_status terrain_state decision_state "
            "reason_bits terrain_candidate risk_candidate terrain_connected "
            "risk_connected risk_bits risk_observed raw_return_count "
            "observed_layer_span_m\n";
  for (std::size_t ix = 0; ix < surface.nx; ++ix) {
    for (std::size_t iy = 0; iy < surface.ny; ++iy) {
      const std::size_t cell_id = surface.CellId(ix, iy);
      const auto& cell = surface.cells[cell_id];
      const auto& decision = traversability.cells[cell_id];
      output << ix << ' ' << iy << ' ' << terrain::ToString(cell.status) << ' ';
      WriteDouble(&output, cell.has_elevation, cell.elevation_z);
      output << ' ';
      WriteDouble(&output, cell.has_slope, cell.slope_gradient);
      output << ' ';
      WriteDouble(&output, cell.has_roughness, cell.roughness_m);
      output << ' ';
      WriteDouble(&output, cell.has_step, cell.step_m);
      output << ' ' << terrain::ToString(cell.step_status) << ' ';
      WriteDouble(&output, cell.has_pit_depth, cell.pit_depth_m);
      output << ' ';
      WriteDouble(&output, cell.has_pit_width, cell.pit_width_x_m);
      output << ' ';
      WriteDouble(&output, cell.has_pit_width, cell.pit_width_y_m);
      output << ' ' << terrain::ToString(cell.pit_status) << ' '
             << terrain::ToString(decision.terrain_state) << ' '
             << terrain::ToString(decision.decision_state) << ' '
             << decision.reason_bits << ' ';
      WriteBool(&output, decision.terrain_candidate);
      output << ' ';
      WriteBool(&output, decision.risk_constrained_candidate);
      output << ' ';
      WriteBool(&output, decision.terrain_connected);
      output << ' ';
      WriteBool(&output, decision.risk_constrained_connected);
      output << ' ' << decision.risk_layers << ' ';
      WriteBool(&output, decision.risk_observed);
      output << ' ' << cell.raw_return_count << ' ';
      WriteDouble(&output, cell.has_layer_span, cell.observed_layer_span_m);
      output << '\n';
    }
  }
  output << "LAYER_COLUMNS ix iy layer_id elevation_z point_count selected "
            "source_point_ids\n";
  for (std::size_t ix = 0; ix < surface.nx; ++ix) {
    for (std::size_t iy = 0; iy < surface.ny; ++iy) {
      const auto& cell = surface.cells[surface.CellId(ix, iy)];
      for (const auto& layer : cell.observed_layers) {
        output << ix << ' ' << iy << ' ' << layer.layer_id << ' ';
        WriteDouble(&output, true, layer.elevation_z);
        output << ' ' << layer.point_count << ' ';
        WriteBool(&output, layer.selected);
        output << ' ';
        for (std::size_t i = 0; i < layer.source_point_ids.size(); ++i) {
          if (i != 0) {
            output << ',';
          }
          output << layer.source_point_ids[i];
        }
        output << '\n';
      }
    }
  }
  output << "POINT_COLUMNS id cell_id layer_id ground_candidate ground risk "
            "surface_z residual_z\n";
  for (std::size_t i = 0; i < surface.points.size(); ++i) {
    const auto& point = surface.points[i];
    output << point.id << ' ' << point.cell_id << ' ' << point.layer_id << ' ';
    WriteBool(&output, point.ground_candidate);
    output << ' ';
    WriteBool(&output, point.ground);
    output << ' ';
    WriteBool(&output, point.risk);
    output << ' ';
    WriteDouble(
        &output,
        point.cell_id >= 0 && surface.cells[point.cell_id].has_elevation,
        point.surface_z);
    output << ' ';
    WriteDouble(
        &output,
        point.cell_id >= 0 && surface.cells[point.cell_id].has_elevation,
        point.residual_z);
    output << '\n';
  }
  output.flush();
  if (!output.good()) {
    throw std::runtime_error("failed while writing output " + path.string());
  }
}

int Run(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    Usage(&std::cout);
    return 0;
  }
  if (argc != 5 || std::string(argv[1]) != "--input" ||
      std::string(argv[3]) != "--output") {
    Usage(&std::cerr);
    return 2;
  }
  Input input = ReadInput(argv[2]);
  terrain::TerrainEstimate surface;
  std::string error;
  if (!terrain::EstimateSurface(input.points, input.config, input.root_id,
                                &surface, &error)) {
    throw std::runtime_error("terrain surface rejected input: " + error);
  }
  if (surface.nx != input.risk.nx || surface.ny != input.risk.ny) {
    throw std::runtime_error("input risk-grid dimensions disagree with config");
  }
  terrain::TraversabilityEstimate traversability;
  if (!terrain::ClassifyTraversability(surface, input.limits, input.risk,
                                       &traversability, &error)) {
    throw std::runtime_error("terrain classifier rejected input: " + error);
  }
  WriteOutput(argv[4], surface, traversability);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return Run(argc, argv);
  } catch (const std::exception& exception) {
    std::cerr << "Terrain diagnostic stopped: " << exception.what()
              << std::endl;
    return 1;
  }
}
