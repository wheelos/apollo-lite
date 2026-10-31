// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_fusion/core/terrain/traversability.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace apollo {
namespace lidar_fusion {
namespace terrain {
namespace {

bool FinitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool IsAmbiguous(const CellResult& cell) {
  return cell.observed_layers.size() > 1 ||
         cell.step_status == StepStatus::kObservedWithinCellLayerSpan ||
         cell.step_status == StepStatus::kLayerSpanAndBoundaryDiscontinuity ||
         cell.pit_status == PitStatus::kUnknownLayerAmbiguity;
}

void Mark(TraversabilityCell* cell, uint32_t reason) {
  cell->reason_bits |= reason;
}

bool NeighborEdgeAllowed(const TerrainEstimate& surface,
                         const TraversabilityLimits& limits, std::size_t lhs,
                         std::size_t rhs) {
  const auto& first = surface.cells[lhs];
  const auto& second = surface.cells[rhs];
  if (!first.has_elevation || !second.has_elevation) {
    return false;
  }
  const double edge_limit = std::min(
      surface.config.max_neighbor_height_change,
      surface.config.cell_size * std::min(surface.config.max_slope_gradient,
                                          limits.max_slope_gradient));
  const double delta = std::abs(first.elevation_z - second.elevation_z);
  return std::isfinite(delta) && delta <= edge_limit;
}

void ConnectedComponent(const TerrainEstimate& surface,
                        const TraversabilityLimits& limits,
                        const std::vector<uint8_t>& candidate,
                        std::vector<uint8_t>* connected) {
  std::fill(connected->begin(), connected->end(), 0);
  if (!surface.root_cell.has_value()) {
    return;
  }
  const auto& root = *surface.root_cell;
  const std::size_t root_id = surface.CellId(root.x, root.y);
  if (!candidate[root_id]) {
    return;
  }
  const std::size_t nx = surface.nx;
  const std::size_t ny = surface.ny;
  std::deque<std::size_t> queue{root_id};
  (*connected)[root_id] = 1;
  while (!queue.empty()) {
    const std::size_t current = queue.front();
    queue.pop_front();
    const std::size_t ix = current / ny;
    const std::size_t iy = current % ny;
    const int dxs[] = {1, -1, 0, 0};
    const int dys[] = {0, 0, 1, -1};
    for (int direction = 0; direction < 4; ++direction) {
      const int xx = static_cast<int>(ix) + dxs[direction];
      const int yy = static_cast<int>(iy) + dys[direction];
      if (xx < 0 || yy < 0 || xx >= static_cast<int>(nx) ||
          yy >= static_cast<int>(ny)) {
        continue;
      }
      const std::size_t neighbor = surface.CellId(static_cast<std::size_t>(xx),
                                                  static_cast<std::size_t>(yy));
      if ((*connected)[neighbor] || !candidate[neighbor] ||
          !NeighborEdgeAllowed(surface, limits, current, neighbor)) {
        continue;
      }
      (*connected)[neighbor] = 1;
      queue.push_back(neighbor);
    }
  }
}

}  // namespace

bool ClassifyTraversability(const TerrainEstimate& surface,
                            const TraversabilityLimits& limits,
                            const RiskGrid& input_risk,
                            TraversabilityEstimate* result,
                            std::string* error) {
  const auto fail = [error](const std::string& message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };
  if (result == nullptr) {
    return fail("result must not be null");
  }
  if (!FinitePositive(limits.max_slope_gradient) ||
      !FinitePositive(limits.max_roughness_m) ||
      !FinitePositive(limits.max_step_m) ||
      !FinitePositive(limits.max_depression_depth_m)) {
    return fail("traversability limits must be finite positive numbers");
  }
  if (surface.nx == 0 || surface.ny == 0 ||
      surface.nx > kMaxGridCells / surface.ny ||
      surface.cells.size() != surface.nx * surface.ny) {
    return fail("surface grid dimensions or storage are invalid");
  }
  RiskGrid risk = input_risk;
  if (risk.nx == 0 && risk.ny == 0 && risk.layers.empty() &&
      risk.observed.empty()) {
    risk.nx = surface.nx;
    risk.ny = surface.ny;
    risk.layers.assign(surface.cells.size(), 0);
    risk.observed.assign(surface.cells.size(), 0);
  }
  if (risk.nx != surface.nx || risk.ny != surface.ny ||
      risk.layers.size() != surface.cells.size() ||
      risk.observed.size() != surface.cells.size()) {
    return fail("risk grid must exactly match the terrain grid");
  }
  for (std::size_t cell_id = 0; cell_id < risk.layers.size(); ++cell_id) {
    if ((risk.layers[cell_id] & ~kKnownRiskBits) != 0 ||
        risk.observed[cell_id] > 1) {
      return fail("risk grid contains unsupported bits or coverage values");
    }
  }
  for (const auto& point : surface.points) {
    if (point.cell_id < -1 ||
        point.cell_id >= static_cast<int64_t>(surface.cells.size())) {
      return fail("point cell ID is outside the surface grid");
    }
    if (point.risk && point.cell_id >= 0) {
      auto& layers = risk.layers[static_cast<std::size_t>(point.cell_id)];
      if (layers == 0) {
        layers |= kUnknownRisk;
      }
    }
  }
  if (surface.root_cell.has_value() && (surface.root_cell->x >= surface.nx ||
                                        surface.root_cell->y >= surface.ny)) {
    return fail("surface root cell is outside the terrain grid");
  }
  TraversabilityEstimate estimate;
  estimate.cells.resize(surface.cells.size());
  estimate.root_cell = surface.root_cell;
  std::vector<uint8_t> terrain_candidate(surface.cells.size(), 0);
  std::vector<uint8_t> risk_candidate(surface.cells.size(), 0);
  std::vector<uint8_t> terrain_connected(surface.cells.size(), 0);
  std::vector<uint8_t> risk_connected(surface.cells.size(), 0);

  for (std::size_t cell_id = 0; cell_id < surface.cells.size(); ++cell_id) {
    const auto& source = surface.cells[cell_id];
    auto& output = estimate.cells[cell_id];
    output.risk_layers = risk.layers[cell_id];
    output.risk_observed = risk.observed[cell_id] != 0;
    const bool supported = source.status == CellStatus::kSupported;
    const bool slope_known =
        source.has_slope && std::isfinite(source.slope_gradient);
    const bool roughness_known =
        source.has_roughness && std::isfinite(source.roughness_m);
    const bool ambiguous = IsAmbiguous(source);
    const bool boundary_step =
        source.step_status == StepStatus::kObservedBoundaryDiscontinuity ||
        source.step_status == StepStatus::kLayerSpanAndBoundaryDiscontinuity;
    const bool step_measured =
        boundary_step && source.has_step && std::isfinite(source.step_m);
    const bool step_known =
        source.step_status == StepStatus::kNoDiscontinuityObserved ||
        step_measured;
    const bool pit_measured = source.pit_status == PitStatus::kMeasured &&
                              source.has_pit_depth &&
                              std::isfinite(source.pit_depth_m);
    const bool pit_known =
        source.pit_status == PitStatus::kNoDepressionObserved || pit_measured;
    const bool slope_exceeds =
        slope_known && source.slope_gradient > limits.max_slope_gradient;
    const bool roughness_exceeds =
        roughness_known && source.roughness_m > limits.max_roughness_m;
    const bool step_exceeds =
        step_measured && source.step_m > limits.max_step_m;
    const bool pit_exceeds =
        pit_measured && source.pit_depth_m > limits.max_depression_depth_m;

    if (!supported) {
      Mark(&output, kReasonUnsupported);
    }
    if (!slope_known) {
      Mark(&output, kReasonSlopeUnobserved);
    }
    if (!roughness_known) {
      Mark(&output, kReasonRoughnessUnobserved);
    }
    if (!step_known) {
      Mark(&output, kReasonStepUnknown);
    }
    if (!pit_known) {
      Mark(&output, kReasonPitUnknown);
    }
    if (ambiguous) {
      Mark(&output, kReasonLayerAmbiguity);
    }
    if (slope_exceeds) {
      Mark(&output, kReasonSlopeExceeds);
    }
    if (roughness_exceeds) {
      Mark(&output, kReasonRoughnessExceeds);
    }
    if (step_exceeds) {
      Mark(&output, kReasonStepExceeds);
    }
    if (pit_exceeds) {
      Mark(&output, kReasonPitExceeds);
    }
    if ((output.risk_layers & kMeasuredRisk) != 0) {
      Mark(&output, kReasonMeasuredRisk);
    }
    if ((output.risk_layers & kObjectRisk) != 0) {
      Mark(&output, kReasonObjectRisk);
    }
    if ((output.risk_layers & kUnknownRisk) != 0) {
      Mark(&output, kReasonUnknownRisk);
    }
    if (!output.risk_observed) {
      Mark(&output, kReasonRiskUnobserved);
    }

    const bool blocker =
        slope_exceeds || roughness_exceeds || step_exceeds || pit_exceeds;
    const bool candidate = supported && source.has_elevation &&
                           std::isfinite(source.elevation_z) && slope_known &&
                           roughness_known && step_known && pit_known &&
                           !ambiguous && !blocker;
    output.terrain_state = TerrainState::kUnknown;
    if (supported && !ambiguous) {
      output.terrain_state = TerrainState::kGroundSupported;
    }
    if (candidate) {
      output.terrain_state = TerrainState::kTerrainPassable;
      terrain_candidate[cell_id] = 1;
    }
    if (blocker) {
      output.terrain_state = TerrainState::kBlocked;
    }
    const bool risk_cell_candidate =
        candidate && output.risk_observed && output.risk_layers == 0;
    if (risk_cell_candidate) {
      risk_candidate[cell_id] = 1;
    }
    output.decision_state = DecisionState::kUnknown;
    if (risk_cell_candidate) {
      output.decision_state = DecisionState::kCandidate;
    }
    if (blocker || (output.risk_layers & (kMeasuredRisk | kObjectRisk)) != 0) {
      output.decision_state = DecisionState::kBlocked;
    }
  }

  ConnectedComponent(surface, limits, terrain_candidate, &terrain_connected);
  ConnectedComponent(surface, limits, risk_candidate, &risk_connected);
  for (std::size_t cell_id = 0; cell_id < surface.cells.size(); ++cell_id) {
    auto& output = estimate.cells[cell_id];
    output.terrain_candidate = terrain_candidate[cell_id] != 0;
    output.risk_constrained_candidate = risk_candidate[cell_id] != 0;
    output.terrain_connected = terrain_connected[cell_id] != 0;
    output.risk_constrained_connected = risk_connected[cell_id] != 0;
    if ((terrain_candidate[cell_id] && !terrain_connected[cell_id]) ||
        (risk_candidate[cell_id] && !risk_connected[cell_id])) {
      Mark(&output, kReasonDisconnected);
    }
  }
  if (surface.root_cell.has_value()) {
    const std::size_t root_id =
        surface.CellId(surface.root_cell->x, surface.root_cell->y);
    estimate.overall_status = terrain_connected[root_id]
                                  ? TraversabilityStatus::kObservedRootConnected
                                  : TraversabilityStatus::kObservedRootVetoed;
  }
  *result = std::move(estimate);
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

const char* ToString(TerrainState state) {
  switch (state) {
    case TerrainState::kUnknown:
      return "UNKNOWN";
    case TerrainState::kGroundSupported:
      return "GROUND_SUPPORTED";
    case TerrainState::kTerrainPassable:
      return "TERRAIN_PASSABLE";
    case TerrainState::kBlocked:
      return "BLOCKED";
  }
  return "INVALID";
}

const char* ToString(DecisionState state) {
  switch (state) {
    case DecisionState::kUnknown:
      return "UNKNOWN";
    case DecisionState::kCandidate:
      return "CANDIDATE";
    case DecisionState::kBlocked:
      return "BLOCKED";
  }
  return "INVALID";
}

const char* ToString(TraversabilityStatus status) {
  switch (status) {
    case TraversabilityStatus::kUnknownNoRoot:
      return "UNKNOWN_NO_ROOT";
    case TraversabilityStatus::kObservedRootConnected:
      return "OBSERVED_ROOT_CONNECTED";
    case TraversabilityStatus::kObservedRootVetoed:
      return "OBSERVED_ROOT_VETOED";
  }
  return "INVALID";
}

}  // namespace terrain
}  // namespace lidar_fusion
}  // namespace apollo
