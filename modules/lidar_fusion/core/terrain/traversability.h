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

#include <cstdint>
#include <string>
#include <vector>

#include "modules/lidar_fusion/core/terrain/surface.h"

namespace apollo {
namespace lidar_fusion {
namespace terrain {

constexpr uint32_t kMeasuredRisk = 1U;
constexpr uint32_t kObjectRisk = 2U;
constexpr uint32_t kUnknownRisk = 4U;
constexpr uint32_t kKnownRiskBits = kMeasuredRisk | kObjectRisk | kUnknownRisk;

enum Reason : uint32_t {
  kReasonUnsupported = 1U << 0,
  kReasonSlopeUnobserved = 1U << 1,
  kReasonRoughnessUnobserved = 1U << 2,
  kReasonStepUnknown = 1U << 3,
  kReasonPitUnknown = 1U << 4,
  kReasonLayerAmbiguity = 1U << 5,
  kReasonSlopeExceeds = 1U << 6,
  kReasonRoughnessExceeds = 1U << 7,
  kReasonStepExceeds = 1U << 8,
  kReasonPitExceeds = 1U << 9,
  kReasonMeasuredRisk = 1U << 10,
  kReasonObjectRisk = 1U << 11,
  kReasonUnknownRisk = 1U << 12,
  kReasonRiskUnobserved = 1U << 13,
  kReasonDisconnected = 1U << 14,
};

struct TraversabilityLimits {
  double max_slope_gradient = 0.0;
  double max_roughness_m = 0.0;
  double max_step_m = 0.0;
  double max_depression_depth_m = 0.0;
};

enum class TerrainState {
  kUnknown,
  kGroundSupported,
  kTerrainPassable,
  kBlocked,
};

enum class DecisionState { kUnknown, kCandidate, kBlocked };

enum class TraversabilityStatus {
  kUnknownNoRoot,
  kObservedRootConnected,
  kObservedRootVetoed,
};

struct RiskGrid {
  std::size_t nx = 0;
  std::size_t ny = 0;
  std::vector<uint32_t> layers;
  std::vector<uint8_t> observed;
};

struct TraversabilityCell {
  TerrainState terrain_state = TerrainState::kUnknown;
  DecisionState decision_state = DecisionState::kUnknown;
  uint32_t reason_bits = 0;
  bool terrain_candidate = false;
  bool risk_constrained_candidate = false;
  bool terrain_connected = false;
  bool risk_constrained_connected = false;
  uint32_t risk_layers = 0;
  bool risk_observed = false;
};

struct TraversabilityEstimate {
  std::vector<TraversabilityCell> cells;
  TraversabilityStatus overall_status = TraversabilityStatus::kUnknownNoRoot;
  std::optional<CellIndex> root_cell;
};

bool ClassifyTraversability(const TerrainEstimate& surface,
                            const TraversabilityLimits& limits,
                            const RiskGrid& risk,
                            TraversabilityEstimate* result, std::string* error);

const char* ToString(TerrainState state);
const char* ToString(DecisionState state);
const char* ToString(TraversabilityStatus status);

}  // namespace terrain
}  // namespace lidar_fusion
}  // namespace apollo
