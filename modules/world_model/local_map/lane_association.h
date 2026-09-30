// Copyright 2026 WheelOS All Rights Reserved.
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
#include <map>

#include "modules/common/status/status.h"
#include "modules/world_model/local_map/lane_types.h"

namespace apollo {
namespace world_model {

struct LaneAssociationResult {
  std::map<int, TrackedLaneSample> samples;
  size_t overlap_samples = 0;
};

// Associates candidate samples on a fixed longitudinal grid with fresh
// history. Restrictive inward changes always take effect immediately.
common::Status AssociateLaneSamples(
    const std::map<int, TrackedLaneSample>& history,
    const std::map<int, TrackedLaneSample>& candidates,
    const TemporalLanePolicy& policy, double now,
    LaneAssociationResult* result);

}  // namespace world_model
}  // namespace apollo
