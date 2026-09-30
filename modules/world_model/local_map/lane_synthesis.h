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

#include "modules/common/status/status.h"
#include "modules/world_model/local_map/lane_types.h"
#include "modules/world_model/local_map/local_scene.h"

namespace apollo {
namespace world_model {

// Materializes the paired temporal track as typed lane and boundary entities.
common::Status SynthesizeObservedLane(const SceneSource& source,
                                      const TemporalLaneSnapshot& snapshot,
                                      const TemporalLanePolicy& policy,
                                      uint64_t left_id, uint64_t right_id,
                                      LaneLayer* layer);

}  // namespace world_model
}  // namespace apollo
