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
#include "modules/local_planning/trajectory_continuity/trajectory.h"

namespace apollo {
namespace local_planning {

common::Status CheckLaneFootprint(const CycleInput& input,
                                  const LaneFollowConfig& config,
                                  const TrajectoryPoint& point);
// Does not trust the generator or trajectory.valid. No mutable planner state.
common::Status ValidateTrajectory(const CycleInput& input,
                                  const LaneFollowConfig& config,
                                  const LocalTrajectory& trajectory);

}  // namespace local_planning
}  // namespace apollo
