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

#include <optional>

#include "modules/common/status/status.h"
#include "modules/local_planning/trajectory_continuity/trajectory_validation.h"

namespace apollo {
namespace local_planning {

struct TrajectoryStitchPolicy {
  double position_tolerance = 0.0;
  double heading_tolerance = 0.0;
  double speed_tolerance = 0.0;
};

enum class TrajectoryContinuityState {
  NO_HISTORY,
  SOURCE_CHANGED,
  EXPIRED,
  OUTSIDE_HORIZON,
  REANCHORED,
  STITCHED
};

struct TrajectoryStart {
  TrajectoryPoint point;
  TrajectoryContinuityState continuity = TrajectoryContinuityState::NO_HISTORY;
};

// Evaluate a previously validated output at the current absolute ODOM time.
// The measured pose remains authoritative if tracking diverged; compatible
// acceleration and curvature still carry across the cycle boundary.
common::Status AlignTrajectoryStart(const CycleInput& input,
                                    const LocalTrajectory* previous,
                                    const TrajectoryStitchPolicy& policy,
                                    TrajectoryStart* start);

// Own only validated output history. Generators and input builders do not
// own each other's temporal state.
class TrajectoryContinuity {
 public:
  void Reset();
  common::Status Start(const CycleInput& input,
                       const TrajectoryStitchPolicy& policy,
                       TrajectoryStart* start) const;
  common::Status Accept(const CycleInput& input, const LaneFollowConfig& config,
                        const LocalTrajectory& trajectory);

 private:
  std::optional<LocalTrajectory> previous_;
};

}  // namespace local_planning
}  // namespace apollo
