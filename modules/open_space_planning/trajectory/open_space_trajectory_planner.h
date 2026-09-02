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

#include <vector>

#include "modules/common/math/vec2d.h"
#include "modules/open_space_planning/common/status.h"
#include "modules/open_space_planning/common/types.h"
#include "modules/open_space_planning/trajectory/path_optimizer/fem_pos_deviation_osqp.h"
#include "modules/open_space_planning/trajectory/speed_optimizer/piecewise_jerk_speed_osqp.h"
#include "modules/open_space_planning/trajectory/trajectory_planner.h"
#include "modules/planning/proto/planner_open_space_config.pb.h"

namespace apollo {
namespace open_space_planning {

class OpenSpaceTrajectoryPlanner : public TrajectoryPlanner {
 public:
  OpenSpaceTrajectoryPlanner();
  explicit OpenSpaceTrajectoryPlanner(
      const planning::PlannerOpenSpaceConfig& config);
  virtual ~OpenSpaceTrajectoryPlanner() = default;

  Status Plan(const TrajectoryPlanningRequest& request,
              PhysicalTrajectory* trajectory) override;

 private:
  bool PlanQpPathAndSpeed(
      const PlanningProblem& problem, const RouteCandidate& route,
      PhysicalTrajectory* trajectory);

  bool PlanKinematicFallback(
      const PlanningProblem& problem, const RouteCandidate& route,
      PhysicalTrajectory* trajectory);

  planning::PlannerOpenSpaceConfig config_;
  FemPosDeviationOsqp path_optimizer_;
  PiecewiseJerkSpeedOsqp speed_optimizer_;
};

}  // namespace open_space_planning
}  // namespace apollo


