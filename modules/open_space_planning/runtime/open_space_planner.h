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
#include <memory>

#include "modules/open_space_planning/common/status.h"
#include "modules/open_space_planning/common/types.h"
#include "modules/open_space_planning/route/route_planner.h"
#include "modules/open_space_planning/safety/fallback_planner.h"
#include "modules/open_space_planning/safety/trajectory_validator.h"
#include "modules/open_space_planning/trajectory/trajectory_planner.h"

namespace apollo {
namespace open_space_planning {

struct OpenSpacePlannerConfig {
  std::size_t maximum_route_candidates = 3;
  RouteSearchParadigm default_search_paradigm = RouteSearchParadigm::kAuto;
};

class OpenSpacePlanner {
 public:
  OpenSpacePlanner(OpenSpacePlannerConfig config,
                   std::unique_ptr<RoutePlanner> route_planner,
                   std::unique_ptr<TrajectoryPlanner> trajectory_planner,
                   std::unique_ptr<TrajectoryValidator> trajectory_validator,
                   std::unique_ptr<FallbackPlanner> fallback_planner);

  Status Plan(const PlanningProblem& problem, PlanningResult* result);

 private:
  bool RevisionsMatch(const PlanningProblem& problem,
                      const RouteCandidate& route) const;
  bool RevisionsMatch(const PlanningProblem& problem,
                      const PhysicalTrajectory& trajectory) const;
  Status PlanFallback(const PlanningProblem& problem, PlanningResult* result);

  OpenSpacePlannerConfig config_;
  std::unique_ptr<RoutePlanner> route_planner_;
  std::unique_ptr<TrajectoryPlanner> trajectory_planner_;
  std::unique_ptr<TrajectoryValidator> trajectory_validator_;
  std::unique_ptr<FallbackPlanner> fallback_planner_;
};

}  // namespace open_space_planning
}  // namespace apollo
