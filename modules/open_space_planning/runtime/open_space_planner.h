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

