#pragma once

#include "modules/open_space_planning/common/status.h"
#include "modules/open_space_planning/common/types.h"

namespace apollo {
namespace open_space_planning {

struct TrajectoryPlanningRequest {
  const PlanningProblem& problem;
  const RouteCandidate& route;
};

class TrajectoryPlanner {
 public:
  virtual ~TrajectoryPlanner() = default;

  virtual Status Plan(const TrajectoryPlanningRequest& request,
                      PhysicalTrajectory* trajectory) = 0;
};

}  // namespace open_space_planning
}  // namespace apollo

