#pragma once

#include "modules/open_space_planning/common/status.h"
#include "modules/open_space_planning/common/types.h"

namespace apollo {
namespace open_space_planning {

struct FallbackPlanningRequest {
  const PlanningProblem& problem;
};

class FallbackPlanner {
 public:
  virtual ~FallbackPlanner() = default;

  virtual Status Plan(const FallbackPlanningRequest& request,
                      PhysicalTrajectory* trajectory) = 0;
};

}  // namespace open_space_planning
}  // namespace apollo

