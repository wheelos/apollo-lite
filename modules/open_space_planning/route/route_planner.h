#pragma once

#include <vector>

#include "modules/open_space_planning/common/status.h"
#include "modules/open_space_planning/common/types.h"

namespace apollo {
namespace open_space_planning {

struct RoutePlanningRequest {
  const PlanningProblem& problem;
  std::size_t maximum_candidate_count = 1;
};

class RoutePlanner {
 public:
  virtual ~RoutePlanner() = default;

  // Successful results are ordered from lowest to highest route cost.
  virtual Status Plan(const RoutePlanningRequest& request,
                      std::vector<RouteCandidate>* candidates) = 0;
};

}  // namespace open_space_planning
}  // namespace apollo
