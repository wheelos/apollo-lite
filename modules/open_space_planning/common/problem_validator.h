#pragma once

#include "modules/open_space_planning/common/status.h"
#include "modules/open_space_planning/common/types.h"

namespace apollo {
namespace open_space_planning {

class ProblemValidator {
 public:
  static Status Validate(const PlanningProblem& problem);
};

}  // namespace open_space_planning
}  // namespace apollo

