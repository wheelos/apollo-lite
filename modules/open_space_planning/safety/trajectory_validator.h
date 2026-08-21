#pragma once

#include "modules/open_space_planning/common/types.h"

namespace apollo {
namespace open_space_planning {

struct TrajectoryValidationRequest {
  const PlanningProblem& problem;
  const PhysicalTrajectory& trajectory;
};

class TrajectoryValidator {
 public:
  virtual ~TrajectoryValidator() = default;

  virtual ValidationReport Validate(
      const TrajectoryValidationRequest& request) const = 0;
};

}  // namespace open_space_planning
}  // namespace apollo

