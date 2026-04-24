#pragma once

#include <string>

#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/planning/common/local_view.h"
#include "modules/planning/environment/capability_extractor.h"
#include "modules/planning/planning_runtime_context.h"

namespace apollo {
namespace planning {

struct ValidationInput {
  const LocalView* local_view = nullptr;
  const PlanningCoordinatorState* planning_state = nullptr;
  const ADCTrajectory* trajectory = nullptr;
};

struct ValidationResult {
  bool command_admissible = true;
  bool trajectory_valid = true;
  bool should_publish = true;
  bool should_hold = false;
  bool fallback_active = false;
  std::string reason;
};

class ValidationSupervisor {
 public:
  ValidationResult Validate(const ValidationInput& input) const;
};

}  // namespace planning
}  // namespace apollo
