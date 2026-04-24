#pragma once

#include <string>
#include <vector>

#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "modules/planning/environment/capability_extractor.h"

namespace apollo {
namespace planning {

struct ModeShellAvailability {
  bool lane_graph_available = false;
  bool corridor_available = false;
  bool open_space_available = false;
  bool free_space_available = false;
  bool safety_hold_available = false;
};

struct ModeResolutionResult {
  PlanningMode requested_mode = MODE_UNKNOWN;
  PlanningMode resolved_mode = MODE_UNKNOWN;
  std::string reason;
  std::vector<std::string> blockers;
};

class ModeResolution {
 public:
  static PlanningMode InferRequestedMode(const PlanningCommand* command,
                                         PlanningMode legacy_mode);

  static ModeResolutionResult Resolve(const PlanningCommand* command,
                                      const CapabilitySet* capability,
                                      const ModeShellAvailability& availability,
                                      PlanningMode legacy_mode);
};

}  // namespace planning
}  // namespace apollo
