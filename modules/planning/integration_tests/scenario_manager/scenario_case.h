#pragma once

#include <string>
#include <vector>

#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {
namespace scenario {

struct EnvInput {
  double timestamp = 0.0;
  double speed = 0.0;
  std::string pad_message = "NONE";
  bool internal_emergency = false;
  double history_stuck_time = 0.0;
  bool has_routing = false;

  std::string current_scenario = "LANE_FOLLOW";
  std::string current_status = "STATUS_PROCESSING";
};

struct Expected {
  std::string scenario;
  std::string grade;
};

struct ScenarioCase {
  std::string id;
  std::string description;
  EnvInput input;
  Expected expected;
};

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
