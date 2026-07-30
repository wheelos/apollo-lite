#pragma once

#include <string>

#include "modules/common_msgs/planning_msgs/motion_execution.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"

namespace apollo {
namespace control {

class MotionCommandAdapter {
 public:
  bool ToLegacyControllerInput(
      const planning::MotionExecutionCommand& command,
      planning::ADCTrajectory* trajectory, std::string* reason) const;
};

}  // namespace control
}  // namespace apollo
