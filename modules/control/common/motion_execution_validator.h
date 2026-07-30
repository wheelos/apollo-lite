#pragma once

#include <set>
#include <string>

#include "modules/common_msgs/planning_msgs/motion_execution.pb.h"

namespace apollo {
namespace control {

struct MotionExecutionCapabilities {
  std::set<planning::MotionCapability> supported;

  bool Supports(planning::MotionCapability capability) const {
    return supported.count(capability) > 0;
  }
};

struct MotionValidationResult {
  bool accepted = false;
  planning::MotionCommandRejectReason reject_reason =
      planning::MOTION_REJECT_NONE;
  planning::MotionExecutionType execution_type =
      planning::MOTION_EXECUTION_TYPE_UNKNOWN;
  std::string reason;
};

class MotionExecutionValidator {
 public:
  explicit MotionExecutionValidator(MotionExecutionCapabilities capabilities);

  MotionValidationResult Validate(
      const planning::MotionExecutionCommand& command,
      double now_sec) const;

 private:
  MotionValidationResult Reject(
      planning::MotionCommandRejectReason reject_reason,
      const std::string& reason) const;
  MotionValidationResult ValidateConstraints(
      const planning::MotionExecutionCommand& command) const;
  MotionValidationResult ValidateStartCondition(
      const planning::MotionExecutionCommand& command) const;
  MotionValidationResult ValidateTrajectory(
      const planning::MotionExecutionCommand& command) const;
  MotionValidationResult ValidatePrimitive(
      const planning::MotionExecutionCommand& command) const;

  MotionExecutionCapabilities capabilities_;
};

}  // namespace control
}  // namespace apollo
