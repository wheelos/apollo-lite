#include "modules/control/common/executor_arbiter.h"

namespace apollo {
namespace control {

bool ExecutorArbiter::Acquire(
    const planning::MotionExecutionCommand& command) {
  switch (command.payload_case()) {
    case planning::MotionExecutionCommand::kTrajectory:
      owner_ = NormalExecutorOwner::kTrajectory;
      return true;
    case planning::MotionExecutionCommand::kPrimitive:
      owner_ = NormalExecutorOwner::kPrimitive;
      return true;
    case planning::MotionExecutionCommand::PAYLOAD_NOT_SET:
    default:
      owner_ = NormalExecutorOwner::kNone;
      return false;
  }
}

void ExecutorArbiter::Release() {
  owner_ = NormalExecutorOwner::kNone;
}

}  // namespace control
}  // namespace apollo
