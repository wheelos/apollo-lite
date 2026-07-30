#pragma once

#include "modules/common_msgs/planning_msgs/motion_execution.pb.h"

namespace apollo {
namespace control {

enum class NormalExecutorOwner {
  kNone = 0,
  kTrajectory,
  kPrimitive,
};

class ExecutorArbiter {
 public:
  bool Acquire(const planning::MotionExecutionCommand& command);
  void Release();

  NormalExecutorOwner owner() const { return owner_; }
  bool has_owner() const { return owner_ != NormalExecutorOwner::kNone; }

 private:
  NormalExecutorOwner owner_ = NormalExecutorOwner::kNone;
};

}  // namespace control
}  // namespace apollo
