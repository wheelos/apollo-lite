#include "modules/control/common/executor_arbiter.h"

#include "gtest/gtest.h"

namespace apollo {
namespace control {

TEST(ExecutorArbiterTest, SelectsOnlyFromPayloadOneof) {
  ExecutorArbiter arbiter;
  planning::MotionExecutionCommand trajectory;
  trajectory.mutable_trajectory();
  EXPECT_TRUE(arbiter.Acquire(trajectory));
  EXPECT_EQ(arbiter.owner(), NormalExecutorOwner::kTrajectory);

  planning::MotionExecutionCommand primitive;
  primitive.mutable_primitive();
  EXPECT_TRUE(arbiter.Acquire(primitive));
  EXPECT_EQ(arbiter.owner(), NormalExecutorOwner::kPrimitive);

  planning::MotionExecutionCommand empty;
  EXPECT_FALSE(arbiter.Acquire(empty));
  EXPECT_EQ(arbiter.owner(), NormalExecutorOwner::kNone);
}

}  // namespace control
}  // namespace apollo
