#include "modules/control/common/strategy_orchestrator.h"

#include "gtest/gtest.h"

namespace apollo {
namespace control {

TEST(StrategyOrchestratorTest, LegacyTrajectoryFallsBackToTracking) {
  planning::ADCTrajectory trajectory;
  trajectory.add_trajectory_point();

  const auto goal = BuildControlCommandGoal(trajectory);
  StrategyOrchestrator orchestrator;
  const auto profile = orchestrator.Resolve(goal);

  EXPECT_EQ(profile.profile_key, "default-tracking");
  EXPECT_FALSE(profile.enforce_hold_stop);
  orchestrator.Apply(profile, &trajectory);
  EXPECT_EQ(trajectory.control_intent().tracking_mode(),
            planning::TRACKING_MODE_TRAJECTORY);
  EXPECT_EQ(trajectory.control_intent().primitive_type(),
            planning::CONTROL_PRIMITIVE_NONE);
  EXPECT_EQ(trajectory.control_intent().execution_channel(),
            planning::EXECUTION_CHANNEL_TRAJECTORY);
}

TEST(StrategyOrchestratorTest, PoseServoIntentSelectsPoseServoProfile) {
  planning::ADCTrajectory trajectory;
  auto* intent = trajectory.mutable_control_intent();
  intent->set_tracking_mode(planning::TRACKING_MODE_POSE_SERVO);
  intent->mutable_target_stop_point()->set_x(1.0);
  intent->mutable_target_stop_point()->set_y(2.0);

  const auto goal = BuildControlCommandGoal(trajectory);
  StrategyOrchestrator orchestrator;
  const auto profile = orchestrator.Resolve(goal);

  EXPECT_EQ(profile.profile_key, "pose-servo");
  EXPECT_TRUE(profile.prefer_pose_servo);
  orchestrator.Apply(profile, &trajectory);
  EXPECT_TRUE(trajectory.control_intent().suppress_large_steer());
  EXPECT_EQ(trajectory.control_intent().tracking_mode(),
            planning::TRACKING_MODE_POSE_SERVO);
  EXPECT_EQ(trajectory.control_intent().execution_channel(),
            planning::EXECUTION_CHANNEL_PRIMITIVE);
}

TEST(StrategyOrchestratorTest, HoldSceneForcesStandstillHold) {
  planning::ADCTrajectory trajectory;
  trajectory.mutable_execution()->set_active_scene(planning::SCENE_HOLD);

  const auto goal = BuildControlCommandGoal(trajectory);
  StrategyOrchestrator orchestrator;
  const auto profile = orchestrator.Resolve(goal);
  orchestrator.Apply(profile, &trajectory);

  EXPECT_EQ(profile.profile_key, "standstill-hold");
  EXPECT_EQ(trajectory.control_intent().tracking_mode(),
            planning::TRACKING_MODE_STANDSTILL_HOLD);
  EXPECT_EQ(trajectory.control_intent().longitudinal_intent(),
            planning::LON_INTENT_HOLD_STOP);
  EXPECT_EQ(trajectory.control_intent().primitive_type(),
            planning::CONTROL_PRIMITIVE_STANDSTILL_HOLD);
  EXPECT_EQ(trajectory.control_intent().execution_channel(),
            planning::EXECUTION_CHANNEL_PRIMITIVE);
}

}  // namespace control
}  // namespace apollo
