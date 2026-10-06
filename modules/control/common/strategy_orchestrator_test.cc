#include "modules/control/common/strategy_orchestrator.h"

#include "gtest/gtest.h"

namespace apollo {
namespace control {

namespace {

void SetTrajectoryIntent(planning::MotionExecutionCommand* command,
                        planning::LongitudinalIntent longitudinal) {
  auto* intent = command->mutable_control_intent();
  intent->set_tracking_mode(planning::TRACKING_MODE_TRAJECTORY);
  intent->set_execution_channel(planning::EXECUTION_CHANNEL_TRAJECTORY);
  intent->set_longitudinal_intent(longitudinal);
  intent->set_lateral_intent(planning::LAT_INTENT_TRACK_PATH);
  intent->set_primitive_type(planning::CONTROL_PRIMITIVE_NONE);
}

void SetPrimitiveIntent(planning::MotionExecutionCommand* command,
                        planning::ControlPrimitiveType primitive_type) {
  auto* intent = command->mutable_control_intent();
  intent->set_tracking_mode(planning::TRACKING_MODE_POSE_SERVO);
  intent->set_execution_channel(planning::EXECUTION_CHANNEL_PRIMITIVE);
  intent->set_longitudinal_intent(planning::LON_INTENT_CRUISE);
  intent->set_lateral_intent(planning::LAT_INTENT_ALIGN_GOAL_HEADING);
  intent->set_primitive_type(primitive_type);
}

}  // namespace

TEST(StrategyOrchestratorTest, ControlledStopPreservesAuthorizedPathIntent) {
  planning::ADCTrajectory trajectory;
  trajectory.add_trajectory_point();
  auto* intent = trajectory.mutable_control_intent();
  intent->set_tracking_mode(planning::TRACKING_MODE_TRAJECTORY);
  intent->set_execution_channel(planning::EXECUTION_CHANNEL_TRAJECTORY);
  intent->set_primitive_type(planning::CONTROL_PRIMITIVE_NONE);
  intent->set_longitudinal_intent(planning::LON_INTENT_MRM_STOP);
  intent->set_lateral_intent(planning::LAT_INTENT_TRACK_PATH);
  intent->set_require_full_stop(true);
  StrategyOrchestrator strategy;
  strategy.Apply(strategy.Resolve(BuildControlCommandGoal(trajectory)), &trajectory);
  EXPECT_EQ(intent->tracking_mode(), planning::TRACKING_MODE_TRAJECTORY);
  EXPECT_EQ(intent->execution_channel(), planning::EXECUTION_CHANNEL_TRAJECTORY);
  EXPECT_EQ(intent->primitive_type(), planning::CONTROL_PRIMITIVE_NONE);
  EXPECT_EQ(intent->longitudinal_intent(), planning::LON_INTENT_MRM_STOP);
  EXPECT_EQ(intent->lateral_intent(), planning::LAT_INTENT_TRACK_PATH);
  EXPECT_TRUE(intent->require_full_stop());
}

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

TEST(StrategyOrchestratorTest, AuthorizedTrajectoryTargetDoesNotImplyPoseServo) {
  planning::MotionExecutionCommand command;
  command.mutable_trajectory()->set_gear(canbus::Chassis::GEAR_DRIVE);
  command.mutable_trajectory()->add_point();
  command.mutable_trajectory()->add_point();
  SetTrajectoryIntent(&command, planning::LON_INTENT_CRUISE);
  auto* intent = command.mutable_control_intent();
  intent->mutable_target_stop_point()->set_x(1.0);
  const auto before = command.SerializeAsString();
  const auto profile = StrategyOrchestrator().Resolve(command);
  EXPECT_TRUE(profile.supported);
  EXPECT_EQ(profile.parameter_profile, CONTROL_PROFILE_ROAD_TRACKING);
  EXPECT_EQ(profile.profile_key, "road-tracking");
  EXPECT_EQ(command.SerializeAsString(), before);
}

TEST(StrategyOrchestratorTest, AuthorizedPreciseStopSelectsLocalPrecisionTuning) {
  planning::MotionExecutionCommand command;
  command.mutable_trajectory()->set_gear(canbus::Chassis::GEAR_DRIVE);
  command.mutable_trajectory()->add_point();
  command.mutable_trajectory()->add_point();
  SetTrajectoryIntent(&command, planning::LON_INTENT_PRECISE_STOP);
  const auto profile = StrategyOrchestrator().Resolve(command);
  EXPECT_EQ(profile.parameter_profile, CONTROL_PROFILE_LOW_SPEED_PRECISION);
  SetTrajectoryIntent(&command, planning::LON_INTENT_MRM_STOP);
  EXPECT_EQ(StrategyOrchestrator().Resolve(command).parameter_profile,
            CONTROL_PROFILE_ROAD_TRACKING);
  EXPECT_EQ(StrategyOrchestrator().Resolve(command).profile_key,
            "controlled-stop");
}

TEST(StrategyOrchestratorTest, AuthorizedPayloadSelectsProfileNotNewCapability) {
  planning::MotionExecutionCommand command;
  command.mutable_primitive()->set_type(planning::MOTION_PRIMITIVE_POSE_SERVO);
  SetPrimitiveIntent(&command, planning::CONTROL_PRIMITIVE_POSE_SERVO);
  EXPECT_EQ(StrategyOrchestrator().Resolve(command).parameter_profile,
            CONTROL_PROFILE_LOW_SPEED_PRECISION);
  command.mutable_primitive()->set_type(
      planning::MOTION_PRIMITIVE_CORRIDOR_SERVO);
  auto* intent = command.mutable_control_intent();
  intent->set_tracking_mode(planning::TRACKING_MODE_PATH_SPEED);
  intent->set_lateral_intent(planning::LAT_INTENT_TRACK_PATH);
  intent->set_primitive_type(planning::CONTROL_PRIMITIVE_NONE);
  EXPECT_EQ(StrategyOrchestrator().Resolve(command).parameter_profile,
            CONTROL_PROFILE_LOW_SPEED_PRECISION);
  EXPECT_EQ(StrategyOrchestrator().Resolve(command).profile_key,
            "spatial-primitive");
  command.mutable_primitive()->set_type(
      planning::MOTION_PRIMITIVE_STANDSTILL_HOLD);
  auto* hold_intent = command.mutable_control_intent();
  hold_intent->set_tracking_mode(planning::TRACKING_MODE_STANDSTILL_HOLD);
  hold_intent->set_longitudinal_intent(planning::LON_INTENT_HOLD_STOP);
  hold_intent->set_primitive_type(
      planning::CONTROL_PRIMITIVE_STANDSTILL_HOLD);
  EXPECT_EQ(StrategyOrchestrator().Resolve(command).parameter_profile,
            CONTROL_PROFILE_STANDSTILL_HOLD);
  command.mutable_primitive()->set_type(planning::MOTION_PRIMITIVE_ROTATE_IN_PLACE);
  hold_intent->set_tracking_mode(planning::TRACKING_MODE_PATH_SPEED);
  hold_intent->set_longitudinal_intent(planning::LON_INTENT_CRUISE);
  hold_intent->set_lateral_intent(planning::LAT_INTENT_TRACK_PATH);
  hold_intent->set_primitive_type(planning::CONTROL_PRIMITIVE_NONE);
  EXPECT_FALSE(StrategyOrchestrator().Resolve(command).supported);
}

TEST(StrategyOrchestratorTest, RejectsSemanticChannelAndHoldMismatches) {
  planning::MotionExecutionCommand command;
  command.mutable_trajectory()->set_gear(canbus::Chassis::GEAR_DRIVE);
  SetTrajectoryIntent(&command, planning::LON_INTENT_CRUISE);
  command.mutable_control_intent()->set_execution_channel(
      planning::EXECUTION_CHANNEL_PRIMITIVE);
  EXPECT_FALSE(StrategyOrchestrator().Resolve(command).supported);

  command.mutable_control_intent()->set_tracking_mode(
      planning::TRACKING_MODE_STANDSTILL_HOLD);
  command.mutable_control_intent()->set_longitudinal_intent(
      planning::LON_INTENT_HOLD_STOP);
  command.mutable_control_intent()->set_execution_channel(
      planning::EXECUTION_CHANNEL_TRAJECTORY);
  command.mutable_control_intent()->set_primitive_type(
      planning::CONTROL_PRIMITIVE_STANDSTILL_HOLD);
  const auto profile = StrategyOrchestrator().Resolve(command);
  EXPECT_FALSE(profile.supported);
}

TEST(StrategyOrchestratorTest, HeadingHoldSelectsPrecisionBehavior) {
  planning::MotionExecutionCommand command;
  command.mutable_trajectory()->set_gear(canbus::Chassis::GEAR_DRIVE);
  SetTrajectoryIntent(&command, planning::LON_INTENT_PRECISE_STOP);
  auto* intent = command.mutable_control_intent();
  intent->set_execution_channel(planning::EXECUTION_CHANNEL_PRIMITIVE);
  intent->set_primitive_type(planning::CONTROL_PRIMITIVE_HEADING_HOLD);
  intent->set_lateral_intent(planning::LAT_INTENT_ALIGN_GOAL_HEADING);

  const auto profile = StrategyOrchestrator().Resolve(command);
  EXPECT_TRUE(profile.supported);
  EXPECT_EQ(profile.profile_key, "heading-hold");
  EXPECT_EQ(profile.parameter_profile, CONTROL_PROFILE_LOW_SPEED_PRECISION);
  EXPECT_TRUE(profile.suppress_large_steer);
}

}  // namespace control
}  // namespace apollo
