#include "modules/control/common/executor_arbiter.h"
#include "modules/control/common/motion_execution_manager.h"
#include "modules/planning/common/motion_plan_builder.h"

#include "gtest/gtest.h"

namespace apollo {
namespace control {
namespace {

planning::PlanningCoordinatorState MissionState(
    planning::MissionSessionState state) {
  planning::PlanningCoordinatorState output;
  output.mission_session_state = state;
  output.mission_identity.set_producer_epoch("mission-boot");
  output.mission_identity.set_aggregate_id("mission");
  output.mission_identity.set_command_id("a-to-b");
  output.mission_identity.set_revision(1);
  return output;
}

localization::LocalizationEstimate Localization(double now_sec) {
  localization::LocalizationEstimate localization;
  localization.mutable_header()->set_timestamp_sec(now_sec);
  localization.mutable_header()->set_frame_id("map");
  localization.set_measurement_time(now_sec);
  localization.mutable_pose()->mutable_position()->set_x(0.0);
  localization.mutable_pose()->mutable_position()->set_y(0.0);
  localization.mutable_pose()->mutable_position()->set_z(0.0);
  localization.mutable_pose()->set_heading(0.0);
  return localization;
}

canbus::Chassis Chassis() {
  canbus::Chassis chassis;
  chassis.set_speed_mps(0.0);
  chassis.set_gear_location(canbus::Chassis::GEAR_DRIVE);
  return chassis;
}

planning::ADCTrajectory Trajectory() {
  planning::ADCTrajectory trajectory;
  trajectory.set_gear(canbus::Chassis::GEAR_DRIVE);
  trajectory.set_total_path_time(1.0);
  for (int i = 0; i < 2; ++i) {
    auto* point = trajectory.add_trajectory_point();
    point->mutable_path_point()->set_x(i);
    point->mutable_path_point()->set_y(0.0);
    point->mutable_path_point()->set_z(0.0);
    point->mutable_path_point()->set_theta(0.0);
    point->mutable_path_point()->set_s(i);
    point->mutable_path_point()->set_kappa(0.0);
    point->mutable_path_point()->set_dkappa(0.0);
    point->set_v(0.0);
    point->set_a(0.0);
    point->set_relative_time(i);
  }
  return trajectory;
}

MotionExecutionVehicleState VehicleState(double now_sec) {
  MotionExecutionVehicleState state;
  state.position.set_x(0.0);
  state.position.set_y(0.0);
  state.position.set_z(0.0);
  state.heading = 0.0;
  state.gear = canbus::Chassis::GEAR_DRIVE;
  state.speed_mps = 0.0;
  state.timestamp_sec = now_sec;
  state.reference_frame_id = "map";
  return state;
}

TEST(MotionChainContractTest, CancelsMissionMotionThenOwnsIdleHold) {
  MotionExecutionCapabilities capabilities;
  capabilities.supported = {
      planning::MOTION_CAPABILITY_TRAJECTORY_TRACKING,
      planning::MOTION_CAPABILITY_STANDSTILL_HOLD,
  };
  MotionExecutionManager manager{
      MotionExecutionValidator(std::move(capabilities))};
  planning::MotionPlanBuilder builder("planning-boot");
  planning::PlanningSemanticSummary semantics;
  ExecutorArbiter arbiter;

  auto execute = builder.Build(
      MissionState(planning::MISSION_SESSION_EXECUTING), semantics,
      Chassis(), Localization(10.0), Trajectory(), 10.0);
  ASSERT_TRUE(execute.has_directive);
  ASSERT_EQ(manager.Apply(execute.directive, 10.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  ASSERT_TRUE(arbiter.Acquire(*manager.active_command()));
  ASSERT_EQ(manager.Arm(10.01).state(),
            planning::MOTION_EXECUTION_ARMED);
  auto running = manager.Start(VehicleState(10.02), 10.02);
  ASSERT_EQ(running.state(),
            planning::MOTION_EXECUTION_EXECUTING_TRAJECTORY);
  builder.ObserveControlStatus(
      running, planning::MOTION_SCOPE_MISSION_DESCENDANT);

  auto cancel = builder.Build(
      MissionState(planning::MISSION_SESSION_CANCELLING), semantics,
      Chassis(), Localization(10.1), Trajectory(), 10.1);
  ASSERT_TRUE(cancel.has_directive);
  ASSERT_EQ(manager.Apply(cancel.directive, 10.1).state(),
            planning::MOTION_EXECUTION_CANCELLING);
  arbiter.Release();
  auto cancelled =
      manager.ConfirmExecutorRevoked(10.11, "executor revoked");
  ASSERT_EQ(cancelled.state(),
            planning::MOTION_EXECUTION_CANCELLED);
  builder.ObserveControlStatus(
      cancelled, planning::MOTION_SCOPE_MISSION_DESCENDANT);

  auto hold = builder.Build(
      MissionState(planning::MISSION_SESSION_CANCELLING), semantics,
      Chassis(), Localization(10.2), planning::ADCTrajectory(), 10.2);
  ASSERT_TRUE(hold.has_directive);
  ASSERT_EQ(hold.directive.scope(),
            planning::MOTION_SCOPE_PLANNING_IDLE_HOLD);
  ASSERT_EQ(manager.Apply(hold.directive, 10.2).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  ASSERT_TRUE(arbiter.Acquire(*manager.active_command()));
  ASSERT_EQ(arbiter.owner(), NormalExecutorOwner::kPrimitive);
  ASSERT_EQ(manager.Arm(10.21).state(),
            planning::MOTION_EXECUTION_ARMED);
  EXPECT_EQ(manager.Start(VehicleState(10.22), 10.22).state(),
            planning::MOTION_EXECUTION_EXECUTING_PRIMITIVE);
}

}  // namespace
}  // namespace control
}  // namespace apollo
