#include "modules/planning/common/motion_plan_builder.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {
namespace {

localization::LocalizationEstimate Localization(double time = 10.0) {
  localization::LocalizationEstimate localization;
  localization.mutable_header()->set_timestamp_sec(time);
  localization.mutable_header()->set_frame_id("map");
  localization.set_measurement_time(time);
  localization.mutable_pose()->mutable_position()->set_x(0.0);
  localization.mutable_pose()->mutable_position()->set_y(0.0);
  localization.mutable_pose()->set_heading(0.0);
  return localization;
}

canbus::Chassis Chassis(double speed = 0.0) {
  canbus::Chassis chassis;
  chassis.set_speed_mps(speed);
  chassis.set_gear_location(canbus::Chassis::GEAR_DRIVE);
  return chassis;
}

PlanningCoordinatorState State(MissionSessionState state) {
  PlanningCoordinatorState output;
  output.mission_session_state = state;
  output.mission_identity.set_producer_epoch("mission-boot");
  output.mission_identity.set_aggregate_id("mission");
  output.mission_identity.set_command_id("a-to-b");
  output.mission_identity.set_revision(1);
  return output;
}

ADCTrajectory Trajectory() {
  ADCTrajectory trajectory;
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
    point->set_v(i == 0 ? 0.1 : 0.0);
    point->set_a(0.0);
    point->set_relative_time(i);
  }
  return trajectory;
}

TEST(MotionPlanBuilderTest, WaitsForAckAndUsesExactReplacement) {
  MotionPlanBuilder builder("planning-boot");
  PlanningSemanticSummary semantics;
  auto first = builder.Build(State(MISSION_SESSION_EXECUTING), semantics,
                             Chassis(), Localization(), Trajectory(), 10.0);
  ASSERT_TRUE(first.has_directive);
  ASSERT_TRUE(first.directive.has_execute());
  EXPECT_EQ(first.directive.scope(), MOTION_SCOPE_MISSION_DESCENDANT);

  auto waiting = builder.Build(State(MISSION_SESSION_EXECUTING), semantics,
                               Chassis(), Localization(10.1), Trajectory(),
                               10.1);
  EXPECT_FALSE(waiting.has_directive);

  MotionExecutionStatus accepted;
  accepted.mutable_identity()->CopyFrom(
      first.directive.execute().command().identity());
  accepted.mutable_parent_mission_identity()->CopyFrom(
      first.directive.parent_mission_identity());
  accepted.set_state(MOTION_EXECUTION_EXECUTING_TRAJECTORY);
  builder.ObserveControlStatus(accepted,
                               MOTION_SCOPE_MISSION_DESCENDANT);

  auto replacement = builder.Build(
      State(MISSION_SESSION_EXECUTING), semantics, Chassis(),
      Localization(10.1), Trajectory(), 10.1);
  ASSERT_TRUE(replacement.has_directive);
  ASSERT_TRUE(replacement.directive.has_replace());
  EXPECT_EQ(
      replacement.directive.replace().expected_active_identity().revision(),
      first.directive.execute().command().identity().revision());
}

TEST(MotionPlanBuilderTest, CancelsThenTransfersToPlanningIdleHold) {
  MotionPlanBuilder builder("planning-boot");
  PlanningSemanticSummary semantics;
  auto execute = builder.Build(State(MISSION_SESSION_EXECUTING), semantics,
                               Chassis(), Localization(), Trajectory(), 10.0);
  MotionExecutionStatus accepted;
  accepted.mutable_identity()->CopyFrom(
      execute.directive.execute().command().identity());
  accepted.mutable_parent_mission_identity()->CopyFrom(
      execute.directive.parent_mission_identity());
  accepted.set_state(MOTION_EXECUTION_EXECUTING_TRAJECTORY);
  builder.ObserveControlStatus(accepted,
                               MOTION_SCOPE_MISSION_DESCENDANT);

  auto cancel = builder.Build(State(MISSION_SESSION_CANCELLING), semantics,
                              Chassis(), Localization(10.1), Trajectory(),
                              10.1);
  ASSERT_TRUE(cancel.has_directive);
  ASSERT_TRUE(cancel.directive.has_cancel());
  EXPECT_TRUE(cancel.directive.cancel().fence_parent_mission());

  MotionExecutionStatus cancelled = accepted;
  cancelled.set_state(MOTION_EXECUTION_CANCELLED);
  builder.ObserveControlStatus(cancelled,
                               MOTION_SCOPE_MISSION_DESCENDANT);

  auto hold = builder.Build(State(MISSION_SESSION_CANCELLING), semantics,
                            Chassis(), Localization(10.2), ADCTrajectory(),
                            10.2);
  ASSERT_TRUE(hold.has_directive);
  EXPECT_EQ(hold.directive.scope(), MOTION_SCOPE_PLANNING_IDLE_HOLD);
  EXPECT_FALSE(hold.directive.has_parent_mission_identity());
  EXPECT_EQ(hold.directive.execute().command().primitive().type(),
            MOTION_PRIMITIVE_STANDSTILL_HOLD);
}

TEST(MotionPlanBuilderTest, ReplacesMovingMissionWithControlledStop) {
  MotionPlanBuilder builder("planning-boot");
  PlanningSemanticSummary semantics;
  auto execute = builder.Build(State(MISSION_SESSION_EXECUTING), semantics,
                               Chassis(1.0), Localization(), Trajectory(),
                               10.0);
  MotionExecutionStatus accepted;
  accepted.mutable_identity()->CopyFrom(
      execute.directive.execute().command().identity());
  accepted.mutable_parent_mission_identity()->CopyFrom(
      execute.directive.parent_mission_identity());
  accepted.set_state(MOTION_EXECUTION_EXECUTING_TRAJECTORY);
  builder.ObserveControlStatus(accepted,
                               MOTION_SCOPE_MISSION_DESCENDANT);

  auto stopping = builder.Build(State(MISSION_SESSION_CANCELLING), semantics,
                                Chassis(1.0), Localization(10.1),
                                Trajectory(), 10.1);
  ASSERT_TRUE(stopping.has_directive);
  ASSERT_TRUE(stopping.directive.has_replace());
  EXPECT_EQ(stopping.directive.replace().command().identity().command_id(),
            "controlled-stop");
  EXPECT_EQ(stopping.directive.replace().command().trajectory().point(1)
                .speed_mps(),
            0.0);
}

TEST(MotionPlanBuilderTest, BuildsReverseOrderedControlledStop) {
  MotionPlanBuilder builder("planning-boot");
  PlanningSemanticSummary semantics;
  auto chassis = Chassis(1.0);
  chassis.set_gear_location(canbus::Chassis::GEAR_REVERSE);
  auto execute = builder.Build(State(MISSION_SESSION_EXECUTING), semantics,
                               chassis, Localization(), Trajectory(), 10.0);
  MotionExecutionStatus accepted;
  accepted.mutable_identity()->CopyFrom(
      execute.directive.execute().command().identity());
  accepted.mutable_parent_mission_identity()->CopyFrom(
      execute.directive.parent_mission_identity());
  accepted.set_state(MOTION_EXECUTION_EXECUTING_TRAJECTORY);
  builder.ObserveControlStatus(accepted,
                               MOTION_SCOPE_MISSION_DESCENDANT);

  auto stopping = builder.Build(State(MISSION_SESSION_CANCELLING), semantics,
                                chassis, Localization(10.1), Trajectory(),
                                10.1);
  ASSERT_TRUE(stopping.has_directive);
  const auto& stop = stopping.directive.replace().command().trajectory();
  EXPECT_EQ(stop.gear(), canbus::Chassis::GEAR_REVERSE);
  EXPECT_LT(stop.point(1).path_point().s(),
            stop.point(0).path_point().s());
}

}  // namespace
}  // namespace planning
}  // namespace apollo
