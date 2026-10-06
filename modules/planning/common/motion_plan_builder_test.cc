#include "modules/planning/common/motion_plan_builder.h"

#include <utility>

#include "gtest/gtest.h"

#include "modules/planning/common/planning_gflags.h"

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
  auto* intent = trajectory.mutable_control_intent();
  intent->set_tracking_mode(TRACKING_MODE_TRAJECTORY);
  intent->set_longitudinal_intent(LON_INTENT_CRUISE);
  intent->set_lateral_intent(LAT_INTENT_TRACK_PATH);
  intent->set_execution_channel(EXECUTION_CHANNEL_TRAJECTORY);
  intent->set_primitive_type(CONTROL_PRIMITIVE_NONE);
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

MotionSpatialEnvelope AuthorizedEnvelope() {
  MotionSpatialEnvelope envelope;
  for (const auto& point : {std::pair<double, double>{-10.0, -2.0},
                            {10.0, -2.0},
                            {10.0, 2.0},
                            {-10.0, 2.0}}) {
    auto* boundary = envelope.add_boundary();
    boundary->set_x(point.first);
    boundary->set_y(point.second);
  }
  envelope.set_max_lateral_deviation_m(1.0);
  return envelope;
}

TEST(MotionPlanBuilderTest, WaitsForAckAndUsesExactReplacement) {
  MotionPlanBuilder builder("planning-boot");
  builder.SetSpatialEnvelope(AuthorizedEnvelope());
  PlanningSemanticSummary semantics;
  auto first = builder.Build(State(MISSION_SESSION_EXECUTING), semantics,
                             Chassis(), Localization(), Trajectory(), 10.0);
  ASSERT_TRUE(first.has_directive);
  ASSERT_TRUE(first.directive.has_execute());
  EXPECT_EQ(first.directive.scope(), MOTION_SCOPE_MISSION_DESCENDANT);
  EXPECT_EQ(first.directive.execute().command().authority_generation(),
            State(MISSION_SESSION_EXECUTING).mission_identity.revision());
  EXPECT_EQ(first.directive.execute()
                .command()
                .authorized_mission_identity()
                .SerializeAsString(),
            first.directive.parent_mission_identity().SerializeAsString());

  auto waiting =
      builder.Build(State(MISSION_SESSION_EXECUTING), semantics, Chassis(),
                    Localization(10.1), Trajectory(), 10.1);
  EXPECT_FALSE(waiting.has_directive);

  MotionExecutionStatus accepted;
  accepted.mutable_identity()->CopyFrom(
      first.directive.execute().command().identity());
  accepted.mutable_parent_mission_identity()->CopyFrom(
      first.directive.parent_mission_identity());
  accepted.set_state(MOTION_EXECUTION_EXECUTING_TRAJECTORY);
  builder.ObserveControlStatus(accepted, MOTION_SCOPE_MISSION_DESCENDANT);

  auto replacement =
      builder.Build(State(MISSION_SESSION_EXECUTING), semantics, Chassis(),
                    Localization(10.1), Trajectory(), 10.1);
  ASSERT_TRUE(replacement.has_directive);
  ASSERT_TRUE(replacement.directive.has_replace());
  EXPECT_EQ(
      replacement.directive.replace().expected_active_identity().revision(),
      first.directive.execute().command().identity().revision());
}

TEST(MotionPlanBuilderTest,
     RetransmitsExactPendingCommandAndIgnoresOtherEpoch) {
  MotionPlanBuilder builder("planning-boot");
  builder.SetSpatialEnvelope(AuthorizedEnvelope());
  PlanningSemanticSummary semantics;
  const auto first =
      builder.Build(State(MISSION_SESSION_EXECUTING), semantics, Chassis(),
                    Localization(), Trajectory(), 10.0);
  ASSERT_TRUE(first.has_directive);
  MotionExecutionStatus unrelated;
  unrelated.mutable_identity()->CopyFrom(
      first.directive.execute().command().identity());
  unrelated.mutable_identity()->set_producer_epoch("old-planning");
  unrelated.set_state(MOTION_EXECUTION_EXECUTING_TRAJECTORY);
  EXPECT_FALSE(builder.IsCorrelatedStatus(unrelated));
  builder.ObserveControlStatus(unrelated, MOTION_SCOPE_MISSION_DESCENDANT);
  const auto retry =
      builder.Build(State(MISSION_SESSION_EXECUTING), semantics, Chassis(),
                    Localization(10.3), Trajectory(), 10.3);
  ASSERT_TRUE(retry.has_directive);
  EXPECT_EQ(retry.directive.SerializeAsString(),
            first.directive.SerializeAsString());
}

TEST(MotionPlanBuilderTest, RejectsMissingFrameAndSpeed) {
  MotionPlanBuilder builder("planning-boot");
  PlanningSemanticSummary semantics;
  auto localization = Localization();
  localization.mutable_header()->clear_frame_id();
  EXPECT_FALSE(builder
                   .Build(State(MISSION_SESSION_EXECUTING), semantics,
                          Chassis(), localization, Trajectory(), 10.0)
                   .has_directive);
  auto chassis = Chassis();
  chassis.clear_speed_mps();
  EXPECT_FALSE(builder
                   .Build(State(MISSION_SESSION_EXECUTING), semantics, chassis,
                          Localization(), Trajectory(), 10.0)
                   .has_directive);
}

TEST(MotionPlanBuilderTest, RejectsMissingControlExecutionSemantics) {
  MotionPlanBuilder builder("planning-boot");
  builder.SetSpatialEnvelope(AuthorizedEnvelope());
  PlanningSemanticSummary semantics;
  auto trajectory = Trajectory();
  trajectory.clear_control_intent();

  const auto result =
      builder.Build(State(MISSION_SESSION_EXECUTING), semantics, Chassis(),
                    Localization(), trajectory, 10.0);
  EXPECT_FALSE(result.has_directive);
  EXPECT_EQ(result.reason,
            "Planning output lacks explicit Control execution semantics");
}

TEST(MotionPlanBuilderTest, RequiresAuthorizedBoundaryAndPreservesExactPolicy) {
  MotionPlanBuilder builder("planning-boot");
  PlanningSemanticSummary semantics;
  semantics.has_position_tolerance = true;
  semantics.terminal_position_tolerance_m = 0.001;
  semantics.has_heading_tolerance = true;
  semantics.terminal_heading_tolerance_rad = 0.002;
  semantics.max_terminal_speed_mps = 0.003;
  auto trajectory = Trajectory();
  trajectory.mutable_control_intent()->set_require_full_stop(true);
  trajectory.mutable_execution()->set_mission_id("mission");
  EXPECT_FALSE(builder
                   .Build(State(MISSION_SESSION_EXECUTING), semantics,
                          Chassis(), Localization(), trajectory, 10.0)
                   .has_directive);
  const auto envelope = AuthorizedEnvelope();
  builder.SetSpatialEnvelope(envelope);
  auto result = builder.Build(State(MISSION_SESSION_EXECUTING), semantics,
                              Chassis(), Localization(), trajectory, 10.0);
  ASSERT_TRUE(result.has_directive);
  const auto& command = result.directive.execute().command();
  EXPECT_DOUBLE_EQ(command.constraints().max_speed_mps(),
                   FLAGS_speed_upper_bound);
  EXPECT_DOUBLE_EQ(command.constraints().max_abs_curvature_per_m(),
                   FLAGS_kappa_bound);
  EXPECT_DOUBLE_EQ(command.completion().position_tolerance_m(), 0.001);
  EXPECT_DOUBLE_EQ(command.completion().heading_tolerance_rad(), 0.002);
  EXPECT_DOUBLE_EQ(command.completion().speed_tolerance_mps(), 0.003);
  EXPECT_TRUE(command.control_intent().require_full_stop());
  EXPECT_EQ(command.execution().mission_id(), "mission");
  ASSERT_EQ(command.spatial_envelope().boundary_size(),
            envelope.boundary_size());
  EXPECT_EQ(command.spatial_envelope().boundary(0).SerializeAsString(),
            envelope.boundary(0).SerializeAsString());
}

TEST(MotionPlanBuilderTest, CancelsThenTransfersToPlanningIdleHold) {
  MotionPlanBuilder builder("planning-boot");
  builder.SetSpatialEnvelope(AuthorizedEnvelope());
  PlanningSemanticSummary semantics;
  auto execute = builder.Build(State(MISSION_SESSION_EXECUTING), semantics,
                               Chassis(), Localization(), Trajectory(), 10.0);
  MotionExecutionStatus accepted;
  accepted.mutable_identity()->CopyFrom(
      execute.directive.execute().command().identity());
  accepted.mutable_parent_mission_identity()->CopyFrom(
      execute.directive.parent_mission_identity());
  accepted.set_state(MOTION_EXECUTION_EXECUTING_TRAJECTORY);
  builder.ObserveControlStatus(accepted, MOTION_SCOPE_MISSION_DESCENDANT);

  auto cancel =
      builder.Build(State(MISSION_SESSION_CANCELLING), semantics, Chassis(),
                    Localization(10.1), Trajectory(), 10.1);
  ASSERT_TRUE(cancel.has_directive);
  ASSERT_TRUE(cancel.directive.has_cancel());
  EXPECT_TRUE(cancel.directive.cancel().fence_parent_mission());

  MotionExecutionStatus cancelled = accepted;
  cancelled.set_state(MOTION_EXECUTION_CANCELLED);
  builder.ObserveControlStatus(cancelled, MOTION_SCOPE_MISSION_DESCENDANT);

  auto hold =
      builder.Build(State(MISSION_SESSION_CANCELLING), semantics, Chassis(),
                    Localization(10.2), ADCTrajectory(), 10.2);
  ASSERT_TRUE(hold.has_directive);
  EXPECT_EQ(hold.directive.scope(), MOTION_SCOPE_PLANNING_IDLE_HOLD);
  EXPECT_FALSE(hold.directive.has_parent_mission_identity());
  EXPECT_EQ(hold.directive.execute().command().primitive().type(),
            MOTION_PRIMITIVE_STANDSTILL_HOLD);
  EXPECT_EQ(hold.directive.execute().command().authority_generation(),
            State(MISSION_SESSION_CANCELLING).mission_identity.revision());
  EXPECT_EQ(hold.directive.execute()
                .command()
                .authorized_mission_identity()
                .SerializeAsString(),
            State(MISSION_SESSION_CANCELLING)
                .mission_identity.SerializeAsString());
}

TEST(MotionPlanBuilderTest, FailedMissionCannotAuthorizeMoreMotion) {
  MotionPlanBuilder builder("planning-boot");
  builder.SetSpatialEnvelope(AuthorizedEnvelope());
  PlanningSemanticSummary semantics;

  const auto result = builder.Build(
      State(MISSION_SESSION_FAILED), semantics, Chassis(), Localization(),
      Trajectory(), 10.0);

  EXPECT_FALSE(result.has_directive);
  EXPECT_EQ(result.reason, "failed Mission requires a new Mission directive");
}

TEST(MotionPlanBuilderTest, ReplacesMovingMissionWithControlledStop) {
  MotionPlanBuilder builder("planning-boot");
  builder.SetSpatialEnvelope(AuthorizedEnvelope());
  PlanningSemanticSummary semantics;
  auto execute =
      builder.Build(State(MISSION_SESSION_EXECUTING), semantics, Chassis(1.0),
                    Localization(), Trajectory(), 10.0);
  MotionExecutionStatus accepted;
  accepted.mutable_identity()->CopyFrom(
      execute.directive.execute().command().identity());
  accepted.mutable_parent_mission_identity()->CopyFrom(
      execute.directive.parent_mission_identity());
  accepted.set_state(MOTION_EXECUTION_EXECUTING_TRAJECTORY);
  builder.ObserveControlStatus(accepted, MOTION_SCOPE_MISSION_DESCENDANT);

  auto stopping =
      builder.Build(State(MISSION_SESSION_CANCELLING), semantics, Chassis(1.0),
                    Localization(10.1), Trajectory(), 10.1);
  ASSERT_TRUE(stopping.has_directive);
  ASSERT_TRUE(stopping.directive.has_replace());
  EXPECT_EQ(stopping.directive.replace().command().identity().command_id(),
            "controlled-stop");
  EXPECT_EQ(
      stopping.directive.replace().command().trajectory().point(1).speed_mps(),
      0.0);
}

TEST(MotionPlanBuilderTest, BuildsReverseOrderedControlledStop) {
  MotionPlanBuilder builder("planning-boot");
  builder.SetSpatialEnvelope(AuthorizedEnvelope());
  PlanningSemanticSummary semantics;
  auto chassis = Chassis(1.0);
  chassis.set_gear_location(canbus::Chassis::GEAR_REVERSE);
  auto reverse_trajectory = Trajectory();
  reverse_trajectory.set_gear(canbus::Chassis::GEAR_REVERSE);
  for (auto& point : *reverse_trajectory.mutable_trajectory_point()) {
    point.mutable_path_point()->set_x(-point.path_point().x());
    point.mutable_path_point()->set_s(-point.path_point().s());
    point.set_v(-point.v());
  }
  auto execute =
      builder.Build(State(MISSION_SESSION_EXECUTING), semantics, chassis,
                    Localization(), reverse_trajectory, 10.0);
  MotionExecutionStatus accepted;
  accepted.mutable_identity()->CopyFrom(
      execute.directive.execute().command().identity());
  accepted.mutable_parent_mission_identity()->CopyFrom(
      execute.directive.parent_mission_identity());
  accepted.set_state(MOTION_EXECUTION_EXECUTING_TRAJECTORY);
  builder.ObserveControlStatus(accepted, MOTION_SCOPE_MISSION_DESCENDANT);

  auto stopping =
      builder.Build(State(MISSION_SESSION_CANCELLING), semantics, chassis,
                    Localization(10.1), reverse_trajectory, 10.1);
  ASSERT_TRUE(stopping.has_directive);
  const auto& stop = stopping.directive.replace().command().trajectory();
  EXPECT_EQ(stop.gear(), canbus::Chassis::GEAR_REVERSE);
  EXPECT_LT(stop.point(1).path_point().s(), stop.point(0).path_point().s());
  EXPECT_EQ(stopping.directive.replace()
                .command()
                .control_intent()
                .longitudinal_intent(),
            LON_INTENT_MRM_STOP);
}

}  // namespace
}  // namespace planning
}  // namespace apollo
