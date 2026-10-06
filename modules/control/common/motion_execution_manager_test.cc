#include "modules/control/common/motion_execution_manager.h"

#include "gtest/gtest.h"

namespace apollo {
namespace control {
namespace {

MotionExecutionCapabilities TrajectoryCapabilities() {
  MotionExecutionCapabilities capabilities;
  capabilities.supported = {
      planning::MOTION_CAPABILITY_TRAJECTORY_TRACKING,
  };
  return capabilities;
}

MotionExecutionCapabilities HoldCapabilities() {
  MotionExecutionCapabilities capabilities;
  capabilities.supported = {
      planning::MOTION_CAPABILITY_STANDSTILL_HOLD,
  };
  return capabilities;
}

MotionExecutionCapabilities TrajectoryAndHoldCapabilities() {
  auto capabilities = TrajectoryCapabilities();
  capabilities.supported.insert(
      planning::MOTION_CAPABILITY_STANDSTILL_HOLD);
  return capabilities;
}

planning::MotionExecutionCommand TrajectoryCommand(
    const std::string& command_id, uint64_t revision, uint32_t priority,
    const std::string& aggregate_id = "motion-stream") {
  planning::MotionExecutionCommand command;
  command.mutable_header()->set_timestamp_sec(1.0);
  command.mutable_header()->set_module_name("planning");
  command.mutable_header()->set_frame_id("map");
  command.mutable_identity()->set_producer_epoch("planning-boot-1");
  command.mutable_identity()->set_aggregate_id(aggregate_id);
  command.mutable_identity()->set_command_id(command_id);
  command.mutable_identity()->set_revision(revision);
  command.set_authority_generation(1);
  command.mutable_authorized_mission_identity()->set_producer_epoch(
      "mission-boot-1");
  command.mutable_authorized_mission_identity()->set_aggregate_id("mission-1");
  command.mutable_authorized_mission_identity()->set_command_id(
      "mission-command");
  command.mutable_authorized_mission_identity()->set_revision(1);
  command.set_reference_frame_id("map");
  command.set_effective_time_sec(1.0);
  command.set_expiry_time_sec(20.0);
  command.set_priority(priority);
  command.set_preemptible(true);
  command.add_required_capability(
      planning::MOTION_CAPABILITY_TRAJECTORY_TRACKING);

  auto* start = command.mutable_start_condition();
  start->mutable_expected_position()->set_x(0.0);
  start->mutable_expected_position()->set_y(0.0);
  start->mutable_expected_position()->set_z(0.0);
  start->set_expected_heading(0.0);
  start->set_expected_gear(canbus::Chassis::GEAR_DRIVE);
  start->set_max_position_error_m(0.2);
  start->set_max_heading_error_rad(0.1);
  start->set_max_abs_speed_mps(0.5);
  start->set_snapshot_time_sec(1.0);
  start->set_reference_frame_id("map");

  auto* constraints = command.mutable_constraints();
  constraints->set_max_speed_mps(2.0);
  constraints->set_max_acceleration_mps2(1.0);
  constraints->set_max_deceleration_mps2(1.0);
  constraints->set_max_jerk_mps3(1.0);
  constraints->set_max_abs_curvature_per_m(0.5);
  constraints->set_max_abs_curvature_derivative_per_m2(0.5);

  auto* completion = command.mutable_completion();
  completion->set_position_tolerance_m(0.1);
  completion->set_heading_tolerance_rad(0.1);
  completion->set_speed_tolerance_mps(0.1);
  completion->set_settle_time_sec(0.2);
  completion->set_execution_timeout_sec(5.0);
  auto* envelope = command.mutable_spatial_envelope();
  auto* boundary = envelope->add_boundary();
  boundary->set_x(-1.0);
  boundary->set_y(-1.0);
  boundary = envelope->add_boundary();
  boundary->set_x(2.0);
  boundary->set_y(-1.0);
  boundary = envelope->add_boundary();
  boundary->set_x(2.0);
  boundary->set_y(1.0);
  boundary = envelope->add_boundary();
  boundary->set_x(-1.0);
  boundary->set_y(1.0);
  auto* trajectory = command.mutable_trajectory();
  trajectory->set_trajectory_id("trajectory-" + command_id);
  trajectory->set_gear(canbus::Chassis::GEAR_DRIVE);
  auto* point = trajectory->add_point();
  point->mutable_path_point()->set_x(0.0);
  point->mutable_path_point()->set_y(0.0);
  point->mutable_path_point()->set_z(0.0);
  point->mutable_path_point()->set_theta(0.0);
  point->mutable_path_point()->set_s(0.0);
  point->mutable_path_point()->set_kappa(0.0);
  point->mutable_path_point()->set_dkappa(0.0);
  point->set_speed_mps(0.0);
  point->set_acceleration_mps2(0.0);
  point->set_relative_time_sec(0.0);
  auto* next_point = trajectory->add_point();
  next_point->CopyFrom(*point);
  next_point->mutable_path_point()->set_x(1.0);
  next_point->mutable_path_point()->set_s(1.0);
  next_point->set_relative_time_sec(1.0);
  return command;
}

planning::MotionExecutionCommand HoldCommand(uint64_t revision) {
  planning::MotionExecutionCommand command;
  command.mutable_header()->set_timestamp_sec(1.0);
  command.mutable_header()->set_module_name("planning");
  command.mutable_header()->set_frame_id("map");
  command.mutable_identity()->set_producer_epoch("planning-boot-1");
  command.mutable_identity()->set_aggregate_id("hold-stream");
  command.mutable_identity()->set_command_id("hold");
  command.mutable_identity()->set_revision(revision);
  command.set_authority_generation(1);
  command.mutable_authorized_mission_identity()->set_producer_epoch(
      "mission-boot-1");
  command.mutable_authorized_mission_identity()->set_aggregate_id("mission-1");
  command.mutable_authorized_mission_identity()->set_command_id(
      "mission-command");
  command.mutable_authorized_mission_identity()->set_revision(1);
  command.set_reference_frame_id("map");
  command.set_effective_time_sec(1.0);
  command.set_expiry_time_sec(20.0);
  command.set_priority(1);
  command.set_preemptible(true);
  command.add_required_capability(
      planning::MOTION_CAPABILITY_STANDSTILL_HOLD);

  auto* start = command.mutable_start_condition();
  start->mutable_expected_position()->set_x(0.0);
  start->mutable_expected_position()->set_y(0.0);
  start->mutable_expected_position()->set_z(0.0);
  start->set_expected_heading(0.0);
  start->set_expected_gear(canbus::Chassis::GEAR_DRIVE);
  start->set_max_position_error_m(0.1);
  start->set_max_heading_error_rad(0.1);
  start->set_max_abs_speed_mps(0.1);
  start->set_snapshot_time_sec(1.0);
  start->set_reference_frame_id("map");

  auto* constraints = command.mutable_constraints();
  constraints->set_max_speed_mps(0.0);
  constraints->set_max_acceleration_mps2(1.0);
  constraints->set_max_deceleration_mps2(1.0);
  constraints->set_max_jerk_mps3(1.0);
  constraints->set_max_abs_curvature_per_m(0.1);
  constraints->set_max_abs_curvature_derivative_per_m2(0.1);

  auto* completion = command.mutable_completion();
  completion->set_position_tolerance_m(0.1);
  completion->set_heading_tolerance_rad(0.1);
  completion->set_speed_tolerance_mps(0.05);
  completion->set_settle_time_sec(0.2);
  completion->set_execution_timeout_sec(10.0);
  auto* envelope = command.mutable_spatial_envelope();
  auto* boundary = envelope->add_boundary();
  boundary->set_x(-0.2);
  boundary->set_y(-0.2);
  boundary = envelope->add_boundary();
  boundary->set_x(0.2);
  boundary->set_y(-0.2);
  boundary = envelope->add_boundary();
  boundary->set_x(0.2);
  boundary->set_y(0.2);
  boundary = envelope->add_boundary();
  boundary->set_x(-0.2);
  boundary->set_y(0.2);
  auto* primitive = command.mutable_primitive();
  primitive->set_type(planning::MOTION_PRIMITIVE_STANDSTILL_HOLD);
  primitive->mutable_standstill_hold()->set_reauthorization_period_sec(1.0);
  return command;
}

planning::MissionCommandIdentity Parent(uint64_t revision = 1,
                                        const std::string& command_id =
                                            "mission-command") {
  planning::MissionCommandIdentity identity;
  identity.set_producer_epoch("mission-boot-1");
  identity.set_aggregate_id("mission-1");
  identity.set_command_id(command_id);
  identity.set_revision(revision);
  return identity;
}

planning::MotionTerminalEvidence EvidenceFor(
    const planning::MotionExecutionCommand& command,
    planning::MotionTerminalEvidenceKind kind,
    planning::MotionExecutorOwnership ownership, double observed_at_sec,
    double settled_duration_sec = 0.2) {
  planning::MotionTerminalEvidence evidence;
  evidence.set_contract_version(1);
  evidence.set_kind(kind);
  evidence.mutable_motion_identity()->CopyFrom(command.identity());
  evidence.mutable_parent_mission_identity()->CopyFrom(
      command.authorized_mission_identity());
  evidence.set_authority_generation(command.authority_generation());
  evidence.set_observed_at_sec(observed_at_sec);
  evidence.set_reference_frame_id(command.reference_frame_id());
  evidence.set_position_error_m(0.0);
  evidence.set_heading_error_rad(0.0);
  evidence.set_absolute_speed_mps(0.0);
  evidence.set_settled_duration_sec(settled_duration_sec);
  evidence.set_executor_ownership(ownership);
  evidence.set_executor_type(
      command.has_trajectory() ? planning::MOTION_EXECUTION_TYPE_TRAJECTORY
                               : planning::MOTION_EXECUTION_TYPE_PRIMITIVE);
  evidence.set_safety_state(planning::MOTION_EVIDENCE_SAFETY_NORMAL);
  return evidence;
}

planning::MotionDirective Execute(
    const planning::MissionCommandIdentity& parent,
    const planning::MotionExecutionCommand& command) {
  planning::MotionDirective directive;
  directive.set_scope(planning::MOTION_SCOPE_MISSION_DESCENDANT);
  directive.mutable_parent_mission_identity()->CopyFrom(parent);
  directive.mutable_execute()->mutable_command()->CopyFrom(command);
  return directive;
}

planning::MotionDirective Replace(
    const planning::MissionCommandIdentity& parent,
    const planning::MotionCommandIdentity& expected,
    const planning::MotionExecutionCommand& command) {
  planning::MotionDirective directive;
  directive.set_scope(planning::MOTION_SCOPE_MISSION_DESCENDANT);
  directive.mutable_parent_mission_identity()->CopyFrom(parent);
  directive.mutable_replace()->mutable_expected_active_identity()->CopyFrom(
      expected);
  directive.mutable_replace()->mutable_command()->CopyFrom(command);
  return directive;
}

planning::MotionDirective Cancel(
    const planning::MissionCommandIdentity& parent,
    const planning::MotionCommandIdentity& target, bool fence_parent) {
  planning::MotionDirective directive;
  directive.set_scope(planning::MOTION_SCOPE_MISSION_DESCENDANT);
  directive.mutable_parent_mission_identity()->CopyFrom(parent);
  directive.mutable_cancel()->mutable_target_identity()->CopyFrom(target);
  directive.mutable_cancel()->set_fence_parent_mission(fence_parent);
  directive.mutable_cancel()->set_reason("cancelled");
  return directive;
}

planning::MotionDirective FenceOnly(
    const planning::MissionCommandIdentity& parent) {
  planning::MotionDirective directive;
  directive.set_scope(planning::MOTION_SCOPE_MISSION_DESCENDANT);
  directive.mutable_parent_mission_identity()->CopyFrom(parent);
  directive.mutable_cancel()->set_fence_parent_mission(true);
  directive.mutable_cancel()->set_reason("mission cancelled");
  return directive;
}

planning::MotionDirective PlanningIdleHold(uint64_t revision) {
  planning::MotionDirective directive;
  directive.set_scope(planning::MOTION_SCOPE_PLANNING_IDLE_HOLD);
  directive.mutable_execute()->mutable_command()->CopyFrom(
      HoldCommand(revision));
  return directive;
}

MotionExecutionVehicleState VehicleState(double timestamp_sec = 2.2) {
  MotionExecutionVehicleState state;
  state.position.set_x(0.0);
  state.position.set_y(0.0);
  state.position.set_z(0.0);
  state.heading = 0.0;
  state.gear = canbus::Chassis::GEAR_DRIVE;
  state.speed_mps = 0.0;
  state.timestamp_sec = timestamp_sec;
  state.reference_frame_id = "map";
  return state;
}

planning::MotionExecutionStatus StartIdleHold(
    MotionExecutionManager* manager, uint64_t revision, double now_sec) {
  const auto applied =
      manager->Apply(PlanningIdleHold(revision), now_sec);
  if (applied.state() != planning::MOTION_EXECUTION_VALIDATED) {
    return applied;
  }
  if (manager->Arm(now_sec + 0.01).state() !=
          planning::MOTION_EXECUTION_ARMED ||
      manager->Start(VehicleState(now_sec + 0.02), now_sec + 0.02).state() !=
          planning::MOTION_EXECUTION_EXECUTING_PRIMITIVE) {
    return manager->status();
  }
  const auto evidence = EvidenceFor(
      *manager->active_command(),
      planning::MOTION_TERMINAL_EVIDENCE_STANDSTILL_HOLD,
      planning::MOTION_EXECUTOR_OWNERSHIP_ACTIVE, now_sec + 0.22);
  return manager->EnterHolding(evidence, now_sec + 0.22);
}

planning::MotionExecutionStatus RevokeExecutor(
    MotionExecutionManager* manager, double now_sec) {
  const auto evidence = EvidenceFor(
      *manager->active_command(),
      planning::MOTION_TERMINAL_EVIDENCE_EXECUTOR_REVOCATION,
      planning::MOTION_EXECUTOR_OWNERSHIP_REVOKED, now_sec, 0.0);
  return manager->ConfirmExecutorRevoked(evidence, now_sec);
}

TEST(MotionExecutionManagerTest, RunsValidatedLifecycle) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  const auto command = TrajectoryCommand("command", 1, 1);

  EXPECT_EQ(manager.Apply(Execute(Parent(), command), 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  EXPECT_EQ(manager.Arm(2.1).state(), planning::MOTION_EXECUTION_ARMED);
  EXPECT_EQ(manager.Start(VehicleState(), 2.2).state(),
            planning::MOTION_EXECUTION_EXECUTING_TRAJECTORY);
  const auto evidence = EvidenceFor(
      command, planning::MOTION_TERMINAL_EVIDENCE_COMPLETION,
      planning::MOTION_EXECUTOR_OWNERSHIP_ACTIVE, 3.0);
  EXPECT_EQ(manager.Succeed(evidence, 3.0).state(),
            planning::MOTION_EXECUTION_SUCCEEDED);
  EXPECT_EQ(manager.last_terminal_status().identity().command_id(), "command");
}

TEST(MotionExecutionManagerTest,
     RejectsMismatchedAndUnsafeCompletionEvidence) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  const auto command = TrajectoryCommand("command", 1, 1);
  ASSERT_EQ(manager.Apply(Execute(Parent(), command), 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  ASSERT_EQ(manager.Arm(2.1).state(), planning::MOTION_EXECUTION_ARMED);
  ASSERT_EQ(manager.Start(VehicleState(), 2.2).state(),
            planning::MOTION_EXECUTION_EXECUTING_TRAJECTORY);

  auto evidence = EvidenceFor(
      command, planning::MOTION_TERMINAL_EVIDENCE_COMPLETION,
      planning::MOTION_EXECUTOR_OWNERSHIP_ACTIVE, 2.5);
  evidence.mutable_parent_mission_identity()->set_revision(2);
  EXPECT_EQ(manager.Succeed(evidence, 2.5).reject_reason(),
            planning::MOTION_REJECT_INVALID_TRANSITION);
  EXPECT_EQ(manager.status().state(),
            planning::MOTION_EXECUTION_EXECUTING_TRAJECTORY);

  evidence = EvidenceFor(command,
                         planning::MOTION_TERMINAL_EVIDENCE_COMPLETION,
                         planning::MOTION_EXECUTOR_OWNERSHIP_ACTIVE, 2.5);
  evidence.set_observed_at_sec(2.1);
  EXPECT_EQ(manager.Succeed(evidence, 2.5).reject_reason(),
            planning::MOTION_REJECT_INVALID_TRANSITION);
  evidence = EvidenceFor(command,
                         planning::MOTION_TERMINAL_EVIDENCE_COMPLETION,
                         planning::MOTION_EXECUTOR_OWNERSHIP_ACTIVE, 2.5);
  evidence.set_authority_generation(2);
  EXPECT_EQ(manager.Succeed(evidence, 2.5).reject_reason(),
            planning::MOTION_REJECT_INVALID_TRANSITION);
  evidence = EvidenceFor(command,
                         planning::MOTION_TERMINAL_EVIDENCE_COMPLETION,
                         planning::MOTION_EXECUTOR_OWNERSHIP_ACTIVE, 2.5);
  evidence.clear_settled_duration_sec();
  EXPECT_EQ(manager.Succeed(evidence, 2.5).reject_reason(),
            planning::MOTION_REJECT_INVALID_TRANSITION);
  evidence = EvidenceFor(command,
                         planning::MOTION_TERMINAL_EVIDENCE_COMPLETION,
                         planning::MOTION_EXECUTOR_OWNERSHIP_ACTIVE, 2.5);
  evidence.set_position_error_m(1.0);
  EXPECT_EQ(manager.Succeed(evidence, 2.5).reject_reason(),
            planning::MOTION_REJECT_INVALID_TRANSITION);
  evidence.set_position_error_m(0.0);
  evidence.set_safety_state(planning::MOTION_EVIDENCE_SAFETY_SOFT_STOP);
  EXPECT_EQ(manager.Succeed(evidence, 2.5).reject_reason(),
            planning::MOTION_REJECT_INVALID_TRANSITION);
  EXPECT_EQ(manager.status().state(),
            planning::MOTION_EXECUTION_EXECUTING_TRAJECTORY);
}

TEST(MotionExecutionManagerTest, RejectsAngularEvidenceWithoutChangingOwner) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  const auto command = TrajectoryCommand("command", 1, 1);
  ASSERT_EQ(manager.Apply(Execute(Parent(), command), 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  ASSERT_EQ(manager.Arm(2.1).state(), planning::MOTION_EXECUTION_ARMED);
  ASSERT_EQ(manager.Start(VehicleState(), 2.2).state(),
            planning::MOTION_EXECUTION_EXECUTING_TRAJECTORY);
  auto evidence = EvidenceFor(
      command, planning::MOTION_TERMINAL_EVIDENCE_COMPLETION,
      planning::MOTION_EXECUTOR_OWNERSHIP_ACTIVE, 2.5);
  evidence.set_absolute_angular_speed_radps(0.0);
  EXPECT_EQ(manager.Succeed(evidence, 2.5).reject_reason(),
            planning::MOTION_REJECT_INVALID_TRANSITION);
  evidence.clear_absolute_angular_speed_radps();
  evidence.mutable_rotation_progress()->set_continuous_heading_observation(true);
  EXPECT_EQ(manager.Succeed(evidence, 2.5).reject_reason(),
            planning::MOTION_REJECT_INVALID_TRANSITION);
  evidence.clear_rotation_progress();
  evidence.set_contract_version(2);
  EXPECT_EQ(manager.Succeed(evidence, 2.5).reject_reason(),
            planning::MOTION_REJECT_INVALID_TRANSITION);
  EXPECT_EQ(manager.status().state(),
            planning::MOTION_EXECUTION_EXECUTING_TRAJECTORY);
  ASSERT_NE(manager.active_command(), nullptr);
  EXPECT_EQ(manager.active_command()->identity().SerializeAsString(),
            command.identity().SerializeAsString());
  evidence.set_contract_version(1);
  EXPECT_EQ(manager.Succeed(evidence, 2.5).state(),
            planning::MOTION_EXECUTION_SUCCEEDED);
}

TEST(MotionExecutionManagerTest, ActiveHoldingRetainsCommandAndDeadline) {
  MotionExecutionManager manager{MotionExecutionValidator(HoldCapabilities())};
  const auto directive = PlanningIdleHold(1);
  ASSERT_EQ(manager.Apply(directive, 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  ASSERT_EQ(manager.Arm(2.0).state(), planning::MOTION_EXECUTION_ARMED);
  ASSERT_EQ(manager.Start(VehicleState(2.0), 2.0).state(),
            planning::MOTION_EXECUTION_EXECUTING_PRIMITIVE);
  const auto evidence = EvidenceFor(
      *manager.active_command(),
      planning::MOTION_TERMINAL_EVIDENCE_STANDSTILL_HOLD,
      planning::MOTION_EXECUTOR_OWNERSHIP_ACTIVE, 2.3);
  ASSERT_EQ(manager.EnterHolding(evidence, 2.3).state(),
            planning::MOTION_EXECUTION_HOLDING);
  EXPECT_FALSE(manager.status().has_parent_mission_identity());
  EXPECT_EQ(manager.status().authorized_mission_identity().SerializeAsString(),
            directive.execute().command().authorized_mission_identity()
                .SerializeAsString());
  EXPECT_EQ(manager.status().authority_generation(),
            directive.execute().command().authority_generation());
  ASSERT_NE(manager.active_command(), nullptr);
  EXPECT_EQ(manager.active_command()->identity().revision(), 1);
  EXPECT_FALSE(manager.status().has_terminal_time_sec());
  EXPECT_EQ(manager.Succeed(evidence, 2.4).reject_reason(),
            planning::MOTION_REJECT_INVALID_TRANSITION);
  EXPECT_EQ(manager.status().state(), planning::MOTION_EXECUTION_HOLDING);
  EXPECT_EQ(manager.Tick(3.1).state(), planning::MOTION_EXECUTION_TIMED_OUT);
  EXPECT_EQ(manager.active_command(), nullptr);
}

TEST(MotionExecutionManagerTest, ActiveHoldingIsCancellableAndReplaceable) {
  MotionExecutionManager manager{MotionExecutionValidator(HoldCapabilities())};
  const auto command = HoldCommand(1);
  ASSERT_EQ(manager.Apply(Execute(Parent(), command), 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  manager.Arm(2.0);
  manager.Start(VehicleState(2.0), 2.0);
  auto hold_evidence = EvidenceFor(
      *manager.active_command(),
      planning::MOTION_TERMINAL_EVIDENCE_STANDSTILL_HOLD,
      planning::MOTION_EXECUTOR_OWNERSHIP_ACTIVE, 2.2);
  manager.EnterHolding(hold_evidence, 2.2);
  auto next = HoldCommand(2);
  ASSERT_EQ(manager.Apply(Replace(Parent(), command.identity(), next), 2.3).state(),
            planning::MOTION_EXECUTION_CANCELLING);
  const auto revoke_evidence = EvidenceFor(
      *manager.active_command(),
      planning::MOTION_TERMINAL_EVIDENCE_EXECUTOR_REVOCATION,
      planning::MOTION_EXECUTOR_OWNERSHIP_REVOKED, 2.3, 0.0);
  EXPECT_EQ(manager.ConfirmExecutorRevoked(revoke_evidence, 2.3).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  ASSERT_NE(manager.active_command(), nullptr);
  EXPECT_EQ(manager.active_command()->identity().revision(), 2);
}

TEST(MotionExecutionManagerTest, MovingExecutionCannotClaimActiveHolding) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  const auto command = TrajectoryCommand("command", 1, 1);
  manager.Apply(Execute(Parent(), command), 2.0);
  manager.Arm(2.0);
  manager.Start(VehicleState(2.0), 2.0);
  const auto evidence = EvidenceFor(
      *manager.active_command(),
      planning::MOTION_TERMINAL_EVIDENCE_STANDSTILL_HOLD,
      planning::MOTION_EXECUTOR_OWNERSHIP_ACTIVE, 2.2);
  EXPECT_EQ(manager.EnterHolding(evidence, 2.2).reject_reason(),
            planning::MOTION_REJECT_INVALID_TRANSITION);
  EXPECT_EQ(manager.status().state(),
            planning::MOTION_EXECUTION_EXECUTING_TRAJECTORY);
}

TEST(MotionExecutionManagerTest, FencedCleanupRequiresExplicitStopAuthorization) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  manager.Apply(FenceOnly(Parent()), 2.0);
  auto command = TrajectoryCommand("controlled-stop", 1, 1);
  auto* intent = command.mutable_control_intent();
  intent->set_tracking_mode(planning::TRACKING_MODE_TRAJECTORY);
  intent->set_execution_channel(planning::EXECUTION_CHANNEL_TRAJECTORY);
  intent->set_primitive_type(planning::CONTROL_PRIMITIVE_NONE);
  intent->set_longitudinal_intent(planning::LON_INTENT_MRM_STOP);
  intent->set_require_full_stop(true);
  EXPECT_EQ(manager.Apply(Execute(Parent(), command), 2.1).reject_reason(),
            planning::MOTION_REJECT_PARENT_FENCED);
  auto spoofed = command;
  spoofed.mutable_control_intent()->set_longitudinal_intent(planning::LON_INTENT_CRUISE);
  EXPECT_EQ(manager.Apply(Execute(Parent(), spoofed), 2.1, true).reject_reason(),
            planning::MOTION_REJECT_PARENT_FENCED);
  auto invalid = command;
  invalid.clear_spatial_envelope();
  EXPECT_EQ(manager.Apply(Execute(Parent(), invalid), 2.1, true).state(),
            planning::MOTION_EXECUTION_REJECTED);
  EXPECT_EQ(manager.Apply(Execute(Parent(), command), 2.1, true).state(),
            planning::MOTION_EXECUTION_VALIDATED);
}

TEST(MotionExecutionManagerTest, IncomingRejectionKeepsCausalParentAndActiveOwner) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  const auto active = TrajectoryCommand("active", 1, 1);
  ASSERT_EQ(manager.Apply(Execute(Parent(), active), 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  auto incoming = TrajectoryCommand("incoming", 1, 1);
  const auto other_parent = Parent(2, "other-parent");
  incoming.clear_spatial_envelope();
  const auto rejection = manager.Apply(Execute(other_parent, incoming), 2.1);
  EXPECT_EQ(rejection.state(), planning::MOTION_EXECUTION_REJECTED);
  EXPECT_EQ(rejection.parent_mission_identity().SerializeAsString(),
            other_parent.SerializeAsString());
  EXPECT_EQ(rejection.identity().command_id(), "incoming");
  EXPECT_EQ(rejection.authorized_mission_identity().SerializeAsString(),
            incoming.authorized_mission_identity().SerializeAsString());
  EXPECT_EQ(rejection.authority_generation(), incoming.authority_generation());
  ASSERT_NE(manager.active_command(), nullptr);
  EXPECT_EQ(manager.active_command()->identity().command_id(), "active");
  EXPECT_EQ(manager.status().parent_mission_identity().SerializeAsString(),
            Parent().SerializeAsString());
}

TEST(MotionExecutionManagerTest, RejectsReplayRevision) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  const auto command = TrajectoryCommand("command", 2, 1);
  ASSERT_EQ(manager.Apply(Execute(Parent(), command), 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);

  const auto replay = manager.Apply(Execute(Parent(), command), 2.1);
  EXPECT_EQ(replay.state(), planning::MOTION_EXECUTION_REJECTED);
  EXPECT_EQ(replay.reject_reason(), planning::MOTION_REJECT_REPLAY);
  EXPECT_EQ(manager.status().state(), planning::MOTION_EXECUTION_VALIDATED);
}

TEST(MotionExecutionManagerTest, RejectsReplayAfterTerminalHold) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryAndHoldCapabilities())};
  const auto command = TrajectoryCommand("command", 1, 1);
  ASSERT_EQ(manager.Apply(Execute(Parent(), command), 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  ASSERT_EQ(manager.Apply(Cancel(Parent(), command.identity(), false), 2.1).state(),
            planning::MOTION_EXECUTION_CANCELLING);
  ASSERT_EQ(RevokeExecutor(&manager, 2.2).state(),
            planning::MOTION_EXECUTION_CANCELLED);
  ASSERT_EQ(StartIdleHold(&manager, 1, 2.3).state(),
            planning::MOTION_EXECUTION_HOLDING);

  const auto replay = manager.Apply(Execute(Parent(), command), 2.6);
  EXPECT_EQ(replay.reject_reason(), planning::MOTION_REJECT_REPLAY);
  EXPECT_EQ(manager.status().state(), planning::MOTION_EXECUTION_HOLDING);
}

TEST(MotionExecutionManagerTest, KeepsReplayRecordUntilLongestExpiry) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryAndHoldCapabilities())};
  auto revision_one = TrajectoryCommand("command", 1, 1);
  revision_one.set_expiry_time_sec(20.0);
  ASSERT_EQ(manager.Apply(Execute(Parent(), revision_one), 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);

  auto revision_two = TrajectoryCommand("command", 2, 1);
  revision_two.set_expiry_time_sec(10.0);
  ASSERT_EQ(manager
                .Apply(Replace(Parent(), revision_one.identity(), revision_two),
                       2.1)
                .state(),
            planning::MOTION_EXECUTION_CANCELLING);
  ASSERT_EQ(RevokeExecutor(&manager, 2.15).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  ASSERT_EQ(
      manager.Apply(Cancel(Parent(), revision_two.identity(), false), 2.2)
          .state(),
            planning::MOTION_EXECUTION_CANCELLING);
  ASSERT_EQ(RevokeExecutor(&manager, 2.3).state(),
            planning::MOTION_EXECUTION_CANCELLED);
  ASSERT_EQ(StartIdleHold(&manager, 1, 2.4).state(),
            planning::MOTION_EXECUTION_HOLDING);

  const auto replay = manager.Apply(Execute(Parent(), revision_one), 11.0);
  EXPECT_EQ(replay.reject_reason(), planning::MOTION_REJECT_REPLAY);
}

TEST(MotionExecutionManagerTest, RejectsImplicitPreemption) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  ASSERT_EQ(
      manager
          .Apply(Execute(Parent(), TrajectoryCommand("first", 1, 10)), 2.0)
          .state(),
            planning::MOTION_EXECUTION_VALIDATED);

  const auto rejected = manager.Apply(
      Execute(Parent(), TrajectoryCommand("second", 1, 20)), 2.1);
  EXPECT_EQ(rejected.reject_reason(), planning::MOTION_REJECT_BUSY);
  ASSERT_NE(manager.active_command(), nullptr);
  EXPECT_EQ(manager.active_command()->identity().command_id(), "first");
}

TEST(MotionExecutionManagerTest, AllowsOnlyExactReplacement) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  const auto first = TrajectoryCommand("first", 1, 5);
  ASSERT_EQ(manager.Apply(Execute(Parent(), first), 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);

  const auto accepted = manager.Apply(
      Replace(Parent(), first.identity(),
              TrajectoryCommand("second", 1, 10)),
      2.1);
  EXPECT_EQ(accepted.state(), planning::MOTION_EXECUTION_CANCELLING);
  ASSERT_NE(manager.active_command(), nullptr);
  EXPECT_EQ(manager.active_command()->identity().command_id(), "first");
  EXPECT_EQ(RevokeExecutor(&manager, 2.2).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  EXPECT_EQ(manager.last_terminal_status().state(),
            planning::MOTION_EXECUTION_CANCELLED);
  ASSERT_NE(manager.active_command(), nullptr);
  EXPECT_EQ(manager.active_command()->identity().command_id(), "second");
}

TEST(MotionExecutionManagerTest, InvalidReplacementPreservesActiveMotion) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  const auto first = TrajectoryCommand("first", 1, 5);
  ASSERT_EQ(manager.Apply(Execute(Parent(), first), 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  auto invalid = TrajectoryCommand("second", 1, 10);
  invalid.clear_start_condition();

  const auto rejected =
      manager.Apply(Replace(Parent(), first.identity(), invalid), 2.1);
  EXPECT_EQ(rejected.reject_reason(),
            planning::MOTION_REJECT_INVALID_START_CONDITION);
  ASSERT_NE(manager.active_command(), nullptr);
  EXPECT_EQ(manager.active_command()->identity().command_id(), "first");
  EXPECT_EQ(manager.status().state(), planning::MOTION_EXECUTION_VALIDATED);
}

TEST(MotionExecutionManagerTest, AppliesPreemptionAcrossMissionBoundary) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  auto first = TrajectoryCommand("shared-id", 1, 10, "mission-a");
  first.mutable_authorized_mission_identity()->CopyFrom(
      Parent(1, "mission-a"));
  first.set_preemptible(false);
  ASSERT_EQ(manager.Apply(Execute(Parent(1, "mission-a"), first), 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);

  auto replacement =
      TrajectoryCommand("shared-id", 2, 20, "other-stream");
  replacement.mutable_authorized_mission_identity()->CopyFrom(
      Parent(1, "mission-b"));
  const auto rejected = manager.Apply(
      Replace(Parent(1, "mission-b"), first.identity(),
              replacement),
      2.1);
  EXPECT_EQ(rejected.reject_reason(), planning::MOTION_REJECT_CAS_MISMATCH);
  ASSERT_NE(manager.active_command(), nullptr);
  EXPECT_EQ(manager.active_command()->identity().aggregate_id(),
            "mission-a");
}

TEST(MotionExecutionManagerTest, NewRevisionDoesNotImplicitlySupersede) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  ASSERT_EQ(
      manager.Apply(Execute(Parent(), TrajectoryCommand("command", 1, 1)),
                    2.0)
          .state(),
      planning::MOTION_EXECUTION_VALIDATED);

  const auto rejected =
      manager.Apply(Execute(Parent(), TrajectoryCommand("command", 2, 1)),
                    2.1);
  EXPECT_EQ(rejected.reject_reason(), planning::MOTION_REJECT_BUSY);
  EXPECT_EQ(manager.active_command()->identity().revision(), 1);
}

TEST(MotionExecutionManagerTest, ParentFenceRejectsDelayedDescendant) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryAndHoldCapabilities())};
  const auto command = TrajectoryCommand("command", 1, 1);
  ASSERT_EQ(manager.Apply(Execute(Parent(), command), 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  ASSERT_EQ(manager.Apply(Cancel(Parent(), command.identity(), true), 2.1)
                .state(),
            planning::MOTION_EXECUTION_CANCELLING);
  ASSERT_EQ(RevokeExecutor(&manager, 2.2).state(),
            planning::MOTION_EXECUTION_CANCELLED);
  ASSERT_EQ(StartIdleHold(&manager, 1, 2.3).state(),
            planning::MOTION_EXECUTION_HOLDING);

  auto delayed = TrajectoryCommand("delayed", 1, 1);
  const auto rejected = manager.Apply(Execute(Parent(), delayed), 2.3);
  EXPECT_EQ(rejected.reject_reason(),
            planning::MOTION_REJECT_PARENT_FENCED);
}

TEST(MotionExecutionManagerTest, FencesParentWithoutActiveDescendant) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  EXPECT_EQ(manager.Apply(FenceOnly(Parent()), 2.0).state(),
            planning::MOTION_EXECUTION_CANCELLED);

  const auto rejected = manager.Apply(
      Execute(Parent(), TrajectoryCommand("delayed", 1, 1)), 2.1);
  EXPECT_EQ(rejected.reject_reason(),
            planning::MOTION_REJECT_PARENT_FENCED);
}

TEST(MotionExecutionManagerTest, AcceptsOnlyExplicitPlanningIdleHold) {
  MotionExecutionManager manager{
      MotionExecutionValidator(HoldCapabilities())};
  EXPECT_EQ(manager.Apply(PlanningIdleHold(1), 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);

  planning::MotionDirective invalid;
  invalid.set_scope(planning::MOTION_SCOPE_PLANNING_IDLE_HOLD);
  invalid.mutable_execute()->mutable_command()->CopyFrom(
      TrajectoryCommand("unowned", 1, 1));
  EXPECT_EQ(manager.Apply(invalid, 2.1).reject_reason(),
            planning::MOTION_REJECT_INVALID_PAYLOAD);
}

TEST(MotionExecutionManagerTest, RenewsPlanningIdleHoldByExactCas) {
  MotionExecutionManager manager{
      MotionExecutionValidator(HoldCapabilities())};
  auto first = PlanningIdleHold(1);
  ASSERT_EQ(manager.Apply(first, 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  auto renewal = PlanningIdleHold(2);
  renewal.clear_execute();
  renewal.mutable_replace()
      ->mutable_expected_active_identity()
      ->CopyFrom(first.execute().command().identity());
  renewal.mutable_replace()->mutable_command()->CopyFrom(HoldCommand(2));

  ASSERT_EQ(manager.Apply(renewal, 2.1).state(),
            planning::MOTION_EXECUTION_CANCELLING);
  EXPECT_EQ(RevokeExecutor(&manager, 2.2).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  ASSERT_NE(manager.active_command(), nullptr);
  EXPECT_EQ(manager.active_command()->identity().revision(), 2);
}

TEST(MotionExecutionManagerTest, TimesOutExecutingCommand) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  const auto command = TrajectoryCommand("command", 1, 1);
  ASSERT_EQ(manager.Submit(command, 2.0).state(),
      planning::MOTION_EXECUTION_VALIDATED);
  ASSERT_EQ(manager.Arm(2.1).state(), planning::MOTION_EXECUTION_ARMED);
  ASSERT_EQ(manager.Start(VehicleState(), 2.2).state(),
            planning::MOTION_EXECUTION_EXECUTING_TRAJECTORY);

  EXPECT_EQ(manager.Tick(7.3).state(),
            planning::MOTION_EXECUTION_TIMED_OUT);
  EXPECT_EQ(manager.status().requested_fallback(),
            planning::MOTION_FALLBACK_HOLD);
  const auto fake_hold = EvidenceFor(
      command,
      planning::MOTION_TERMINAL_EVIDENCE_STANDSTILL_HOLD,
      planning::MOTION_EXECUTOR_OWNERSHIP_ACTIVE, 7.4);
  EXPECT_EQ(manager.EnterHolding(fake_hold, 7.4).reject_reason(),
            planning::MOTION_REJECT_INVALID_TRANSITION);
  EXPECT_EQ(manager.active_command(), nullptr);
}

TEST(MotionExecutionManagerTest, CannotArmExpiredValidatedCommand) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  auto command = TrajectoryCommand("command", 1, 1);
  command.set_expiry_time_sec(3.0);
  ASSERT_EQ(manager.Submit(command, 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);

  EXPECT_EQ(manager.Arm(3.1).state(),
            planning::MOTION_EXECUTION_TIMED_OUT);
}

TEST(MotionExecutionManagerTest, EnforcesHoldReauthorizationDeadline) {
  MotionExecutionManager manager{
      MotionExecutionValidator(HoldCapabilities())};
  ASSERT_EQ(manager.Submit(HoldCommand(1), 2.0).state(),
            planning::MOTION_EXECUTION_VALIDATED);
  ASSERT_EQ(manager.Arm(2.1).state(), planning::MOTION_EXECUTION_ARMED);
  ASSERT_EQ(manager.Start(VehicleState(), 2.2).state(),
            planning::MOTION_EXECUTION_EXECUTING_PRIMITIVE);

  EXPECT_EQ(manager.Tick(3.3).state(),
            planning::MOTION_EXECUTION_TIMED_OUT);
  EXPECT_EQ(manager.status().reason(),
            "standstill hold reauthorization timeout");
}

TEST(MotionExecutionManagerTest, RejectsInvalidTransition) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  ASSERT_EQ(
      manager.Submit(TrajectoryCommand("command", 1, 1), 2.0).state(),
      planning::MOTION_EXECUTION_VALIDATED);

  const auto invalid = manager.Start(VehicleState(2.1), 2.1);
  EXPECT_EQ(invalid.reject_reason(),
            planning::MOTION_REJECT_INVALID_TRANSITION);
  EXPECT_EQ(manager.status().state(), planning::MOTION_EXECUTION_VALIDATED);
}

TEST(MotionExecutionManagerTest, RejectsVehicleStartMismatch) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  ASSERT_EQ(
      manager.Submit(TrajectoryCommand("command", 1, 1), 2.0).state(),
      planning::MOTION_EXECUTION_VALIDATED);
  ASSERT_EQ(manager.Arm(2.1).state(), planning::MOTION_EXECUTION_ARMED);
  auto state = VehicleState();
  state.position.set_x(1.0);

  const auto rejected = manager.Start(state, 2.2);
  EXPECT_EQ(rejected.reject_reason(),
            planning::MOTION_REJECT_INVALID_START_CONDITION);
  EXPECT_EQ(manager.status().state(), planning::MOTION_EXECUTION_ARMED);
}

TEST(MotionExecutionManagerTest, RejectsStaleVehicleStateAtHandoff) {
  MotionExecutionManager manager{
      MotionExecutionValidator(TrajectoryCapabilities())};
  ASSERT_EQ(
      manager.Submit(TrajectoryCommand("command", 1, 1), 2.0).state(),
      planning::MOTION_EXECUTION_VALIDATED);
  ASSERT_EQ(manager.Arm(2.1).state(), planning::MOTION_EXECUTION_ARMED);

  const auto rejected = manager.Start(VehicleState(2.0), 2.3);
  EXPECT_EQ(rejected.reject_reason(),
            planning::MOTION_REJECT_INVALID_START_CONDITION);
  EXPECT_EQ(manager.status().state(), planning::MOTION_EXECUTION_ARMED);
}

}  // namespace
}  // namespace control
}  // namespace apollo
