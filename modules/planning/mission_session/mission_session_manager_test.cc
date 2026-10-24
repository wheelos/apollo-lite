#include "modules/planning/mission_session/mission_session_manager.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {
namespace {

localization::LocalizationEstimate Localization() {
  localization::LocalizationEstimate localization;
  localization.mutable_header()->set_frame_id("map");
  localization.mutable_header()->set_timestamp_sec(10.0);
  localization.mutable_pose()->mutable_position()->set_x(1.0);
  localization.mutable_pose()->mutable_position()->set_y(2.0);
  localization.mutable_pose()->mutable_position()->set_z(0.0);
  localization.mutable_pose()->set_heading(0.5);
  return localization;
}

MissionPlan Plan(MissionTaskType task_type) {
  MissionPlan plan;
  plan.set_task_type(task_type);
  plan.mutable_start()->set_current_pose_at_acceptance(true);
  plan.mutable_goal()->mutable_goal_pose()->set_x(10.0);
  plan.mutable_goal()->mutable_goal_pose()->set_y(20.0);
  plan.mutable_goal()->mutable_goal_pose()->set_z(0.0);
  plan.mutable_completion()->set_timeout_sec(120.0);
  plan.set_preemptible(true);
  return plan;
}

MissionDirective Activate(uint64_t revision,
                          MissionTaskType task_type = MISSION_TASK_A_TO_B) {
  MissionDirective directive;
  directive.mutable_identity()->set_producer_epoch("mission-boot-1");
  directive.mutable_identity()->set_aggregate_id("mission-1");
  directive.mutable_identity()->set_command_id("move");
  directive.mutable_identity()->set_revision(revision);
  directive.mutable_activate()->mutable_plan()->CopyFrom(Plan(task_type));
  return directive;
}

MissionDirective Replace(const MissionCommandIdentity& expected,
                         uint64_t revision) {
  auto directive = Activate(revision);
  directive.clear_activate();
  directive.mutable_replace()->mutable_expected_active_identity()->CopyFrom(
      expected);
  directive.mutable_replace()->mutable_plan()->CopyFrom(
      Plan(MISSION_TASK_PARK_IN));
  return directive;
}

MissionDirective Cancel(const MissionCommandIdentity& expected,
                        uint64_t revision) {
  auto directive = Activate(revision);
  directive.clear_activate();
  directive.mutable_cancel()->mutable_expected_active_identity()->CopyFrom(
      expected);
  directive.mutable_cancel()->set_postcondition(
      MISSION_CANCEL_CONTROLLED_STOP_THEN_HOLD);
  directive.mutable_cancel()->set_reason("operator cancelled");
  return directive;
}

MotionTerminalEvidence HoldEvidence(
    const MissionCommandIdentity& parent) {
  MotionTerminalEvidence evidence;
  evidence.set_contract_version(1);
  evidence.set_kind(MOTION_TERMINAL_EVIDENCE_STANDSTILL_HOLD);
  evidence.mutable_motion_identity()->set_producer_epoch("planning-boot");
  evidence.mutable_motion_identity()->set_aggregate_id("idle-hold");
  evidence.mutable_motion_identity()->set_command_id("hold");
  evidence.mutable_motion_identity()->set_revision(1);
  evidence.mutable_parent_mission_identity()->CopyFrom(parent);
  evidence.set_authority_generation(parent.revision());
  evidence.set_observed_at_sec(13.0);
  evidence.set_reference_frame_id("map");
  evidence.set_position_error_m(0.0);
  evidence.set_heading_error_rad(0.0);
  evidence.set_absolute_speed_mps(0.0);
  evidence.set_settled_duration_sec(0.2);
  evidence.set_executor_ownership(MOTION_EXECUTOR_OWNERSHIP_ACTIVE);
  evidence.set_executor_type(MOTION_EXECUTION_TYPE_PRIMITIVE);
  evidence.set_safety_state(MOTION_EVIDENCE_SAFETY_NORMAL);
  return evidence;
}

TEST(MissionSessionManagerTest, PersistsAcceptanceSnapshotAcrossDuplicate) {
  MissionSessionManager manager;
  const auto directive = Activate(1);
  ASSERT_TRUE(manager.Apply(directive, Localization(), 11.0).accepted);
  const auto snapshot = manager.guidance().accepted_start;

  auto moved = Localization();
  moved.mutable_pose()->mutable_position()->set_x(100.0);
  const auto duplicate = manager.Apply(directive, moved, 12.0);

  EXPECT_TRUE(duplicate.accepted);
  EXPECT_EQ(duplicate.code, MissionAdmissionCode::kDuplicate);
  EXPECT_EQ(manager.guidance().accepted_start.SerializeAsString(),
            snapshot.SerializeAsString());
}

TEST(MissionSessionManagerTest, DeclaredTaskProfilesDoNotImplySupport) {
  MissionSessionManager manager;
  ASSERT_TRUE(manager.Apply(Activate(1), Localization(), 11.0).accepted);
  for (const auto task :
       {MISSION_TASK_PARK_OUT, MISSION_TASK_ESCAPE,
        MISSION_TASK_ROTATE_IN_PLACE, MISSION_TASK_REACH_POSE,
        MISSION_TASK_STANDSTILL_WAIT}) {
    auto directive = Replace(manager.guidance().identity, 2);
    directive.mutable_replace()->mutable_plan()->set_task_type(task);
    EXPECT_EQ(manager.Apply(directive, Localization(), 12.0).code,
              MissionAdmissionCode::kUnsupportedOperation);
    EXPECT_EQ(manager.guidance().identity.revision(), 1);
    EXPECT_EQ(manager.guidance().plan.task_type(), MISSION_TASK_A_TO_B);
  }
  auto directive = Replace(manager.guidance().identity, 2);
  directive.mutable_replace()->mutable_plan()->set_travel_permission(
      MISSION_TRAVEL_FORWARD_ONLY);
  EXPECT_EQ(manager.Apply(directive, Localization(), 12.0).code,
            MissionAdmissionCode::kUnsupportedOperation);
  directive.mutable_replace()->mutable_plan()->clear_travel_permission();
  directive.mutable_replace()->mutable_plan()->mutable_goal()
      ->set_reference_frame_id("other-frame");
  EXPECT_EQ(manager.Apply(directive, Localization(), 12.0).code,
            MissionAdmissionCode::kUnsupportedOperation);
  EXPECT_EQ(manager.guidance().identity.revision(), 1);
}

TEST(MissionSessionManagerTest, ReplacesOnlyExactActiveRevision) {
  MissionSessionManager manager;
  ASSERT_TRUE(manager.Apply(Activate(1), Localization(), 11.0).accepted);
  auto wrong = manager.guidance().identity;
  wrong.set_revision(2);

  EXPECT_EQ(manager.Apply(Replace(wrong, 2), Localization(), 12.0).code,
            MissionAdmissionCode::kCasMismatch);
  const auto accepted = manager.Apply(Replace(manager.guidance().identity, 2),
                                      Localization(), 12.0);
  EXPECT_TRUE(accepted.accepted);
  EXPECT_EQ(manager.guidance().plan.task_type(), MISSION_TASK_PARK_IN);
  EXPECT_EQ(manager.guidance().identity.revision(), 2u);
}

TEST(MissionSessionManagerTest, DuplicateIdentityCannotChangePlan) {
  MissionSessionManager manager;
  auto directive = Activate(1);
  ASSERT_TRUE(manager.Apply(directive, Localization(), 11.0).accepted);
  directive.mutable_activate()->mutable_plan()->set_task_type(
      MISSION_TASK_PARK_IN);
  EXPECT_FALSE(manager.Apply(directive, Localization(), 11.1).accepted);
  EXPECT_EQ(manager.guidance().plan.task_type(), MISSION_TASK_A_TO_B);
}

TEST(MissionSessionManagerTest, PreservesMissionPolicyWhenLowering) {
  MissionSessionManager manager;
  auto directive = Activate(1, MISSION_TASK_PARK_IN);
  auto* plan = directive.mutable_activate()->mutable_plan();
  plan->set_preferred_mode(MODE_OPEN_SPACE);
  plan->set_priority(7);
  plan->add_tags("park");
  plan->mutable_recovery()->set_retry_budget(3);
  ASSERT_TRUE(manager.Apply(directive, Localization(), 11.0).accepted);
  const auto command = manager.BuildPlanningCommand();
  EXPECT_EQ(command.requested_scene(), SCENE_PARK_IN);
  EXPECT_EQ(command.preferred_mode(), MODE_OPEN_SPACE);
  EXPECT_EQ(command.priority(), 7);
  EXPECT_EQ(command.recovery().retry_budget(), 3);
  EXPECT_EQ(command.completion().SerializeAsString(),
            plan->completion().SerializeAsString());
  EXPECT_EQ(command.tags(0), "park");
}

TEST(MissionSessionManagerTest, CancellationRequiresTerminalMotionEvidence) {
  MissionSessionManager manager;
  ASSERT_TRUE(manager.Apply(Activate(1), Localization(), 11.0).accepted);
  ASSERT_TRUE(manager.MarkExecuting().accepted);
  ASSERT_TRUE(
      manager
          .Apply(Cancel(manager.guidance().identity, 2), Localization(), 12.0)
          .accepted);
  EXPECT_TRUE(manager.guidance().cancellation_fenced);
  EXPECT_EQ(manager.guidance().state, MISSION_SESSION_CANCELLING);
  EXPECT_EQ(manager.last_accepted_directive_identity().revision(), 2);

  auto invalid_evidence = HoldEvidence(manager.guidance().identity);
  invalid_evidence.set_authority_generation(99);
  EXPECT_FALSE(manager.ConfirmCancellation(invalid_evidence).accepted);
  const auto evidence = HoldEvidence(manager.guidance().identity);
  EXPECT_TRUE(manager.ConfirmCancellation(evidence).accepted);
  EXPECT_EQ(manager.guidance().state, MISSION_SESSION_CANCELLED);
  EXPECT_FALSE(manager.HasActiveSession());
}

TEST(MissionSessionManagerTest, DeclaredSettlingCannotReplaceLegacySession) {
  MissionSessionManager manager;
  ASSERT_TRUE(manager.Apply(Activate(1), Localization(), 11.0).accepted);
  ASSERT_TRUE(manager.MarkExecuting().accepted);
  const auto identity = manager.guidance().identity;
  auto directive = Replace(identity, 2);
  auto* completion = directive.mutable_replace()->mutable_plan()
                         ->mutable_completion();
  completion->set_speed_tolerance_mps(0.0);
  EXPECT_EQ(manager.Apply(directive, Localization(), 12.0).code,
            MissionAdmissionCode::kUnsupportedOperation);
  completion->clear_speed_tolerance_mps();
  completion->set_angular_speed_tolerance_radps(0.0);
  EXPECT_EQ(manager.Apply(directive, Localization(), 12.0).code,
            MissionAdmissionCode::kUnsupportedOperation);
  completion->clear_angular_speed_tolerance_radps();
  completion->set_settle_time_sec(0.5);
  EXPECT_EQ(manager.Apply(directive, Localization(), 12.0).code,
            MissionAdmissionCode::kUnsupportedOperation);
  EXPECT_EQ(manager.guidance().state, MISSION_SESSION_EXECUTING);
  EXPECT_EQ(manager.guidance().identity.SerializeAsString(),
            identity.SerializeAsString());
  completion->clear_settle_time_sec();
  EXPECT_TRUE(manager.Apply(directive, Localization(), 12.0).accepted);
}

TEST(MissionSessionManagerTest, CompletionRequiresCorrelatedHoldEvidence) {
  MissionSessionManager manager;
  ASSERT_TRUE(manager.Apply(Activate(1), Localization(), 11.0).accepted);
  ASSERT_TRUE(manager.MarkExecuting().accepted);
  ASSERT_TRUE(manager.BeginCompleting().accepted);

  auto evidence = HoldEvidence(manager.guidance().identity);
  evidence.mutable_parent_mission_identity()->set_revision(2);
  EXPECT_FALSE(manager.Complete(evidence).accepted);
  EXPECT_EQ(manager.guidance().state, MISSION_SESSION_COMPLETING);

  evidence = HoldEvidence(manager.guidance().identity);
  EXPECT_TRUE(manager.Complete(evidence).accepted);
  EXPECT_EQ(manager.guidance().state, MISSION_SESSION_COMPLETED);
}

TEST(MissionSessionManagerTest, RejectsAngularHoldEvidenceWithoutTransition) {
  MissionSessionManager manager;
  ASSERT_TRUE(manager.Apply(Activate(1), Localization(), 11.0).accepted);
  ASSERT_TRUE(manager.MarkExecuting().accepted);
  ASSERT_TRUE(manager.BeginCompleting().accepted);
  const auto identity = manager.guidance().identity;
  auto evidence = HoldEvidence(identity);
  evidence.set_absolute_angular_speed_radps(0.0);
  EXPECT_FALSE(manager.Complete(evidence).accepted);
  evidence.clear_absolute_angular_speed_radps();
  evidence.mutable_rotation_progress();
  EXPECT_FALSE(manager.Complete(evidence).accepted);
  evidence.clear_rotation_progress();
  evidence.set_contract_version(2);
  EXPECT_FALSE(manager.Complete(evidence).accepted);
  EXPECT_EQ(manager.guidance().state, MISSION_SESSION_COMPLETING);
  EXPECT_EQ(manager.guidance().identity.SerializeAsString(),
            identity.SerializeAsString());
  evidence.set_contract_version(1);
  EXPECT_TRUE(manager.Complete(evidence).accepted);
}

TEST(MissionSessionManagerTest, AngularEvidenceCannotConfirmCancellation) {
  MissionSessionManager manager;
  ASSERT_TRUE(manager.Apply(Activate(1), Localization(), 11.0).accepted);
  ASSERT_TRUE(manager.MarkExecuting().accepted);
  ASSERT_TRUE(manager.Apply(Cancel(manager.guidance().identity, 2),
                            Localization(), 12.0).accepted);
  auto evidence = HoldEvidence(manager.guidance().identity);
  evidence.set_absolute_angular_speed_radps(0.0);
  EXPECT_FALSE(manager.ConfirmCancellation(evidence).accepted);
  evidence.clear_absolute_angular_speed_radps();
  evidence.mutable_rotation_progress();
  EXPECT_FALSE(manager.ConfirmCancellation(evidence).accepted);
  EXPECT_EQ(manager.guidance().state, MISSION_SESSION_CANCELLING);
  evidence.clear_rotation_progress();
  EXPECT_TRUE(manager.ConfirmCancellation(evidence).accepted);
}

TEST(MissionSessionManagerTest,
     UnimplementedSessionControlsRejectWithoutMutation) {
  MissionSessionManager manager;
  ASSERT_TRUE(manager.Apply(Activate(1), Localization(), 11.0).accepted);
  ASSERT_TRUE(manager.MarkExecuting().accepted);
  const auto identity = manager.guidance().identity;
  auto directive = Activate(2);
  directive.clear_activate();
  directive.mutable_suspend()->mutable_expected_active_identity()->CopyFrom(
      identity);
  EXPECT_EQ(manager.Apply(directive, Localization(), 12.0).code,
            MissionAdmissionCode::kUnsupportedOperation);
  directive.clear_suspend();
  directive.mutable_resume()->mutable_expected_active_identity()->CopyFrom(
      identity);
  EXPECT_EQ(manager.Apply(directive, Localization(), 12.0).code,
            MissionAdmissionCode::kUnsupportedOperation);
  EXPECT_EQ(manager.guidance().state, MISSION_SESSION_EXECUTING);
  EXPECT_EQ(manager.guidance().identity.SerializeAsString(),
            identity.SerializeAsString());
  EXPECT_EQ(manager.last_accepted_directive_identity().revision(), 1);
}

TEST(MissionSessionManagerTest, ReplacementAndCancellationAreIdempotent) {
  MissionSessionManager manager;
  ASSERT_TRUE(manager.Apply(Activate(1), Localization(), 11.0).accepted);
  const auto replace = Replace(manager.guidance().identity, 2);
  ASSERT_TRUE(manager.Apply(replace, Localization(), 12.0).accepted);
  EXPECT_EQ(manager.Apply(replace, Localization(), 12.1).code,
            MissionAdmissionCode::kDuplicate);
  const auto cancel = Cancel(manager.guidance().identity, 3);
  ASSERT_TRUE(manager.Apply(cancel, Localization(), 12.2).accepted);
  EXPECT_EQ(manager.Apply(cancel, Localization(), 12.3).code,
            MissionAdmissionCode::kDuplicate);
  auto changed = cancel;
  changed.mutable_cancel()->set_reason("changed");
  EXPECT_FALSE(manager.Apply(changed, Localization(), 12.4).accepted);
  EXPECT_EQ(manager.guidance().state, MISSION_SESSION_CANCELLING);
}

TEST(MissionSessionManagerTest, CancellationFenceCannotBeRevivedByReplace) {
  MissionSessionManager manager;
  ASSERT_TRUE(manager.Apply(Activate(1), Localization(), 11.0).accepted);
  const auto active = manager.guidance().identity;
  ASSERT_TRUE(manager.Apply(Cancel(active, 2), Localization(), 12.0).accepted);

  const auto result = manager.Apply(Replace(active, 2), Localization(), 12.1);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.code, MissionAdmissionCode::kInvalidTransition);
  EXPECT_TRUE(manager.guidance().cancellation_fenced);
  EXPECT_EQ(manager.guidance().state, MISSION_SESSION_CANCELLING);
}

TEST(MissionSessionManagerTest, CriticalSuspensionPreservesSession) {
  MissionSessionManager manager;
  ASSERT_TRUE(manager.Apply(Activate(1), Localization(), 11.0).accepted);
  ASSERT_TRUE(manager.MarkExecuting().accepted);
  ASSERT_TRUE(manager.Suspend("critical preemption").accepted);
  EXPECT_EQ(manager.guidance().state, MISSION_SESSION_SUSPENDED);
  EXPECT_TRUE(manager.HasActiveSession());
  ASSERT_TRUE(manager.Resume().accepted);
  EXPECT_EQ(manager.guidance().state, MISSION_SESSION_EXECUTING);
}

TEST(MissionSessionManagerTest, PersistsCorrelatedRouteAndPhase) {
  MissionSessionManager manager;
  ASSERT_TRUE(manager.Apply(Activate(1), Localization(), 11.0).accepted);
  const auto identity = manager.guidance().identity;

  MissionRouteContext requested;
  requested.set_request_id("route-request-1");
  requested.set_state(MISSION_ROUTE_REQUESTED);
  ASSERT_TRUE(manager.UpdateRoute(identity, requested).accepted);
  EXPECT_EQ(manager.guidance().phase, MISSION_PHASE_ROUTING);

  MissionRouteContext ready = requested;
  ready.set_state(MISSION_ROUTE_READY);
  ready.set_map_version("map-v1");
  ready.set_route_id("route-1");
  ASSERT_TRUE(manager.UpdateRoute(identity, ready).accepted);
  EXPECT_EQ(manager.guidance().phase, MISSION_PHASE_ENROUTE);
  EXPECT_EQ(manager.guidance().route.route_id(), "route-1");

  ASSERT_TRUE(manager.AdvancePhase(identity, MISSION_PHASE_APPROACH).accepted);
  ASSERT_TRUE(manager.AdvancePhase(identity, MISSION_PHASE_SETTLING).accepted);
  EXPECT_EQ(manager.guidance().phase, MISSION_PHASE_SETTLING);
}

TEST(MissionSessionManagerTest, RejectsRouteFromDifferentMissionRevision) {
  MissionSessionManager manager;
  ASSERT_TRUE(manager.Apply(Activate(1), Localization(), 11.0).accepted);
  auto wrong = manager.guidance().identity;
  wrong.set_revision(2);
  MissionRouteContext route;
  route.set_request_id("route-request-1");
  route.set_state(MISSION_ROUTE_REQUESTED);

  EXPECT_EQ(manager.UpdateRoute(wrong, route).code,
            MissionAdmissionCode::kCasMismatch);
}

TEST(MissionSessionManagerTest, RejectsExplicitStartMismatch) {
  MissionSessionManager manager;
  auto directive = Activate(1);
  auto* start = directive.mutable_activate()->mutable_plan()->mutable_start();
  start->clear_current_pose_at_acceptance();
  auto* explicit_start = start->mutable_explicit_start();
  explicit_start->mutable_position()->set_x(50.0);
  explicit_start->mutable_position()->set_y(50.0);
  explicit_start->set_heading(0.5);
  explicit_start->set_reference_frame_id("map");
  explicit_start->set_max_position_error_m(0.5);
  explicit_start->set_max_heading_error_rad(0.1);

  const auto result = manager.Apply(directive, Localization(), 11.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.code, MissionAdmissionCode::kInvalidStart);
}

}  // namespace
}  // namespace planning
}  // namespace apollo
