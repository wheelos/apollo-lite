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

TEST(MissionSessionManagerTest, ReplacesOnlyExactActiveRevision) {
  MissionSessionManager manager;
  ASSERT_TRUE(manager.Apply(Activate(1), Localization(), 11.0).accepted);
  auto wrong = manager.guidance().identity;
  wrong.set_revision(2);

  EXPECT_EQ(manager.Apply(Replace(wrong, 2), Localization(), 12.0).code,
            MissionAdmissionCode::kCasMismatch);
  const auto accepted =
      manager.Apply(Replace(manager.guidance().identity, 2), Localization(),
                    12.0);
  EXPECT_TRUE(accepted.accepted);
  EXPECT_EQ(manager.guidance().plan.task_type(), MISSION_TASK_PARK_IN);
  EXPECT_EQ(manager.guidance().identity.revision(), 2u);
}

TEST(MissionSessionManagerTest, CancellationRequiresTerminalMotionEvidence) {
  MissionSessionManager manager;
  ASSERT_TRUE(manager.Apply(Activate(1), Localization(), 11.0).accepted);
  ASSERT_TRUE(manager.MarkExecuting().accepted);
  ASSERT_TRUE(manager
                  .Apply(Cancel(manager.guidance().identity, 2),
                         Localization(), 12.0)
                  .accepted);
  EXPECT_TRUE(manager.guidance().cancellation_fenced);
  EXPECT_EQ(manager.guidance().state, MISSION_SESSION_CANCELLING);
  EXPECT_EQ(manager.last_accepted_directive_identity().revision(), 2);

  EXPECT_FALSE(manager.ConfirmCancellation(false).accepted);
  EXPECT_TRUE(manager.ConfirmCancellation(true).accepted);
  EXPECT_EQ(manager.guidance().state, MISSION_SESSION_CANCELLED);
  EXPECT_FALSE(manager.HasActiveSession());
}

TEST(MissionSessionManagerTest, CancellationFenceCannotBeRevivedByReplace) {
  MissionSessionManager manager;
  ASSERT_TRUE(manager.Apply(Activate(1), Localization(), 11.0).accepted);
  const auto active = manager.guidance().identity;
  ASSERT_TRUE(
      manager.Apply(Cancel(active, 2), Localization(), 12.0).accepted);

  const auto result =
      manager.Apply(Replace(active, 2), Localization(), 12.1);
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

  ASSERT_TRUE(
      manager.AdvancePhase(identity, MISSION_PHASE_APPROACH).accepted);
  ASSERT_TRUE(
      manager.AdvancePhase(identity, MISSION_PHASE_SETTLING).accepted);
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
  auto* start =
      directive.mutable_activate()->mutable_plan()->mutable_start();
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
