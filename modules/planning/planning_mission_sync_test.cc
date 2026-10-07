#include <unistd.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "gtest/gtest.h"

#include "cyber/init.h"
#include "cyber/time/clock.h"
#include "modules/planning/planning_component.h"

namespace apollo {
namespace planning {

class PlanningMissionSyncTestPeer {
 public:
  static std::string CheckInput(const PlanningComponent& component,
                                const PlanningCoordinatorState& preview_state) {
    return component.CheckInput(preview_state);
  }

  static void SetReadinessInputs(PlanningComponent* component,
                                 bool localization_ready, bool chassis_ready) {
    if (localization_ready) {
      component->local_view_.localization_estimate =
          std::make_shared<localization::LocalizationEstimate>();
    }
    if (chassis_ready) {
      component->local_view_.chassis = std::make_shared<canbus::Chassis>();
    }
  }

  static execution_state_sync::Result Init(PlanningComponent* component,
                                           const std::string& path,
                                           const std::string& node_name) {
    component->node_ = cyber::CreateNode(node_name);
    component->planning_coordinator_ =
        std::make_unique<PlanningCoordinator>(nullptr);
    component->execution_state_transport_ =
        std::make_unique<PlanningExecutionStateTransport>();
    return component->execution_state_transport_->Init(path, node_name);
  }

  static bool Poll(PlanningComponent* component,
                   const localization::LocalizationEstimate& localization) {
    return component->PollExecutionState(localization);
  }

  static void Prepare(PlanningComponent* component,
                      const MissionDirective& directive, uint64_t sequence,
                      const localization::LocalizationEstimate& localization) {
    component->mission_directive_ = directive;
    component->mission_event_sequence_ = sequence;
    component->ApplyPendingMissionDirective(localization);
  }

  static MissionAdmissionResult MarkExecuting(PlanningComponent* component) {
    return component->planning_coordinator_->MarkMissionExecuting();
  }

  static MotionDirective BuildMotion(
      PlanningComponent* component,
      const localization::LocalizationEstimate& localization) {
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
    component->motion_plan_builder_.SetSpatialEnvelope(envelope);

    PlanningCoordinatorState state;
    const auto& guidance =
        component->planning_coordinator_->mission_session_manager().guidance();
    state.mission_identity.CopyFrom(guidance.identity);
    state.mission_session_state = guidance.state;
    PlanningSemanticSummary semantics;
    canbus::Chassis chassis;
    chassis.set_speed_mps(0.0);
    chassis.set_gear_location(canbus::Chassis::GEAR_DRIVE);
    ADCTrajectory trajectory;
    trajectory.set_gear(canbus::Chassis::GEAR_DRIVE);
    trajectory.set_total_path_time(1.0);
    auto* intent = trajectory.mutable_control_intent();
    intent->set_tracking_mode(TRACKING_MODE_TRAJECTORY);
    intent->set_longitudinal_intent(LON_INTENT_CRUISE);
    intent->set_lateral_intent(LAT_INTENT_TRACK_PATH);
    intent->set_execution_channel(EXECUTION_CHANNEL_TRAJECTORY);
    intent->set_primitive_type(CONTROL_PRIMITIVE_NONE);
    for (int index = 0; index < 2; ++index) {
      auto* point = trajectory.add_trajectory_point();
      point->mutable_path_point()->set_x(1.0 + index);
      point->mutable_path_point()->set_y(2.0);
      point->mutable_path_point()->set_z(0.0);
      point->mutable_path_point()->set_theta(0.5);
      point->mutable_path_point()->set_s(index);
      point->mutable_path_point()->set_kappa(0.0);
      point->mutable_path_point()->set_dkappa(0.0);
      point->set_v(index == 0 ? 0.1 : 0.0);
      point->set_a(0.0);
      point->set_relative_time(index);
    }
    const double now = cyber::Clock::NowInSeconds();
    return component->motion_plan_builder_
        .Build(state, semantics, chassis, localization, trajectory, now)
        .directive;
  }

  static bool SubmitMotion(PlanningComponent* component,
                           const MotionDirective& directive) {
    return component->SubmitExecutionState(
        execution_state_sync::Channel::kMotion, directive.SerializeAsString(),
        false);
  }

  static bool SubmitIdleRuntimeStatus(PlanningComponent* component) {
    PlanningRuntimeStatus status;
    status.set_state(RUNTIME_IDLE);
    status.set_reason("awaiting a live Mission authorization");
    return component->SubmitExecutionState(
        execution_state_sync::Channel::kPlanningStatus,
        status.SerializeAsString(), false);
  }

  static PlanningCommand BuildPlanningCommand(
      const PlanningComponent& component) {
    return component.planning_coordinator_->mission_session_manager()
        .BuildPlanningCommand();
  }

  static const MissionSessionManager& Session(
      const PlanningComponent& component) {
    return component.planning_coordinator_->mission_session_manager();
  }

  static bool Ready(const PlanningComponent& component) {
    return component.execution_state_transport_->Ready();
  }

  static bool Faulted(const PlanningComponent& component) {
    return component.execution_state_fault_;
  }

  static size_t Pending(const PlanningComponent& component) {
    return component.pending_mission_admissions_;
  }

  static uint64_t Cursor(const PlanningComponent& component) {
    return component.execution_state_transport_->cursor();
  }

  static uint64_t MotionSequence(const PlanningComponent& component) {
    return component.motion_event_sequence_;
  }

  static std::shared_ptr<const execution_state_sync::WorkerView> Latest(
      const PlanningComponent& component) {
    return component.execution_state_transport_->Latest();
  }
};

namespace {

template <typename Predicate>
bool Await(Predicate predicate) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  do {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}

MissionDirective Activate(uint64_t revision) {
  MissionDirective directive;
  auto* identity = directive.mutable_identity();
  identity->set_producer_epoch("mission-sync-test");
  identity->set_aggregate_id("mission");
  identity->set_command_id("move");
  identity->set_revision(revision);
  auto* plan = directive.mutable_activate()->mutable_plan();
  plan->set_task_type(MISSION_TASK_A_TO_B);
  plan->mutable_start()->set_current_pose_at_acceptance(true);
  plan->mutable_goal()->mutable_goal_pose()->set_x(10.0);
  plan->mutable_goal()->mutable_goal_pose()->set_y(20.0);
  plan->mutable_completion()->set_timeout_sec(120.0);
  plan->set_preemptible(true);
  return directive;
}

TEST(PlanningMissionAdmissionTest, StagingRequiresMatchingCommitIdentity) {
  PlanningCoordinator coordinator(nullptr);
  localization::LocalizationEstimate localization;
  localization.mutable_header()->set_frame_id("map");
  localization.mutable_header()->set_timestamp_sec(10.0);
  localization.mutable_pose()->mutable_position()->set_x(1.0);
  localization.mutable_pose()->mutable_position()->set_y(2.0);
  localization.mutable_pose()->mutable_position()->set_z(0.0);
  localization.mutable_pose()->set_heading(0.5);
  const auto directive = Activate(1);
  ASSERT_TRUE(coordinator.PrepareMissionDirective(directive, localization, 11.0)
                  .accepted);
  EXPECT_FALSE(
      coordinator.mission_session_manager().guidance().identity.has_revision());
  EXPECT_EQ(
      coordinator.PrepareMissionDirective(Activate(2), localization, 11.0).code,
      MissionAdmissionCode::kBusy);
  EXPECT_EQ(
      coordinator.ApplyMissionDirective(Activate(2), localization, 11.0).code,
      MissionAdmissionCode::kBusy);
  auto wrong_identity = directive.identity();
  wrong_identity.set_revision(2);
  EXPECT_FALSE(coordinator.CommitPreparedMissionDirective(wrong_identity));
  EXPECT_FALSE(
      coordinator.mission_session_manager().guidance().identity.has_revision());
  ASSERT_TRUE(coordinator.CommitPreparedMissionDirective(directive.identity()));
  EXPECT_EQ(
      coordinator.mission_session_manager().guidance().identity.revision(), 1);
  EXPECT_EQ(coordinator.prepared_mission_session_manager(), nullptr);
  auto cancel = Activate(2);
  cancel.clear_activate();
  cancel.mutable_cancel()->mutable_expected_active_identity()->CopyFrom(
      directive.identity());
  cancel.mutable_cancel()->set_postcondition(
      MISSION_CANCEL_CONTROLLED_STOP_THEN_HOLD);
  ASSERT_TRUE(
      coordinator.PrepareMissionDirective(cancel, localization, 12.0).accepted);
  coordinator.DiscardPreparedMissionDirective();
  EXPECT_EQ(coordinator.mission_session_manager().guidance().state,
            MISSION_SESSION_ACCEPTED);
  EXPECT_EQ(coordinator.mission_session_manager()
                .last_accepted_directive_identity()
                .revision(),
            1);
}

TEST(PlanningMissionAdmissionTest,
     ControlFailureIsReportedAsMissionFailureNotCancellation) {
  PlanningCoordinator coordinator(nullptr);
  localization::LocalizationEstimate localization;
  localization.mutable_header()->set_frame_id("map");
  localization.mutable_header()->set_timestamp_sec(10.0);
  localization.mutable_pose()->mutable_position()->set_x(1.0);
  localization.mutable_pose()->mutable_position()->set_y(2.0);
  localization.mutable_pose()->mutable_position()->set_z(0.0);
  localization.mutable_pose()->set_heading(0.5);

  const auto directive = Activate(1);
  ASSERT_TRUE(
      coordinator.ApplyMissionDirective(directive, localization, 11.0).accepted);
  ASSERT_TRUE(coordinator.MarkMissionExecuting().accepted);

  const auto failure = coordinator.FailMission("Control execution timed out");
  ASSERT_TRUE(failure.accepted) << failure.reason;
  const auto& guidance = coordinator.mission_session_manager().guidance();
  EXPECT_EQ(guidance.state, MISSION_SESSION_FAILED);
  EXPECT_FALSE(guidance.cancellation_fenced);
  EXPECT_EQ(guidance.identity.SerializeAsString(),
            directive.identity().SerializeAsString());
}

TEST(PlanningComponentReadinessTest, ReportsMissingLocalization) {
  PlanningComponent component;
  PlanningCoordinatorState preview_state;
  EXPECT_EQ(PlanningMissionSyncTestPeer::CheckInput(component, preview_state),
            "localization not ready");
}

TEST(PlanningComponentReadinessTest, ReportsMissingChassis) {
  PlanningComponent component;
  PlanningMissionSyncTestPeer::SetReadinessInputs(&component, true, false);
  PlanningCoordinatorState preview_state;
  EXPECT_EQ(PlanningMissionSyncTestPeer::CheckInput(component, preview_state),
            "chassis not ready");
}

TEST(PlanningComponentReadinessTest, PreservesCoordinatorModeFailureReason) {
  PlanningComponent component;
  PlanningMissionSyncTestPeer::SetReadinessInputs(&component, true, true);
  PlanningCoordinatorState preview_state;
  preview_state.resolved_mode = MODE_UNKNOWN;
  preview_state.reason = "no executable shell";
  EXPECT_EQ(PlanningMissionSyncTestPeer::CheckInput(component, preview_state),
            "no executable shell");
}

TEST(PlanningComponentReadinessTest, RequiresRelativeMapForCorridorMode) {
  PlanningComponent component;
  PlanningMissionSyncTestPeer::SetReadinessInputs(&component, true, true);
  PlanningCoordinatorState preview_state;
  preview_state.resolved_mode = MODE_CORRIDOR;
  EXPECT_EQ(PlanningMissionSyncTestPeer::CheckInput(component, preview_state),
            "relative map not ready for mapless planning");
}

class PlanningMissionSyncTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    ASSERT_TRUE(cyber::Init("planning_mission_sync_test"));
  }

  static void TearDownTestSuite() { cyber::Clear(); }

  void SetUp() override {
    static std::atomic<unsigned> counter{0};
    const auto suffix =
        std::to_string(getpid()) + "_" + std::to_string(counter++);
    directory_ = std::filesystem::temp_directory_path() /
                 ("planning_mission_sync_" + suffix);
    ASSERT_TRUE(std::filesystem::create_directory(directory_));
    path_ = (directory_ / "state.db").string();
    component_ = std::make_unique<PlanningComponent>();
    const auto initialized = PlanningMissionSyncTestPeer::Init(
        component_.get(), path_, "planning_sync_" + suffix);
    ASSERT_TRUE(initialized.ok()) << initialized.message;
    ASSERT_TRUE(Await([this] {
      Poll();
      return PlanningMissionSyncTestPeer::Ready(*component_);
    }));
    execution_state_sync::Result opened;
    const bool open_completed = Await([this, &opened] {
      opened = execution_state_sync::Store::Open(
          {path_, execution_state_sync::Role::kMission}, &mission_);
      return opened.code != execution_state_sync::Code::kBusy ||
             opened.sqlite_code == 0;
    });
    ASSERT_TRUE(open_completed && opened.ok())
        << "code=" << static_cast<int>(opened.code)
        << " sqlite_code=" << opened.sqlite_code << " " << opened.message;
  }

  void TearDown() override {
    component_.reset();
    control_.reset();
    mission_.reset();
    std::filesystem::remove(path_ + "-shm");
    std::filesystem::remove(path_ + "-wal");
    std::filesystem::remove(path_);
    std::filesystem::remove(directory_);
  }

  localization::LocalizationEstimate Localization() const {
    localization::LocalizationEstimate localization;
    localization.mutable_header()->set_frame_id("map");
    localization.mutable_header()->set_timestamp_sec(
        cyber::Clock::NowInSeconds());
    localization.mutable_pose()->mutable_position()->set_x(1.0);
    localization.mutable_pose()->mutable_position()->set_y(2.0);
    localization.mutable_pose()->mutable_position()->set_z(0.0);
    localization.mutable_pose()->set_heading(0.5);
    return localization;
  }

  void Poll() {
    PlanningMissionSyncTestPeer::Poll(component_.get(), Localization());
  }

  execution_state_sync::Result Submit(const MissionDirective& directive,
                                      execution_state_sync::Commit* commit) {
    execution_state_sync::Operation operation;
    operation.operation_id =
        "mission-request-" + std::to_string(transport_revision_ + 1);
    operation.identity = {"mission-sync-test", "mission", "move",
                          transport_revision_ + 1};
    operation.expected_revision = transport_revision_;
    operation.payload = directive.SerializeAsString();
    operation.mission_fenced = directive.has_cancel();
    execution_state_sync::Result result;
    Await([&] {
      result = mission_->Submit(operation, commit);
      return result.code != execution_state_sync::Code::kBusy ||
             result.sqlite_code == 0;
    });
    if (result.ok()) {
      ++transport_revision_;
    }
    return result;
  }

  execution_state_sync::Result SubmitControlMotionStatus(
      const MotionDirective& directive, uint64_t mission_sequence,
      uint64_t motion_sequence, uint64_t revision,
      bool unrelated_parent, uint64_t* commit_sequence,
      MotionExecutionState state = MOTION_EXECUTION_FAILED,
      bool owner_runtime = false) {
    control::ControlRuntimeStatus status;
    status.mutable_header()->set_timestamp_sec(cyber::Clock::NowInSeconds());
    status.set_motion_scope(directive.scope());
    status.set_executor_owner_active(false);
    auto* motion = status.mutable_motion_execution();
    motion->mutable_identity()->CopyFrom(
        directive.execute().command().identity());
    motion->mutable_parent_mission_identity()->CopyFrom(
        directive.parent_mission_identity());
    if (unrelated_parent) {
      motion->mutable_parent_mission_identity()->set_command_id("unrelated");
    }
    motion->set_state(state);
    motion->set_reason("Control reported execution failure");

    execution_state_sync::Operation operation;
    operation.channel = execution_state_sync::Channel::kControlStatus;
    operation.operation_id = "control-status-" + std::to_string(revision);
    operation.identity = {"control-peer", "mission", "motion-status",
                          revision};
    operation.expected_revision = revision - 1;
    if (owner_runtime) {
      operation.control_status_kind =
          execution_state_sync::ControlStatusKind::kOwnerRuntime;
    } else {
      operation.guards = {
          {execution_state_sync::Channel::kMission, mission_sequence},
          {execution_state_sync::Channel::kMotion, motion_sequence}};
    }
    operation.payload = status.SerializeAsString();
    execution_state_sync::Commit commit;
    execution_state_sync::Result result;
    Await([&]() {
      result = control_->Submit(operation, &commit);
      return result.code != execution_state_sync::Code::kBusy ||
             result.sqlite_code == 0;
    });
    if (result.ok() && commit_sequence != nullptr) {
      *commit_sequence = commit.sequence;
    }
    return result;
  }

  void AcceptFirstMission() {
    execution_state_sync::Commit commit;
    const auto submitted = Submit(Activate(1), &commit);
    ASSERT_TRUE(submitted.ok()) << submitted.message;
    ASSERT_TRUE(Await([this] {
      Poll();
      return PlanningMissionSyncTestPeer::Session(*component_)
                 .guidance()
                 .identity.revision() == 1;
    }));
    ASSERT_FALSE(PlanningMissionSyncTestPeer::Faulted(*component_));
  }

  std::filesystem::path directory_;
  std::string path_;
  std::unique_ptr<PlanningComponent> component_;
  std::unique_ptr<execution_state_sync::Store> mission_;
  std::unique_ptr<execution_state_sync::Store> control_;
  uint64_t transport_revision_ = 0;
};

TEST_F(PlanningMissionSyncTest,
       SessionChangesOnlyAfterCommittedReceiptIsConsumed) {
  auto directive = Activate(1);
  directive.mutable_activate()->mutable_plan()->set_preferred_mode(
      MODE_CORRIDOR);
  execution_state_sync::Commit commit;
  const auto submitted = Submit(directive, &commit);
  ASSERT_TRUE(submitted.ok()) << submitted.message;
  PlanningMissionSyncTestPeer::Prepare(component_.get(), directive,
                                       commit.sequence, Localization());
  ASSERT_EQ(PlanningMissionSyncTestPeer::Pending(*component_), 1);
  EXPECT_FALSE(PlanningMissionSyncTestPeer::Session(*component_)
                   .guidance()
                   .identity.has_revision());
  ASSERT_TRUE(Await([this] {
    const auto view = PlanningMissionSyncTestPeer::Latest(*component_);
    return view && view->result.ok() && view->snapshot &&
           view->snapshot->accepted_authority.has_value();
  }));
  EXPECT_FALSE(PlanningMissionSyncTestPeer::Session(*component_)
                   .guidance()
                   .identity.has_revision());
  ASSERT_TRUE(Await([this] {
    Poll();
    return PlanningMissionSyncTestPeer::Session(*component_)
                   .guidance()
                   .identity.revision() == 1 &&
           PlanningMissionSyncTestPeer::Pending(*component_) == 0;
  }));
  EXPECT_EQ(PlanningMissionSyncTestPeer::BuildPlanningCommand(*component_)
                .preferred_mode(),
            MODE_CORRIDOR);
  EXPECT_FALSE(PlanningMissionSyncTestPeer::Faulted(*component_));
}

TEST_F(PlanningMissionSyncTest,
       PlanningCanPublishIdleOwnerStatusWithoutMissionAuthority) {
  ASSERT_TRUE(
      PlanningMissionSyncTestPeer::SubmitIdleRuntimeStatus(component_.get()));
  ASSERT_TRUE(Await([this] {
    Poll();
    const auto view = PlanningMissionSyncTestPeer::Latest(*component_);
    return view && view->result.ok() && view->snapshot &&
           view->snapshot
               ->latest[static_cast<size_t>(
                   execution_state_sync::Channel::kPlanningStatus)]
               .has_value();
  }));
  const auto view = PlanningMissionSyncTestPeer::Latest(*component_);
  ASSERT_TRUE(view->snapshot->latest[static_cast<size_t>(
      execution_state_sync::Channel::kPlanningStatus)]);
  const auto& status_event = *view->snapshot->latest[static_cast<size_t>(
      execution_state_sync::Channel::kPlanningStatus)];
  EXPECT_TRUE(status_event.operation.guards.empty());
  EXPECT_FALSE(view->snapshot->accepted_authority);
  EXPECT_FALSE(PlanningMissionSyncTestPeer::Faulted(*component_));
}

TEST_F(PlanningMissionSyncTest, FailedAdmissionCommitPreservesAcceptedSession) {
  AcceptFirstMission();
  ASSERT_FALSE(HasFatalFailure());
  const auto original =
      PlanningMissionSyncTestPeer::Session(*component_).guidance();
  auto replacement = Activate(2);
  replacement.clear_activate();
  replacement.mutable_replace()->mutable_expected_active_identity()->CopyFrom(
      original.identity);
  replacement.mutable_replace()->mutable_plan()->CopyFrom(original.plan);
  replacement.mutable_replace()
      ->mutable_plan()
      ->mutable_goal()
      ->mutable_goal_pose()
      ->set_x(100.0);
  execution_state_sync::Commit replacement_commit;
  auto submitted = Submit(replacement, &replacement_commit);
  ASSERT_TRUE(submitted.ok()) << submitted.message;
  execution_state_sync::Commit newer_commit;
  submitted = Submit(Activate(3), &newer_commit);
  ASSERT_TRUE(submitted.ok()) << submitted.message;
  PlanningMissionSyncTestPeer::Prepare(component_.get(), replacement,
                                       replacement_commit.sequence,
                                       Localization());
  EXPECT_EQ(PlanningMissionSyncTestPeer::Session(*component_)
                .guidance()
                .identity.SerializeAsString(),
            original.identity.SerializeAsString());
  ASSERT_TRUE(Await([this] {
    Poll();
    return PlanningMissionSyncTestPeer::Faulted(*component_);
  }));
  EXPECT_EQ(PlanningMissionSyncTestPeer::Pending(*component_), 0);
  const auto& after =
      PlanningMissionSyncTestPeer::Session(*component_).guidance();
  EXPECT_EQ(after.identity.SerializeAsString(),
            original.identity.SerializeAsString());
  EXPECT_EQ(after.plan.SerializeAsString(), original.plan.SerializeAsString());
  EXPECT_EQ(after.accepted_start.SerializeAsString(),
            original.accepted_start.SerializeAsString());
}

TEST_F(PlanningMissionSyncTest, ProcessesQueuedIntentsWithoutSkippingReceipts) {
  execution_state_sync::Commit first_commit;
  auto submitted = Submit(Activate(1), &first_commit);
  ASSERT_TRUE(submitted.ok()) << submitted.message;
  execution_state_sync::Commit second_commit;
  submitted = Submit(Activate(2), &second_commit);
  ASSERT_TRUE(submitted.ok()) << submitted.message;
  ASSERT_TRUE(Await([this, &second_commit] {
    const auto view = PlanningMissionSyncTestPeer::Latest(*component_);
    return view && view->result.ok() && view->snapshot &&
           view->snapshot->sequence >= second_commit.sequence;
  }));
  ASSERT_TRUE(Await([this, &second_commit] {
    Poll();
    return PlanningMissionSyncTestPeer::Cursor(*component_) >=
               second_commit.sequence &&
           PlanningMissionSyncTestPeer::Session(*component_)
                   .guidance()
                   .identity.revision() == 2;
  }));
  EXPECT_FALSE(PlanningMissionSyncTestPeer::Faulted(*component_));
  execution_state_sync::EventBatch batch;
  const auto read = mission_->ReadEvents(0, 64, &batch);
  ASSERT_TRUE(read.ok()) << read.message;
  size_t receipts = 0;
  for (const auto& event : batch.events) {
    if (event.operation.channel !=
        execution_state_sync::Channel::kPlanningStatus) {
      continue;
    }
    PlanningRuntimeStatus status;
    ASSERT_TRUE(status.ParseFromString(event.operation.payload));
    ASSERT_TRUE(status.has_admission_directive_identity());
    ASSERT_EQ(event.operation.guards.size(), 1);
    ++receipts;
    if (status.admission_directive_identity().revision() == 1) {
      EXPECT_EQ(status.mission_admission_outcome(), MISSION_ADMISSION_REJECTED);
      EXPECT_EQ(event.operation.guards.front().sequence, first_commit.sequence);
    } else {
      EXPECT_EQ(status.admission_directive_identity().revision(), 2);
      EXPECT_EQ(status.mission_admission_outcome(), MISSION_ADMISSION_ACCEPTED);
      EXPECT_EQ(event.operation.guards.front().sequence,
                second_commit.sequence);
    }
  }
  EXPECT_EQ(receipts, 2);
}

TEST_F(PlanningMissionSyncTest,
       CancelKeepsTaskIdentityUntilPhysicalRetirement) {
  AcceptFirstMission();
  ASSERT_FALSE(HasFatalFailure());
  const auto identity =
      PlanningMissionSyncTestPeer::Session(*component_).guidance().identity;
  auto cancel = Activate(2);
  cancel.clear_activate();
  cancel.mutable_cancel()->mutable_expected_active_identity()->CopyFrom(
      identity);
  cancel.mutable_cancel()->set_postcondition(
      MISSION_CANCEL_CONTROLLED_STOP_THEN_HOLD);
  execution_state_sync::Commit commit;
  const auto submitted = Submit(cancel, &commit);
  ASSERT_TRUE(submitted.ok()) << submitted.message;
  ASSERT_TRUE(Await([this] {
    Poll();
    return PlanningMissionSyncTestPeer::Session(*component_)
               .last_accepted_directive_identity()
               .revision() == 2;
  }));
  const auto& guidance =
      PlanningMissionSyncTestPeer::Session(*component_).guidance();
  EXPECT_EQ(guidance.identity.SerializeAsString(),
            identity.SerializeAsString());
  EXPECT_EQ(guidance.state, MISSION_SESSION_CANCELLING);
  EXPECT_TRUE(guidance.cancellation_fenced);
  EXPECT_FALSE(PlanningMissionSyncTestPeer::Faulted(*component_));
}

TEST_F(PlanningMissionSyncTest,
       ControlFailureNeedsMatchingMissionAndMotionIdentities) {
  AcceptFirstMission();
  ASSERT_FALSE(HasFatalFailure());
  ASSERT_TRUE(PlanningMissionSyncTestPeer::MarkExecuting(component_.get())
                  .accepted);
  const auto localization = Localization();
  const auto motion_directive =
      PlanningMissionSyncTestPeer::BuildMotion(component_.get(), localization);
  ASSERT_TRUE(motion_directive.has_execute());
  ASSERT_EQ(motion_directive.scope(), MOTION_SCOPE_MISSION_DESCENDANT);
  ASSERT_TRUE(PlanningMissionSyncTestPeer::SubmitMotion(
      component_.get(), motion_directive));
  ASSERT_TRUE(Await([this] {
    Poll();
    return PlanningMissionSyncTestPeer::MotionSequence(*component_) != 0;
  }));

  execution_state_sync::Options options;
  options.path = path_;
  options.role = execution_state_sync::Role::kControl;
  options.writer_epoch = "control-peer";
  execution_state_sync::Result opened;
  ASSERT_TRUE(Await([&]() {
    opened = execution_state_sync::Store::Open(options, &control_);
    return opened.code != execution_state_sync::Code::kBusy ||
           opened.sqlite_code == 0;
  })) << opened.message;
  ASSERT_TRUE(opened.ok()) << opened.message;

  execution_state_sync::Snapshot snapshot;
  ASSERT_TRUE(mission_->ReadSnapshot(&snapshot).ok());
  const auto& mission_event = *snapshot.latest[static_cast<size_t>(
      execution_state_sync::Channel::kMission)];
  const auto motion_sequence =
      PlanningMissionSyncTestPeer::MotionSequence(*component_);
  uint64_t control_sequence = 0;
  auto submitted = SubmitControlMotionStatus(
      motion_directive, mission_event.sequence, motion_sequence, 1, false,
      &control_sequence, MOTION_EXECUTION_FAILED, true);
  ASSERT_TRUE(submitted.ok()) << submitted.message;
  ASSERT_TRUE(Await([this, control_sequence] {
    Poll();
    return PlanningMissionSyncTestPeer::Cursor(*component_) >=
           control_sequence;
  }));
  EXPECT_EQ(PlanningMissionSyncTestPeer::Session(*component_).guidance().state,
            MISSION_SESSION_EXECUTING);

  submitted = SubmitControlMotionStatus(
      motion_directive, mission_event.sequence, motion_sequence, 2, true,
      &control_sequence);
  ASSERT_TRUE(submitted.ok()) << submitted.message;
  ASSERT_TRUE(Await([this, control_sequence] {
    Poll();
    return PlanningMissionSyncTestPeer::Cursor(*component_) >=
           control_sequence;
  }));
  EXPECT_EQ(PlanningMissionSyncTestPeer::Session(*component_).guidance().state,
            MISSION_SESSION_EXECUTING);

  submitted = SubmitControlMotionStatus(
      motion_directive, mission_event.sequence, motion_sequence, 3, false,
      &control_sequence);
  ASSERT_TRUE(submitted.ok()) << submitted.message;
  ASSERT_TRUE(Await([this] {
    Poll();
    return PlanningMissionSyncTestPeer::Session(*component_).guidance().state ==
           MISSION_SESSION_FAILED;
  }));
  EXPECT_FALSE(
      PlanningMissionSyncTestPeer::Session(*component_).guidance()
          .cancellation_fenced);
  EXPECT_FALSE(PlanningMissionSyncTestPeer::Faulted(*component_));
}

TEST_F(PlanningMissionSyncTest,
       ControlRejectionFailsOnlyTheMatchingMissionMotion) {
  AcceptFirstMission();
  ASSERT_FALSE(HasFatalFailure());
  ASSERT_TRUE(PlanningMissionSyncTestPeer::MarkExecuting(component_.get())
                  .accepted);
  const auto localization = Localization();
  const auto motion_directive =
      PlanningMissionSyncTestPeer::BuildMotion(component_.get(), localization);
  ASSERT_TRUE(motion_directive.has_execute());
  ASSERT_EQ(motion_directive.scope(), MOTION_SCOPE_MISSION_DESCENDANT);
  ASSERT_TRUE(PlanningMissionSyncTestPeer::SubmitMotion(
      component_.get(), motion_directive));
  ASSERT_TRUE(Await([this] {
    Poll();
    return PlanningMissionSyncTestPeer::MotionSequence(*component_) != 0;
  }));

  execution_state_sync::Options options;
  options.path = path_;
  options.role = execution_state_sync::Role::kControl;
  options.writer_epoch = "control-rejection-peer";
  execution_state_sync::Result opened;
  ASSERT_TRUE(Await([&]() {
    opened = execution_state_sync::Store::Open(options, &control_);
    return opened.code != execution_state_sync::Code::kBusy ||
           opened.sqlite_code == 0;
  })) << opened.message;
  ASSERT_TRUE(opened.ok()) << opened.message;

  execution_state_sync::Snapshot snapshot;
  ASSERT_TRUE(mission_->ReadSnapshot(&snapshot).ok());
  const auto& mission_event = *snapshot.latest[static_cast<size_t>(
      execution_state_sync::Channel::kMission)];
  const auto motion_sequence =
      PlanningMissionSyncTestPeer::MotionSequence(*component_);
  uint64_t control_sequence = 0;
  auto submitted = SubmitControlMotionStatus(
      motion_directive, mission_event.sequence, motion_sequence, 1, true,
      &control_sequence, MOTION_EXECUTION_REJECTED);
  ASSERT_TRUE(submitted.ok()) << submitted.message;
  ASSERT_TRUE(Await([this, control_sequence] {
    Poll();
    return PlanningMissionSyncTestPeer::Cursor(*component_) >=
           control_sequence;
  }));
  EXPECT_EQ(PlanningMissionSyncTestPeer::Session(*component_).guidance().state,
            MISSION_SESSION_EXECUTING);

  submitted = SubmitControlMotionStatus(
      motion_directive, mission_event.sequence, motion_sequence, 2, false,
      &control_sequence, MOTION_EXECUTION_REJECTED);
  ASSERT_TRUE(submitted.ok()) << submitted.message;
  ASSERT_TRUE(Await([this] {
    Poll();
    return PlanningMissionSyncTestPeer::Session(*component_).guidance().state ==
           MISSION_SESSION_FAILED;
  }));
  EXPECT_FALSE(PlanningMissionSyncTestPeer::Session(*component_).guidance()
                   .cancellation_fenced);
  EXPECT_FALSE(PlanningMissionSyncTestPeer::Faulted(*component_));
}

}  // namespace
}  // namespace planning
}  // namespace apollo
