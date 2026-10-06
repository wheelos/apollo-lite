#include "modules/mission/mission_component.h"

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "cyber/timer/timing_wheel.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/execution_state_sync/execution_state_gflags.h"
#include "modules/execution_state_sync/store.h"
#include "modules/mission/common/mission_context.h"
#include "modules/mission/common/mission_request_contract.h"
#include "modules/mission/nodes/action/move_to_node.h"
#include "modules/mission/nodes/action/park_in_node.h"

namespace apollo {
namespace mission {
namespace {

class MissionHarness : public MissionComponent {
 public:
  bool InitializeRuntime(const std::string& name, const std::string& config) {
    node_ = cyber::CreateNode(name);
    config_file_path_ = config;
    return Init();
  }

  void StopRuntime() { Clear(); }
};

struct Observations {
  std::mutex mutex;
  std::condition_variable changed;
  MissionRuntimeStatus latest;
};

template <typename Predicate>
bool Await(Predicate predicate) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(5);
  do {
    if (predicate()) {
      return true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}

bool SqlBusy(const execution_state_sync::Result& result) {
  return result.code == execution_state_sync::Code::kBusy &&
         result.sqlite_code != 0;
}

class MissionLifecycleTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    ASSERT_TRUE(cyber::Init("mission_lifecycle_test"));
  }

  static void TearDownTestSuite() { cyber::Clear(); }

  void SetUp() override {
    static std::atomic<unsigned> count{0};
    suffix_ = std::to_string(getpid()) + "_" + std::to_string(count++);
    directory_ = std::filesystem::temp_directory_path() /
                 ("mission_lifecycle_" + suffix_);
    ASSERT_TRUE(std::filesystem::create_directory(directory_));
    ASSERT_TRUE(std::filesystem::create_directory(directory_ / "trees"));
    std::ofstream tree(directory_ / "trees" / "move.xml");
    tree << R"(<root BTCPP_format="4">
      <BehaviorTree ID="FunctionalMove">
        <MoveTo goal="{Goal}"/>
      </BehaviorTree>
    </root>)";
    tree.close();
    ASSERT_TRUE(tree.good());
    saved_database_ = FLAGS_execution_state_db_path;
    saved_chassis_topic_ = FLAGS_chassis_topic;
    saved_localization_topic_ = FLAGS_localization_topic;
    FLAGS_execution_state_db_path = (directory_ / "state.db").string();
    FLAGS_chassis_topic = "/test/mission/chassis/" + suffix_;
    FLAGS_localization_topic = "/test/mission/localization/" + suffix_;
    WriteConfig(10);
  }

  void TearDown() override {
    if (component_) {
      component_->StopRuntime();
      component_.reset();
    }
    MissionContext::Instance()->ShutdownExecutionState();
    status_reader_.reset();
    observer_.reset();
    peer_.reset();
    FLAGS_execution_state_db_path = saved_database_;
    FLAGS_chassis_topic = saved_chassis_topic_;
    FLAGS_localization_topic = saved_localization_topic_;
    std::filesystem::remove(directory_ / "state.db-wal");
    std::filesystem::remove(directory_ / "state.db-shm");
    std::filesystem::remove(directory_ / "state.db");
    std::filesystem::remove(directory_ / "requests.db-wal");
    std::filesystem::remove(directory_ / "requests.db-shm");
    std::filesystem::remove(directory_ / "requests.db");
    std::filesystem::remove(directory_ / "config.pb.txt");
    std::filesystem::remove(directory_ / "trees" / "move.xml");
    std::filesystem::remove(directory_ / "trees");
    std::filesystem::remove(directory_);
  }

  void WriteConfig(int interval) {
    std::ofstream config(directory_ / "config.pb.txt");
    config << "bt_tree_dir: \"" << (directory_ / "trees").string() << "\"\n"
           << "tick_interval_ms: " << interval << "\n"
           << "mission_directive_topic: \"/test/mission/directive/"
           << suffix_ << "\"\n"
           << "mission_runtime_status_topic: \"/test/mission/status/"
           << suffix_ << "\"\n"
           << "mission_request_ledger_path: \""
           << (directory_ / "requests.db").string() << "\"\n";
    config.close();
    ASSERT_TRUE(config.good());
  }

  MissionRequest Request() const {
    MissionRequest request;
    request.set_mission_id("mission-" + suffix_);
    request.set_task_name("FunctionalMove");
    auto* goal = request.add_waypoints();
    goal->set_name("Goal");
    goal->mutable_pose()->set_x(1.0);
    goal->mutable_pose()->set_y(2.0);
    return request;
  }

  execution_state_sync::Result PublishPlanning(
      const planning::MissionDirective& directive, uint64_t parent_sequence,
      uint64_t revision, planning::MissionSessionState session,
      bool admission) {
    planning::PlanningRuntimeStatus status;
    status.set_mission_id(directive.identity().aggregate_id());
    status.set_command_id(directive.identity().command_id());
    status.mutable_mission_identity()->CopyFrom(directive.identity());
    status.mutable_accepted_directive_identity()->CopyFrom(directive.identity());
    status.set_mission_session_state(session);
    status.set_state(admission ? planning::RUNTIME_ACCEPTED
                               : planning::RUNTIME_COMPLETED);
    if (admission) {
      status.mutable_admission_directive_identity()->CopyFrom(
          directive.identity());
      status.set_mission_admission_outcome(planning::MISSION_ADMISSION_ACCEPTED);
    }
    execution_state_sync::Operation operation;
    operation.channel = execution_state_sync::Channel::kPlanningStatus;
    operation.operation_id = "planning-" + std::to_string(revision);
    operation.identity = {"planning-peer", "statuses", operation.operation_id,
                          revision};
    operation.expected_revision = revision - 1;
    operation.guards = {
        {execution_state_sync::Channel::kMission, parent_sequence}};
    operation.payload = status.SerializeAsString();
    operation.planning_status_kind =
        admission ? execution_state_sync::PlanningStatusKind::kAdmissionAccepted
                  : execution_state_sync::PlanningStatusKind::kRuntime;
    execution_state_sync::Commit commit;
    execution_state_sync::Result result;
    Await([&]() {
      result = peer_->Submit(operation, &commit);
      return !SqlBusy(result);
    });
    return result;
  }

  std::string suffix_;
  std::filesystem::path directory_;
  std::string saved_database_;
  std::string saved_chassis_topic_;
  std::string saved_localization_topic_;
  std::unique_ptr<MissionHarness> component_;
  std::shared_ptr<cyber::Node> observer_;
  std::shared_ptr<cyber::Reader<MissionRuntimeStatus>> status_reader_;
  std::unique_ptr<execution_state_sync::Store> peer_;
};

TEST_F(MissionLifecycleTest, StartupWaitsForReadyAndDoesNotRestoreAuthority) {
  auto* context = MissionContext::Instance();
  ASSERT_TRUE(context->InitExecutionState(FLAGS_execution_state_db_path,
                                         "mission-first",
                                         (directory_ / "requests.db").string()));
  EXPECT_FALSE(context->ExecutionStateHealthy());
  ASSERT_TRUE(Await([context]() {
    context->PollExecutionState();
    return context->ExecutionStateHealthy();
  }));
  context->SetCurrentMissionId("old-mission");
  context->SetCurrentTaskName("old-task");
  context->ShutdownExecutionState();
  EXPECT_FALSE(context->ExecutionStateHealthy());
  ASSERT_TRUE(context->InitExecutionState(FLAGS_execution_state_db_path,
                                         "mission-restarted",
                                         (directory_ / "requests.db").string()));
  EXPECT_FALSE(context->ExecutionStateHealthy());
  const auto snapshot = context->GetMissionCommandSnapshot();
  EXPECT_TRUE(snapshot.mission_id.empty());
  EXPECT_TRUE(snapshot.active_command_id.empty());
  EXPECT_FALSE(snapshot.accepted_mission_identity.has_revision());
}

TEST_F(MissionLifecycleTest, TypedSubmitIsDurableAndIdempotent) {
  const auto observations = std::make_shared<Observations>();
  observer_ = cyber::CreateNode("typed_mission_observer_" + suffix_);
  status_reader_ = observer_->CreateReader<MissionRuntimeStatus>(
      "/test/mission/status/" + suffix_,
      [observations](const std::shared_ptr<MissionRuntimeStatus>& status) {
        std::lock_guard<std::mutex> lock(observations->mutex);
        observations->latest.CopyFrom(*status);
        observations->changed.notify_all();
      });
  ASSERT_NE(status_reader_, nullptr);
  component_ = std::make_unique<MissionHarness>();
  ASSERT_TRUE(component_->InitializeRuntime(
      "typed_mission_runtime_" + suffix_,
      (directory_ / "config.pb.txt").string()));
  auto* context = MissionContext::Instance();
  ASSERT_TRUE(Await([context]() { return context->ExecutionStateHealthy(); }));

  MissionRequest request;
  request.set_mission_id("typed-" + suffix_);
  request.mutable_identity()->set_client_epoch("client-epoch");
  request.mutable_identity()->set_request_id("request-" + suffix_);
  request.set_contract_version(kMissionRequestContractVersion);
  auto* plan = request.mutable_submit_task()->mutable_plan();
  plan->set_task_type(planning::MISSION_TASK_A_TO_B);
  plan->set_preferred_mode(planning::MODE_CORRIDOR);
  plan->mutable_start()->set_current_pose_at_acceptance(true);
  plan->mutable_goal()->mutable_goal_pose()->set_x(1.0);
  plan->mutable_goal()->mutable_goal_pose()->set_y(2.0);
  plan->mutable_completion()->set_timeout_sec(120.0);
  plan->set_preemptible(true);

  ASSERT_TRUE(component_->Proc(std::make_shared<MissionRequest>(request)));
  MissionRequest query;
  query.mutable_identity()->set_client_epoch("query-client");
  query.mutable_identity()->set_request_id("query-" + suffix_);
  query.set_contract_version(kMissionRequestContractVersion);
  query.mutable_query()->mutable_request_identity()->CopyFrom(
      request.identity());
  MissionRequestResult result;
  ASSERT_TRUE(Await([&]() {
    if (!component_->Proc(std::make_shared<MissionRequest>(query))) {
      return false;
    }
    std::lock_guard<std::mutex> lock(observations->mutex);
    return observations->latest.has_last_request_result() &&
           observations->latest.last_request_result().outcome() ==
               MISSION_REQUEST_COMMITTED;
  }));
  {
    std::lock_guard<std::mutex> lock(observations->mutex);
    result.CopyFrom(observations->latest.last_request_result());
  }
  EXPECT_TRUE(result.durably_committed());
  EXPECT_GT(result.commit_sequence(), 0u);
  ASSERT_TRUE(result.has_affected_task_identity());
  EXPECT_EQ(result.affected_task_identity().aggregate_id(),
            request.mission_id());

  execution_state_sync::Options options;
  options.path = FLAGS_execution_state_db_path;
  options.role = execution_state_sync::Role::kPlanning;
  options.writer_epoch = "typed-planning-peer";
  execution_state_sync::Result opened;
  ASSERT_TRUE(Await([&]() {
    opened = execution_state_sync::Store::Open(options, &peer_);
    return !SqlBusy(opened);
  })) << opened.message;
  ASSERT_TRUE(opened.ok()) << opened.message;
  execution_state_sync::Snapshot snapshot;
  ASSERT_TRUE(Await([this, &snapshot]() {
    const auto read = peer_->ReadSnapshot(&snapshot);
    return read.ok() &&
           snapshot.latest[static_cast<size_t>(
               execution_state_sync::Channel::kMission)]
               .has_value();
  }));
  const auto& mission_event = *snapshot.latest[static_cast<size_t>(
      execution_state_sync::Channel::kMission)];
  planning::MissionDirective directive;
  ASSERT_TRUE(directive.ParseFromString(mission_event.operation.payload));
  ASSERT_TRUE(directive.has_activate());
  EXPECT_EQ(directive.activate().plan().preferred_mode(),
            planning::MODE_CORRIDOR);
  ASSERT_TRUE(PublishPlanning(directive, mission_event.sequence, 1,
                              planning::MISSION_SESSION_ACCEPTED, true)
                  .ok());
  ASSERT_TRUE(Await([&]() {
    MissionRequestResult current;
    return context->GetMissionRequestResult(request.identity(), &current) &&
           current.outcome() == MISSION_REQUEST_ACCEPTED;
  }));
  ASSERT_TRUE(PublishPlanning(directive, mission_event.sequence, 2,
                              planning::MISSION_SESSION_COMPLETED, false)
                  .ok());
  ASSERT_TRUE(Await([&]() {
    MissionRequestResult current;
    return context->GetMissionRequestResult(request.identity(), &current) &&
           current.outcome() == MISSION_REQUEST_TERMINAL &&
           current.terminal_disposition() == MISSION_TASK_COMPLETED;
  }));

  MissionRequest duplicate = request;
  ASSERT_TRUE(component_->Proc(std::make_shared<MissionRequest>(duplicate)));
  MissionRequest conflict = request;
  conflict.mutable_submit_task()
      ->mutable_plan()
      ->mutable_goal()
      ->mutable_goal_pose()
      ->set_x(3.0);
  EXPECT_FALSE(component_->Proc(std::make_shared<MissionRequest>(conflict)));
  ASSERT_TRUE(Await([&observations]() {
    std::lock_guard<std::mutex> lock(observations->mutex);
    return observations->latest.has_last_request_result() &&
           observations->latest.last_request_result().code() ==
               MISSION_REQUEST_CONFLICT;
  }));
}

TEST_F(MissionLifecycleTest, ControlSafetyOutcomeIsCorrelatedAndDurable) {
  component_ = std::make_unique<MissionHarness>();
  ASSERT_TRUE(component_->InitializeRuntime(
      "safety_mission_runtime_" + suffix_,
      (directory_ / "config.pb.txt").string()));
  auto* context = MissionContext::Instance();
  ASSERT_TRUE(Await([context]() { return context->ExecutionStateHealthy(); }));

  MissionRequest request;
  request.set_mission_id("safety-" + suffix_);
  request.mutable_identity()->set_client_epoch("safety-client");
  request.mutable_identity()->set_request_id("stop-" + suffix_);
  request.set_contract_version(kMissionRequestContractVersion);
  auto* stop = request.mutable_safety_stop();
  stop->mutable_identity()->set_requester_epoch(
      request.identity().client_epoch());
  stop->mutable_identity()->set_request_id(request.identity().request_id());
  stop->set_policy(control::SAFETY_STOP_CONTROLLED);
  stop->set_reason("operator-requested controlled safety stop");
  ASSERT_TRUE(component_->Proc(std::make_shared<MissionRequest>(request)));

  execution_state_sync::Options options;
  options.path = FLAGS_execution_state_db_path;
  options.role = execution_state_sync::Role::kControl;
  options.writer_epoch = "safety-control-peer";
  execution_state_sync::Result opened;
  ASSERT_TRUE(Await([&]() {
    opened = execution_state_sync::Store::Open(options, &peer_);
    return !SqlBusy(opened);
  })) << opened.message;
  ASSERT_TRUE(opened.ok()) << opened.message;
  execution_state_sync::Snapshot snapshot;
  ASSERT_TRUE(Await([this, &snapshot]() {
    const auto read = peer_->ReadSnapshot(&snapshot);
    return read.ok() &&
           snapshot.latest[static_cast<size_t>(
               execution_state_sync::Channel::kSafetyRequest)]
               .has_value();
  }));

  control::SafetyStopObservation observation;
  observation.mutable_operation_identity()->set_requester_epoch(
      request.identity().client_epoch());
  observation.mutable_operation_identity()->set_request_id(
      request.identity().request_id());
  observation.mutable_safety_identity()->set_control_epoch(
      "safety-control-peer");
  observation.mutable_safety_identity()->set_generation(1);
  observation.set_operation_accepted(true);
  observation.set_enforced(true);
  observation.set_safety_latched(true);
  observation.set_effective_policy(control::SAFETY_STOP_CONTROLLED);
  observation.set_reason("controlled stop latched in Control");
  execution_state_sync::Operation operation;
  operation.operation_id = "safety-observation-" + suffix_;
  operation.channel = execution_state_sync::Channel::kSafetyStatus;
  operation.identity.epoch = "safety-control-peer";
  operation.identity.aggregate_id = request.mission_id();
  operation.identity.command_id = request.identity().request_id();
  operation.identity.revision = 1;
  operation.expected_revision = 0;
  operation.payload = observation.SerializeAsString();
  execution_state_sync::Commit commit;
  ASSERT_TRUE(peer_->Submit(operation, &commit).ok());
  ASSERT_GT(commit.sequence, 0u);

  MissionRequestResult result;
  ASSERT_TRUE(Await([&]() {
    return context->GetMissionRequestResult(request.identity(), &result) &&
           result.outcome() == MISSION_REQUEST_ACCEPTED;
  }));
  ASSERT_TRUE(result.has_safety_observation());
  EXPECT_TRUE(result.safety_observation().durably_committed());
  EXPECT_EQ(result.safety_observation().commit_sequence(), commit.sequence);
  EXPECT_EQ(result.safety_observation().safety_identity().generation(), 1u);
}

TEST_F(MissionLifecycleTest, PeriodicTickCommitsTaskAndConsumesTerminalFeedback) {
  const auto observations = std::make_shared<Observations>();
  observer_ = cyber::CreateNode("mission_observer_" + suffix_);
  status_reader_ = observer_->CreateReader<MissionRuntimeStatus>(
      "/test/mission/status/" + suffix_,
      [observations](const std::shared_ptr<MissionRuntimeStatus>& status) {
        std::lock_guard<std::mutex> lock(observations->mutex);
        observations->latest.CopyFrom(*status);
        observations->changed.notify_all();
      });
  ASSERT_NE(status_reader_, nullptr);
  component_ = std::make_unique<MissionHarness>();
  ASSERT_TRUE(component_->InitializeRuntime(
      "mission_runtime_" + suffix_, (directory_ / "config.pb.txt").string()));
  ASSERT_TRUE(component_->Proc(std::make_shared<MissionRequest>(Request())));
  execution_state_sync::Options options;
  options.path = FLAGS_execution_state_db_path;
  options.role = execution_state_sync::Role::kPlanning;
  options.writer_epoch = "planning-peer";
  execution_state_sync::Result opened;
  ASSERT_TRUE(Await([&]() {
    opened = execution_state_sync::Store::Open(options, &peer_);
    return !SqlBusy(opened);
  })) << opened.message;
  ASSERT_TRUE(opened.ok()) << opened.message;
  execution_state_sync::Snapshot snapshot;
  ASSERT_TRUE(Await([this, &snapshot]() {
    const auto read = peer_->ReadSnapshot(&snapshot);
    return read.ok() &&
           snapshot.latest[static_cast<size_t>(
               execution_state_sync::Channel::kMission)].has_value();
  }));
  const auto& event = *snapshot.latest[static_cast<size_t>(
      execution_state_sync::Channel::kMission)];
  planning::MissionDirective directive;
  ASSERT_TRUE(directive.ParseFromString(event.operation.payload));
  ASSERT_TRUE(directive.has_activate());
  EXPECT_EQ(directive.identity().aggregate_id(), Request().mission_id());
  ASSERT_TRUE(component_->Proc(std::make_shared<MissionRequest>(Request())));
  auto conflicting = Request();
  conflicting.mutable_waypoints(0)->mutable_pose()->set_x(9.0);
  EXPECT_FALSE(component_->Proc(
      std::make_shared<MissionRequest>(conflicting)));
  auto replacement = Request();
  replacement.set_mission_id("other-mission");
  EXPECT_FALSE(component_->Proc(
      std::make_shared<MissionRequest>(replacement)));
  EXPECT_EQ(MissionContext::Instance()->GetCurrentMissionId(),
            Request().mission_id());

  auto published = PublishPlanning(directive, event.sequence, 1,
                                   planning::MISSION_SESSION_EXECUTING, true);
  ASSERT_TRUE(published.ok()) << published.message;
  ASSERT_TRUE(Await([&directive]() {
    const auto accepted =
        MissionContext::Instance()->GetMissionCommandSnapshot();
    return accepted.accepted_mission_identity.SerializeAsString() ==
           directive.identity().SerializeAsString();
  }));
  auto duplicate_component = std::make_unique<MissionHarness>();
  EXPECT_FALSE(duplicate_component->InitializeRuntime(
      "duplicate_mission_" + suffix_,
      (directory_ / "config.pb.txt").string()));
  duplicate_component.reset();
  published = PublishPlanning(directive, event.sequence, 2,
                              planning::MISSION_SESSION_COMPLETING, false);
  ASSERT_TRUE(published.ok()) << published.message;
  ASSERT_TRUE(Await([]() {
    const auto status = MissionContext::Instance()->GetPlanningRuntimeStatus();
    return status &&
           status->mission_session_state() ==
               planning::MISSION_SESSION_COMPLETING;
  }));
  EXPECT_FALSE(MissionContext::Instance()->GetMissionCommandSnapshot()
                   .active_command_id.empty());
  published = PublishPlanning(directive, event.sequence, 3,
                              planning::MISSION_SESSION_COMPLETED, false);
  ASSERT_TRUE(published.ok()) << published.message;
  {
    std::unique_lock<std::mutex> lock(observations->mutex);
    ASSERT_TRUE(observations->changed.wait_for(
        lock, std::chrono::seconds(5), [&observations]() {
          return observations->latest.state() == MISSION_RUNTIME_COMPLETED;
        }));
    EXPECT_EQ(observations->latest.last_terminal_task_identity()
                  .SerializeAsString(),
              directive.identity().SerializeAsString());
    EXPECT_TRUE(observations->latest.active_command_id().empty());
  }
  component_->StopRuntime();
  EXPECT_FALSE(MissionContext::Instance()->ExecutionStateHealthy());
  std::vector<execution_state_sync::Participant> participants;
  ASSERT_TRUE(peer_->ReadParticipants(&participants).ok());
  for (const auto& participant : participants) {
    if (participant.role == execution_state_sync::Role::kMission) {
      EXPECT_FALSE(participant.ready);
    }
  }
}

TEST_F(MissionLifecycleTest, InvalidTickIntervalsFailBeforeStartingRuntime) {
  for (const int interval :
       {0, -1, static_cast<int>(cyber::TIMER_MAX_INTERVAL_MS)}) {
    WriteConfig(interval);
    component_ = std::make_unique<MissionHarness>();
    EXPECT_FALSE(component_->InitializeRuntime(
        "invalid_mission_" + suffix_, (directory_ / "config.pb.txt").string()));
    EXPECT_FALSE(MissionContext::Instance()->ExecutionStateHealthy());
    component_.reset();
  }
}

TEST_F(MissionLifecycleTest, TaskSubmissionFailureDoesNotReportRunning) {
  MissionContext::Instance()->ShutdownExecutionState();
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<MoveToNode>("MoveTo");
  factory.registerNodeType<ParkInNode>("ParkIn");
  auto blackboard = BT::Blackboard::create();
  common::PointENU goal;
  goal.set_x(1.0);
  goal.set_y(2.0);
  blackboard->set("Goal", goal);
  auto move = factory.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="Move">
           <MoveTo goal="{Goal}"/>
         </BehaviorTree></root>)", blackboard);
  EXPECT_EQ(move.tickOnce(), BT::NodeStatus::FAILURE);
  auto park = factory.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="Park">
           <ParkIn parking_space_id="parking-1"/>
         </BehaviorTree></root>)");
  EXPECT_EQ(park.tickOnce(), BT::NodeStatus::FAILURE);
}

}  // namespace
}  // namespace mission
}  // namespace apollo
