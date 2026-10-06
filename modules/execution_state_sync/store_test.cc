// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/execution_state_sync/store.h"

#include <sqlite3.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "modules/execution_state_sync/client.h"
#include "modules/execution_state_sync/worker.h"

namespace apollo {
namespace execution_state_sync {
namespace {

Operation Mission(uint64_t revision, std::string id = "") {
  Operation op;
  op.operation_id = id.empty() ? "op-" + std::to_string(revision) : id;
  op.identity = {"mission-epoch", "mission-1", "command-1", revision};
  op.expected_revision = revision - 1;
  op.payload = std::string("binary\0proto", 12);
  return op;
}

class StoreTest : public ::testing::Test {
 protected:
  void SetUp() override {
    static std::atomic<unsigned> count{0};
    path_ = "execution-state-sync-test-" + std::to_string(getpid()) + "-" +
            std::to_string(count++) + ".db";
    ASSERT_TRUE(Store::Open({path_, Role::kMission, 1000}, &mission_).ok());
  }
  void TearDown() override {
    mission_.reset();
    std::remove(path_.c_str());
    std::remove((path_ + "-wal").c_str());
    std::remove((path_ + "-shm").c_str());
  }
  std::string path_;
  std::unique_ptr<Store> mission_;
};

TEST_F(StoreTest, IdempotencyChecksEntireImmutableOperation) {
  const auto op = Mission(1);
  Commit first;
  ASSERT_TRUE(mission_->Submit(op, &first).ok());
  EXPECT_EQ(first.sequence, 1);
  EXPECT_FALSE(first.duplicate);
  Commit duplicate;
  ASSERT_TRUE(mission_->Submit(op, &duplicate).ok());
  EXPECT_EQ(duplicate.sequence, first.sequence);
  EXPECT_TRUE(duplicate.duplicate);
  auto changed = op;
  changed.payload += "changed";
  EXPECT_EQ(mission_->Submit(changed, &duplicate).code, Code::kConflict);
  changed = op;
  changed.identity.epoch = "new-epoch";
  EXPECT_EQ(mission_->Submit(changed, &duplicate).code, Code::kConflict);
  changed = op;
  changed.mission_fenced = true;
  EXPECT_EQ(mission_->Submit(changed, &duplicate).code, Code::kConflict);
  ASSERT_TRUE(mission_->Submit(Mission(2), &duplicate).ok());
  ASSERT_TRUE(mission_->Submit(op, &duplicate).ok());
  EXPECT_EQ(duplicate.sequence, 1);
  EXPECT_TRUE(duplicate.duplicate);
}

TEST_F(StoreTest, OwnershipGuardsAndMissionFence) {
  std::unique_ptr<Store> planning;
  ASSERT_TRUE(Store::Open({path_, Role::kPlanning}, &planning).ok());
  Commit commit;
  EXPECT_EQ(planning->Submit(Mission(1), &commit).code, Code::kUnauthorized);
  auto mission = Mission(1);
  mission.mission_fenced = true;
  ASSERT_TRUE(mission_->Submit(mission, &commit).ok());
  Operation motion = Mission(1, "motion-1");
  motion.channel = Channel::kMotion;
  EXPECT_EQ(planning->Submit(motion, &commit).code, Code::kInvalidArgument);
  motion.guards = {{Channel::kMission, 1}};
  EXPECT_EQ(planning->Submit(motion, &commit).code, Code::kFenced);
  motion.allow_when_fenced = true;
  ASSERT_TRUE(planning->Submit(motion, &commit).ok());
  EXPECT_EQ(commit.sequence, 2);
  ASSERT_TRUE(mission_->Submit(Mission(2), &commit).ok());
  motion.operation_id = "motion-2";
  motion.identity.revision = 2;
  motion.expected_revision = 1;
  EXPECT_EQ(planning->Submit(motion, &commit).code, Code::kConflict);
  motion.guards = {{Channel::kMission, 3}};
  ASSERT_TRUE(planning->Submit(motion, &commit).ok());
  Snapshot snapshot;
  ASSERT_TRUE(planning->ReadSnapshot(&snapshot).ok());
  EXPECT_EQ(snapshot.sequence, 4);
  ASSERT_TRUE(snapshot.latest[0]);
  ASSERT_TRUE(snapshot.latest[1]);
  EXPECT_EQ(snapshot.latest[0]->sequence, 3);
  EXPECT_EQ(snapshot.latest[1]->sequence, 4);
  EXPECT_FALSE(snapshot.latest[2]);
}

TEST_F(StoreTest, ControlRequiresBothParents) {
  Commit commit;
  ASSERT_TRUE(mission_->Submit(Mission(1), &commit).ok());
  std::unique_ptr<Store> planning;
  std::unique_ptr<Store> control;
  ASSERT_TRUE(Store::Open({path_, Role::kPlanning}, &planning).ok());
  ASSERT_TRUE(Store::Open({path_, Role::kControl}, &control).ok());
  auto motion = Mission(1, "motion");
  motion.channel = Channel::kMotion;
  motion.guards = {{Channel::kMission, 1}};
  ASSERT_TRUE(planning->Submit(motion, &commit).ok());
  auto status = Mission(1, "control-status");
  status.channel = Channel::kControlStatus;
  status.guards = {{Channel::kMission, 1}};
  EXPECT_EQ(control->Submit(status, &commit).code, Code::kInvalidArgument);
  status.guards.push_back({Channel::kMotion, 2});
  ASSERT_TRUE(control->Submit(status, &commit).ok());
  EXPECT_EQ(commit.sequence, 3);
}

TEST_F(StoreTest, OwnerRuntimeObservationsDoNotRequireTaskAuthority) {
  std::unique_ptr<Store> planning;
  std::unique_ptr<Store> control;
  ASSERT_TRUE(Store::Open({path_, Role::kPlanning}, &planning).ok());
  ASSERT_TRUE(Store::Open({path_, Role::kControl}, &control).ok());

  auto planning_status = Mission(1, "planning-runtime");
  planning_status.channel = Channel::kPlanningStatus;
  planning_status.identity.epoch = "planning-epoch";
  planning_status.planning_status_kind = PlanningStatusKind::kRuntime;
  Commit commit;
  ASSERT_TRUE(planning->Submit(planning_status, &commit).ok());

  auto control_status = Mission(1, "control-runtime");
  control_status.channel = Channel::kControlStatus;
  control_status.identity.epoch = "control-epoch";
  control_status.control_status_kind = ControlStatusKind::kOwnerRuntime;
  ASSERT_TRUE(control->Submit(control_status, &commit).ok());

  Snapshot snapshot;
  ASSERT_TRUE(planning->ReadSnapshot(&snapshot).ok());
  EXPECT_FALSE(snapshot.accepted_authority);
  ASSERT_TRUE(snapshot.latest[static_cast<size_t>(Channel::kPlanningStatus)]);
  EXPECT_TRUE(snapshot.latest[static_cast<size_t>(Channel::kPlanningStatus)]
                  ->operation.guards.empty());
  ASSERT_TRUE(snapshot.latest[static_cast<size_t>(Channel::kControlStatus)]);
  EXPECT_TRUE(snapshot.latest[static_cast<size_t>(Channel::kControlStatus)]
                  ->operation.guards.empty());
  EXPECT_EQ(snapshot.latest[static_cast<size_t>(Channel::kControlStatus)]
                ->operation.control_status_kind,
            ControlStatusKind::kOwnerRuntime);

  auto unguarded_result = Mission(1, "unguarded-control-result");
  unguarded_result.channel = Channel::kControlStatus;
  unguarded_result.identity.epoch = "control-epoch";
  EXPECT_EQ(control->Submit(unguarded_result, &commit).code,
            Code::kInvalidArgument);

  planning_status.planning_status_kind =
      PlanningStatusKind::kAdmissionAccepted;
  planning_status.operation_id = "unguarded-admission";
  planning_status.identity.command_id = "unguarded-admission";
  planning_status.identity.revision = 2;
  planning_status.expected_revision = 1;
  EXPECT_EQ(planning->Submit(planning_status, &commit).code,
            Code::kInvalidArgument);
}

TEST_F(StoreTest, V4UnguardedControlStatusesMigrateAsOwnerObservations) {
  Commit commit;
  ASSERT_TRUE(mission_->Submit(Mission(1), &commit).ok());
  {
    std::unique_ptr<Store> planning;
    std::unique_ptr<Store> control;
    ASSERT_TRUE(Store::Open({path_, Role::kPlanning}, &planning).ok());
    ASSERT_TRUE(Store::Open({path_, Role::kControl}, &control).ok());
    auto motion = Mission(1, "motion");
    motion.channel = Channel::kMotion;
    motion.guards = {{Channel::kMission, 1}};
    ASSERT_TRUE(planning->Submit(motion, &commit).ok());
    auto observation = Mission(1, "control-runtime");
    observation.channel = Channel::kControlStatus;
    observation.control_status_kind = ControlStatusKind::kOwnerRuntime;
    ASSERT_TRUE(control->Submit(observation, &commit).ok());
  }
  mission_.reset();

  sqlite3* db = nullptr;
  ASSERT_EQ(sqlite3_open(path_.c_str(), &db), SQLITE_OK);
  const int migration_fixture_rc = sqlite3_exec(
      db,
      "ALTER TABLE events DROP COLUMN control_status_kind;"
      "PRAGMA user_version=4;",
      nullptr, nullptr, nullptr);
  EXPECT_EQ(migration_fixture_rc, SQLITE_OK) << sqlite3_errmsg(db);
  EXPECT_EQ(sqlite3_close(db), SQLITE_OK);

  ASSERT_TRUE(Store::Open({path_, Role::kMission}, &mission_).ok());
  Snapshot snapshot;
  ASSERT_TRUE(mission_->ReadSnapshot(&snapshot).ok());
  ASSERT_TRUE(snapshot.latest[static_cast<size_t>(Channel::kControlStatus)]);
  const auto& observation =
      snapshot.latest[static_cast<size_t>(Channel::kControlStatus)]
          ->operation;
  EXPECT_TRUE(observation.guards.empty());
  EXPECT_EQ(observation.control_status_kind, ControlStatusKind::kOwnerRuntime);
}

TEST_F(StoreTest, IndependentSafetyChannelsAreRoleOwnedAndUngarded) {
  std::unique_ptr<Store> control;
  std::unique_ptr<Store> planning;
  ASSERT_TRUE(Store::Open({path_, Role::kControl}, &control).ok());
  ASSERT_TRUE(Store::Open({path_, Role::kPlanning}, &planning).ok());

  auto request = Mission(1, "safety-request");
  request.channel = Channel::kSafetyRequest;
  Commit commit;
  ASSERT_TRUE(mission_->Submit(request, &commit).ok());
  EXPECT_EQ(commit.sequence, 1u);
  EXPECT_EQ(planning->Submit(request, &commit).code, Code::kUnauthorized);

  auto observation = Mission(1, "safety-observation");
  observation.channel = Channel::kSafetyStatus;
  ASSERT_TRUE(control->Submit(observation, &commit).ok());
  EXPECT_EQ(commit.sequence, 2u);
  EXPECT_EQ(planning->Submit(observation, &commit).code, Code::kUnauthorized);

  Snapshot snapshot;
  ASSERT_TRUE(control->ReadSnapshot(&snapshot).ok());
  ASSERT_TRUE(snapshot.latest[static_cast<size_t>(Channel::kSafetyRequest)]);
  ASSERT_TRUE(snapshot.latest[static_cast<size_t>(Channel::kSafetyStatus)]);
}

TEST_F(StoreTest, HistoricalResultsDoNotGrantAuthorityOrLoseCausality) {
  Commit commit;
  ASSERT_TRUE(mission_->Submit(Mission(1), &commit).ok());
  std::unique_ptr<Store> planning;
  std::unique_ptr<Store> control;
  ASSERT_TRUE(Store::Open({path_, Role::kPlanning}, &planning).ok());
  ASSERT_TRUE(Store::Open({path_, Role::kControl}, &control).ok());
  auto motion = Mission(1, "motion-1");
  motion.channel = Channel::kMotion;
  motion.guards = {{Channel::kMission, 1}};
  ASSERT_TRUE(planning->Submit(motion, &commit).ok());
  ASSERT_TRUE(mission_->Submit(Mission(2), &commit).ok());
  auto replacement = Mission(2, "motion-2");
  replacement.channel = Channel::kMotion;
  replacement.guards = {{Channel::kMission, 3}};
  ASSERT_TRUE(planning->Submit(replacement, &commit).ok());

  auto result = Mission(1, "old-owner-terminal");
  result.channel = Channel::kControlStatus;
  result.guards = {{Channel::kMission, 1}, {Channel::kMotion, 2}};
  ASSERT_TRUE(control->Submit(result, &commit).ok());
  Snapshot snapshot;
  ASSERT_TRUE(control->ReadSnapshot(&snapshot).ok());
  const auto& observed =
      snapshot.latest[static_cast<size_t>(Channel::kControlStatus)];
  ASSERT_TRUE(observed);
  ASSERT_EQ(observed->operation.guards.size(), 2);
  EXPECT_EQ(observed->operation.guards[0].sequence, 1);
  EXPECT_EQ(observed->operation.guards[1].sequence, 2);
  EXPECT_FALSE(snapshot.accepted_authority);

  auto status = Mission(1, "old-planning-result");
  status.channel = Channel::kPlanningStatus;
  status.guards = {{Channel::kMission, 1}, {Channel::kMotion, 2}};
  ASSERT_TRUE(planning->Submit(status, &commit).ok());
  status = Mission(2, "old-rejected-admission");
  status.channel = Channel::kPlanningStatus;
  status.guards = {{Channel::kMission, 1}};
  status.planning_status_kind = PlanningStatusKind::kAdmissionRejected;
  ASSERT_TRUE(planning->Submit(status, &commit).ok());
  status = Mission(3, "stale-authority");
  status.channel = Channel::kPlanningStatus;
  status.guards = {{Channel::kMission, 1}};
  status.planning_status_kind = PlanningStatusKind::kAdmissionAccepted;
  EXPECT_EQ(planning->Submit(status, &commit).code, Code::kConflict);
  status.planning_status_kind = PlanningStatusKind::kAdmissionDuplicate;
  EXPECT_EQ(planning->Submit(status, &commit).code, Code::kConflict);
  result = Mission(2, "wrong-parent-channel");
  result.channel = Channel::kControlStatus;
  result.guards = {{Channel::kMission, 2}, {Channel::kMotion, 1}};
  EXPECT_EQ(control->Submit(result, &commit).code, Code::kConflict);
  result.guards = {{Channel::kMission, 1}, {Channel::kMotion, 999}};
  EXPECT_EQ(control->Submit(result, &commit).code, Code::kConflict);
  motion = Mission(3, "stale-motion");
  motion.channel = Channel::kMotion;
  motion.guards = {{Channel::kMission, 1}};
  EXPECT_EQ(planning->Submit(motion, &commit).code, Code::kConflict);
}

TEST_F(StoreTest, CrossProcessFeedbackCommitsAgainstHistoricalParents) {
  Commit commit;
  ASSERT_TRUE(mission_->Submit(Mission(1), &commit).ok());
  std::unique_ptr<Store> planning;
  ASSERT_TRUE(Store::Open({path_, Role::kPlanning}, &planning).ok());
  auto motion = Mission(1, "motion");
  motion.channel = Channel::kMotion;
  motion.guards = {{Channel::kMission, 1}};
  ASSERT_TRUE(planning->Submit(motion, &commit).ok());
  ASSERT_TRUE(mission_->Submit(Mission(2), &commit).ok());
  planning.reset();
  mission_.reset();

  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    bool succeeded = false;
    {
      std::unique_ptr<Store> control;
      if (Store::Open({path_, Role::kControl}, &control).ok()) {
        auto result = Mission(1, "child-terminal-result");
        result.channel = Channel::kControlStatus;
        result.guards = {{Channel::kMission, 1}, {Channel::kMotion, 2}};
        succeeded = control->Submit(result, &commit).ok();
      }
    }
    _exit(succeeded ? 0 : 1);
  }
  int child_status = 0;
  ASSERT_EQ(waitpid(child, &child_status, 0), child);
  ASSERT_TRUE(WIFEXITED(child_status));
  ASSERT_EQ(WEXITSTATUS(child_status), 0);
  ASSERT_TRUE(Store::Open({path_, Role::kMission}, &mission_).ok());
  EventBatch batch;
  ASSERT_TRUE(mission_->ReadEvents(3, 8, &batch).ok());
  ASSERT_EQ(batch.events.size(), 1);
  EXPECT_EQ(batch.events.front().owner, Role::kControl);
  EXPECT_EQ(batch.events.front().operation.operation_id,
            "child-terminal-result");
  EXPECT_EQ(batch.events.front().operation.guards[1].sequence, 2);
}

TEST_F(StoreTest, RejectedAdmissionDoesNotReplaceAcceptedAuthorityProjection) {
  Commit mission_commit;
  ASSERT_TRUE(mission_->Submit(Mission(1), &mission_commit).ok());
  std::unique_ptr<Store> planning;
  ASSERT_TRUE(Store::Open({path_, Role::kPlanning}, &planning).ok());

  auto admission = Mission(1, "planning-admission-accepted");
  admission.channel = Channel::kPlanningStatus;
  admission.identity = {"planning-epoch", "mission-1", "command-1", 1};
  admission.guards = {{Channel::kMission, mission_commit.sequence}};
  admission.payload = "accepted-authority";
  admission.planning_status_kind = PlanningStatusKind::kAdmissionAccepted;
  Commit accepted_commit;
  ASSERT_TRUE(planning->Submit(admission, &accepted_commit).ok());
  Snapshot snapshot;
  ASSERT_TRUE(planning->ReadSnapshot(&snapshot).ok());
  ASSERT_TRUE(snapshot.accepted_authority);
  EXPECT_EQ(snapshot.accepted_authority->sequence, accepted_commit.sequence);
  EXPECT_EQ(snapshot.accepted_authority->operation.payload,
            "accepted-authority");

  admission.operation_id = "planning-admission-rejected";
  admission.identity.command_id = "rejected-replace";
  admission.identity.revision = 2;
  admission.expected_revision = 1;
  admission.payload = "rejected-replacement";
  admission.planning_status_kind = PlanningStatusKind::kAdmissionRejected;
  Commit rejected_commit;
  ASSERT_TRUE(planning->Submit(admission, &rejected_commit).ok());
  ASSERT_TRUE(planning->ReadSnapshot(&snapshot).ok());
  ASSERT_TRUE(
      snapshot.latest[static_cast<size_t>(Channel::kPlanningStatus)]);
  EXPECT_EQ(snapshot.latest[static_cast<size_t>(Channel::kPlanningStatus)]
                ->sequence,
            rejected_commit.sequence);
  ASSERT_TRUE(snapshot.accepted_authority);
  EXPECT_EQ(snapshot.accepted_authority->sequence, accepted_commit.sequence);

  Commit next_mission_commit;
  ASSERT_TRUE(mission_->Submit(Mission(2), &next_mission_commit).ok());
  admission.operation_id = "planning-admission-stale-accepted";
  admission.identity.command_id = "stale-accept";
  admission.identity.revision = 3;
  admission.expected_revision = 2;
  admission.guards = {{Channel::kMission, mission_commit.sequence}};
  admission.planning_status_kind = PlanningStatusKind::kAdmissionAccepted;
  EXPECT_EQ(planning->Submit(admission, &rejected_commit).code, Code::kConflict);
  ASSERT_TRUE(planning->ReadSnapshot(&snapshot).ok());
  ASSERT_TRUE(snapshot.accepted_authority);
  EXPECT_EQ(snapshot.accepted_authority->sequence, accepted_commit.sequence);
}

TEST_F(StoreTest, AcceptedAuthorityProjectionFailureRollsBackAdmission) {
  Commit mission_commit;
  ASSERT_TRUE(mission_->Submit(Mission(1), &mission_commit).ok());
  std::unique_ptr<Store> planning;
  ASSERT_TRUE(Store::Open({path_, Role::kPlanning}, &planning).ok());
  sqlite3* raw = nullptr;
  ASSERT_EQ(sqlite3_open(path_.c_str(), &raw), SQLITE_OK);
  std::unique_ptr<sqlite3, decltype(&sqlite3_close)> db(raw, sqlite3_close);
  ASSERT_EQ(sqlite3_exec(
                raw,
                "CREATE TRIGGER fail_authority BEFORE INSERT ON "
                "accepted_authority BEGIN SELECT RAISE(ABORT,'injected "
                "failure'); END",
                nullptr, nullptr, nullptr),
            SQLITE_OK);

  auto admission = Mission(1, "planning-admission");
  admission.channel = Channel::kPlanningStatus;
  admission.identity = {"planning-epoch", "mission-1", "command-1", 1};
  admission.guards = {{Channel::kMission, mission_commit.sequence}};
  admission.planning_status_kind = PlanningStatusKind::kAdmissionAccepted;
  Commit unchanged{99, 99, false};
  EXPECT_EQ(planning->Submit(admission, &unchanged).code, Code::kConflict);
  EXPECT_EQ(unchanged.sequence, 99);

  Snapshot snapshot;
  ASSERT_TRUE(planning->ReadSnapshot(&snapshot).ok());
  EXPECT_FALSE(snapshot.latest[static_cast<size_t>(Channel::kPlanningStatus)]);
  EXPECT_FALSE(snapshot.accepted_authority);
  EXPECT_EQ(snapshot.sequence, mission_commit.sequence);

  ASSERT_EQ(sqlite3_exec(raw, "DROP TRIGGER fail_authority", nullptr, nullptr,
                         nullptr),
            SQLITE_OK);
  Commit committed;
  ASSERT_TRUE(planning->Submit(admission, &committed).ok());
  ASSERT_TRUE(planning->ReadSnapshot(&snapshot).ok());
  ASSERT_TRUE(snapshot.accepted_authority);
  EXPECT_EQ(snapshot.accepted_authority->sequence, committed.sequence);
}

TEST_F(StoreTest, RestartOrderedReplayAndDurableCursor) {
  Commit commit;
  for (uint64_t i = 1; i <= 7; ++i) {
    ASSERT_TRUE(mission_->Submit(Mission(i), &commit).ok());
  }
  const Consumer consumer{Role::kMission, "reader-epoch"};
  uint64_t cursor = 100;
  ASSERT_TRUE(mission_->ReadCursor(consumer, &cursor).ok());
  EXPECT_EQ(cursor, 0);
  EventBatch batch;
  ASSERT_TRUE(mission_->ReadEvents(cursor, 3, &batch).ok());
  ASSERT_EQ(batch.events.size(), 3);
  EXPECT_EQ(batch.high_watermark, 7);
  EXPECT_EQ(batch.next_sequence, 3);
  EXPECT_EQ(batch.events[1].sequence, 2);
  ASSERT_TRUE(mission_->Acknowledge(consumer, 0, 3).ok());
  EXPECT_EQ(mission_->Acknowledge(consumer, 0, 4).code, Code::kConflict);
  EXPECT_EQ(mission_->Acknowledge(consumer, 3, 8).code, Code::kNotFound);
  EXPECT_EQ(mission_->Acknowledge({Role::kControl, "reader-epoch"}, 0, 3).code,
            Code::kUnauthorized);
  mission_.reset();
  ASSERT_TRUE(Store::Open({path_, Role::kMission}, &mission_).ok());
  ASSERT_TRUE(mission_->ReadCursor(consumer, &cursor).ok());
  EXPECT_EQ(cursor, 3);
  ASSERT_TRUE(mission_->ReadEvents(cursor, 3, &batch).ok());
  EXPECT_EQ(batch.events.front().sequence, 4);
  EXPECT_EQ(batch.next_sequence, 6);
  ASSERT_TRUE(
      mission_->ReadCursor({Role::kMission, "new-epoch"}, &cursor).ok());
  EXPECT_EQ(cursor, 0);
  EXPECT_EQ(mission_->ReadEvents(8, 1, &batch).code, Code::kNotFound);
  EXPECT_EQ(mission_->ReadEvents(0, Store::kMaxBatchSize + 1, &batch).code,
            Code::kInvalidArgument);
}

TEST_F(StoreTest, WriterClaimsPublishReadinessAndFenceSupersededEpochs) {
  const uint64_t old_generation = mission_->fencing_generation();
  ASSERT_GT(old_generation, 0);
  std::vector<Participant> participants;
  ASSERT_TRUE(mission_->ReadParticipants(&participants).ok());
  ASSERT_EQ(participants.size(), 1);
  EXPECT_FALSE(participants.front().ready);
  ASSERT_TRUE(mission_->SetReady(true).ok());
  ASSERT_TRUE(mission_->ReadParticipants(&participants).ok());
  EXPECT_TRUE(participants.front().ready);

  std::unique_ptr<Store> duplicate;
  EXPECT_EQ(Store::Open({path_, Role::kMission, 1000, "duplicate"}, &duplicate)
                .code,
            Code::kBusy);
  EXPECT_EQ(duplicate, nullptr);

  ASSERT_TRUE(mission_->ReleaseWriter().ok());
  std::unique_ptr<Store> replacement;
  ASSERT_TRUE(Store::Open(
                  {path_, Role::kMission, 1000, "replacement", "v2",
                   {"mission-v2"}},
                  &replacement)
                  .ok());
  EXPECT_GT(replacement->fencing_generation(), old_generation);
  Commit stale_commit;
  EXPECT_EQ(mission_->Submit(Mission(1), &stale_commit).code, Code::kFenced);
  ASSERT_TRUE(replacement->ReadParticipants(&participants).ok());
  ASSERT_EQ(participants.size(), 1);
  EXPECT_EQ(participants.front().epoch, "replacement");
  EXPECT_EQ(participants.front().contract_version, "v2");
  EXPECT_EQ(participants.front().capabilities,
            (std::vector<std::string>{"mission-v2"}));
  EXPECT_FALSE(participants.front().ready);

  Commit commit;
  ASSERT_TRUE(replacement->Submit(Mission(1), &commit).ok());
  EXPECT_EQ(commit.fencing_generation, replacement->fencing_generation());
  EventBatch batch;
  ASSERT_TRUE(replacement->ReadEvents(0, 64, &batch).ok());
  ASSERT_EQ(batch.events.size(), 1);
  EXPECT_EQ(batch.events.front().operation.fencing_generation,
            replacement->fencing_generation());
}

TEST_F(StoreTest, ExpiredWriterCanBeTakenOverAndOldGenerationIsFenced) {
  ASSERT_TRUE(mission_->ReleaseWriter().ok());
  mission_.reset();
  std::unique_ptr<Store> expiring;
  ASSERT_TRUE(Store::Open(
                  {path_, Role::kMission, 1000, "expiring", "v1", {}, 25},
                  &expiring)
                  .ok());
  std::this_thread::sleep_for(std::chrono::milliseconds(60));

  std::unique_ptr<Store> replacement;
  ASSERT_TRUE(Store::Open(
                  {path_, Role::kMission, 1000, "lease-takeover", "v1", {},
                   1000},
                  &replacement)
                  .ok());
  Commit stale_commit;
  EXPECT_EQ(expiring->Submit(Mission(1), &stale_commit).code, Code::kFenced);
  Commit commit;
  ASSERT_TRUE(replacement->Submit(Mission(1), &commit).ok());
  EXPECT_GT(commit.fencing_generation, 1);
}

TEST_F(StoreTest, SnapshotContainsCoherentLatestHead) {
  for (uint64_t i = 1; i <= 50; ++i) {
    Commit commit;
    ASSERT_TRUE(mission_->Submit(Mission(i), &commit).ok());
    Snapshot snapshot;
    ASSERT_TRUE(mission_->ReadSnapshot(&snapshot).ok());
    ASSERT_TRUE(snapshot.latest[0].has_value());
    EXPECT_EQ(snapshot.latest[0]->sequence, snapshot.sequence);
    EXPECT_EQ(snapshot.latest[0]->operation.identity.revision,
              snapshot.sequence);
    EXPECT_EQ(snapshot.latest[0]->operation.fencing_generation,
              mission_->fencing_generation());
  }
}

TEST_F(StoreTest, InvalidInputsAreBoundedAndPreserveOutputs) {
  Commit commit{99, 99, false};
  auto op = Mission(1);
  op.payload.resize(Store::kMaxPayloadBytes + 1);
  EXPECT_EQ(mission_->Submit(op, &commit).code, Code::kInvalidArgument);
  op = Mission(1);
  op.identity.revision = 2;
  EXPECT_EQ(mission_->Submit(op, &commit).code, Code::kInvalidArgument);
  op = Mission(1);
  op.channel = static_cast<Channel>(9);
  EXPECT_EQ(mission_->Submit(op, &commit).code, Code::kInvalidArgument);
  EXPECT_EQ(commit.sequence, 99);
  EventBatch batch;
  batch.high_watermark = 99;
  EXPECT_EQ(mission_->ReadEvents(0, 0, &batch).code, Code::kInvalidArgument);
  EXPECT_EQ(batch.high_watermark, 99);
}

TEST_F(StoreTest, BusyAndFailedTransactionPreserveOutputsAndHistory) {
  mission_.reset();
  ASSERT_TRUE(Store::Open({path_, Role::kMission, 0}, &mission_).ok());
  sqlite3* raw = nullptr;
  ASSERT_EQ(sqlite3_open(path_.c_str(), &raw), SQLITE_OK);
  std::unique_ptr<sqlite3, decltype(&sqlite3_close)> db(raw, sqlite3_close);
  ASSERT_EQ(sqlite3_exec(raw, "BEGIN IMMEDIATE", nullptr, nullptr, nullptr),
            SQLITE_OK);
  Commit unchanged{99, 99, false};
  EXPECT_EQ(mission_->Submit(Mission(1), &unchanged).code, Code::kBusy);
  EXPECT_EQ(unchanged.sequence, 99);
  ASSERT_EQ(sqlite3_exec(raw, "ROLLBACK", nullptr, nullptr, nullptr),
            SQLITE_OK);
  ASSERT_EQ(
      sqlite3_exec(raw,
                   "CREATE TRIGGER fail_head BEFORE INSERT ON heads BEGIN "
                   "SELECT RAISE(ABORT,'injected failure'); END",
                   nullptr, nullptr, nullptr),
      SQLITE_OK);
  EXPECT_EQ(mission_->Submit(Mission(1), &unchanged).code, Code::kConflict);
  EXPECT_EQ(unchanged.sequence, 99);
  EventBatch batch;
  ASSERT_TRUE(mission_->ReadEvents(0, 64, &batch).ok());
  EXPECT_TRUE(batch.events.empty());
  EXPECT_EQ(batch.high_watermark, 0);
  Snapshot snapshot;
  ASSERT_TRUE(mission_->ReadSnapshot(&snapshot).ok());
  EXPECT_FALSE(snapshot.latest[0]);
  ASSERT_EQ(
      sqlite3_exec(raw, "DROP TRIGGER fail_head", nullptr, nullptr, nullptr),
      SQLITE_OK);
  ASSERT_TRUE(mission_->Submit(Mission(1), &unchanged).ok());
  EXPECT_EQ(unchanged.sequence, 1);
}

TEST_F(StoreTest, CorruptionIsExplicitAndOpenOutputPreserved) {
  const std::string corrupt = path_ + ".corrupt";
  {
    std::ofstream stream(corrupt, std::ios::binary);
    stream << std::string(4096, 'x');
  }
  Store* original = mission_.get();
  EXPECT_EQ(Store::Open({corrupt, Role::kMission}, &mission_).code,
            Code::kCorrupt);
  EXPECT_EQ(mission_.get(), original);
  std::remove(corrupt.c_str());
}

TEST_F(StoreTest, WorkerBackpressureCompletionsAndSnapshots) {
  mission_.reset();
  WorkerOptions options;
  options.store = {path_, Role::kMission};
  options.capacity = 1;
  std::unique_ptr<Worker> worker;
  ASSERT_TRUE(Worker::Start(options, &worker).ok());
  uint64_t ticket = 0;
  Result submitted;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  do {
    submitted = worker->TrySubmit(Mission(1), &ticket);
    std::this_thread::yield();
  } while (submitted.code == Code::kBusy &&
           std::chrono::steady_clock::now() < deadline);
  ASSERT_TRUE(submitted.ok());
  Result full;
  uint64_t unused = 123;
  do {
    full = worker->TrySubmit(Mission(2), &unused);
    std::this_thread::yield();
  } while (full.code == Code::kBusy &&
           std::chrono::steady_clock::now() < deadline);
  EXPECT_EQ(full.code, Code::kQueueFull);
  EXPECT_EQ(unused, 123);
  worker->Stop();
  Completion completion;
  ASSERT_TRUE(worker->TryTakeCompletion(&completion).ok());
  EXPECT_EQ(completion.ticket, ticket);
  EXPECT_TRUE(completion.result.ok());
  EXPECT_EQ(completion.commit.sequence, 1);
  const auto view = worker->Latest();
  ASSERT_TRUE(view);
  EXPECT_TRUE(view->result.ok());
  ASSERT_TRUE(view->snapshot);
  EXPECT_EQ(view->snapshot->sequence, 1);
  EXPECT_EQ(worker->TrySubmit(Mission(2), &unused).code, Code::kStopped);
  EXPECT_EQ(worker->TryTakeCompletion(&completion).code, Code::kNotFound);
}

bool WaitReady(Client* client) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  do {
    std::vector<Event> events;
    const Result result = client->Poll(&events);
    if (client->Ready()) {
      return true;
    }
    if (!result.ok() && result.code != Code::kBusy) {
      return false;
    }
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}

TEST_F(StoreTest, ClientStartsAtTailAndReplaysOnlyLiveOrderedEvents) {
  Commit commit;
  ASSERT_TRUE(mission_->Submit(Mission(1), &commit).ok());
  Client client;
  ASSERT_TRUE(client
                  .Init(std::filesystem::absolute(path_).string(),
                        Role::kPlanning, "planning-live", 8, "test-v2",
                        {"motion-directive-v1"})
                  .ok());
  ASSERT_TRUE(WaitReady(&client));
  EXPECT_EQ(client.cursor(), 1);
  bool planning_ready = false;
  const auto readiness_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  do {
    for (const auto& participant : client.Participants()) {
      if (participant.role == Role::kPlanning &&
          participant.epoch == "planning-live") {
        planning_ready = participant.ready &&
                         participant.contract_version == "test-v2" &&
                         participant.capabilities ==
                             std::vector<std::string>{"motion-directive-v1"};
      }
    }
    if (!planning_ready) {
      std::this_thread::yield();
    }
  } while (!planning_ready &&
           std::chrono::steady_clock::now() < readiness_deadline);
  EXPECT_TRUE(planning_ready);
  std::vector<Event> events;
  ASSERT_TRUE(client.Poll(&events).ok());
  EXPECT_TRUE(events.empty());

  const auto live_mission_result = mission_->Submit(Mission(2), &commit);
  ASSERT_TRUE(live_mission_result.ok())
      << "code=" << static_cast<int>(live_mission_result.code)
      << " sqlite_code=" << live_mission_result.sqlite_code
      << " " << live_mission_result.message;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  do {
    ASSERT_TRUE(client.Poll(&events).ok());
    std::this_thread::yield();
  } while (events.empty() && std::chrono::steady_clock::now() < deadline);
  ASSERT_EQ(events.size(), 1);
  EXPECT_EQ(events.front().sequence, 2);
  ASSERT_TRUE(client.Acknowledge(events.back().sequence).ok());
  do {
    ASSERT_TRUE(client.Poll(&events).ok());
    std::this_thread::yield();
  } while (client.cursor() != 2 && std::chrono::steady_clock::now() < deadline);
  EXPECT_EQ(client.cursor(), 2);
}

TEST_F(StoreTest, ClientSeparatesAdmissionCommitAndTransportRevision) {
  mission_.reset();
  Client client;
  ASSERT_TRUE(client
                  .Init(std::filesystem::absolute(path_).string(),
                        Role::kMission, "mission-live", 1)
                  .ok());
  ASSERT_TRUE(WaitReady(&client));
  uint64_t first_ticket = 0;
  ASSERT_TRUE(client
                  .Submit(Channel::kMission, "proto-revision-900", {}, false,
                          false, &first_ticket)
                  .ok());
  uint64_t duplicate_ticket = 0;
  ASSERT_TRUE(client
                  .Submit(Channel::kMission, "proto-revision-900", {}, false,
                          false, &duplicate_ticket)
                  .ok());
  EXPECT_EQ(duplicate_ticket, first_ticket);
  uint64_t blocked_ticket = 0;
  EXPECT_EQ(client
                .Submit(Channel::kMission, "different", {}, false, false,
                        &blocked_ticket)
                .code,
            Code::kQueueFull);

  Submission submission;
  bool completed = false;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  do {
    std::vector<Event> events;
    ASSERT_TRUE(client.Poll(&events).ok());
    if (client.TakeSubmission(&submission).ok()) {
      completed = true;
      break;
    }
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < deadline);
  ASSERT_TRUE(completed);
  ASSERT_TRUE(submission.result.ok());
  EXPECT_EQ(submission.operation.identity.revision, 1);
  EXPECT_EQ(submission.operation.payload, "proto-revision-900");
  EXPECT_EQ(submission.commit.sequence, 1);
}

TEST_F(StoreTest, ClientRetriesBusyWithoutLosingTicketsOrAdvancingCursor) {
  mission_.reset();
  Client client;
  ASSERT_TRUE(client.Init(std::filesystem::absolute(path_).string(),
                          Role::kMission, "mission-busy", 4).ok());
  ASSERT_TRUE(WaitReady(&client));
  sqlite3* raw = nullptr;
  const int opened = sqlite3_open(path_.c_str(), &raw);
  std::unique_ptr<sqlite3, decltype(&sqlite3_close)> locker(raw, &sqlite3_close);
  ASSERT_EQ(opened, SQLITE_OK);
  ASSERT_EQ(sqlite3_busy_timeout(raw, 1000), SQLITE_OK);
  ASSERT_EQ(sqlite3_exec(raw, "BEGIN IMMEDIATE", nullptr, nullptr, nullptr),
            SQLITE_OK) << sqlite3_errmsg(raw);
  uint64_t first_ticket = 0;
  uint64_t second_ticket = 0;
  ASSERT_TRUE(client.Submit(Channel::kMission, "first", {}, false, false,
                            &first_ticket).ok());
  ASSERT_TRUE(client.Submit(Channel::kMission, "second", {}, false, false,
                            &second_ticket).ok());
  const auto blocked_until = std::chrono::steady_clock::now() +
                             std::chrono::milliseconds(250);
  std::vector<Event> events;
  while (std::chrono::steady_clock::now() < blocked_until) {
    const auto poll = client.Poll(&events);
    ASSERT_TRUE(poll.ok() || poll.code == Code::kBusy) << poll.message;
    Submission premature;
    EXPECT_EQ(client.TakeSubmission(&premature).code, Code::kNotFound);
    EXPECT_EQ(client.cursor(), 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  uint64_t retry_ticket = 0;
  ASSERT_TRUE(client.Submit(Channel::kMission, "first", {}, false, false,
                            &retry_ticket).ok());
  EXPECT_EQ(retry_ticket, first_ticket);
  ASSERT_EQ(sqlite3_exec(raw, "COMMIT", nullptr, nullptr, nullptr), SQLITE_OK);
  std::vector<Submission> committed;
  bool recovered = false;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(5);
  do {
    const auto poll = client.Poll(&events);
    ASSERT_TRUE(poll.ok() || poll.code == Code::kBusy) << poll.message;
    Submission submission;
    while (client.TakeSubmission(&submission).ok()) {
      committed.push_back(submission);
    }
    const auto view = client.Latest();
    recovered = client.Healthy() && view && view->snapshot &&
                view->snapshot->sequence == 2;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while ((committed.size() < 2 || !recovered) &&
           std::chrono::steady_clock::now() < deadline);
  ASSERT_EQ(committed.size(), 2);
  ASSERT_TRUE(committed[0].result.ok()) << committed[0].result.message;
  ASSERT_TRUE(committed[1].result.ok()) << committed[1].result.message;
  EXPECT_EQ(committed[0].ticket, first_ticket);
  EXPECT_EQ(committed[1].ticket, second_ticket);
  EXPECT_EQ(committed[0].commit.sequence, 1);
  EXPECT_EQ(committed[1].commit.sequence, 2);
  EXPECT_EQ(committed[0].operation.identity.revision, 1);
  EXPECT_EQ(committed[1].operation.identity.revision, 2);
  ASSERT_TRUE(client.Healthy());
  ASSERT_TRUE(client.Fault().ok());
  ASSERT_TRUE(client.Latest()->snapshot);
  EXPECT_EQ(client.Latest()->snapshot->sequence, 2);
  if (events.empty()) {
    do {
      ASSERT_TRUE(client.Poll(&events).ok());
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (events.empty() && std::chrono::steady_clock::now() < deadline);
  }
  ASSERT_FALSE(events.empty());
  const uint64_t through = events.back().sequence;
  ASSERT_EQ(sqlite3_exec(raw, "BEGIN IMMEDIATE", nullptr, nullptr, nullptr),
            SQLITE_OK);
  ASSERT_TRUE(client.Acknowledge(through).ok());
  const auto ack_blocked_until = std::chrono::steady_clock::now() +
                                 std::chrono::milliseconds(250);
  while (std::chrono::steady_clock::now() < ack_blocked_until) {
    const auto poll = client.Poll(&events);
    ASSERT_TRUE(poll.ok() || poll.code == Code::kBusy) << poll.message;
    EXPECT_EQ(client.cursor(), 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_EQ(sqlite3_exec(raw, "COMMIT", nullptr, nullptr, nullptr), SQLITE_OK);
  const auto ack_deadline = std::chrono::steady_clock::now() +
                            std::chrono::seconds(5);
  do {
    const auto poll = client.Poll(&events);
    ASSERT_TRUE(poll.ok() || poll.code == Code::kBusy) << poll.message;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while ((client.cursor() != through || !client.Healthy()) &&
           std::chrono::steady_clock::now() < ack_deadline);
  EXPECT_EQ(client.cursor(), through);
  EXPECT_TRUE(client.Healthy());
  EXPECT_TRUE(client.Fault().ok());
}

TEST_F(StoreTest, ClientRequiresAbsolutePersistentPath) {
  Client client;
  EXPECT_EQ(client.Init("", Role::kMission, "epoch").code,
            Code::kInvalidArgument);
  EXPECT_EQ(client.Init("relative.db", Role::kMission, "epoch").code,
            Code::kInvalidArgument);
  EXPECT_EQ(client
                .Init("/directory-that-does-not-exist/state.db", Role::kMission,
                      "epoch")
                .code,
            Code::kInvalidArgument);
}

TEST_F(StoreTest, HistoricalControlStatusCommitsWithoutRelabeling) {
  Commit commit;
  ASSERT_TRUE(mission_->Submit(Mission(1), &commit).ok());
  std::unique_ptr<Store> planning;
  ASSERT_TRUE(Store::Open({path_, Role::kPlanning}, &planning).ok());
  auto motion = Mission(1, "motion");
  motion.channel = Channel::kMotion;
  motion.guards = {{Channel::kMission, 1}};
  ASSERT_TRUE(planning->Submit(motion, &commit).ok());

  Client control;
  ASSERT_TRUE(control
                  .Init(std::filesystem::absolute(path_).string(),
                        Role::kControl, "control-live")
                  .ok());
  ASSERT_TRUE(WaitReady(&control));

  const auto next_mission_result = mission_->Submit(Mission(2), &commit);
  ASSERT_TRUE(next_mission_result.ok())
      << "code=" << static_cast<int>(next_mission_result.code)
      << " sqlite_code=" << next_mission_result.sqlite_code
      << " " << next_mission_result.message;
  uint64_t ticket = 0;
  ASSERT_TRUE(control
                  .Submit(Channel::kControlStatus, "stale-status",
                          {{Channel::kMission, 1}, {Channel::kMotion, 2}},
                          false, false, &ticket)
                  .ok());

  Submission submission;
  bool completed = false;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  do {
    std::vector<Event> events;
    ASSERT_TRUE(control.Poll(&events).ok());
    if (control.TakeSubmission(&submission).ok()) {
      completed = true;
      break;
    }
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < deadline);
  ASSERT_TRUE(completed);
  EXPECT_EQ(submission.ticket, ticket);
  EXPECT_TRUE(submission.result.ok());
  EXPECT_EQ(submission.operation.guards[0].sequence, 1);
  EXPECT_EQ(submission.operation.guards[1].sequence, 2);
  EXPECT_TRUE(control.Healthy());
}

TEST_F(StoreTest, MissionClientUsesReleasedSingleWriterClaim) {
  ASSERT_TRUE(mission_->ReleaseWriter().ok());
  mission_.reset();
  Client mission;
  ASSERT_TRUE(mission
                  .Init(std::filesystem::absolute(path_).string(),
                        Role::kMission, "mission-live")
                  .ok());
  ASSERT_TRUE(WaitReady(&mission));

  uint64_t ticket = 0;
  ASSERT_TRUE(
      mission.Submit(Channel::kMission, "mission-intent", {}, false, false,
                     &ticket)
          .ok());

  Submission submission;
  bool completed = false;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  do {
    std::vector<Event> events;
    const Result poll = mission.Poll(&events);
    if (!poll.ok() && poll.code != Code::kConflict) {
      FAIL() << poll.message;
    }
    if (mission.TakeSubmission(&submission).ok()) {
      completed = true;
      break;
    }
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < deadline);
  ASSERT_TRUE(completed);
  EXPECT_EQ(submission.ticket, ticket);
  ASSERT_TRUE(submission.result.ok());
  EXPECT_EQ(submission.commit.sequence, 1);
  EXPECT_TRUE(mission.Healthy());
}

}  // namespace
}  // namespace execution_state_sync
}  // namespace apollo
