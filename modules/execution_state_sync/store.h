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

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace apollo {
namespace execution_state_sync {

enum class Role { kMission = 0, kPlanning = 1, kControl = 2 };
enum class Channel {
  kMission = 0,
  kMotion = 1,
  kPlanningStatus = 2,
  kControlStatus = 3,
  kSafetyRequest = 4,
  kSafetyStatus = 5
};
enum class PlanningStatusKind {
  kRuntime = 0,
  kAdmissionAccepted = 1,
  kAdmissionRejected = 2,
  kAdmissionDuplicate = 3
};
enum class ControlStatusKind { kMotionResult = 0, kOwnerRuntime = 1 };
enum class Code {
  kOk,
  kInvalidArgument,
  kUnauthorized,
  kNotFound,
  kConflict,
  kFenced,
  kBusy,
  kCorrupt,
  kIoError,
  kQueueFull,
  kStopped,
  kInternal
};

struct Result {
  Code code = Code::kOk;
  std::string message;
  int sqlite_code = 0;
  bool ok() const { return code == Code::kOk; }
};

struct Identity {
  std::string epoch;
  std::string aggregate_id;
  std::string command_id;
  uint64_t revision = 0;
};

struct Guard {
  // Commands/accepted admission require the current head. Observational
  // statuses reference an existing immutable event, even after head advances.
  Channel channel = Channel::kMission;
  uint64_t sequence = 0;
};

struct Operation {
  std::string operation_id;
  Channel channel = Channel::kMission;
  Identity identity;
  uint64_t expected_revision = 0;
  // Serialized protobuf bytes. The caller owns schema and domain validation.
  std::string payload;
  std::vector<Guard> guards;
  // Only mission writes this flag. It fences ordinary subsequent motion writes.
  bool mission_fenced = false;
  // Only motion may set this, for caller-validated stop/cleanup operations.
  bool allow_when_fenced = false;
  // Assigned by Store from its live writer claim; callers cannot choose it.
  uint64_t fencing_generation = 0;
  PlanningStatusKind planning_status_kind = PlanningStatusKind::kRuntime;
  ControlStatusKind control_status_kind =
      ControlStatusKind::kMotionResult;
};

struct Commit {
  uint64_t sequence = 0;
  uint64_t revision = 0;
  bool duplicate = false;
  uint64_t fencing_generation = 0;
};

struct Event {
  uint64_t sequence = 0;
  Role owner = Role::kMission;
  Operation operation;
};

struct Snapshot {
  uint64_t sequence = 0;
  std::array<std::optional<Event>, 6> latest;
  std::optional<Event> accepted_authority;
};

struct EventBatch {
  uint64_t high_watermark = 0;
  uint64_t next_sequence = 0;
  std::vector<Event> events;
};

struct Consumer {
  Role role = Role::kMission;
  std::string epoch;
};

struct Options {
  std::string path;
  Role role = Role::kMission;
  int busy_timeout_ms = 50;
  std::string writer_epoch;
  std::string contract_version = "v1";
  std::vector<std::string> capabilities;
  uint64_t writer_lease_ms = 5000;
};

struct Participant {
  Role role = Role::kMission;
  std::string epoch;
  uint64_t fencing_generation = 0;
  uint64_t lease_expiry_monotonic_ms = 0;
  bool ready = false;
  std::string contract_version;
  std::vector<std::string> capabilities;
};

// One connection per Store, confined to one thread. Multiple Stores/processes
// may share the same local database. Role checks are API policy, not a security
// boundary against processes with direct filesystem access.
class Store {
 public:
  static constexpr size_t kMaxPayloadBytes = 1024 * 1024;
  static constexpr size_t kMaxBatchSize = 64;
  static Result Open(const Options& options, std::unique_ptr<Store>* store);
  ~Store();
  Store(const Store&) = delete;
  Store& operator=(const Store&) = delete;

  Result Submit(const Operation& operation, Commit* commit);
  Result RenewWriter();
  Result SetReady(bool ready);
  Result ReadParticipants(std::vector<Participant>* participants);
  Result ReleaseWriter();
  Result ReadSnapshot(Snapshot* snapshot);
  Result ReadEvents(uint64_t after_sequence, size_t limit, EventBatch* batch);
  uint64_t fencing_generation() const;
  // An absent cursor is a successful zero cursor (replay from the beginning).
  Result ReadCursor(const Consumer& consumer, uint64_t* sequence);
  // Only the connection's role may acknowledge its consumer. Monotonic CAS;
  // through_sequence must identify an existing event. Ack only after
  // processing.
  Result Acknowledge(const Consumer& consumer, uint64_t expected_sequence,
                     uint64_t through_sequence);

 private:
  struct Impl;
  explicit Store(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace execution_state_sync
}  // namespace apollo
