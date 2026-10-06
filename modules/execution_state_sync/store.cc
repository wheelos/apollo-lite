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

#include <atomic>
#include <chrono>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <unistd.h>

namespace apollo {
namespace execution_state_sync {
namespace {

constexpr uint64_t kMaxInteger = std::numeric_limits<int64_t>::max();
constexpr int kApplicationId = 1163088729;
constexpr uint64_t kMaxWriterLeaseMs = 60000;

uint64_t MonotonicMs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

std::string DefaultEpoch() {
  static std::atomic<uint64_t> counter{0};
  return "store-" + std::to_string(getpid()) + "-" +
         std::to_string(MonotonicMs()) + "-" +
         std::to_string(counter.fetch_add(1));
}

Result ReadBootId(std::string* boot_id) {
  if (boot_id == nullptr) {
    return {Code::kInvalidArgument, "null boot identity output"};
  }
  std::ifstream input("/proc/sys/kernel/random/boot_id");
  if (!input) {
    return {Code::kIoError, "cannot read host boot identity"};
  }
  std::getline(input, *boot_id);
  if (boot_id->empty() || boot_id->size() > 128) {
    return {Code::kIoError, "host boot identity is invalid"};
  }
  return {};
}

std::string EncodeCapabilities(const std::vector<std::string>& capabilities) {
  std::string encoded;
  for (const auto& capability : capabilities) {
    if (!encoded.empty()) {
      encoded.push_back('\n');
    }
    encoded.append(capability);
  }
  return encoded;
}

std::vector<std::string> DecodeCapabilities(const std::string& encoded) {
  std::vector<std::string> capabilities;
  size_t begin = 0;
  while (begin < encoded.size()) {
    const size_t end = encoded.find('\n', begin);
    capabilities.push_back(encoded.substr(
        begin, end == std::string::npos ? std::string::npos : end - begin));
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1;
  }
  return capabilities;
}

Result Error(Code code, const std::string& message) {
  return {code, message, 0};
}

Result SqlResult(sqlite3* db, int rc) {
  if (rc == SQLITE_OK || rc == SQLITE_ROW || rc == SQLITE_DONE) {
    return {};
  }
  Code code = Code::kInternal;
  switch (rc & 0xff) {
    case SQLITE_BUSY:
    case SQLITE_LOCKED:
      code = Code::kBusy;
      break;
    case SQLITE_CORRUPT:
    case SQLITE_NOTADB:
      code = Code::kCorrupt;
      break;
    case SQLITE_IOERR:
    case SQLITE_CANTOPEN:
    case SQLITE_FULL:
    case SQLITE_READONLY:
    case SQLITE_PERM:
      code = Code::kIoError;
      break;
    case SQLITE_CONSTRAINT:
      code = Code::kConflict;
      break;
  }
  return {code, db ? sqlite3_errmsg(db) : sqlite3_errstr(rc), rc};
}

#define SYNC_RETURN_IF_ERROR(expression) \
  do {                                   \
    Result result = (expression);        \
    if (!result.ok()) {                  \
      return result;                     \
    }                                    \
  } while (false)

class Statement {
 public:
  explicit Statement(sqlite3* db) : db_(db) {}
  ~Statement() { sqlite3_finalize(statement_); }
  Statement(const Statement&) = delete;
  Statement& operator=(const Statement&) = delete;
  Result Prepare(const char* sql) {
    return SqlResult(db_,
                     sqlite3_prepare_v2(db_, sql, -1, &statement_, nullptr));
  }
  Result Bind(int index, uint64_t value) {
    return SqlResult(db_, sqlite3_bind_int64(statement_, index,
                                             static_cast<int64_t>(value)));
  }
  Result Bind(int index, const std::string& value) {
    return SqlResult(db_, sqlite3_bind_blob(statement_, index, value.data(),
                                            static_cast<int>(value.size()),
                                            SQLITE_TRANSIENT));
  }
  Result Step(bool* row = nullptr) {
    const int rc = sqlite3_step(statement_);
    if (row != nullptr) {
      *row = rc == SQLITE_ROW;
    }
    return SqlResult(db_, rc);
  }
  uint64_t Integer(int index) const {
    return static_cast<uint64_t>(sqlite3_column_int64(statement_, index));
  }
  std::string Bytes(int index) const {
    const int size = sqlite3_column_bytes(statement_, index);
    const auto* data =
        static_cast<const char*>(sqlite3_column_blob(statement_, index));
    return size == 0 ? std::string() : std::string(data, size);
  }

 private:
  sqlite3* db_;
  sqlite3_stmt* statement_ = nullptr;
};

Result Exec(sqlite3* db, const char* sql) {
  return SqlResult(db, sqlite3_exec(db, sql, nullptr, nullptr, nullptr));
}

class Transaction {
 public:
  explicit Transaction(sqlite3* db) : db_(db) {}
  ~Transaction() {
    if (active_) {
      sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, nullptr);
    }
  }
  Result Begin(bool write) {
    Result result = Exec(db_, write ? "BEGIN IMMEDIATE" : "BEGIN");
    active_ = result.ok();
    return result;
  }
  Result Commit() {
    Result result = Exec(db_, "COMMIT");
    if (result.ok()) {
      active_ = false;
    }
    return result;
  }

 private:
  sqlite3* db_;
  bool active_ = false;
};

bool ValidRole(Role role) {
  return role == Role::kMission || role == Role::kPlanning ||
         role == Role::kControl;
}

bool ValidChannel(Channel channel) {
  return channel == Channel::kMission || channel == Channel::kMotion ||
         channel == Channel::kPlanningStatus ||
         channel == Channel::kControlStatus ||
         channel == Channel::kSafetyRequest ||
         channel == Channel::kSafetyStatus;
}

bool ValidPlanningStatusKind(PlanningStatusKind kind) {
  return kind == PlanningStatusKind::kRuntime ||
         kind == PlanningStatusKind::kAdmissionAccepted ||
         kind == PlanningStatusKind::kAdmissionRejected ||
         kind == PlanningStatusKind::kAdmissionDuplicate;
}

bool ValidControlStatusKind(ControlStatusKind kind) {
  return kind == ControlStatusKind::kMotionResult ||
         kind == ControlStatusKind::kOwnerRuntime;
}

bool UpdatesAcceptedAuthority(PlanningStatusKind kind) {
  return kind == PlanningStatusKind::kAdmissionAccepted ||
         kind == PlanningStatusKind::kAdmissionDuplicate;
}

Result UpgradeSchemaToV5(sqlite3* db) {
  return Exec(
      db,
      "ALTER TABLE events ADD COLUMN control_status_kind INTEGER NOT NULL "
      "DEFAULT 0 CHECK(control_status_kind BETWEEN 0 AND 1);"
      "DROP TRIGGER IF EXISTS immutable_event_update;"
      "UPDATE events SET control_status_kind=1 WHERE channel=3 AND "
      "mission_parent=0 AND motion_parent=0;"
      "CREATE TRIGGER immutable_event_update BEFORE UPDATE ON events BEGIN "
      "SELECT RAISE(ABORT,'immutable event'); END;"
      "PRAGMA user_version=5;");
}

Role Owner(Channel channel) {
  if (channel == Channel::kMission || channel == Channel::kSafetyRequest) {
    return Role::kMission;
  }
  return channel == Channel::kControlStatus || channel == Channel::kSafetyStatus
             ? Role::kControl
             : Role::kPlanning;
}

bool ValidId(const std::string& id) { return !id.empty() && id.size() <= 256; }

Result UpgradeSchemaToV4(sqlite3* db) {
  return Exec(
      db,
      "DROP TRIGGER IF EXISTS immutable_event_update;"
      "DROP TRIGGER IF EXISTS immutable_event_delete;"
      "DROP TABLE heads;"
      "DROP TABLE accepted_authority;"
      "ALTER TABLE events RENAME TO old_events;"
      "CREATE TABLE events("
      "sequence INTEGER PRIMARY KEY AUTOINCREMENT,"
      "operation_id BLOB NOT NULL UNIQUE,"
      "owner INTEGER NOT NULL CHECK(owner BETWEEN 0 AND 2),"
      "channel INTEGER NOT NULL CHECK(channel BETWEEN 0 AND 5),"
      "epoch BLOB NOT NULL,aggregate_id BLOB NOT NULL,command_id BLOB NOT "
      "NULL,"
      "revision INTEGER NOT NULL CHECK(revision>0),"
      "expected_revision INTEGER NOT NULL CHECK(expected_revision>=0),"
      "payload BLOB NOT NULL CHECK(length(payload)<=1048576),"
      "mission_fenced INTEGER NOT NULL CHECK(mission_fenced IN (0,1)),"
      "allow_when_fenced INTEGER NOT NULL CHECK(allow_when_fenced IN (0,1)),"
      "mission_parent INTEGER NOT NULL CHECK(mission_parent>=0),"
      "motion_parent INTEGER NOT NULL CHECK(motion_parent>=0),"
      "fencing_generation INTEGER NOT NULL CHECK(fencing_generation>0),"
      "planning_status_kind INTEGER NOT NULL "
      "CHECK(planning_status_kind BETWEEN 0 AND 3),"
      "UNIQUE(channel,revision)) STRICT;"
      "INSERT INTO events(sequence,operation_id,owner,channel,epoch,"
      "aggregate_id,command_id,revision,expected_revision,payload,"
      "mission_fenced,allow_when_fenced,mission_parent,motion_parent,"
      "fencing_generation,planning_status_kind) "
      "SELECT sequence,operation_id,"
      "owner,channel,epoch,aggregate_id,command_id,revision,expected_revision,"
      "payload,mission_fenced,allow_when_fenced,mission_parent,motion_parent,"
      "fencing_generation,planning_status_kind "
      "FROM old_events;"
      "DROP TABLE old_events;"
      "CREATE TABLE heads(channel INTEGER PRIMARY KEY CHECK(channel BETWEEN "
      "0 AND 5),sequence INTEGER NOT NULL REFERENCES events(sequence)) "
      "STRICT;"
      "INSERT INTO heads(channel,sequence) SELECT channel,MAX(sequence) FROM "
      "events GROUP BY channel;"
      "CREATE TABLE accepted_authority("
      "singleton INTEGER PRIMARY KEY CHECK(singleton=1),"
      "sequence INTEGER NOT NULL REFERENCES events(sequence)) STRICT;"
      "INSERT INTO accepted_authority(singleton,sequence) SELECT 1,sequence "
      "FROM events WHERE channel=2 AND planning_status_kind IN (1,3) "
      "ORDER BY sequence DESC LIMIT 1;"
      "CREATE TRIGGER immutable_event_update BEFORE UPDATE ON events BEGIN "
      "SELECT RAISE(ABORT,'immutable event'); END;"
      "CREATE TRIGGER immutable_event_delete BEFORE DELETE ON events BEGIN "
      "SELECT RAISE(ABORT,'immutable event'); END;"
      "PRAGMA user_version=4;");
}

uint64_t Parent(const Operation& operation, Channel channel) {
  for (const auto& guard : operation.guards) {
    if (guard.channel == channel) {
      return guard.sequence;
    }
  }
  return 0;
}

Result Validate(const Operation& op, Role role) {
  if (!ValidChannel(op.channel) || !ValidId(op.operation_id) ||
      !ValidId(op.identity.epoch) || !ValidId(op.identity.aggregate_id) ||
      !ValidId(op.identity.command_id) || op.expected_revision >= kMaxInteger ||
      op.identity.revision != op.expected_revision + 1 ||
      op.payload.size() > Store::kMaxPayloadBytes || op.guards.size() > 2) {
    return Error(Code::kInvalidArgument,
                 "invalid operation identity or bounds");
  }
  if (!ValidPlanningStatusKind(op.planning_status_kind) ||
      (op.planning_status_kind != PlanningStatusKind::kRuntime &&
       op.channel != Channel::kPlanningStatus)) {
    return Error(Code::kInvalidArgument, "invalid planning status kind");
  }
  if (!ValidControlStatusKind(op.control_status_kind) ||
      (op.control_status_kind != ControlStatusKind::kMotionResult &&
       op.channel != Channel::kControlStatus)) {
    return Error(Code::kInvalidArgument, "invalid Control status kind");
  }
  if (Owner(op.channel) != role) {
    return Error(Code::kUnauthorized, "channel belongs to another role");
  }
  bool mission = false;
  bool motion = false;
  for (const auto& guard : op.guards) {
    if (guard.sequence == 0 || guard.sequence > kMaxInteger) {
      return Error(Code::kInvalidArgument, "invalid parent sequence");
    }
    if (guard.channel == Channel::kMission && !mission) {
      mission = true;
    } else if (guard.channel == Channel::kMotion && !motion) {
      motion = true;
    } else {
      return Error(Code::kInvalidArgument, "duplicate or unsupported guard");
    }
  }
  const bool mission_owned_channel =
      op.channel == Channel::kMission || op.channel == Channel::kSafetyRequest;
  const bool independent_safety_status = op.channel == Channel::kSafetyStatus;
  const bool owner_runtime_observation =
      op.guards.empty() &&
      ((op.channel == Channel::kControlStatus &&
        op.control_status_kind == ControlStatusKind::kOwnerRuntime) ||
       (op.channel == Channel::kPlanningStatus &&
        op.planning_status_kind == PlanningStatusKind::kRuntime));
  if ((mission_owned_channel && !op.guards.empty()) ||
      (independent_safety_status && !op.guards.empty()) ||
      (!mission_owned_channel && !independent_safety_status && !mission &&
       !owner_runtime_observation) ||
      (op.channel == Channel::kControlStatus &&
       (op.control_status_kind == ControlStatusKind::kOwnerRuntime
            ? !op.guards.empty()
            : (!mission || !motion))) ||
      (op.channel != Channel::kMission && op.mission_fenced) ||
      (op.channel != Channel::kMotion && op.allow_when_fenced)) {
    return Error(Code::kInvalidArgument,
                 "invalid causal guards or fence flags");
  }
  return {};
}

Event Decode(const Statement& statement) {
  Event event;
  event.sequence = statement.Integer(0);
  auto& op = event.operation;
  op.operation_id = statement.Bytes(1);
  event.owner = static_cast<Role>(statement.Integer(2));
  op.channel = static_cast<Channel>(statement.Integer(3));
  op.identity.epoch = statement.Bytes(4);
  op.identity.aggregate_id = statement.Bytes(5);
  op.identity.command_id = statement.Bytes(6);
  op.identity.revision = statement.Integer(7);
  op.expected_revision = statement.Integer(8);
  op.payload = statement.Bytes(9);
  op.mission_fenced = statement.Integer(10) != 0;
  op.allow_when_fenced = statement.Integer(11) != 0;
  if (statement.Integer(12) != 0) {
    op.guards.push_back({Channel::kMission, statement.Integer(12)});
  }
  if (statement.Integer(13) != 0) {
    op.guards.push_back({Channel::kMotion, statement.Integer(13)});
  }
  op.fencing_generation = statement.Integer(14);
  op.planning_status_kind =
      static_cast<PlanningStatusKind>(statement.Integer(15));
  op.control_status_kind =
      static_cast<ControlStatusKind>(statement.Integer(16));
  return event;
}

bool Same(const Operation& a, const Operation& b) {
  return a.operation_id == b.operation_id && a.channel == b.channel &&
         a.identity.epoch == b.identity.epoch &&
         a.identity.aggregate_id == b.identity.aggregate_id &&
         a.identity.command_id == b.identity.command_id &&
         a.identity.revision == b.identity.revision &&
         a.expected_revision == b.expected_revision && a.payload == b.payload &&
         a.mission_fenced == b.mission_fenced &&
         a.allow_when_fenced == b.allow_when_fenced &&
         a.planning_status_kind == b.planning_status_kind &&
         a.control_status_kind == b.control_status_kind &&
         Parent(a, Channel::kMission) == Parent(b, Channel::kMission) &&
         Parent(a, Channel::kMotion) == Parent(b, Channel::kMotion);
}

Result Scalar(sqlite3* db, const char* sql, uint64_t* value) {
  Statement statement(db);
  SYNC_RETURN_IF_ERROR(statement.Prepare(sql));
  bool row = false;
  SYNC_RETURN_IF_ERROR(statement.Step(&row));
  if (!row) {
    return Error(Code::kCorrupt, "missing scalar row");
  }
  *value = statement.Integer(0);
  return {};
}

constexpr char kEventColumns[] =
    "sequence,operation_id,owner,channel,epoch,aggregate_id,command_id,"
    "revision,expected_revision,payload,mission_fenced,allow_when_fenced,"
    "mission_parent,motion_parent,fencing_generation,planning_status_kind,"
    "control_status_kind";

Result Head(sqlite3* db, Channel channel, std::optional<Event>* event) {
  Statement statement(db);
  const std::string sql = std::string("SELECT ") + kEventColumns +
                          " FROM events WHERE sequence=(SELECT sequence FROM "
                          "heads WHERE channel=?)";
  SYNC_RETURN_IF_ERROR(statement.Prepare(sql.c_str()));
  SYNC_RETURN_IF_ERROR(statement.Bind(1, static_cast<uint64_t>(channel)));
  bool row = false;
  SYNC_RETURN_IF_ERROR(statement.Step(&row));
  *event = row ? std::optional<Event>(Decode(statement)) : std::nullopt;
  return {};
}

Result EventBySequence(sqlite3* db, uint64_t sequence,
                       std::optional<Event>* event) {
  Statement statement(db);
  const std::string sql =
      std::string("SELECT ") + kEventColumns + " FROM events WHERE sequence=?";
  SYNC_RETURN_IF_ERROR(statement.Prepare(sql.c_str()));
  SYNC_RETURN_IF_ERROR(statement.Bind(1, sequence));
  bool row = false;
  SYNC_RETURN_IF_ERROR(statement.Step(&row));
  *event = row ? std::optional<Event>(Decode(statement)) : std::nullopt;
  return {};
}

Result Cursor(sqlite3* db, const Consumer& consumer, uint64_t* sequence) {
  Statement statement(db);
  SYNC_RETURN_IF_ERROR(statement.Prepare(
      "SELECT sequence FROM cursors WHERE role=? AND epoch=?"));
  SYNC_RETURN_IF_ERROR(statement.Bind(1, static_cast<uint64_t>(consumer.role)));
  SYNC_RETURN_IF_ERROR(statement.Bind(2, consumer.epoch));
  bool row = false;
  SYNC_RETURN_IF_ERROR(statement.Step(&row));
  *sequence = row ? statement.Integer(0) : 0;
  return {};
}

}  // namespace

struct Store::Impl {
  sqlite3* db = nullptr;
  Role role = Role::kMission;
  std::string writer_epoch;
  std::string boot_id;
  std::string contract_version;
  std::string capabilities;
  uint64_t fencing_generation = 0;
  uint64_t writer_lease_ms = 0;
  ~Impl() {
    if (db != nullptr) {
      sqlite3_close_v2(db);
    }
  }
};

Store::Store(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
Store::~Store() {
  if (impl_ != nullptr && impl_->db != nullptr) {
    ReleaseWriter();
  }
}

Result Store::Open(const Options& options, std::unique_ptr<Store>* store) {
  if (store == nullptr || options.path.empty() || options.path == ":memory:" ||
      options.path.find('\0') != std::string::npos ||
      !ValidRole(options.role) || options.busy_timeout_ms < 0 ||
      options.busy_timeout_ms > 5000 ||
      options.contract_version.empty() || options.contract_version.size() > 64 ||
      options.writer_lease_ms == 0 ||
      options.writer_lease_ms > kMaxWriterLeaseMs ||
      options.capabilities.size() > 128) {
    return Error(Code::kInvalidArgument, "invalid persistent database options");
  }
  for (const auto& capability : options.capabilities) {
    if (!ValidId(capability) || capability.find('\n') != std::string::npos) {
      return Error(Code::kInvalidArgument, "invalid capability declaration");
    }
  }
  auto impl = std::make_unique<Impl>();
  impl->role = options.role;
  impl->writer_epoch =
      options.writer_epoch.empty() ? DefaultEpoch() : options.writer_epoch;
  if (!ValidId(impl->writer_epoch)) {
    return Error(Code::kInvalidArgument, "invalid writer epoch");
  }
  impl->contract_version = options.contract_version;
  impl->capabilities = EncodeCapabilities(options.capabilities);
  impl->writer_lease_ms = options.writer_lease_ms;
  SYNC_RETURN_IF_ERROR(ReadBootId(&impl->boot_id));
  const int rc = sqlite3_open_v2(
      options.path.c_str(), &impl->db,
      SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX,
      nullptr);
  SYNC_RETURN_IF_ERROR(SqlResult(impl->db, rc));
  SYNC_RETURN_IF_ERROR(
      SqlResult(impl->db, sqlite3_extended_result_codes(impl->db, 1)));
  SYNC_RETURN_IF_ERROR(SqlResult(
      impl->db, sqlite3_busy_timeout(impl->db, options.busy_timeout_ms)));
  SYNC_RETURN_IF_ERROR(Exec(impl->db, "PRAGMA foreign_keys=ON"));
  // Reject foreign databases before changing their journal mode.
  uint64_t application = 0;
  SYNC_RETURN_IF_ERROR(Scalar(impl->db, "PRAGMA application_id", &application));
  if (application != 0 && application != kApplicationId) {
    return Error(Code::kCorrupt, "database application ID mismatch");
  }
  {
    Statement wal(impl->db);
    SYNC_RETURN_IF_ERROR(wal.Prepare("PRAGMA journal_mode=WAL"));
    bool row = false;
    SYNC_RETURN_IF_ERROR(wal.Step(&row));
    if (!row || wal.Bytes(0) != "wal") {
      return Error(Code::kIoError, "persistent WAL mode unavailable");
    }
  }
  SYNC_RETURN_IF_ERROR(Exec(impl->db, "PRAGMA synchronous=FULL"));
  Transaction transaction(impl->db);
  SYNC_RETURN_IF_ERROR(transaction.Begin(true));
  uint64_t version = 0;
  SYNC_RETURN_IF_ERROR(Scalar(impl->db, "PRAGMA user_version", &version));
  SYNC_RETURN_IF_ERROR(Scalar(impl->db, "PRAGMA application_id", &application));
  if (version == 0 && application == 0) {
    uint64_t tables = 0;
    SYNC_RETURN_IF_ERROR(
        Scalar(impl->db,
               "SELECT COUNT(*) FROM sqlite_schema WHERE name NOT LIKE "
               "'sqlite_%'",
               &tables));
    if (tables != 0) {
      return Error(Code::kCorrupt, "refusing to initialize nonempty database");
    }
    SYNC_RETURN_IF_ERROR(Exec(
        impl->db,
        "CREATE TABLE events("
        "sequence INTEGER PRIMARY KEY AUTOINCREMENT,"
        "operation_id BLOB NOT NULL UNIQUE,"
        "owner INTEGER NOT NULL CHECK(owner BETWEEN 0 AND 2),"
        "channel INTEGER NOT NULL CHECK(channel BETWEEN 0 AND 5),"
        "epoch BLOB NOT NULL,aggregate_id BLOB NOT NULL,command_id BLOB NOT "
        "NULL,"
        "revision INTEGER NOT NULL CHECK(revision>0),"
        "expected_revision INTEGER NOT NULL CHECK(expected_revision>=0),"
        "payload BLOB NOT NULL CHECK(length(payload)<=1048576),"
        "mission_fenced INTEGER NOT NULL CHECK(mission_fenced IN (0,1)),"
        "allow_when_fenced INTEGER NOT NULL CHECK(allow_when_fenced IN (0,1)),"
        "mission_parent INTEGER NOT NULL CHECK(mission_parent>=0),"
        "motion_parent INTEGER NOT NULL CHECK(motion_parent>=0),"
        "fencing_generation INTEGER NOT NULL CHECK(fencing_generation>0),"
        "planning_status_kind INTEGER NOT NULL "
        "CHECK(planning_status_kind BETWEEN 0 AND 3),"
        "control_status_kind INTEGER NOT NULL "
        "CHECK(control_status_kind BETWEEN 0 AND 1),"
        "UNIQUE(channel,revision)) STRICT;"
        "CREATE TABLE heads(channel INTEGER PRIMARY KEY CHECK(channel BETWEEN "
        "0 AND 5),sequence INTEGER NOT NULL REFERENCES events(sequence)) "
        "STRICT;"
        "CREATE TABLE cursors(role INTEGER NOT NULL CHECK(role BETWEEN 0 AND "
        "2),"
        "epoch BLOB NOT NULL,sequence INTEGER NOT NULL CHECK(sequence>=0),"
        "PRIMARY KEY(role,epoch)) STRICT;"
        "CREATE TABLE writer_claims("
        "role INTEGER PRIMARY KEY CHECK(role BETWEEN 0 AND 2),"
        "epoch BLOB NOT NULL,"
        "generation INTEGER NOT NULL CHECK(generation>0),"
        "boot_id BLOB NOT NULL,"
        "expires_at_ms INTEGER NOT NULL CHECK(expires_at_ms>=0),"
        "ready INTEGER NOT NULL CHECK(ready IN (0,1)),"
        "contract_version BLOB NOT NULL,"
        "capabilities BLOB NOT NULL) STRICT;"
        "CREATE TABLE accepted_authority("
        "singleton INTEGER PRIMARY KEY CHECK(singleton=1),"
        "sequence INTEGER NOT NULL REFERENCES events(sequence)) STRICT;"
        "CREATE TRIGGER immutable_event_update BEFORE UPDATE ON events BEGIN "
        "SELECT RAISE(ABORT,'immutable event'); END;"
        "CREATE TRIGGER immutable_event_delete BEFORE DELETE ON events BEGIN "
        "SELECT RAISE(ABORT,'immutable event'); END;"
        "PRAGMA application_id=1163088729; PRAGMA user_version=5;"));
  } else if (version == 1 && application == kApplicationId) {
    SYNC_RETURN_IF_ERROR(Exec(
        impl->db,
        "ALTER TABLE events ADD COLUMN fencing_generation INTEGER NOT NULL "
        "DEFAULT 0 CHECK(fencing_generation>=0);"
        "ALTER TABLE events ADD COLUMN planning_status_kind INTEGER NOT NULL "
        "DEFAULT 0 CHECK(planning_status_kind BETWEEN 0 AND 3);"
        "CREATE TABLE writer_claims("
        "role INTEGER PRIMARY KEY CHECK(role BETWEEN 0 AND 2),"
        "epoch BLOB NOT NULL,"
        "generation INTEGER NOT NULL CHECK(generation>0),"
        "boot_id BLOB NOT NULL,"
        "expires_at_ms INTEGER NOT NULL CHECK(expires_at_ms>=0),"
        "ready INTEGER NOT NULL CHECK(ready IN (0,1)),"
        "contract_version BLOB NOT NULL,"
        "capabilities BLOB NOT NULL) STRICT;"
        "CREATE TABLE accepted_authority("
        "singleton INTEGER PRIMARY KEY CHECK(singleton=1),"
        "sequence INTEGER NOT NULL REFERENCES events(sequence)) STRICT;"
        "PRAGMA user_version=3;"));
  } else if (version == 2 && application == kApplicationId) {
    SYNC_RETURN_IF_ERROR(Exec(
        impl->db,
        "ALTER TABLE events ADD COLUMN planning_status_kind INTEGER NOT NULL "
        "DEFAULT 0 CHECK(planning_status_kind BETWEEN 0 AND 3);"
        "CREATE TABLE accepted_authority("
        "singleton INTEGER PRIMARY KEY CHECK(singleton=1),"
        "sequence INTEGER NOT NULL REFERENCES events(sequence)) STRICT;"
        "PRAGMA user_version=3;"));
  } else if ((version != 3 && version != 4 && version != 5) ||
             application != kApplicationId) {
    return Error(Code::kCorrupt, "unsupported database schema or application");
  }
  if (version == 1 || version == 2 || version == 3) {
    SYNC_RETURN_IF_ERROR(UpgradeSchemaToV4(impl->db));
  }
  if (version != 0 && version != 5) {
    SYNC_RETURN_IF_ERROR(UpgradeSchemaToV5(impl->db));
  }
  SYNC_RETURN_IF_ERROR(transaction.Commit());
  {
    Transaction claim(impl->db);
    SYNC_RETURN_IF_ERROR(claim.Begin(true));
    uint64_t generation = 0;
    bool occupied = false;
    std::string claimed_boot_id;
    Statement existing(impl->db);
    SYNC_RETURN_IF_ERROR(existing.Prepare(
        "SELECT generation,boot_id,expires_at_ms FROM writer_claims WHERE role=?"));
    SYNC_RETURN_IF_ERROR(
        existing.Bind(1, static_cast<uint64_t>(impl->role)));
    SYNC_RETURN_IF_ERROR(existing.Step(&occupied));
    if (occupied) {
      generation = existing.Integer(0);
      claimed_boot_id = existing.Bytes(1);
      if (claimed_boot_id == impl->boot_id &&
          existing.Integer(2) > MonotonicMs()) {
        return Error(Code::kBusy, "role already has a live writer claim");
      }
      if (generation >= kMaxInteger) {
        return Error(Code::kStopped, "writer fencing generation exhausted");
      }
      ++generation;
    } else {
      generation = 1;
    }
    const uint64_t expiry = MonotonicMs() + impl->writer_lease_ms;
    Statement write_claim(impl->db);
    SYNC_RETURN_IF_ERROR(write_claim.Prepare(
        "INSERT INTO writer_claims(role,epoch,generation,boot_id,"
        "expires_at_ms,ready,contract_version,capabilities) "
        "VALUES(?,?,?,?,?,0,?,?) ON CONFLICT(role) DO UPDATE SET "
        "epoch=excluded.epoch,generation=excluded.generation,"
        "boot_id=excluded.boot_id,expires_at_ms=excluded.expires_at_ms,"
        "ready=0,contract_version=excluded.contract_version,"
        "capabilities=excluded.capabilities"));
    SYNC_RETURN_IF_ERROR(
        write_claim.Bind(1, static_cast<uint64_t>(impl->role)));
    SYNC_RETURN_IF_ERROR(write_claim.Bind(2, impl->writer_epoch));
    SYNC_RETURN_IF_ERROR(write_claim.Bind(3, generation));
    SYNC_RETURN_IF_ERROR(write_claim.Bind(4, impl->boot_id));
    SYNC_RETURN_IF_ERROR(write_claim.Bind(5, expiry));
    SYNC_RETURN_IF_ERROR(write_claim.Bind(6, impl->contract_version));
    SYNC_RETURN_IF_ERROR(write_claim.Bind(7, impl->capabilities));
    SYNC_RETURN_IF_ERROR(write_claim.Step());
    SYNC_RETURN_IF_ERROR(claim.Commit());
    impl->fencing_generation = generation;
  }
  *store = std::unique_ptr<Store>(new Store(std::move(impl)));
  return {};
}

Result Store::RenewWriter() {
  const uint64_t now = MonotonicMs();
  if (now > kMaxInteger - impl_->writer_lease_ms) {
    return Error(Code::kStopped, "monotonic lease clock exhausted");
  }
  Statement renew(impl_->db);
  SYNC_RETURN_IF_ERROR(renew.Prepare(
      "UPDATE writer_claims SET expires_at_ms=? WHERE role=? AND epoch=? AND "
      "generation=? AND boot_id=?"));
  SYNC_RETURN_IF_ERROR(
      renew.Bind(1, now + impl_->writer_lease_ms));
  SYNC_RETURN_IF_ERROR(renew.Bind(2, static_cast<uint64_t>(impl_->role)));
  SYNC_RETURN_IF_ERROR(renew.Bind(3, impl_->writer_epoch));
  SYNC_RETURN_IF_ERROR(renew.Bind(4, impl_->fencing_generation));
  SYNC_RETURN_IF_ERROR(renew.Bind(5, impl_->boot_id));
  SYNC_RETURN_IF_ERROR(renew.Step());
  if (sqlite3_changes(impl_->db) != 1) {
    return Error(Code::kFenced, "writer claim was superseded");
  }
  return {};
}

Result Store::SetReady(bool ready) {
  SYNC_RETURN_IF_ERROR(RenewWriter());
  Statement update(impl_->db);
  SYNC_RETURN_IF_ERROR(update.Prepare(
      "UPDATE writer_claims SET ready=? WHERE role=? AND epoch=? AND "
      "generation=? AND boot_id=?"));
  SYNC_RETURN_IF_ERROR(update.Bind(1, static_cast<uint64_t>(ready)));
  SYNC_RETURN_IF_ERROR(update.Bind(2, static_cast<uint64_t>(impl_->role)));
  SYNC_RETURN_IF_ERROR(update.Bind(3, impl_->writer_epoch));
  SYNC_RETURN_IF_ERROR(update.Bind(4, impl_->fencing_generation));
  SYNC_RETURN_IF_ERROR(update.Bind(5, impl_->boot_id));
  SYNC_RETURN_IF_ERROR(update.Step());
  if (sqlite3_changes(impl_->db) != 1) {
    return Error(Code::kFenced, "writer claim is no longer current");
  }
  return {};
}

Result Store::ReadParticipants(std::vector<Participant>* participants) {
  if (participants == nullptr) {
    return Error(Code::kInvalidArgument, "null participant output");
  }
  SYNC_RETURN_IF_ERROR(RenewWriter());
  Statement query(impl_->db);
  SYNC_RETURN_IF_ERROR(query.Prepare(
      "SELECT role,epoch,generation,boot_id,expires_at_ms,ready,"
      "contract_version,capabilities FROM writer_claims ORDER BY role"));
  std::vector<Participant> next;
  const uint64_t now = MonotonicMs();
  for (;;) {
    bool row = false;
    SYNC_RETURN_IF_ERROR(query.Step(&row));
    if (!row) {
      break;
    }
    Participant participant;
    participant.role = static_cast<Role>(query.Integer(0));
    participant.epoch = query.Bytes(1);
    participant.fencing_generation = query.Integer(2);
    const std::string boot_id = query.Bytes(3);
    participant.lease_expiry_monotonic_ms = query.Integer(4);
    participant.ready =
        query.Integer(5) != 0 && boot_id == impl_->boot_id &&
        participant.lease_expiry_monotonic_ms > now;
    participant.contract_version = query.Bytes(6);
    participant.capabilities = DecodeCapabilities(query.Bytes(7));
    next.push_back(std::move(participant));
  }
  *participants = std::move(next);
  return {};
}

Result Store::ReleaseWriter() {
  Statement release(impl_->db);
  SYNC_RETURN_IF_ERROR(release.Prepare(
      "UPDATE writer_claims SET ready=0,expires_at_ms=0 WHERE role=? AND "
      "epoch=? AND generation=? AND boot_id=?"));
  SYNC_RETURN_IF_ERROR(release.Bind(1, static_cast<uint64_t>(impl_->role)));
  SYNC_RETURN_IF_ERROR(release.Bind(2, impl_->writer_epoch));
  SYNC_RETURN_IF_ERROR(release.Bind(3, impl_->fencing_generation));
  SYNC_RETURN_IF_ERROR(release.Bind(4, impl_->boot_id));
  SYNC_RETURN_IF_ERROR(release.Step());
  if (sqlite3_changes(impl_->db) != 1) {
    return Error(Code::kFenced, "writer claim was already superseded");
  }
  return {};
}

Result Store::Submit(const Operation& op, Commit* commit) {
  if (commit == nullptr) {
    return Error(Code::kInvalidArgument, "null commit output");
  }
  SYNC_RETURN_IF_ERROR(RenewWriter());
  Operation submitted = op;
  submitted.fencing_generation = impl_->fencing_generation;
  SYNC_RETURN_IF_ERROR(Validate(submitted, impl_->role));
  Transaction transaction(impl_->db);
  SYNC_RETURN_IF_ERROR(transaction.Begin(true));
  {
    Statement prior(impl_->db);
    const std::string sql = std::string("SELECT ") + kEventColumns +
                            " FROM events WHERE operation_id=?";
    SYNC_RETURN_IF_ERROR(prior.Prepare(sql.c_str()));
    SYNC_RETURN_IF_ERROR(prior.Bind(1, submitted.operation_id));
    bool row = false;
    SYNC_RETURN_IF_ERROR(prior.Step(&row));
    if (row) {
      const auto event = Decode(prior);
      if (event.owner != impl_->role || !Same(event.operation, submitted)) {
        return Error(Code::kConflict,
                     "operation ID reused with different data");
      }
      SYNC_RETURN_IF_ERROR(transaction.Commit());
      *commit = {event.sequence, event.operation.identity.revision, true,
                 event.operation.fencing_generation};
      return {};
    }
  }
  std::optional<Event> current;
  SYNC_RETURN_IF_ERROR(Head(impl_->db, submitted.channel, &current));
  if ((current ? current->operation.identity.revision : 0) !=
      submitted.expected_revision) {
    return Error(Code::kConflict, "channel revision changed");
  }
  for (const auto& guard : submitted.guards) {
    std::optional<Event> parent;
    const bool observation =
        submitted.channel == Channel::kControlStatus ||
        (submitted.channel == Channel::kPlanningStatus &&
         !UpdatesAcceptedAuthority(submitted.planning_status_kind));
    if (observation) {
      SYNC_RETURN_IF_ERROR(
          EventBySequence(impl_->db, guard.sequence, &parent));
    } else {
      SYNC_RETURN_IF_ERROR(Head(impl_->db, guard.channel, &parent));
    }
    if (!parent || parent->sequence != guard.sequence ||
        parent->operation.channel != guard.channel) {
      return Error(Code::kConflict, "parent version changed or absent");
    }
    if (guard.channel == Channel::kMission &&
        submitted.channel == Channel::kMotion &&
        parent->operation.mission_fenced && !submitted.allow_when_fenced) {
      return Error(Code::kFenced, "mission fences ordinary motion");
    }
  }
  Statement insert(impl_->db);
  SYNC_RETURN_IF_ERROR(insert.Prepare(
      "INSERT INTO events(operation_id,owner,channel,epoch,aggregate_id,"
      "command_id,revision,expected_revision,payload,mission_fenced,"
      "allow_when_fenced,mission_parent,motion_parent,fencing_generation,"
      "planning_status_kind,control_status_kind)"
      " VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)"));
  SYNC_RETURN_IF_ERROR(insert.Bind(1, submitted.operation_id));
  SYNC_RETURN_IF_ERROR(insert.Bind(2, static_cast<uint64_t>(impl_->role)));
  SYNC_RETURN_IF_ERROR(
      insert.Bind(3, static_cast<uint64_t>(submitted.channel)));
  SYNC_RETURN_IF_ERROR(insert.Bind(4, submitted.identity.epoch));
  SYNC_RETURN_IF_ERROR(insert.Bind(5, submitted.identity.aggregate_id));
  SYNC_RETURN_IF_ERROR(insert.Bind(6, submitted.identity.command_id));
  SYNC_RETURN_IF_ERROR(insert.Bind(7, submitted.identity.revision));
  SYNC_RETURN_IF_ERROR(insert.Bind(8, submitted.expected_revision));
  SYNC_RETURN_IF_ERROR(insert.Bind(9, submitted.payload));
  SYNC_RETURN_IF_ERROR(
      insert.Bind(10, static_cast<uint64_t>(submitted.mission_fenced)));
  SYNC_RETURN_IF_ERROR(
      insert.Bind(11, static_cast<uint64_t>(submitted.allow_when_fenced)));
  SYNC_RETURN_IF_ERROR(
      insert.Bind(12, Parent(submitted, Channel::kMission)));
  SYNC_RETURN_IF_ERROR(
      insert.Bind(13, Parent(submitted, Channel::kMotion)));
  SYNC_RETURN_IF_ERROR(
      insert.Bind(14, submitted.fencing_generation));
  SYNC_RETURN_IF_ERROR(
      insert.Bind(15, static_cast<uint64_t>(submitted.planning_status_kind)));
  SYNC_RETURN_IF_ERROR(
      insert.Bind(16, static_cast<uint64_t>(submitted.control_status_kind)));
  SYNC_RETURN_IF_ERROR(insert.Step());
  const uint64_t sequence = sqlite3_last_insert_rowid(impl_->db);
  Statement head(impl_->db);
  SYNC_RETURN_IF_ERROR(head.Prepare(
      "INSERT INTO heads(channel,sequence) VALUES(?,?) ON CONFLICT(channel) "
      "DO UPDATE SET sequence=excluded.sequence"));
  SYNC_RETURN_IF_ERROR(
      head.Bind(1, static_cast<uint64_t>(submitted.channel)));
  SYNC_RETURN_IF_ERROR(head.Bind(2, sequence));
  SYNC_RETURN_IF_ERROR(head.Step());
  if (UpdatesAcceptedAuthority(submitted.planning_status_kind)) {
    Statement authority(impl_->db);
    SYNC_RETURN_IF_ERROR(authority.Prepare(
        "INSERT INTO accepted_authority(singleton,sequence) VALUES(1,?) "
        "ON CONFLICT(singleton) DO UPDATE SET sequence=excluded.sequence"));
    SYNC_RETURN_IF_ERROR(authority.Bind(1, sequence));
    SYNC_RETURN_IF_ERROR(authority.Step());
  }
  SYNC_RETURN_IF_ERROR(transaction.Commit());
  *commit = {sequence, submitted.identity.revision, false,
             impl_->fencing_generation};
  return {};
}

Result Store::ReadSnapshot(Snapshot* snapshot) {
  if (snapshot == nullptr) {
    return Error(Code::kInvalidArgument, "null snapshot output");
  }
  SYNC_RETURN_IF_ERROR(RenewWriter());
  Transaction transaction(impl_->db);
  SYNC_RETURN_IF_ERROR(transaction.Begin(false));
  Snapshot next;
  SYNC_RETURN_IF_ERROR(Scalar(impl_->db,
                              "SELECT COALESCE(MAX(sequence),0) FROM events",
                              &next.sequence));
  for (size_t i = 0; i < next.latest.size(); ++i) {
    SYNC_RETURN_IF_ERROR(
        Head(impl_->db, static_cast<Channel>(i), &next.latest[i]));
  }
  {
    Statement authority(impl_->db);
    SYNC_RETURN_IF_ERROR(authority.Prepare(
        "SELECT sequence FROM accepted_authority WHERE singleton=1"));
    bool row = false;
    SYNC_RETURN_IF_ERROR(authority.Step(&row));
    if (row) {
      SYNC_RETURN_IF_ERROR(
          EventBySequence(impl_->db, authority.Integer(0),
                          &next.accepted_authority));
      if (!next.accepted_authority) {
        return Error(Code::kCorrupt,
                     "accepted authority points to a missing event");
      }
    }
  }
  SYNC_RETURN_IF_ERROR(transaction.Commit());
  *snapshot = std::move(next);
  return {};
}

Result Store::ReadEvents(uint64_t after_sequence, size_t limit,
                         EventBatch* batch) {
  if (batch == nullptr || after_sequence > kMaxInteger || limit == 0 ||
      limit > kMaxBatchSize) {
    return Error(Code::kInvalidArgument, "invalid event batch bounds");
  }
  SYNC_RETURN_IF_ERROR(RenewWriter());
  Transaction transaction(impl_->db);
  SYNC_RETURN_IF_ERROR(transaction.Begin(false));
  EventBatch next;
  next.next_sequence = after_sequence;
  SYNC_RETURN_IF_ERROR(Scalar(impl_->db,
                              "SELECT COALESCE(MAX(sequence),0) FROM events",
                              &next.high_watermark));
  if (after_sequence > next.high_watermark) {
    return Error(Code::kNotFound, "event cursor exceeds database history");
  }
  {
    Statement events(impl_->db);
    const std::string sql =
        std::string("SELECT ") + kEventColumns +
        " FROM events WHERE sequence>? ORDER BY sequence LIMIT ?";
    SYNC_RETURN_IF_ERROR(events.Prepare(sql.c_str()));
    SYNC_RETURN_IF_ERROR(events.Bind(1, after_sequence));
    SYNC_RETURN_IF_ERROR(events.Bind(2, limit));
    for (;;) {
      bool row = false;
      SYNC_RETURN_IF_ERROR(events.Step(&row));
      if (!row) {
        break;
      }
      next.events.push_back(Decode(events));
      next.next_sequence = next.events.back().sequence;
    }
  }
  SYNC_RETURN_IF_ERROR(transaction.Commit());
  *batch = std::move(next);
  return {};
}

Result Store::ReadCursor(const Consumer& consumer, uint64_t* sequence) {
  if (sequence == nullptr || !ValidRole(consumer.role) ||
      !ValidId(consumer.epoch)) {
    return Error(Code::kInvalidArgument, "invalid consumer");
  }
  SYNC_RETURN_IF_ERROR(RenewWriter());
  uint64_t next = 0;
  SYNC_RETURN_IF_ERROR(Cursor(impl_->db, consumer, &next));
  *sequence = next;
  return {};
}

Result Store::Acknowledge(const Consumer& consumer, uint64_t expected_sequence,
                          uint64_t through_sequence) {
  if (!ValidRole(consumer.role) || !ValidId(consumer.epoch) ||
      expected_sequence > through_sequence || through_sequence > kMaxInteger) {
    return Error(Code::kInvalidArgument, "invalid acknowledgement");
  }
  SYNC_RETURN_IF_ERROR(RenewWriter());
  if (consumer.role != impl_->role) {
    return Error(Code::kUnauthorized, "cannot acknowledge another role");
  }
  Transaction transaction(impl_->db);
  SYNC_RETURN_IF_ERROR(transaction.Begin(true));
  uint64_t current = 0;
  SYNC_RETURN_IF_ERROR(Cursor(impl_->db, consumer, &current));
  if (current != expected_sequence) {
    return Error(Code::kConflict, "consumer cursor changed");
  }
  if (through_sequence > 0) {
    Statement event(impl_->db);
    SYNC_RETURN_IF_ERROR(
        event.Prepare("SELECT sequence FROM events WHERE sequence=?"));
    SYNC_RETURN_IF_ERROR(event.Bind(1, through_sequence));
    bool row = false;
    SYNC_RETURN_IF_ERROR(event.Step(&row));
    if (!row) {
      return Error(Code::kNotFound, "acknowledged event does not exist");
    }
  }
  Statement ack(impl_->db);
  SYNC_RETURN_IF_ERROR(ack.Prepare(
      "INSERT INTO cursors(role,epoch,sequence) VALUES(?,?,?) "
      "ON CONFLICT(role,epoch) DO UPDATE SET sequence=excluded.sequence"));
  SYNC_RETURN_IF_ERROR(ack.Bind(1, static_cast<uint64_t>(consumer.role)));
  SYNC_RETURN_IF_ERROR(ack.Bind(2, consumer.epoch));
  SYNC_RETURN_IF_ERROR(ack.Bind(3, through_sequence));
  SYNC_RETURN_IF_ERROR(ack.Step());
  return transaction.Commit();
}

uint64_t Store::fencing_generation() const {
  return impl_->fencing_generation;
}

#undef SYNC_RETURN_IF_ERROR

}  // namespace execution_state_sync
}  // namespace apollo
