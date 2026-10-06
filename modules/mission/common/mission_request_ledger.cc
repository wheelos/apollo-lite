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

#include "modules/mission/common/mission_request_ledger.h"

#include <sqlite3.h>

#include <filesystem>
#include <utility>
#include <vector>

namespace apollo {
namespace mission {
namespace {

constexpr int kApplicationId = 1163088730;

MissionRequestLedgerResult SqlError(sqlite3* db, int rc) {
  if (rc == SQLITE_OK || rc == SQLITE_ROW || rc == SQLITE_DONE) {
    return {};
  }
  return {MissionRequestLedgerCode::kStorageError,
          db != nullptr ? sqlite3_errmsg(db) : sqlite3_errstr(rc)};
}

class Statement {
 public:
  explicit Statement(sqlite3* db) : db_(db) {}
  ~Statement() { sqlite3_finalize(statement_); }
  Statement(const Statement&) = delete;
  Statement& operator=(const Statement&) = delete;

  MissionRequestLedgerResult Prepare(const char* sql) {
    return SqlError(
        db_, sqlite3_prepare_v2(db_, sql, -1, &statement_, nullptr));
  }
  MissionRequestLedgerResult Bind(int index, const std::string& value) {
    return SqlError(
        db_, sqlite3_bind_blob(statement_, index, value.data(),
                               static_cast<int>(value.size()), SQLITE_TRANSIENT));
  }
  MissionRequestLedgerResult Bind(int index, uint64_t value) {
    return SqlError(db_, sqlite3_bind_int64(
                             statement_, index, static_cast<sqlite3_int64>(value)));
  }
  MissionRequestLedgerResult Step(bool* row = nullptr) {
    const int rc = sqlite3_step(statement_);
    if (row != nullptr) {
      *row = rc == SQLITE_ROW;
    }
    return SqlError(db_, rc);
  }
  std::string Bytes(int index) const {
    const int size = sqlite3_column_bytes(statement_, index);
    const auto* bytes =
        static_cast<const char*>(sqlite3_column_blob(statement_, index));
    return size == 0 ? std::string() : std::string(bytes, size);
  }
  uint64_t Integer(int index) const {
    return static_cast<uint64_t>(sqlite3_column_int64(statement_, index));
  }

 private:
  sqlite3* db_;
  sqlite3_stmt* statement_ = nullptr;
};

MissionRequestLedgerResult Exec(sqlite3* db, const char* sql) {
  return SqlError(db, sqlite3_exec(db, sql, nullptr, nullptr, nullptr));
}

class Transaction {
 public:
  explicit Transaction(sqlite3* db) : db_(db) {}
  ~Transaction() {
    if (active_) {
      sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, nullptr);
    }
  }
  MissionRequestLedgerResult Begin() {
    auto result = Exec(db_, "BEGIN IMMEDIATE");
    active_ = result.ok();
    return result;
  }
  MissionRequestLedgerResult Commit() {
    auto result = Exec(db_, "COMMIT");
    if (result.ok()) {
      active_ = false;
    }
    return result;
  }

 private:
  sqlite3* db_;
  bool active_ = false;
};

MissionRequestLedgerResult ReadRecord(
    sqlite3* db, const MissionRequestIdentity& identity,
    MissionRequestRecord* record) {
  Statement query(db);
  auto result = query.Prepare(
      "SELECT fingerprint,request_payload,result_payload,version FROM "
      "mission_requests WHERE client_epoch=? AND request_id=?");
  if (!result.ok()) {
    return result;
  }
  if (!(result = query.Bind(1, identity.client_epoch())).ok() ||
      !(result = query.Bind(2, identity.request_id())).ok()) {
    return result;
  }
  bool row = false;
  if (!(result = query.Step(&row)).ok()) {
    return result;
  }
  if (!row) {
    return {MissionRequestLedgerCode::kNotFound,
            "Mission request is not in the durable ledger"};
  }
  MissionRequestRecord next;
  next.identity.CopyFrom(identity);
  next.fingerprint = query.Bytes(0);
  next.request_payload = query.Bytes(1);
  if (!next.result.ParseFromString(query.Bytes(2))) {
    return {MissionRequestLedgerCode::kStorageError,
            "Mission request ledger contains an invalid result"};
  }
  next.version = query.Integer(3);
  *record = std::move(next);
  return {};
}

bool ValidIdentity(const MissionRequestIdentity& identity) {
  return identity.has_client_epoch() && !identity.client_epoch().empty() &&
         identity.client_epoch().size() <= 64 && identity.has_request_id() &&
         !identity.request_id().empty() && identity.request_id().size() <= 128;
}

}  // namespace

struct MissionRequestLedger::Impl {
  sqlite3* db = nullptr;
};

MissionRequestLedger::MissionRequestLedger() : impl_(std::make_unique<Impl>()) {}
MissionRequestLedger::~MissionRequestLedger() { Close(); }

MissionRequestLedgerResult MissionRequestLedger::Open(const std::string& path) {
  if (impl_->db != nullptr || path.empty()) {
    return {MissionRequestLedgerCode::kInvalidArgument,
            "request ledger is already open or path is empty"};
  }
  MissionRequestLedgerResult result;
  const std::filesystem::path db_path(path);
  std::error_code ec;
  if (!db_path.is_absolute() || db_path.parent_path().empty() ||
      !std::filesystem::is_directory(db_path.parent_path(), ec) || ec) {
    return {MissionRequestLedgerCode::kInvalidArgument,
            "request ledger requires an absolute path with an existing parent"};
  }
  const int rc = sqlite3_open_v2(
      path.c_str(), &impl_->db,
      SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX,
      nullptr);
  if (rc != SQLITE_OK) {
    auto result = SqlError(impl_->db, rc);
    Close();
    return result;
  }
  result = SqlError(impl_->db, sqlite3_busy_timeout(impl_->db, 100));
  if (!result.ok()) {
    Close();
    return result;
  }
  bool row = false;
  uint64_t application = 0;
  {
    Statement app_id(impl_->db);
    result = app_id.Prepare("PRAGMA application_id");
    if (!result.ok() || !(result = app_id.Step(&row)).ok() || !row) {
      Close();
      return result.ok()
                 ? MissionRequestLedgerResult{
                       MissionRequestLedgerCode::kStorageError,
                       "request ledger application ID is unavailable"}
                 : result;
    }
    application = app_id.Integer(0);
  }
  if (application != 0 && application != kApplicationId) {
    Close();
    return {MissionRequestLedgerCode::kStorageError,
            "request ledger application ID mismatch"};
  }
  uint64_t schema_version = 0;
  {
    Statement version(impl_->db);
    if (!(result = version.Prepare("PRAGMA user_version")).ok() ||
        !(result = version.Step(&row)).ok() || !row) {
      Close();
      return result.ok()
                 ? MissionRequestLedgerResult{
                       MissionRequestLedgerCode::kStorageError,
                       "request ledger schema version is unavailable"}
                 : result;
    }
    schema_version = version.Integer(0);
  }
  if (schema_version == 0 && application == 0) {
    Transaction transaction(impl_->db);
    if (!(result = transaction.Begin()).ok() ||
        !(result = Exec(
              impl_->db,
              "CREATE TABLE mission_requests("
              "client_epoch BLOB NOT NULL,"
              "request_id BLOB NOT NULL,"
              "fingerprint BLOB NOT NULL,"
              "request_payload BLOB NOT NULL,"
              "result_payload BLOB NOT NULL,"
              "version INTEGER NOT NULL CHECK(version>0),"
              "PRIMARY KEY(client_epoch,request_id)) STRICT;"
              "PRAGMA application_id=1163088730; PRAGMA user_version=1;"))
             .ok() ||
        !(result = transaction.Commit()).ok()) {
      Close();
      return result;
    }
  } else if (schema_version != 1 || application != kApplicationId) {
    Close();
    return {MissionRequestLedgerCode::kStorageError,
            "unsupported request ledger schema"};
  }
  if (!(result = Exec(impl_->db, "PRAGMA journal_mode=WAL")).ok() ||
      !(result = Exec(impl_->db, "PRAGMA synchronous=FULL")).ok()) {
    Close();
    return result;
  }
  return {};
}

MissionRequestLedgerResult MissionRequestLedger::Begin(
    const MissionRequestIdentity& identity, const std::string& fingerprint,
    const std::string& request_payload, const MissionRequestResult& initial,
    MissionRequestRecord* record) {
  if (impl_->db == nullptr || record == nullptr || !ValidIdentity(identity) ||
      fingerprint.empty() || request_payload.empty() ||
      fingerprint.size() > 1024 * 1024 || request_payload.size() > 1024 * 1024 ||
      !initial.has_identity() ||
      initial.identity().SerializeAsString() != identity.SerializeAsString()) {
    return {MissionRequestLedgerCode::kInvalidArgument,
            "invalid durable Mission request record"};
  }
  Transaction transaction(impl_->db);
  auto result = transaction.Begin();
  if (!result.ok()) {
    return result;
  }
  MissionRequestRecord existing;
  result = ReadRecord(impl_->db, identity, &existing);
  if (result.ok()) {
    if (existing.fingerprint != fingerprint ||
        existing.request_payload != request_payload) {
      return {MissionRequestLedgerCode::kConflict,
              "request identity was already used with different content"};
    }
    if (!(result = transaction.Commit()).ok()) {
      return result;
    }
    *record = std::move(existing);
    result.duplicate = true;
    return result;
  }
  if (result.code != MissionRequestLedgerCode::kNotFound) {
    return result;
  }
  Statement insert(impl_->db);
  if (!(result = insert
            .Prepare("INSERT INTO mission_requests(client_epoch,request_id,"
                     "fingerprint,request_payload,result_payload,version) "
                     "VALUES(?,?,?,?,?,1)"))
           .ok() ||
      !(result = insert.Bind(1, identity.client_epoch())).ok() ||
      !(result = insert.Bind(2, identity.request_id())).ok() ||
      !(result = insert.Bind(3, fingerprint)).ok() ||
      !(result = insert.Bind(4, request_payload)).ok() ||
      !(result = insert.Bind(5, initial.SerializeAsString())).ok() ||
      !(result = insert.Step()).ok() ||
      !(result = transaction.Commit()).ok()) {
    return result;
  }
  record->identity.CopyFrom(identity);
  record->fingerprint = fingerprint;
  record->request_payload = request_payload;
  record->result.CopyFrom(initial);
  record->version = 1;
  return {};
}

MissionRequestLedgerResult MissionRequestLedger::Get(
    const MissionRequestIdentity& identity, MissionRequestRecord* record) const {
  if (impl_->db == nullptr || record == nullptr || !ValidIdentity(identity)) {
    return {MissionRequestLedgerCode::kInvalidArgument,
            "invalid Mission request ledger lookup"};
  }
  return ReadRecord(impl_->db, identity, record);
}

MissionRequestLedgerResult MissionRequestLedger::Update(
    const MissionRequestIdentity& identity, const std::string& fingerprint,
    uint64_t expected_version, const MissionRequestResult& result,
    MissionRequestRecord* record) {
  if (impl_->db == nullptr || record == nullptr || !ValidIdentity(identity) ||
      fingerprint.empty() || expected_version == 0 ||
      !result.has_identity() ||
      result.identity().SerializeAsString() != identity.SerializeAsString()) {
    return {MissionRequestLedgerCode::kInvalidArgument,
            "invalid Mission request ledger update"};
  }
  Transaction transaction(impl_->db);
  auto status = transaction.Begin();
  if (!status.ok()) {
    return status;
  }
  MissionRequestRecord current;
  status = ReadRecord(impl_->db, identity, &current);
  if (!status.ok()) {
    return status;
  }
  if (current.fingerprint != fingerprint) {
    return {MissionRequestLedgerCode::kConflict,
            "request identity fingerprint changed"};
  }
  if (current.version != expected_version) {
    if (current.result.SerializeAsString() != result.SerializeAsString()) {
      return {MissionRequestLedgerCode::kConflict,
              "request result version changed"};
    }
    if (!(status = transaction.Commit()).ok()) {
      return status;
    }
    *record = std::move(current);
    status.duplicate = true;
    return status;
  }
  Statement update(impl_->db);
  if (!(status = update
            .Prepare("UPDATE mission_requests SET result_payload=?,version=? "
                     "WHERE client_epoch=? AND request_id=? AND version=?"))
           .ok() ||
      !(status = update.Bind(1, result.SerializeAsString())).ok() ||
      !(status = update.Bind(2, expected_version + 1)).ok() ||
      !(status = update.Bind(3, identity.client_epoch())).ok() ||
      !(status = update.Bind(4, identity.request_id())).ok() ||
      !(status = update.Bind(5, expected_version)).ok() ||
      !(status = update.Step()).ok()) {
    return status;
  }
  if (sqlite3_changes(impl_->db) != 1) {
    return {MissionRequestLedgerCode::kConflict,
            "request result compare-and-swap failed"};
  }
  if (!(status = transaction.Commit()).ok()) {
    return status;
  }
  current.result.CopyFrom(result);
  current.version = expected_version + 1;
  *record = std::move(current);
  return {};
}

MissionRequestLedgerResult MissionRequestLedger::InterruptPendingOnRestart() {
  if (impl_->db == nullptr) {
    return {MissionRequestLedgerCode::kInvalidArgument,
            "request ledger is not open"};
  }
  Statement query(impl_->db);
  auto result = query.Prepare(
      "SELECT client_epoch,request_id,fingerprint,result_payload,version "
      "FROM mission_requests");
  if (!result.ok()) {
    return result;
  }
  std::vector<MissionRequestRecord> pending;
  for (;;) {
    bool row = false;
    if (!(result = query.Step(&row)).ok()) {
      return result;
    }
    if (!row) {
      break;
    }
    MissionRequestRecord record;
    record.identity.set_client_epoch(query.Bytes(0));
    record.identity.set_request_id(query.Bytes(1));
    record.fingerprint = query.Bytes(2);
    if (!record.result.ParseFromString(query.Bytes(3))) {
      return {MissionRequestLedgerCode::kStorageError,
              "Mission request ledger contains an invalid result"};
    }
    record.version = query.Integer(4);
    if (record.result.outcome() == MISSION_REQUEST_PENDING ||
        record.result.outcome() == MISSION_REQUEST_COMMITTED ||
        record.result.outcome() == MISSION_REQUEST_ACCEPTED) {
      pending.push_back(std::move(record));
    }
  }
  Transaction transaction(impl_->db);
  if (!(result = transaction.Begin()).ok()) {
    return result;
  }
  for (auto& record : pending) {
    record.result.set_outcome(MISSION_REQUEST_TERMINAL);
    record.result.set_code(MISSION_REQUEST_RESULT_EXPIRED);
    record.result.set_retry_disposition(MISSION_REQUEST_RECONCILE);
    record.result.set_reason(
        "Mission restarted before the request result was finalized; no prior "
        "execution authority was restored");
    Statement update(impl_->db);
    if (!(result = update
              .Prepare("UPDATE mission_requests SET result_payload=?,version=? "
                       "WHERE client_epoch=? AND request_id=? AND version=?"))
             .ok() ||
        !(result = update.Bind(1, record.result.SerializeAsString())).ok() ||
        !(result = update.Bind(2, record.version + 1)).ok() ||
        !(result = update.Bind(3, record.identity.client_epoch())).ok() ||
        !(result = update.Bind(4, record.identity.request_id())).ok() ||
        !(result = update.Bind(5, record.version)).ok() ||
        !(result = update.Step()).ok()) {
      return result;
    }
    if (sqlite3_changes(impl_->db) != 1) {
      return {MissionRequestLedgerCode::kConflict,
              "pending request changed during restart reconciliation"};
    }
  }
  return transaction.Commit();
}

void MissionRequestLedger::Close() {
  if (impl_ != nullptr && impl_->db != nullptr) {
    sqlite3_close_v2(impl_->db);
    impl_->db = nullptr;
  }
}

}  // namespace mission
}  // namespace apollo
