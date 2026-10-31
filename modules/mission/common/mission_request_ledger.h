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

#include <cstdint>
#include <memory>
#include <string>

#include "wheelos_msgs/mission_msgs/mission_request_result.pb.h"

namespace apollo {
namespace mission {

enum class MissionRequestLedgerCode {
  kOk,
  kNotFound,
  kConflict,
  kInvalidArgument,
  kStorageError
};

struct MissionRequestLedgerResult {
  MissionRequestLedgerCode code = MissionRequestLedgerCode::kOk;
  std::string message;
  bool duplicate = false;
  bool ok() const { return code == MissionRequestLedgerCode::kOk; }
};

struct MissionRequestRecord {
  MissionRequestIdentity identity;
  std::string fingerprint;
  std::string request_payload;
  MissionRequestResult result;
  uint64_t version = 0;
};

class MissionRequestLedger {
 public:
  MissionRequestLedger();
  ~MissionRequestLedger();
  MissionRequestLedger(const MissionRequestLedger&) = delete;
  MissionRequestLedger& operator=(const MissionRequestLedger&) = delete;

  MissionRequestLedgerResult Open(const std::string& path);
  MissionRequestLedgerResult Begin(
      const MissionRequestIdentity& identity, const std::string& fingerprint,
      const std::string& request_payload, const MissionRequestResult& initial,
      MissionRequestRecord* record);
  MissionRequestLedgerResult Get(const MissionRequestIdentity& identity,
                                 MissionRequestRecord* record) const;
  MissionRequestLedgerResult Update(
      const MissionRequestIdentity& identity, const std::string& fingerprint,
      uint64_t expected_version, const MissionRequestResult& result,
      MissionRequestRecord* record);
  MissionRequestLedgerResult InterruptPendingOnRestart();
  void Close();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mission
}  // namespace apollo
