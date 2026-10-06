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

#include <atomic>
#include <filesystem>
#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace mission {
namespace {

class MissionRequestLedgerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    static std::atomic<uint64_t> next{0};
    directory_ = std::filesystem::temp_directory_path() /
                 ("mission-request-ledger-" +
                  std::to_string(next.fetch_add(1)));
    std::filesystem::create_directories(directory_);
    path_ = (directory_ / "ledger.db").string();
  }

  void TearDown() override { std::filesystem::remove_all(directory_); }

  MissionRequestIdentity Identity() const {
    MissionRequestIdentity identity;
    identity.set_client_epoch("client-boot");
    identity.set_request_id("request-1");
    return identity;
  }

  MissionRequestResult PendingResult() const {
    MissionRequestResult result;
    result.mutable_identity()->CopyFrom(Identity());
    result.set_outcome(MISSION_REQUEST_PENDING);
    result.set_code(MISSION_REQUEST_OK);
    result.set_retry_disposition(MISSION_REQUEST_RECONCILE);
    result.set_durably_committed(true);
    return result;
  }

  std::filesystem::path directory_;
  std::string path_;
};

TEST_F(MissionRequestLedgerTest, DeduplicatesAndRejectsIdentityReuse) {
  MissionRequestLedger ledger;
  ASSERT_TRUE(ledger.Open(path_).ok());
  MissionRequestRecord record;
  auto result = ledger.Begin(Identity(), "fingerprint", "request-payload",
                             PendingResult(), &record);
  ASSERT_TRUE(result.ok()) << result.message;
  EXPECT_FALSE(result.duplicate);
  EXPECT_EQ(record.version, 1u);

  MissionRequestRecord duplicate;
  result = ledger.Begin(Identity(), "fingerprint", "request-payload",
                        PendingResult(), &duplicate);
  ASSERT_TRUE(result.ok()) << result.message;
  EXPECT_TRUE(result.duplicate);
  EXPECT_EQ(duplicate.version, 1u);

  result = ledger.Begin(Identity(), "different", "request-payload",
                        PendingResult(), &duplicate);
  EXPECT_EQ(result.code, MissionRequestLedgerCode::kConflict);
}

TEST_F(MissionRequestLedgerTest, PersistsFinalResultAcrossRestart) {
  {
    MissionRequestLedger ledger;
    ASSERT_TRUE(ledger.Open(path_).ok());
    MissionRequestRecord record;
    ASSERT_TRUE(ledger
                    .Begin(Identity(), "fingerprint", "request-payload",
                           PendingResult(), &record)
                    .ok());
    auto committed = PendingResult();
    committed.set_outcome(MISSION_REQUEST_COMMITTED);
    committed.set_commit_sequence(42);
    MissionRequestRecord updated;
    const auto result = ledger.Update(Identity(), "fingerprint",
                                      record.version, committed, &updated);
    ASSERT_TRUE(result.ok()) << result.message;
    EXPECT_EQ(updated.version, 2u);
  }

  MissionRequestLedger restarted;
  ASSERT_TRUE(restarted.Open(path_).ok());
  MissionRequestRecord record;
  const auto result = restarted.Get(Identity(), &record);
  ASSERT_TRUE(result.ok()) << result.message;
  EXPECT_EQ(record.result.outcome(), MISSION_REQUEST_COMMITTED);
  EXPECT_EQ(record.result.commit_sequence(), 42u);
}

TEST_F(MissionRequestLedgerTest, PendingRequestIsNotReauthorizedOnRestart) {
  {
    MissionRequestLedger ledger;
    ASSERT_TRUE(ledger.Open(path_).ok());
    MissionRequestRecord record;
    ASSERT_TRUE(ledger
                    .Begin(Identity(), "fingerprint", "request-payload",
                           PendingResult(), &record)
                    .ok());
  }

  MissionRequestLedger restarted;
  ASSERT_TRUE(restarted.Open(path_).ok());
  const auto result = restarted.InterruptPendingOnRestart();
  ASSERT_TRUE(result.ok()) << result.message;
  MissionRequestRecord record;
  ASSERT_TRUE(restarted.Get(Identity(), &record).ok());
  EXPECT_EQ(record.result.outcome(), MISSION_REQUEST_TERMINAL);
  EXPECT_EQ(record.result.code(), MISSION_REQUEST_RESULT_EXPIRED);
  EXPECT_EQ(record.result.retry_disposition(), MISSION_REQUEST_RECONCILE);
  EXPECT_NE(record.result.reason().find("no prior execution authority was restored"),
            std::string::npos);
}

}  // namespace
}  // namespace mission
}  // namespace apollo
