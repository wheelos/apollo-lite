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
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "modules/execution_state_sync/worker.h"

namespace apollo {
namespace execution_state_sync {

struct Submission {
  uint64_t ticket = 0;
  Result result;
  Commit commit;
  Operation operation;
};

// Component-facing asynchronous adapter. After Init it establishes a durable
// fresh-epoch cursor at the current tail. Historical state is observable
// through Latest(), but no historical operation is replayed as live
// authorization.
class Client {
 public:
  static constexpr size_t kEventBatchSize = 32;

  Result Init(const std::string& path, Role role, const std::string& epoch,
              size_t capacity = 8,
              const std::string& contract_version = "v1",
              std::vector<std::string> capabilities = {});

  // Admission is distinct from commit. Transport revisions are assigned here
  // per channel and are intentionally independent of protobuf identity fields.
  Result Submit(Channel channel, std::string payload, std::vector<Guard> guards,
                bool mission_fenced, bool cleanup, uint64_t* ticket,
                PlanningStatusKind planning_status_kind =
                    PlanningStatusKind::kRuntime,
                ControlStatusKind control_status_kind =
                    ControlStatusKind::kMotionResult);

  // Nonblocking: drains worker completions and returns at most one ordered
  // batch already read by the worker. Callers must process the entire vector
  // before acknowledging events.back().sequence.
  Result Poll(std::vector<Event>* events);
  Result Acknowledge(uint64_t through_sequence);
  Result TakeSubmission(Submission* submission);

  std::shared_ptr<const WorkerView> Latest() const;
  bool Healthy() const;
  Result Fault() const;
  bool Ready() const;
  uint64_t cursor() const;
  std::vector<Participant> Participants() const;

 private:
  struct Pending {
    uint64_t ticket = 0;
    uint64_t worker_ticket = 0;
    Operation operation;
  };

  void LatchFault(const Result& result);
  Result DrainCompletions();
  Result DispatchPending();
  Result RequestAck();
  Result RequestReady();
  Result RequestEvents();
  size_t QueuedSubmissions() const;

  std::unique_ptr<Worker> worker_;
  Role role_ = Role::kMission;
  std::string epoch_;
  uint64_t cursor_ = 0;
  uint64_t startup_tail_ = 0;
  uint64_t event_ticket_ = 0;
  uint64_t delivered_sequence_ = 0;
  uint64_t ack_ticket_ = 0;
  uint64_t ack_target_ = 0;
  uint64_t ready_ticket_ = 0;
  bool ready_ = false;
  bool startup_acknowledged_ = false;
  bool stopped_ = false;
  std::array<uint64_t, 6> revisions_{};
  std::array<std::deque<Pending>, 6> pending_{};
  std::vector<Event> events_;
  std::deque<Submission> submissions_;
  size_t capacity_ = 0;
  uint64_t next_ticket_ = 1;
  Result fault_;
};

}  // namespace execution_state_sync
}  // namespace apollo
