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

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "modules/execution_state_sync/store.h"

namespace apollo {
namespace execution_state_sync {

struct WorkerOptions {
  Options store;
  // Includes completed but not yet consumed results: callers cannot silently
  // lose completion/error results by submitting faster than they consume.
  size_t capacity = 8;
  std::chrono::milliseconds poll_interval{50};
};

struct WorkerView {
  Result result;
  std::shared_ptr<const Snapshot> snapshot;
  std::vector<Participant> participants;
  std::chrono::steady_clock::time_point observed_at;
};

struct Completion {
  uint64_t ticket = 0;
  Result result;
  Commit commit;
  EventBatch batch;
  uint64_t cursor = 0;
};

// No user callbacks. SQLite lives exclusively on the worker thread.
// Try* calls use try_lock, returning kBusy on queue contention; they never wait
// for disk or SQLite. Latest() uses C++17 atomic shared_ptr publication, which
// is not promised lock-free by the standard and is not a hard-real-time API.
class Worker {
 public:
  static Result Start(const WorkerOptions& options,
                      std::unique_ptr<Worker>* worker);
  ~Worker();
  Worker(const Worker&) = delete;
  Worker& operator=(const Worker&) = delete;

  Result TrySubmit(Operation operation, uint64_t* ticket);
  Result TrySetReady(bool ready, uint64_t* ticket);
  Result TryReadEvents(uint64_t after_sequence, size_t limit, uint64_t* ticket);
  Result TryReadCursor(Consumer consumer, uint64_t* ticket);
  Result TryAcknowledge(Consumer consumer, uint64_t expected_sequence,
                        uint64_t through_sequence, uint64_t* ticket);
  Result TryTakeCompletion(Completion* completion);
  std::shared_ptr<const WorkerView> Latest() const;
  // Lifecycle-thread only. Drains accepted requests, then joins. Never call
  // concurrently with other Worker methods or from a control callback.
  void Stop();

 private:
  struct Impl;
  explicit Worker(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace execution_state_sync
}  // namespace apollo
