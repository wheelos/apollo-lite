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

#include "modules/execution_state_sync/worker.h"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <system_error>
#include <thread>
#include <utility>

namespace apollo {
namespace execution_state_sync {

struct Worker::Impl {
  enum class Kind { kSubmit, kEvents, kCursor, kAck, kReady };
  struct Request {
    Kind kind = Kind::kSubmit;
    uint64_t ticket = 0;
    Operation operation;
    Consumer consumer;
    uint64_t after = 0;
    uint64_t through = 0;
    size_t limit = 0;
    bool ready = false;
  };

  explicit Impl(WorkerOptions opts) : options(std::move(opts)) {}

  Result Enqueue(Request request, uint64_t* ticket) {
    if (ticket == nullptr) {
      return {Code::kInvalidArgument, "null ticket output"};
    }
    std::unique_lock<std::mutex> lock(mutex, std::try_to_lock);
    if (!lock.owns_lock()) {
      return {Code::kBusy, "worker queue contended"};
    }
    if (stopping) {
      return {Code::kStopped, "worker stopped"};
    }
    if (outstanding == options.capacity) {
      return {Code::kQueueFull, "consume completions before submitting"};
    }
    if (next_ticket == std::numeric_limits<uint64_t>::max()) {
      return {Code::kStopped, "worker ticket space exhausted"};
    }
    request.ticket = next_ticket++;
    *ticket = request.ticket;
    requests.push_back(std::move(request));
    ++outstanding;
    condition.notify_one();
    return {};
  }

  void Publish(Store* store) {
    auto next = std::make_shared<WorkerView>();
    const auto prior = std::atomic_load(&view);
    if (prior) {
      next->snapshot = prior->snapshot;
      next->observed_at = prior->observed_at;
    }
    auto snapshot = std::make_shared<Snapshot>();
    next->result = store->ReadSnapshot(snapshot.get());
    if (next->result.ok()) {
      std::vector<Participant> participants;
      next->result = store->ReadParticipants(&participants);
      if (next->result.ok()) {
        next->snapshot = std::move(snapshot);
        next->participants = std::move(participants);
      }
    }
    if (next->result.ok()) {
      next->observed_at = std::chrono::steady_clock::now();
    }
    std::shared_ptr<const WorkerView> immutable = std::move(next);
    std::atomic_store(&view, std::move(immutable));
  }

  void Run(std::promise<Result> ready) {
    std::unique_ptr<Store> store;
    Result opened = Store::Open(options.store, &store);
    if (!opened.ok()) {
      ready.set_value(opened);
      return;
    }
    Publish(store.get());
    const auto initial = std::atomic_load(&view);
    ready.set_value(initial->result);
    if (!initial->result.ok()) {
      return;
    }
    for (;;) {
      Request request;
      bool have_request = false;
      {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait_for(lock, options.poll_interval,
                           [this] { return stopping || !requests.empty(); });
        if (stopping && requests.empty()) {
          return;
        }
        if (!requests.empty()) {
          request = std::move(requests.front());
          requests.pop_front();
          have_request = true;
        }
      }
      if (have_request) {
        Completion completion;
        completion.ticket = request.ticket;
        switch (request.kind) {
          case Kind::kSubmit:
            completion.result =
                store->Submit(request.operation, &completion.commit);
            break;
          case Kind::kEvents:
            completion.result = store->ReadEvents(request.after, request.limit,
                                                  &completion.batch);
            break;
          case Kind::kCursor:
            completion.result =
                store->ReadCursor(request.consumer, &completion.cursor);
            break;
          case Kind::kAck:
            completion.result = store->Acknowledge(
                request.consumer, request.after, request.through);
            break;
          case Kind::kReady:
            completion.result = store->SetReady(request.ready);
            break;
        }
        {
          std::lock_guard<std::mutex> lock(mutex);
          completions.push_back(std::move(completion));
        }
      }
      Publish(store.get());
    }
  }

  WorkerOptions options;
  std::mutex mutex;
  std::condition_variable condition;
  std::deque<Request> requests;
  std::deque<Completion> completions;
  size_t outstanding = 0;
  uint64_t next_ticket = 1;
  bool stopping = false;
  std::thread thread;
  std::shared_ptr<const WorkerView> view;
};

Worker::Worker(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
Worker::~Worker() { Stop(); }

Result Worker::Start(const WorkerOptions& options,
                     std::unique_ptr<Worker>* worker) {
  if (worker == nullptr || options.capacity == 0 || options.capacity > 64 ||
      options.poll_interval.count() <= 0 ||
      options.poll_interval > std::chrono::seconds(5)) {
    return {Code::kInvalidArgument, "invalid worker options"};
  }
  auto next =
      std::unique_ptr<Worker>(new Worker(std::make_unique<Impl>(options)));
  std::promise<Result> ready;
  auto initialized = ready.get_future();
  try {
    next->impl_->thread = std::thread(
        [impl = next->impl_.get(), promise = std::move(ready)]() mutable {
          impl->Run(std::move(promise));
        });
  } catch (const std::system_error& error) {
    return {Code::kInternal, error.what()};
  }
  Result result = initialized.get();
  if (!result.ok()) {
    return result;
  }
  *worker = std::move(next);
  return {};
}

Result Worker::TrySubmit(Operation operation, uint64_t* ticket) {
  if (operation.payload.size() > Store::kMaxPayloadBytes ||
      operation.guards.size() > 2 || operation.operation_id.size() > 256 ||
      operation.identity.epoch.size() > 256 ||
      operation.identity.aggregate_id.size() > 256 ||
      operation.identity.command_id.size() > 256) {
    return {Code::kInvalidArgument, "operation exceeds queue bounds"};
  }
  Impl::Request request;
  request.operation = std::move(operation);
  return impl_->Enqueue(std::move(request), ticket);
}

Result Worker::TrySetReady(bool ready, uint64_t* ticket) {
  Impl::Request request;
  request.kind = Impl::Kind::kReady;
  request.ready = ready;
  return impl_->Enqueue(std::move(request), ticket);
}

Result Worker::TryReadEvents(uint64_t after_sequence, size_t limit,
                             uint64_t* ticket) {
  if (limit == 0 || limit > Store::kMaxBatchSize) {
    return {Code::kInvalidArgument, "invalid event batch bounds"};
  }
  Impl::Request request;
  request.kind = Impl::Kind::kEvents;
  request.after = after_sequence;
  request.limit = limit;
  return impl_->Enqueue(std::move(request), ticket);
}

Result Worker::TryReadCursor(Consumer consumer, uint64_t* ticket) {
  if (consumer.epoch.size() > 256) {
    return {Code::kInvalidArgument, "consumer exceeds queue bounds"};
  }
  Impl::Request request;
  request.kind = Impl::Kind::kCursor;
  request.consumer = std::move(consumer);
  return impl_->Enqueue(std::move(request), ticket);
}

Result Worker::TryAcknowledge(Consumer consumer, uint64_t expected_sequence,
                              uint64_t through_sequence, uint64_t* ticket) {
  if (consumer.epoch.size() > 256) {
    return {Code::kInvalidArgument, "consumer exceeds queue bounds"};
  }
  Impl::Request request;
  request.kind = Impl::Kind::kAck;
  request.consumer = std::move(consumer);
  request.after = expected_sequence;
  request.through = through_sequence;
  return impl_->Enqueue(std::move(request), ticket);
}

Result Worker::TryTakeCompletion(Completion* completion) {
  if (completion == nullptr) {
    return {Code::kInvalidArgument, "null completion output"};
  }
  std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
  if (!lock.owns_lock()) {
    return {Code::kBusy, "worker queue contended"};
  }
  if (impl_->completions.empty()) {
    return {Code::kNotFound, "no completed request"};
  }
  *completion = std::move(impl_->completions.front());
  impl_->completions.pop_front();
  --impl_->outstanding;
  return {};
}

std::shared_ptr<const WorkerView> Worker::Latest() const {
  return std::atomic_load(&impl_->view);
}

void Worker::Stop() {
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stopping = true;
    impl_->condition.notify_one();
  }
  if (impl_->thread.joinable()) {
    impl_->thread.join();
  }
}

}  // namespace execution_state_sync
}  // namespace apollo
