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

#include "modules/execution_state_sync/client.h"

#include <algorithm>
#include <filesystem>
#include <limits>
#include <utility>

namespace apollo {
namespace execution_state_sync {
namespace {

size_t Index(Channel channel) { return static_cast<size_t>(channel); }

std::string ChannelName(Channel channel) {
  switch (channel) {
    case Channel::kMission:
      return "mission";
    case Channel::kMotion:
      return "motion";
    case Channel::kPlanningStatus:
      return "planning-status";
    case Channel::kControlStatus:
      return "control-status";
    case Channel::kSafetyRequest:
      return "safety-request";
    case Channel::kSafetyStatus:
      return "safety-status";
  }
  return "invalid";
}

bool IsTransientAdmission(const Result& result) {
  return result.code == Code::kBusy || result.code == Code::kQueueFull;
}

bool IsFatalSubmissionFailure(Channel channel, const Result& result) {
  return result.code != Code::kConflict || channel != Channel::kControlStatus;
}

bool SameGuards(const std::vector<Guard>& left,
                const std::vector<Guard>& right) {
  if (left.size() != right.size()) {
    return false;
  }
  for (size_t i = 0; i < left.size(); ++i) {
    if (left[i].channel != right[i].channel ||
        left[i].sequence != right[i].sequence) {
      return false;
    }
  }
  return true;
}

}  // namespace

Result Client::Init(const std::string& path, Role role,
                    const std::string& epoch, size_t capacity,
                    const std::string& contract_version,
                    std::vector<std::string> capabilities) {
  const std::filesystem::path database_path(path);
  std::error_code filesystem_error;
  const bool parent_exists = !database_path.parent_path().empty() &&
                             std::filesystem::is_directory(
                                 database_path.parent_path(), filesystem_error);
  if (worker_ != nullptr || path.empty() || epoch.empty() ||
      !database_path.is_absolute() || !parent_exists || filesystem_error) {
    return {Code::kInvalidArgument,
            "client requires an absolute path with an existing parent and "
            "a nonempty epoch"};
  }
  role_ = role;
  epoch_ = epoch;
  capacity_ = capacity;
  WorkerOptions options;
  options.store = {path, role};
  options.store.writer_epoch = epoch;
  options.store.contract_version = contract_version;
  options.store.capabilities = std::move(capabilities);
  options.capacity = capacity;
  Result result = Worker::Start(options, &worker_);
  if (!result.ok()) {
    LatchFault(result);
    return result;
  }
  const auto view = worker_->Latest();
  if (!view || !view->result.ok() || !view->snapshot) {
    result = view ? view->result
                  : Result{Code::kInternal, "worker has no initial snapshot"};
    LatchFault(result);
    return result;
  }
  startup_tail_ = view->snapshot->sequence;
  for (size_t i = 0; i < revisions_.size(); ++i) {
    const auto& event = view->snapshot->latest[i];
    revisions_[i] = event ? event->operation.identity.revision : 0;
  }
  result =
      worker_->TryAcknowledge({role_, epoch_}, 0, startup_tail_, &ack_ticket_);
  if (!result.ok()) {
    LatchFault(result);
  } else {
    ack_target_ = startup_tail_;
  }
  return result;
}

Result Client::Submit(Channel channel, std::string payload,
                      std::vector<Guard> guards, bool mission_fenced,
                      bool cleanup, uint64_t* ticket,
                      PlanningStatusKind planning_status_kind,
                      ControlStatusKind control_status_kind) {
  if (ticket == nullptr || worker_ == nullptr) {
    return {Code::kInvalidArgument, "client or ticket output is invalid"};
  }
  if (!ready_) {
    return fault_.ok() ? Result{Code::kBusy, "startup tail is not acknowledged"}
                       : fault_;
  }
  if (!fault_.ok()) {
    return fault_;
  }
  const size_t index = Index(channel);
  if (index >= pending_.size()) {
    return {Code::kInvalidArgument, "invalid channel"};
  }
  auto& queue = pending_[index];
  const auto duplicate =
      std::find_if(queue.begin(), queue.end(), [&](const Pending& pending) {
        return pending.operation.payload == payload &&
               SameGuards(pending.operation.guards, guards) &&
               pending.operation.mission_fenced == mission_fenced &&
               pending.operation.allow_when_fenced == cleanup &&
               pending.operation.planning_status_kind ==
                   planning_status_kind &&
               pending.operation.control_status_kind == control_status_kind;
      });
  if (duplicate != queue.end()) {
    *ticket = duplicate->ticket;
    return {};
  }
  if (QueuedSubmissions() + submissions_.size() >= capacity_) {
    return {Code::kQueueFull, "submission results have not been consumed"};
  }
  Operation operation;
  operation.channel = channel;
  operation.expected_revision = revisions_[index] + queue.size();
  operation.identity = {epoch_, ChannelName(channel), "",
                        operation.expected_revision + 1};
  operation.operation_id = epoch_ + "/" + ChannelName(channel) + "/" +
                           std::to_string(operation.identity.revision);
  operation.identity.command_id = operation.operation_id;
  operation.payload = std::move(payload);
  operation.guards = std::move(guards);
  operation.mission_fenced = mission_fenced;
  operation.allow_when_fenced = cleanup;
  operation.planning_status_kind = planning_status_kind;
  operation.control_status_kind = control_status_kind;
  if (next_ticket_ == std::numeric_limits<uint64_t>::max()) {
    return {Code::kStopped, "client ticket space exhausted"};
  }
  Pending pending;
  pending.ticket = next_ticket_++;
  pending.operation = std::move(operation);
  *ticket = pending.ticket;
  queue.push_back(std::move(pending));
  const Result result = DispatchPending();
  if (!result.ok() && !IsTransientAdmission(result)) {
    LatchFault(result);
    return result;
  }
  return {};
}

Result Client::Poll(std::vector<Event>* events) {
  if (events == nullptr || worker_ == nullptr) {
    return {Code::kInvalidArgument, "client or event output is invalid"};
  }
  events->clear();
  Result result = DrainCompletions();
  if (!result.ok()) {
    return result;
  }
  if (!fault_.ok()) {
    return fault_;
  }
  if (!startup_acknowledged_) {
    result = RequestAck();
    if (!result.ok() && !IsTransientAdmission(result)) {
      LatchFault(result);
      return result;
    }
  }
  if (!ready_ && startup_acknowledged_ && ack_ticket_ == 0 &&
      ready_ticket_ == 0 &&
      cursor_ == startup_tail_) {
    result = RequestReady();
    if (!result.ok() && !IsTransientAdmission(result)) {
      LatchFault(result);
      return result;
    }
  }
  if (!ready_) {
    return {Code::kBusy, "startup tail is not acknowledged"};
  }
  result = DispatchPending();
  if (!result.ok() && !IsTransientAdmission(result)) {
    return result;
  }
  if (!events_.empty()) {
    *events = events_;
    delivered_sequence_ = events->back().sequence;
  }
  result = RequestAck();
  if (!result.ok() && !IsTransientAdmission(result)) {
    return result;
  }
  result = RequestEvents();
  return result.ok() || IsTransientAdmission(result) ? Result{} : result;
}

Result Client::Acknowledge(uint64_t through_sequence) {
  if (worker_ == nullptr || !ready_ || ack_ticket_ != 0 ||
      through_sequence < cursor_ || through_sequence > delivered_sequence_) {
    return {Code::kInvalidArgument,
            "client is not ready or acknowledgement is invalid"};
  }
  if (through_sequence == cursor_) {
    return {};
  }
  ack_target_ = through_sequence;
  events_.clear();
  Result result = RequestAck();
  if (!result.ok() && !IsTransientAdmission(result)) {
    LatchFault(result);
    return result;
  }
  return {};
}

Result Client::TakeSubmission(Submission* submission) {
  if (submission == nullptr) {
    return {Code::kInvalidArgument, "null submission output"};
  }
  if (submissions_.empty()) {
    return {Code::kNotFound, "no completed submission"};
  }
  *submission = std::move(submissions_.front());
  submissions_.erase(submissions_.begin());
  return {};
}

std::shared_ptr<const WorkerView> Client::Latest() const {
  return worker_ ? worker_->Latest() : nullptr;
}

bool Client::Healthy() const {
  const auto view = Latest();
  return fault_.ok() && submissions_.size() < capacity_ && view &&
         view->result.ok();
}

Result Client::Fault() const {
  if (!fault_.ok()) {
    return fault_;
  }
  if (submissions_.size() >= capacity_) {
    return {Code::kQueueFull, "submission results have not been consumed"};
  }
  const auto view = Latest();
  return view ? view->result : Result{Code::kStopped, "client is not running"};
}

bool Client::Ready() const { return ready_ && Healthy(); }

uint64_t Client::cursor() const { return cursor_; }

std::vector<Participant> Client::Participants() const {
  const auto view = Latest();
  return view ? view->participants : std::vector<Participant>{};
}

void Client::LatchFault(const Result& result) {
  if (fault_.ok() && !result.ok()) {
    fault_ = result;
  }
}

Result Client::DrainCompletions() {
  for (;;) {
    if (submissions_.size() >= capacity_) {
      return {};
    }
    Completion completion;
    Result result = worker_->TryTakeCompletion(&completion);
    if (result.code == Code::kNotFound || result.code == Code::kBusy) {
      return {};
    }
    if (!result.ok()) {
      LatchFault(result);
      return result;
    }
    if (completion.ticket == ack_ticket_) {
      ack_ticket_ = 0;
      if (completion.result.code == Code::kBusy) {
        continue;
      }
      if (!completion.result.ok()) {
        LatchFault(completion.result);
        return completion.result;
      }
      cursor_ = ack_target_;
      startup_acknowledged_ = true;
      delivered_sequence_ = 0;
      if (!ready_) {
        ready_ticket_ = 0;
      }
      continue;
    }
    if (completion.ticket == ready_ticket_) {
      ready_ticket_ = 0;
      if (completion.result.code == Code::kBusy) {
        continue;
      }
      if (!completion.result.ok()) {
        LatchFault(completion.result);
        return completion.result;
      }
      ready_ = true;
      continue;
    }
    if (completion.ticket == event_ticket_) {
      event_ticket_ = 0;
      if (completion.result.code == Code::kBusy) {
        continue;
      }
      if (!completion.result.ok()) {
        LatchFault(completion.result);
        return completion.result;
      }
      events_ = std::move(completion.batch.events);
      continue;
    }
    auto queue =
        std::find_if(pending_.begin(), pending_.end(),
                     [&completion](const std::deque<Pending>& items) {
                       return !items.empty() &&
                              items.front().worker_ticket == completion.ticket;
                     });
    if (queue == pending_.end()) {
      result = {Code::kInternal, "completion has no matching request"};
      LatchFault(result);
      return result;
    }
    if (completion.result.code == Code::kBusy) {
      queue->front().worker_ticket = 0;
      continue;
    }
    Submission submission;
    submission.ticket = queue->front().ticket;
    submission.result = completion.result;
    submission.commit = completion.commit;
    submission.operation = queue->front().operation;
    if (completion.result.ok()) {
      revisions_[Index(queue->front().operation.channel)] =
          completion.commit.revision;
    } else if (IsFatalSubmissionFailure(queue->front().operation.channel,
                                        completion.result)) {
      LatchFault(completion.result);
    }
    queue->pop_front();
    submissions_.push_back(std::move(submission));
    if (!completion.result.ok()) {
      while (!queue->empty() && submissions_.size() < capacity_) {
        Submission cancelled;
        cancelled.ticket = queue->front().ticket;
        cancelled.result = {
            Code::kStopped,
            "submission cancelled because its channel predecessor failed"};
        cancelled.operation = std::move(queue->front().operation);
        queue->pop_front();
        submissions_.push_back(std::move(cancelled));
      }
    }
  }
}

Result Client::DispatchPending() {
  if (!fault_.ok()) {
    return fault_;
  }
  for (auto& queue : pending_) {
    if (queue.empty() || queue.front().worker_ticket != 0) {
      continue;
    }
    uint64_t worker_ticket = 0;
    Result result = worker_->TrySubmit(queue.front().operation, &worker_ticket);
    if (result.ok()) {
      queue.front().worker_ticket = worker_ticket;
    } else if (!IsTransientAdmission(result)) {
      return result;
    }
  }
  return {};
}

Result Client::RequestAck() {
  if (ack_ticket_ != 0 ||
      (startup_acknowledged_ && ack_target_ == cursor_)) {
    return {};
  }
  Result result = worker_->TryAcknowledge({role_, epoch_}, cursor_, ack_target_,
                                          &ack_ticket_);
  if (!result.ok()) {
    ack_ticket_ = 0;
  }
  return result;
}

Result Client::RequestReady() {
  if (ready_ticket_ != 0 || ready_) {
    return {};
  }
  return worker_->TrySetReady(true, &ready_ticket_);
}

Result Client::RequestEvents() {
  if (event_ticket_ != 0 || ack_ticket_ != 0 || !events_.empty() ||
      delivered_sequence_ != 0) {
    return {};
  }
  Result result =
      worker_->TryReadEvents(cursor_, kEventBatchSize, &event_ticket_);
  if (!result.ok()) {
    event_ticket_ = 0;
    if (!IsTransientAdmission(result)) {
      LatchFault(result);
    }
  }
  return result;
}

size_t Client::QueuedSubmissions() const {
  size_t count = 0;
  for (const auto& queue : pending_) {
    count += queue.size();
  }
  return count;
}

}  // namespace execution_state_sync
}  // namespace apollo
