/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/planning_execution_state_transport.h"

#include <utility>

namespace apollo {
namespace planning {

execution_state::Result PlanningExecutionStateTransport::Init(
    const std::string& path, const std::string& producer_epoch) {
  client_ = std::make_unique<execution_state::Client>();
  return client_->Init(path, execution_state::Role::kPlanning,
                       producer_epoch, 8, "v1",
                       {"mission-admission-v1", "motion-directive-v1",
                        "planning-runtime-status-v1"});
}

execution_state::Result PlanningExecutionStateTransport::Poll(
    std::vector<execution_state::Event>* events) {
  return client_->Poll(events);
}

execution_state::Result PlanningExecutionStateTransport::Acknowledge(
    uint64_t through_sequence) {
  return client_->Acknowledge(through_sequence);
}

std::vector<execution_state::Submission>
PlanningExecutionStateTransport::DrainSubmissions() {
  std::vector<execution_state::Submission> submissions;
  execution_state::Submission submission;
  while (client_->TakeSubmission(&submission).ok()) {
    submissions.push_back(std::move(submission));
  }
  return submissions;
}

execution_state::Result PlanningExecutionStateTransport::Submit(
    execution_state::Channel channel, std::string payload,
    std::vector<execution_state::Guard> guards, bool cleanup,
    execution_state::PlanningStatusKind planning_status_kind) {
  uint64_t ticket = 0;
  return client_->Submit(channel, std::move(payload), std::move(guards), false,
                         cleanup, &ticket, planning_status_kind);
}

std::shared_ptr<const execution_state::WorkerView>
PlanningExecutionStateTransport::Latest() const {
  return client_ == nullptr ? nullptr : client_->Latest();
}

bool PlanningExecutionStateTransport::Healthy() const {
  return client_ != nullptr && client_->Healthy();
}

bool PlanningExecutionStateTransport::Ready() const {
  return client_ != nullptr && client_->Ready();
}

uint64_t PlanningExecutionStateTransport::cursor() const {
  return client_ == nullptr ? 0 : client_->cursor();
}

}  // namespace planning
}  // namespace apollo
