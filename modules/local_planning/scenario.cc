// Copyright 2026 WheelOS All Rights Reserved.
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

#include "modules/local_planning/scenario.h"

#include <unordered_set>
#include <utility>

namespace apollo {
namespace local_planning {

Stage::Stage(std::vector<std::unique_ptr<Task>> tasks)
    : tasks_(std::move(tasks)) {}

common::Status Stage::Validate() const {
  if (tasks_.empty()) {
    return common::Status(common::ErrorCode::PLANNING_ERROR,
                          "stage has no tasks");
  }
  std::unordered_set<std::string> names;
  for (const auto& task : tasks_) {
    if (!task || task->Name().empty() || !names.insert(task->Name()).second) {
      return common::Status(common::ErrorCode::PLANNING_ERROR,
                            "stage has null, unnamed or duplicate tasks");
    }
  }
  return common::Status::OK();
}

common::Status Stage::Execute(const CycleInput& input) {
  auto status = Validate();
  if (!status.ok()) {
    Reset();
    return status;
  }
  for (const auto& task : tasks_) {
    status = task->Execute(input);
    if (!status.ok()) {
      const std::string reason =
          "task " + task->Name() + ": " + status.error_message();
      Reset();
      return common::Status(status.code(), reason);
    }
  }
  return common::Status::OK();
}

void Stage::Reset() {
  for (const auto& task : tasks_) {
    if (task) {
      task->Reset();
    }
  }
}

Scenario::Scenario(const InputPolicy& policy,
                   std::vector<std::unique_ptr<Task>> tasks)
    : gate_(policy), stage_(std::move(tasks)) {}

common::Status Scenario::BeginEpoch(const OdometryInput& odometry, double now) {
  input_admitted_ = false;
  state_ = ScenarioState::INVALID;
  stage_.Reset();
  auto status = gate_.BeginEpoch(odometry, now);
  if (!status.ok()) {
    return status;
  }
  status = stage_.Validate();
  if (status.ok()) {
    state_ = ScenarioState::IDLE;
  }
  return status;
}

common::Status Scenario::Process(const CycleInput& input) {
  input_admitted_ = false;
  state_ = ScenarioState::INVALID;
  auto status = gate_.Admit(input);
  if (!status.ok()) {
    stage_.Reset();
    return status;
  }
  input_admitted_ = true;
  status = stage_.Execute(input);
  if (status.ok()) {
    state_ = ScenarioState::TASKS_COMPLETED;
  }
  return status;
}

common::Status Scenario::ProcessAdmitted(const CycleInput& input) {
  input_admitted_ = true;
  state_ = ScenarioState::INVALID;
  auto status = stage_.Execute(input);
  if (status.ok()) {
    state_ = ScenarioState::TASKS_COMPLETED;
  }
  return status;
}

void Scenario::Disarm() {
  gate_.Disarm();
  Invalidate();
}

void Scenario::Invalidate() {
  input_admitted_ = false;
  state_ = ScenarioState::INVALID;
  stage_.Reset();
}

}  // namespace local_planning
}  // namespace apollo
