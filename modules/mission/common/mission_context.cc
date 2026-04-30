// Copyright 2025 WheelOS All Rights Reserved.
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

#include "modules/mission/common/mission_context.h"

namespace apollo {
namespace mission {

MissionContext::MissionContext() = default;

void MissionContext::SetRoutingWriter(
    const std::shared_ptr<cyber::Writer<routing::RoutingRequest>>& writer) {
  std::lock_guard<std::mutex> lock(mutex_);
  routing_writer_ = writer;
}

void MissionContext::SetPlanningCommandWriter(
    const std::shared_ptr<cyber::Writer<planning::PlanningCommand>>& writer) {
  std::lock_guard<std::mutex> lock(mutex_);
  planning_command_writer_ = writer;
}

void MissionContext::UpdateChassis(const std::shared_ptr<canbus::Chassis>& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  chassis_ = msg;
}

void MissionContext::UpdateLocalization(
    const std::shared_ptr<localization::LocalizationEstimate>& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  localization_ = msg;
}

void MissionContext::UpdatePlanningRuntimeStatus(
    const std::shared_ptr<planning::PlanningRuntimeStatus>& msg) {
  std::shared_ptr<cyber::Writer<planning::PlanningCommand>> writer;
  std::vector<planning::PlanningCommand> commands_to_publish;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    planning_runtime_status_ = msg;
    if (msg == nullptr) {
      return;
    }
    if (msg->has_mission_id() &&
        !command_supervisor_.ShouldTrackMission(msg->mission_id())) {
      return;
    }
    writer = planning_command_writer_;
    command_supervisor_.UpdatePlanningRuntimeStatus(*msg, &commands_to_publish);
  }
  PublishPlanningCommands(writer, commands_to_publish);
}

void MissionContext::UpdateControlRuntimeStatus(
    const std::shared_ptr<control::ControlRuntimeStatus>& msg) {
  std::shared_ptr<cyber::Writer<planning::PlanningCommand>> writer;
  std::vector<planning::PlanningCommand> commands_to_publish;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    control_runtime_status_ = msg;
    if (msg == nullptr) {
      return;
    }
    if (msg->has_mission_id() &&
        !command_supervisor_.ShouldTrackMission(msg->mission_id())) {
      return;
    }
    writer = planning_command_writer_;
    command_supervisor_.UpdateControlRuntimeStatus(*msg, &commands_to_publish);
  }
  PublishPlanningCommands(writer, commands_to_publish);
}

void MissionContext::SaveWaypoint(const std::string& name,
                                  const common::PointENU& pose) {
  std::lock_guard<std::mutex> lock(mutex_);
  waypoints_[name] = pose;
}

std::shared_ptr<canbus::Chassis> MissionContext::GetChassis() {
  std::lock_guard<std::mutex> lock(mutex_);
  return chassis_;
}

std::shared_ptr<localization::LocalizationEstimate>
MissionContext::GetLocalization() {
  std::lock_guard<std::mutex> lock(mutex_);
  return localization_;
}

std::shared_ptr<planning::PlanningRuntimeStatus>
MissionContext::GetPlanningRuntimeStatus() {
  std::lock_guard<std::mutex> lock(mutex_);
  return planning_runtime_status_;
}

std::shared_ptr<control::ControlRuntimeStatus>
MissionContext::GetControlRuntimeStatus() {
  std::lock_guard<std::mutex> lock(mutex_);
  return control_runtime_status_;
}

CommandLifecycleStatus MissionContext::GetCommandLifecycleStatus(
    const std::string& command_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return command_supervisor_.GetCommandLifecycleStatus(command_id);
}

bool MissionContext::GetWaypoint(const std::string& name,
                                 common::PointENU* out_pose) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto iter = waypoints_.find(name);
  if (iter == waypoints_.end()) {
    return false;
  }
  *out_pose = iter->second;
  return true;
}

void MissionContext::SendRoutingRequest(const common::PointENU& end_pose) {
  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> writer;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    writer = routing_writer_;
  }
  if (writer == nullptr) {
    AWARN << "Routing writer is not ready.";
    return;
  }
  routing::RoutingRequest request;
  auto* waypoint = request.add_waypoint();
  *waypoint->mutable_pose() = end_pose;
  common::util::FillHeader("mission", &request);
  writer->Write(request);
}

void MissionContext::SendPlanningCommand(const planning::PlanningCommand& command) {
  if (!command.has_command_id()) {
    AERROR << "Planning command must set command_id before publish.";
    return;
  }

  planning::PlanningCommand command_copy = command;
  std::shared_ptr<cyber::Writer<planning::PlanningCommand>> writer;
  std::vector<planning::PlanningCommand> commands_to_publish;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    writer = planning_command_writer_;
    if (writer == nullptr) {
      AWARN << "Planning command writer is not ready.";
      return;
    }
    if (!command_copy.has_mission_id() &&
        !command_supervisor_.GetCurrentMissionId().empty()) {
      command_copy.set_mission_id(command_supervisor_.GetCurrentMissionId());
    }
    command_supervisor_.EvaluatePlanningCommand(command_copy,
                                               &commands_to_publish);
  }
  PublishPlanningCommands(writer, commands_to_publish);
}

bool MissionContext::AcknowledgeRecovery() {
  std::lock_guard<std::mutex> lock(mutex_);
  return command_supervisor_.AcknowledgeRecovery();
}

bool MissionContext::ResumeRecovery() {
  std::shared_ptr<cyber::Writer<planning::PlanningCommand>> writer;
  std::vector<planning::PlanningCommand> commands_to_publish;
  bool resumed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    writer = planning_command_writer_;
    resumed = command_supervisor_.ResumeRecovery(&commands_to_publish);
  }
  PublishPlanningCommands(writer, commands_to_publish);
  return resumed;
}

bool MissionContext::RetryRecovery() {
  std::shared_ptr<cyber::Writer<planning::PlanningCommand>> writer;
  std::vector<planning::PlanningCommand> commands_to_publish;
  bool retried = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    writer = planning_command_writer_;
    retried = command_supervisor_.RetryRecovery(&commands_to_publish);
  }
  PublishPlanningCommands(writer, commands_to_publish);
  return retried;
}

bool MissionContext::AbortRecovery() {
  std::lock_guard<std::mutex> lock(mutex_);
  return command_supervisor_.AbortRecovery();
}

void MissionContext::SetCurrentMissionId(const std::string& id) {
  std::lock_guard<std::mutex> lock(mutex_);
  command_supervisor_.SetCurrentMissionId(id);
}

void MissionContext::SetCurrentTaskName(const std::string& task_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  command_supervisor_.SetCurrentTaskName(task_name);
}

std::string MissionContext::GetCurrentMissionId() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return command_supervisor_.GetCurrentMissionId();
}

MissionCommandSnapshot MissionContext::GetMissionCommandSnapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return command_supervisor_.GetSnapshot();
}

void MissionContext::PublishPlanningCommands(
    const std::shared_ptr<cyber::Writer<planning::PlanningCommand>>& writer,
    const std::vector<planning::PlanningCommand>& commands) {
  if (writer == nullptr) {
    return;
  }
  for (auto command : commands) {
    common::util::FillHeader("mission", &command);
    writer->Write(command);
  }
}

}  // namespace mission
}  // namespace apollo
