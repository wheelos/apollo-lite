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

void MissionContext::SetMissionDirectiveWriter(
    const std::shared_ptr<cyber::Writer<planning::MissionDirective>>& writer) {
  std::lock_guard<std::mutex> lock(mutex_);
  mission_directive_writer_ = writer;
}

void MissionContext::SetProducerEpoch(const std::string& producer_epoch) {
  std::lock_guard<std::mutex> lock(mutex_);
  producer_epoch_ = producer_epoch;
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
  std::shared_ptr<cyber::Writer<planning::MissionDirective>> writer;
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
    if (msg->has_command_id()) {
      const auto pending =
          pending_mission_directives_.find(msg->command_id());
      if (pending != pending_mission_directives_.end() &&
          msg->has_accepted_directive_identity() &&
            pending->second.identity().command_id() ==
                msg->accepted_directive_identity().command_id() &&
          pending->second.identity().producer_epoch() ==
              msg->accepted_directive_identity().producer_epoch() &&
          pending->second.identity().aggregate_id() ==
              msg->accepted_directive_identity().aggregate_id() &&
          pending->second.identity().revision() ==
              msg->accepted_directive_identity().revision()) {
        mission_identities_[msg->command_id()].CopyFrom(
            pending->second.identity());
        pending_mission_directives_.erase(pending);
      }
    }
    writer = mission_directive_writer_;
    command_supervisor_.UpdatePlanningRuntimeStatus(*msg, &commands_to_publish);
  }
  PublishPlanningCommands(writer, commands_to_publish);
}

void MissionContext::UpdateControlRuntimeStatus(
    const std::shared_ptr<control::ControlRuntimeStatus>& msg) {
  std::shared_ptr<cyber::Writer<planning::MissionDirective>> writer;
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
    writer = mission_directive_writer_;
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

void MissionContext::SendPlanningCommand(const planning::PlanningCommand& command) {
  if (!command.has_command_id()) {
    AERROR << "Planning command must set command_id before publish.";
    return;
  }

  planning::PlanningCommand command_copy = command;
  std::shared_ptr<cyber::Writer<planning::MissionDirective>> writer;
  std::vector<planning::PlanningCommand> commands_to_publish;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    writer = mission_directive_writer_;
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
  std::shared_ptr<cyber::Writer<planning::MissionDirective>> writer;
  std::vector<planning::PlanningCommand> commands_to_publish;
  bool resumed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    writer = mission_directive_writer_;
    resumed = command_supervisor_.ResumeRecovery(&commands_to_publish);
  }
  PublishPlanningCommands(writer, commands_to_publish);
  return resumed;
}

bool MissionContext::RetryRecovery() {
  std::shared_ptr<cyber::Writer<planning::MissionDirective>> writer;
  std::vector<planning::PlanningCommand> commands_to_publish;
  bool retried = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    writer = mission_directive_writer_;
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
    const std::shared_ptr<cyber::Writer<planning::MissionDirective>>& writer,
    const std::vector<planning::PlanningCommand>& commands) {
  if (writer == nullptr) {
    return;
  }
  for (const auto& command : commands) {
    planning::MissionDirective directive;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!BuildMissionDirective(command, &directive)) {
        AERROR << "Failed to convert PlanningCommand to MissionDirective: "
               << command.command_id();
        continue;
      }
    }
    common::util::FillHeader("mission", &directive);
    writer->Write(directive);
  }
}

planning::MissionPlan MissionContext::BuildMissionPlan(
    const planning::PlanningCommand& command) const {
  planning::MissionPlan plan;
  switch (command.requested_scene()) {
    case planning::SCENE_PARK_IN:
      plan.set_task_type(planning::MISSION_TASK_PARK_IN);
      break;
    case planning::SCENE_PULL_OVER:
      plan.set_task_type(planning::MISSION_TASK_PULL_OVER);
      break;
    case planning::SCENE_DOCK:
      plan.set_task_type(planning::MISSION_TASK_DOCK);
      break;
    case planning::SCENE_SUMMON:
      plan.set_task_type(planning::MISSION_TASK_SUMMON);
      break;
    case planning::SCENE_LANE_CRUISE:
    default:
      plan.set_task_type(planning::MISSION_TASK_A_TO_B);
      break;
  }
  plan.mutable_start()->set_current_pose_at_acceptance(true);
  if (command.has_goal()) {
    plan.mutable_goal()->CopyFrom(command.goal());
  }
  if (command.has_route_hint()) {
    plan.mutable_route_hint()->CopyFrom(command.route_hint());
  }
  if (command.has_preferred_mode()) {
    plan.set_preferred_mode(command.preferred_mode());
  }
  if (command.has_priority()) {
    plan.set_priority(command.priority());
  }
  plan.set_preemptible(command.preemptible());
  if (command.has_completion()) {
    plan.mutable_completion()->CopyFrom(command.completion());
  }
  if (!plan.has_completion()) {
    plan.mutable_completion();
  }
  if (!plan.completion().has_position_tolerance_m()) {
    plan.mutable_completion()->set_position_tolerance_m(0.3);
  }
  if (!plan.completion().has_heading_tolerance_rad()) {
    plan.mutable_completion()->set_heading_tolerance_rad(0.3);
  }
  if (!plan.completion().has_require_full_stop()) {
    plan.mutable_completion()->set_require_full_stop(true);
  }
  if (!plan.completion().has_timeout_sec()) {
    plan.mutable_completion()->set_timeout_sec(600.0);
  }
  if (command.has_fallback()) {
    plan.mutable_fallback()->CopyFrom(command.fallback());
  }
  if (command.has_recovery()) {
    plan.mutable_recovery()->CopyFrom(command.recovery());
  }
  if (command.has_domain_policy()) {
    plan.mutable_domain_policy()->CopyFrom(command.domain_policy());
  }
  if (command.has_open_space()) {
    plan.mutable_open_space()->CopyFrom(command.open_space());
  }
  for (const auto& tag : command.tags()) {
    plan.add_tags(tag);
  }
  return plan;
}

bool MissionContext::BuildMissionDirective(
    const planning::PlanningCommand& command,
    planning::MissionDirective* directive) {
  if (directive == nullptr || command.command_id().empty()) {
    return false;
  }
  const auto pending =
      pending_mission_directives_.find(command.command_id());
  if (pending != pending_mission_directives_.end()) {
    directive->CopyFrom(pending->second);
    return true;
  }
  auto identity_it = mission_identities_.find(command.command_id());
  if (command.action() == planning::COMMAND_CANCEL) {
    if (identity_it == mission_identities_.end()) {
      return false;
    }
    const auto expected = identity_it->second;
    auto next = expected;
    next.set_revision(expected.revision() + 1);
    directive->mutable_identity()->CopyFrom(next);
    directive->mutable_cancel()
        ->mutable_expected_active_identity()
        ->CopyFrom(expected);
    directive->mutable_cancel()->set_postcondition(
        planning::MISSION_CANCEL_CONTROLLED_STOP_THEN_HOLD);
    directive->mutable_cancel()->set_reason("Mission behavior halted");
    pending_mission_directives_[command.command_id()].CopyFrom(*directive);
    return true;
  }

  const auto plan = BuildMissionPlan(command);
  if (identity_it == mission_identities_.end()) {
    auto* identity = directive->mutable_identity();
    identity->set_producer_epoch(producer_epoch_);
    identity->set_aggregate_id(
        command.has_mission_id() ? command.mission_id()
                                 : command.command_id());
    identity->set_command_id(command.command_id());
    identity->set_revision(1);
    directive->mutable_activate()->mutable_plan()->CopyFrom(plan);
  } else {
    const auto expected = identity_it->second;
    auto next = expected;
    next.set_revision(expected.revision() + 1);
    directive->mutable_identity()->CopyFrom(next);
    directive->mutable_replace()
        ->mutable_expected_active_identity()
        ->CopyFrom(expected);
    directive->mutable_replace()->mutable_plan()->CopyFrom(plan);
  }
  mission_plans_[command.command_id()].CopyFrom(plan);
  pending_mission_directives_[command.command_id()].CopyFrom(*directive);
  return true;
}

}  // namespace mission
}  // namespace apollo
