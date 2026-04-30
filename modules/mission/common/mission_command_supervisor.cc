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

#include "modules/mission/common/mission_command_supervisor.h"

#include <algorithm>
#include <chrono>

namespace apollo {
namespace mission {

namespace {

double NowSec() {
  return std::chrono::duration<double>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

bool ContainsAction(const std::vector<RecoveryAction>& actions,
                    RecoveryAction action) {
  return std::find(actions.begin(), actions.end(), action) != actions.end();
}

}  // namespace

void MissionCommandSupervisor::SetCurrentMissionId(const std::string& id) {
  if (current_mission_id_ != id) {
    current_task_name_.clear();
    active_command_id_.clear();
    dispatch_next_queued_after_cancel_command_id_.clear();
    operator_recovery_required_ = false;
    queued_command_ids_.clear();
    command_specs_.clear();
    command_statuses_.clear();
    last_terminal_command_status_ = CommandLifecycleStatus();
    ResetRecoveryState();
  }
  current_mission_id_ = id;
}

void MissionCommandSupervisor::SetCurrentTaskName(const std::string& task_name) {
  current_task_name_ = task_name;
}

const std::string& MissionCommandSupervisor::GetCurrentMissionId() const {
  return current_mission_id_;
}

bool MissionCommandSupervisor::ShouldTrackMission(
    const std::string& mission_id) const {
  return current_mission_id_.empty() || mission_id.empty() ||
         mission_id == current_mission_id_;
}

CommandLifecycleStatus MissionCommandSupervisor::GetCommandLifecycleStatus(
    const std::string& command_id) const {
  const auto iter = command_statuses_.find(command_id);
  if (iter == command_statuses_.end()) {
    CommandLifecycleStatus status;
    status.command_id = command_id;
    return status;
  }
  return iter->second;
}

MissionCommandSnapshot MissionCommandSupervisor::GetSnapshot() const {
  MissionCommandSnapshot snapshot;
  snapshot.mission_id = current_mission_id_;
  snapshot.task_name = current_task_name_;
  snapshot.active_command_id = active_command_id_;
  snapshot.operator_recovery_required = operator_recovery_required_;
  if (!active_command_id_.empty()) {
    const auto iter = command_statuses_.find(active_command_id_);
    if (iter != command_statuses_.end()) {
      snapshot.active_command_status = iter->second;
    } else {
      snapshot.active_command_status.command_id = active_command_id_;
    }
  }
  snapshot.queued_command_ids.assign(queued_command_ids_.begin(),
                                     queued_command_ids_.end());
  snapshot.last_terminal_command_status = last_terminal_command_status_;
  snapshot.recovery_state = recovery_state_;
  return snapshot;
}

bool MissionCommandSupervisor::IsTerminalLifecycleState(
    CommandLifecycleState state) {
  return state == CommandLifecycleState::kCompleted ||
         state == CommandLifecycleState::kFailed ||
         state == CommandLifecycleState::kCancelled;
}

bool MissionCommandSupervisor::HasRecoveryPolicyValue(
    const planning::RecoveryPolicy& policy) {
  return policy.has_require_operator_ack() || policy.has_allow_resume() ||
         policy.has_allow_retry() || policy.has_allow_abort() ||
         policy.has_allow_mrm() || policy.has_retry_budget() ||
         policy.has_recovery_timeout_sec();
}

CommandLifecycleStatus* MissionCommandSupervisor::FindOrCreateCommandLifecycle(
    const std::string& command_id) {
  auto& status = command_statuses_[command_id];
  if (status.command_id.empty()) {
    status.command_id = command_id;
  }
  return &status;
}

void MissionCommandSupervisor::RemoveQueuedCommand(const std::string& command_id) {
  queued_command_ids_.erase(
      std::remove(queued_command_ids_.begin(), queued_command_ids_.end(),
                  command_id),
      queued_command_ids_.end());
}

bool MissionCommandSupervisor::IsQueuedCommand(
    const std::string& command_id) const {
  return std::find(queued_command_ids_.begin(), queued_command_ids_.end(),
                   command_id) != queued_command_ids_.end();
}

planning::PlanningCommand MissionCommandSupervisor::BuildCancelCommand(
    const planning::PlanningCommand& active_command) const {
  planning::PlanningCommand cancel = active_command;
  cancel.clear_header();
  cancel.set_action(planning::COMMAND_CANCEL);
  return cancel;
}

void MissionCommandSupervisor::MarkCommandDispatched(
    CommandLifecycleStatus* status, const std::string& reason) {
  if (status == nullptr) {
    return;
  }
  status->state = CommandLifecycleState::kDispatched;
  status->reason = reason;
  ++status->dispatch_count;
}

void MissionCommandSupervisor::CaptureTerminalStatus(
    const CommandLifecycleStatus& status) {
  last_terminal_command_status_ = status;
}

void MissionCommandSupervisor::ResetRecoveryState() {
  recovery_state_ = RecoveryState();
}

void MissionCommandSupervisor::UpdateRecoveryTimeoutState() {
  if (!recovery_state_.active || recovery_state_.deadline_sec <= 0.0) {
    return;
  }
  recovery_state_.timeout_expired = NowSec() >= recovery_state_.deadline_sec;
}

uint32_t MissionCommandSupervisor::RemainingRetryBudget(
    const planning::PlanningCommand& command,
    const CommandLifecycleStatus& status) const {
  if (!command.has_recovery() || !command.recovery().has_retry_budget()) {
    return 0;
  }
  const uint32_t configured_budget = command.recovery().retry_budget();
  const uint32_t retry_count = status.dispatch_count > 0 ? status.dispatch_count - 1 : 0;
  return configured_budget > retry_count ? configured_budget - retry_count : 0;
}

bool MissionCommandSupervisor::IsActionAllowed(RecoveryAction action) const {
  return recovery_state_.active &&
         ContainsAction(recovery_state_.allowed_actions, action);
}

void MissionCommandSupervisor::EnterRecoveryState(
    const CommandLifecycleStatus& status) {
  RecoveryState recovery;
  recovery.active = true;
  recovery.command_id = status.command_id;
  recovery.reason = status.reason;

  bool require_operator_ack = true;
  bool allow_resume = !queued_command_ids_.empty();
  bool allow_retry = false;
  bool allow_abort = true;
  bool allow_mrm = true;
  double timeout_sec = 0.0;
  uint32_t retry_budget_remaining = 0;

  const auto spec_iter = command_specs_.find(status.command_id);
  if (spec_iter != command_specs_.end() && spec_iter->second.has_recovery() &&
      HasRecoveryPolicyValue(spec_iter->second.recovery())) {
    const auto& policy = spec_iter->second.recovery();
    require_operator_ack =
        policy.has_require_operator_ack() ? policy.require_operator_ack() : true;
    allow_resume = policy.has_allow_resume() ? policy.allow_resume() : allow_resume;
    allow_retry = policy.has_allow_retry() ? policy.allow_retry() : false;
    allow_abort = policy.has_allow_abort() ? policy.allow_abort() : true;
    allow_mrm = policy.has_allow_mrm() ? policy.allow_mrm() : true;
    timeout_sec =
        policy.has_recovery_timeout_sec() ? policy.recovery_timeout_sec() : 0.0;
    retry_budget_remaining = RemainingRetryBudget(spec_iter->second, status);
  }

  recovery.operator_ack_required = require_operator_ack;
  recovery.operator_acknowledged = !require_operator_ack;
  if (require_operator_ack) {
    recovery.allowed_actions.push_back(RecoveryAction::kAcknowledge);
  }
  if (allow_resume && !queued_command_ids_.empty()) {
    recovery.allowed_actions.push_back(RecoveryAction::kResume);
  }
  if (allow_retry && retry_budget_remaining > 0) {
    recovery.allowed_actions.push_back(RecoveryAction::kRetry);
  }
  if (allow_abort) {
    recovery.allowed_actions.push_back(RecoveryAction::kAbort);
  }
  if (allow_mrm) {
    recovery.allowed_actions.push_back(RecoveryAction::kEscalateMrm);
  }

  recovery.retry_budget_remaining = retry_budget_remaining;
  recovery.retry_count = status.dispatch_count > 0 ? status.dispatch_count - 1 : 0;
  if (timeout_sec > 0.0) {
    recovery.deadline_sec = NowSec() + timeout_sec;
  }
  UpdateRecoveryTimeoutState();

  if (ContainsAction(recovery.allowed_actions, RecoveryAction::kRetry)) {
    recovery.recommended_action = RecoveryAction::kRetry;
  } else if (ContainsAction(recovery.allowed_actions, RecoveryAction::kResume)) {
    recovery.recommended_action = RecoveryAction::kResume;
  } else if (ContainsAction(recovery.allowed_actions, RecoveryAction::kAbort)) {
    recovery.recommended_action = RecoveryAction::kAbort;
  } else if (ContainsAction(recovery.allowed_actions,
                            RecoveryAction::kEscalateMrm)) {
    recovery.recommended_action = RecoveryAction::kEscalateMrm;
  }

  recovery_state_ = recovery;
  UpdateRecoveryTimeoutState();
}

void MissionCommandSupervisor::MaybeDispatchNextQueuedCommand(
    std::vector<planning::PlanningCommand>* commands_to_publish) {
  if (commands_to_publish == nullptr || !active_command_id_.empty() ||
      queued_command_ids_.empty()) {
    return;
  }
  const std::string next_command_id = queued_command_ids_.front();
  queued_command_ids_.pop_front();
  const auto spec_iter = command_specs_.find(next_command_id);
  if (spec_iter == command_specs_.end()) {
    return;
  }
  active_command_id_ = next_command_id;
  auto* status = FindOrCreateCommandLifecycle(next_command_id);
  MarkCommandDispatched(status, "mission dispatched queued planning command");
  operator_recovery_required_ = false;
  ResetRecoveryState();
  commands_to_publish->push_back(spec_iter->second);
}

void MissionCommandSupervisor::HandleActiveTerminalState(
    const CommandLifecycleStatus& status,
    std::vector<planning::PlanningCommand>* commands_to_publish) {
  if (active_command_id_ != status.command_id) {
    return;
  }
  CaptureTerminalStatus(status);
  active_command_id_.clear();

  if (status.state == CommandLifecycleState::kCompleted) {
    dispatch_next_queued_after_cancel_command_id_.clear();
    operator_recovery_required_ = false;
    ResetRecoveryState();
    MaybeDispatchNextQueuedCommand(commands_to_publish);
    return;
  }

  if (status.state == CommandLifecycleState::kCancelled) {
    const bool should_resume_queue =
        dispatch_next_queued_after_cancel_command_id_ == status.command_id;
    dispatch_next_queued_after_cancel_command_id_.clear();
    if (should_resume_queue) {
      operator_recovery_required_ = false;
      ResetRecoveryState();
      MaybeDispatchNextQueuedCommand(commands_to_publish);
    }
    return;
  }

  if (status.state == CommandLifecycleState::kFailed) {
    dispatch_next_queued_after_cancel_command_id_.clear();
    operator_recovery_required_ = true;
    EnterRecoveryState(status);
  }
}

void MissionCommandSupervisor::EvaluateActivateOrUpdateCommand(
    const planning::PlanningCommand& command,
    std::vector<planning::PlanningCommand>* commands_to_publish) {
  const std::string& command_id = command.command_id();
  command_specs_[command_id] = command;
  auto* status = FindOrCreateCommandLifecycle(command_id);
  if (command.has_mission_id()) {
    status->mission_id = command.mission_id();
  } else if (!current_mission_id_.empty()) {
    status->mission_id = current_mission_id_;
  }

  if (active_command_id_.empty()) {
    active_command_id_ = command_id;
    MarkCommandDispatched(status, "mission dispatched planning command");
    operator_recovery_required_ = false;
    ResetRecoveryState();
    if (commands_to_publish != nullptr) {
      commands_to_publish->push_back(command);
    }
    return;
  }

  if (active_command_id_ == command_id) {
    MarkCommandDispatched(status, "mission updated active planning command");
    ResetRecoveryState();
    if (commands_to_publish != nullptr) {
      commands_to_publish->push_back(command);
    }
    return;
  }

  if (command.has_replaces_command_id() &&
      command.replaces_command_id() == active_command_id_) {
    const auto active_iter = command_specs_.find(active_command_id_);
    if (active_iter != command_specs_.end() &&
        active_iter->second.has_preemptible() &&
        active_iter->second.preemptible()) {
      RemoveQueuedCommand(command_id);
      queued_command_ids_.push_front(command_id);
      status->state = CommandLifecycleState::kQueued;
      status->reason = "queued replacement command while active command cancels";
      auto* active_status = FindOrCreateCommandLifecycle(active_command_id_);
      active_status->state = CommandLifecycleState::kCancelling;
      active_status->reason = "mission preempted active command by replacement";
      dispatch_next_queued_after_cancel_command_id_ = active_command_id_;
      ResetRecoveryState();
      if (commands_to_publish != nullptr) {
        commands_to_publish->push_back(BuildCancelCommand(active_iter->second));
      }
      return;
    }
  }

  if (command.has_replaces_command_id() &&
      IsQueuedCommand(command.replaces_command_id())) {
    RemoveQueuedCommand(command.replaces_command_id());
    RemoveQueuedCommand(command_id);
    queued_command_ids_.push_front(command_id);
    status->state = CommandLifecycleState::kQueued;
    status->reason = "queued command replaced an older pending command";
    auto* replaced_status =
        FindOrCreateCommandLifecycle(command.replaces_command_id());
    replaced_status->state = CommandLifecycleState::kCancelled;
    replaced_status->reason = "mission replaced pending command";
    CaptureTerminalStatus(*replaced_status);
    return;
  }

  const auto active_iter = command_specs_.find(active_command_id_);
  if (active_iter != command_specs_.end() &&
      active_iter->second.has_preemptible() &&
      active_iter->second.preemptible() && command.has_priority() &&
      (!active_iter->second.has_priority() ||
       command.priority() > active_iter->second.priority())) {
    RemoveQueuedCommand(command_id);
    queued_command_ids_.push_front(command_id);
    status->state = CommandLifecycleState::kQueued;
    status->reason = "queued higher-priority command while active command cancels";
    auto* active_status = FindOrCreateCommandLifecycle(active_command_id_);
    active_status->state = CommandLifecycleState::kCancelling;
    active_status->reason = "mission preempted active command by priority";
    dispatch_next_queued_after_cancel_command_id_ = active_command_id_;
    ResetRecoveryState();
    if (commands_to_publish != nullptr) {
      commands_to_publish->push_back(BuildCancelCommand(active_iter->second));
    }
    return;
  }

  RemoveQueuedCommand(command_id);
  queued_command_ids_.push_back(command_id);
  status->state = CommandLifecycleState::kQueued;
  if (active_iter != command_specs_.end() &&
      active_iter->second.has_preemptible() &&
      !active_iter->second.preemptible()) {
    status->reason = "queued behind non-preemptible active command";
  } else {
    status->reason = "queued behind active planning command";
  }
}

void MissionCommandSupervisor::EvaluateCancelCommand(
    const planning::PlanningCommand& command,
    std::vector<planning::PlanningCommand>* commands_to_publish) {
  const std::string& command_id = command.command_id();
  auto* status = FindOrCreateCommandLifecycle(command_id);
  if (command.has_mission_id()) {
    status->mission_id = command.mission_id();
  } else if (!current_mission_id_.empty()) {
    status->mission_id = current_mission_id_;
  }

  if (active_command_id_ == command_id) {
    status->state = CommandLifecycleState::kCancelling;
    status->reason = "mission requested planning command cancel";
    ResetRecoveryState();
    const auto active_iter = command_specs_.find(command_id);
    if (active_iter != command_specs_.end() && commands_to_publish != nullptr) {
      commands_to_publish->push_back(BuildCancelCommand(active_iter->second));
    }
    return;
  }

  if (IsQueuedCommand(command_id)) {
    RemoveQueuedCommand(command_id);
    status->state = CommandLifecycleState::kCancelled;
    status->reason = "mission cancelled pending planning command";
    CaptureTerminalStatus(*status);
  }
}

void MissionCommandSupervisor::EvaluatePlanningCommand(
    const planning::PlanningCommand& command,
    std::vector<planning::PlanningCommand>* commands_to_publish) {
  if (!command.has_command_id()) {
    return;
  }
  if (command.has_action() && command.action() == planning::COMMAND_CANCEL) {
    EvaluateCancelCommand(command, commands_to_publish);
    return;
  }
  EvaluateActivateOrUpdateCommand(command, commands_to_publish);
}

bool MissionCommandSupervisor::AcknowledgeRecovery() {
  if (!recovery_state_.active || !recovery_state_.operator_ack_required) {
    return false;
  }
  recovery_state_.operator_acknowledged = true;
  recovery_state_.allowed_actions.erase(
      std::remove(recovery_state_.allowed_actions.begin(),
                  recovery_state_.allowed_actions.end(),
                  RecoveryAction::kAcknowledge),
      recovery_state_.allowed_actions.end());
  return true;
}

bool MissionCommandSupervisor::ResumeRecovery(
    std::vector<planning::PlanningCommand>* commands_to_publish) {
  UpdateRecoveryTimeoutState();
  if (!IsActionAllowed(RecoveryAction::kResume) ||
      (recovery_state_.operator_ack_required &&
       !recovery_state_.operator_acknowledged)) {
    return false;
  }
  operator_recovery_required_ = false;
  ResetRecoveryState();
  MaybeDispatchNextQueuedCommand(commands_to_publish);
  return true;
}

bool MissionCommandSupervisor::RetryRecovery(
    std::vector<planning::PlanningCommand>* commands_to_publish) {
  UpdateRecoveryTimeoutState();
  if (!IsActionAllowed(RecoveryAction::kRetry) || commands_to_publish == nullptr ||
      (recovery_state_.operator_ack_required &&
       !recovery_state_.operator_acknowledged)) {
    return false;
  }
  const auto spec_iter = command_specs_.find(recovery_state_.command_id);
  if (spec_iter == command_specs_.end()) {
    return false;
  }
  active_command_id_ = recovery_state_.command_id;
  auto* status = FindOrCreateCommandLifecycle(recovery_state_.command_id);
  MarkCommandDispatched(status, "mission retried failed planning command");
  operator_recovery_required_ = false;
  ResetRecoveryState();
  commands_to_publish->push_back(spec_iter->second);
  return true;
}

bool MissionCommandSupervisor::AbortRecovery() {
  UpdateRecoveryTimeoutState();
  if (!IsActionAllowed(RecoveryAction::kAbort) ||
      (recovery_state_.operator_ack_required &&
       !recovery_state_.operator_acknowledged)) {
    return false;
  }
  for (const auto& queued_command_id : queued_command_ids_) {
    auto* queued_status = FindOrCreateCommandLifecycle(queued_command_id);
    queued_status->state = CommandLifecycleState::kCancelled;
    queued_status->reason = "mission aborted queued command during recovery";
  }
  queued_command_ids_.clear();
  operator_recovery_required_ = false;
  ResetRecoveryState();
  return true;
}

void MissionCommandSupervisor::UpdatePlanningRuntimeStatus(
    const planning::PlanningRuntimeStatus& status,
    std::vector<planning::PlanningCommand>* commands_to_publish) {
  if (!status.has_command_id() || status.command_id().empty()) {
    return;
  }
  auto* command_status = FindOrCreateCommandLifecycle(status.command_id());
  if (status.has_mission_id()) {
    command_status->mission_id = status.mission_id();
  }
  if (status.has_state()) {
    command_status->planning_state = status.state();
    switch (status.state()) {
      case planning::RUNTIME_ACCEPTED:
        command_status->state = CommandLifecycleState::kAccepted;
        break;
      case planning::RUNTIME_RUNNING:
      case planning::RUNTIME_DEGRADED:
        command_status->state = CommandLifecycleState::kRunning;
        break;
      case planning::RUNTIME_HOLDING:
        command_status->state = CommandLifecycleState::kHolding;
        break;
      case planning::RUNTIME_COMPLETED:
        command_status->state = CommandLifecycleState::kCompleted;
        break;
      case planning::RUNTIME_REJECTED:
      case planning::RUNTIME_FAILED:
        command_status->state = CommandLifecycleState::kFailed;
        break;
      case planning::RUNTIME_CANCELLED:
        command_status->state = CommandLifecycleState::kCancelled;
        break;
      case planning::RUNTIME_UNKNOWN:
      case planning::RUNTIME_IDLE:
      default:
        break;
    }
  }

  if (status.has_reason() && !status.reason().empty()) {
    command_status->reason = status.reason();
  } else if (status.has_completion() &&
             status.completion().has_completion_reason() &&
             !status.completion().completion_reason().empty()) {
    command_status->reason = status.completion().completion_reason();
  }

  if (IsTerminalLifecycleState(command_status->state)) {
    HandleActiveTerminalState(*command_status, commands_to_publish);
  }
  UpdateRecoveryTimeoutState();
}

void MissionCommandSupervisor::UpdateControlRuntimeStatus(
    const control::ControlRuntimeStatus& status,
    std::vector<planning::PlanningCommand>* commands_to_publish) {
  if (!status.has_command_id() || status.command_id().empty()) {
    return;
  }
  auto* command_status = FindOrCreateCommandLifecycle(status.command_id());
  if (status.has_mission_id()) {
    command_status->mission_id = status.mission_id();
  }
  if (!status.has_state()) {
    return;
  }
  command_status->control_state = status.state();
  switch (status.state()) {
    case control::CONTROL_RUNTIME_RUNNING:
    case control::CONTROL_RUNTIME_DEGRADED:
      if (!IsTerminalLifecycleState(command_status->state)) {
        command_status->state = CommandLifecycleState::kRunning;
      }
      break;
    case control::CONTROL_RUNTIME_HOLDING:
    case control::CONTROL_RUNTIME_WAITING_INPUT:
      if (!IsTerminalLifecycleState(command_status->state)) {
        command_status->state = CommandLifecycleState::kHolding;
      }
      break;
    case control::CONTROL_RUNTIME_SOFT_STOP:
    case control::CONTROL_RUNTIME_ESTOP:
    case control::CONTROL_RUNTIME_FAULTED:
    case control::CONTROL_RUNTIME_MANUAL:
      command_status->state = CommandLifecycleState::kFailed;
      break;
    case control::CONTROL_RUNTIME_UNKNOWN:
    default:
      break;
  }
  if (status.has_reason() && !status.reason().empty()) {
    command_status->reason = status.reason();
  }

  if (IsTerminalLifecycleState(command_status->state)) {
    HandleActiveTerminalState(*command_status, commands_to_publish);
  }
  UpdateRecoveryTimeoutState();
}

}  // namespace mission
}  // namespace apollo
