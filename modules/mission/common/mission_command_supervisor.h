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

#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/common_msgs/control_msgs/control_runtime_status.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "modules/common_msgs/planning_msgs/planning_runtime_status.pb.h"

namespace apollo {
namespace mission {

enum class CommandLifecycleState {
  kUnknown = 0,
  kDispatched = 1,
  kAccepted = 2,
  kRunning = 3,
  kHolding = 4,
  kQueued = 5,
  kCancelling = 6,
  kCompleted = 7,
  kFailed = 8,
  kCancelled = 9,
};

enum class RecoveryAction {
  kNone = 0,
  kAcknowledge = 1,
  kResume = 2,
  kRetry = 3,
  kAbort = 4,
  kEscalateMrm = 5,
};

struct CommandLifecycleStatus {
  std::string mission_id;
  std::string command_id;
  CommandLifecycleState state = CommandLifecycleState::kUnknown;
  std::string reason;
  planning::RuntimeState planning_state = planning::RUNTIME_UNKNOWN;
  control::ControlRuntimeState control_state =
      control::CONTROL_RUNTIME_UNKNOWN;
  uint32_t dispatch_count = 0;
};

struct RecoveryState {
  bool active = false;
  bool operator_ack_required = false;
  bool operator_acknowledged = false;
  std::vector<RecoveryAction> allowed_actions;
  RecoveryAction recommended_action = RecoveryAction::kNone;
  std::string reason;
  std::string command_id;
  uint32_t retry_count = 0;
  uint32_t retry_budget_remaining = 0;
  double deadline_sec = 0.0;
  bool timeout_expired = false;
};

struct MissionCommandSnapshot {
  std::string mission_id;
  std::string task_name;
  std::string active_command_id;
  CommandLifecycleStatus active_command_status;
  std::vector<std::string> queued_command_ids;
  CommandLifecycleStatus last_terminal_command_status;
  bool operator_recovery_required = false;
  RecoveryState recovery_state;
};

class MissionCommandSupervisor {
 public:
  void SetCurrentMissionId(const std::string& id);
  void SetCurrentTaskName(const std::string& task_name);

  const std::string& GetCurrentMissionId() const;
  bool ShouldTrackMission(const std::string& mission_id) const;

  CommandLifecycleStatus GetCommandLifecycleStatus(
      const std::string& command_id) const;
  MissionCommandSnapshot GetSnapshot() const;

  void EvaluatePlanningCommand(
      const planning::PlanningCommand& command,
      std::vector<planning::PlanningCommand>* commands_to_publish);
  void UpdatePlanningRuntimeStatus(
      const planning::PlanningRuntimeStatus& status,
      std::vector<planning::PlanningCommand>* commands_to_publish);
  void UpdateControlRuntimeStatus(
      const control::ControlRuntimeStatus& status,
      std::vector<planning::PlanningCommand>* commands_to_publish);
  bool AcknowledgeRecovery();
  bool ResumeRecovery(std::vector<planning::PlanningCommand>* commands_to_publish);
  bool RetryRecovery(std::vector<planning::PlanningCommand>* commands_to_publish);
  bool AbortRecovery();

 private:
  static bool IsTerminalLifecycleState(CommandLifecycleState state);
  static bool HasRecoveryPolicyValue(const planning::RecoveryPolicy& policy);

  CommandLifecycleStatus* FindOrCreateCommandLifecycle(
      const std::string& command_id);
  void RemoveQueuedCommand(const std::string& command_id);
  bool IsQueuedCommand(const std::string& command_id) const;
  planning::PlanningCommand BuildCancelCommand(
      const planning::PlanningCommand& active_command) const;
  void MaybeDispatchNextQueuedCommand(
      std::vector<planning::PlanningCommand>* commands_to_publish);
  void MarkCommandDispatched(CommandLifecycleStatus* status,
                             const std::string& reason);
  void CaptureTerminalStatus(const CommandLifecycleStatus& status);
  void ResetRecoveryState();
  void UpdateRecoveryTimeoutState();
  uint32_t RemainingRetryBudget(const planning::PlanningCommand& command,
                                const CommandLifecycleStatus& status) const;
  void EnterRecoveryState(const CommandLifecycleStatus& status);
  bool IsActionAllowed(RecoveryAction action) const;
  void HandleActiveTerminalState(
      const CommandLifecycleStatus& status,
      std::vector<planning::PlanningCommand>* commands_to_publish);
  void EvaluateActivateOrUpdateCommand(
      const planning::PlanningCommand& command,
      std::vector<planning::PlanningCommand>* commands_to_publish);
  void EvaluateCancelCommand(
      const planning::PlanningCommand& command,
      std::vector<planning::PlanningCommand>* commands_to_publish);

  std::string current_mission_id_;
  std::string current_task_name_;
  std::string active_command_id_;
  std::string dispatch_next_queued_after_cancel_command_id_;
  bool operator_recovery_required_ = false;

  std::deque<std::string> queued_command_ids_;
  std::unordered_map<std::string, planning::PlanningCommand> command_specs_;
  std::unordered_map<std::string, CommandLifecycleStatus> command_statuses_;
  CommandLifecycleStatus last_terminal_command_status_;
  RecoveryState recovery_state_;
};

}  // namespace mission
}  // namespace apollo
