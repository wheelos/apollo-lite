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
#include "modules/mission/common/mission_request_contract.h"

#include <algorithm>
#include <limits>

namespace apollo {
namespace mission {

bool MissionContext::SendPlanningPad(
    planning::PadMessage::DrivingAction action) {
  planning::PlanningCommand command;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto snapshot = command_supervisor_.GetSnapshot();
    if (snapshot.active_command_id.empty()) {
      AERROR << "Legacy mission action requires an active mission command";
      return false;
    }
    command.set_mission_id(snapshot.mission_id);
    command.set_command_id(snapshot.active_command_id);
    if (action == planning::PadMessage::STOP) {
      command.set_action(planning::COMMAND_CANCEL);
    } else if (action == planning::PadMessage::PULL_OVER) {
      const auto plan = mission_plans_.find(snapshot.active_command_id);
      if (plan == mission_plans_.end()) {
        AERROR << "Pull-over requires the active mission plan";
        return false;
      }
      command.set_action(planning::COMMAND_UPDATE);
      command.set_requested_scene(planning::SCENE_PULL_OVER);
      command.mutable_goal()->CopyFrom(plan->second.goal());
      command.mutable_completion()->CopyFrom(plan->second.completion());
      command.mutable_fallback()->CopyFrom(plan->second.fallback());
      command.mutable_recovery()->CopyFrom(plan->second.recovery());
      command.mutable_domain_policy()->CopyFrom(plan->second.domain_policy());
      command.set_preferred_mode(plan->second.preferred_mode());
      command.set_preemptible(plan->second.preemptible());
      command.set_priority(plan->second.priority());
    } else {
      AERROR << "Unsupported legacy mission action: " << action;
      return false;
    }
  }
  return SendPlanningCommand(command);
}

MissionContext::MissionContext() = default;

void MissionContext::SetMissionDirectiveWriter(
    const std::shared_ptr<cyber::Writer<planning::MissionDirective>>& writer) {
  std::lock_guard<std::mutex> lock(mutex_);
  mission_directive_writer_ = writer;
}

bool MissionContext::InitExecutionState(const std::string& path,
                                        const std::string& producer_epoch,
                                        const std::string& request_ledger_path) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (execution_state_client_ != nullptr) {
    AERROR << "Mission execution-state runtime must shut down before re-init";
    return false;
  }
  execution_state_fault_ = true;
  command_supervisor_ = MissionCommandSupervisor();
  mission_identities_.clear();
  accepted_mission_identities_.clear();
  pending_mission_directives_.clear();
  mission_plans_.clear();
  pending_typed_requests_.clear();
  request_identity_by_command_.clear();
  last_safety_status_sequence_ = 0;
  planning_runtime_status_.reset();
  control_runtime_status_.reset();
  producer_epoch_ = producer_epoch;
  mission_request_ledger_ = std::make_unique<MissionRequestLedger>();
  auto ledger_result = mission_request_ledger_->Open(request_ledger_path);
  if (!ledger_result.ok()) {
    AERROR << "Failed to open Mission request ledger: "
           << ledger_result.message;
    mission_request_ledger_.reset();
    return false;
  }
  ledger_result = mission_request_ledger_->InterruptPendingOnRestart();
  if (!ledger_result.ok()) {
    AERROR << "Failed to reconcile Mission request ledger: "
           << ledger_result.message;
    mission_request_ledger_->Close();
    mission_request_ledger_.reset();
    return false;
  }
  execution_state_client_ = std::make_unique<execution_state_sync::Client>();
  const auto result = execution_state_client_->Init(
      path, execution_state_sync::Role::kMission, producer_epoch_, 8, "v1",
      {"mission-directive-v1", "mission-business-status-v1"});
  if (!result.ok()) {
    AERROR << "Failed to initialize execution-state client: " << result.message;
    execution_state_client_.reset();
    mission_request_ledger_->Close();
    mission_request_ledger_.reset();
    return false;
  }
  execution_state_fault_ = false;
  return true;
}

void MissionContext::ShutdownExecutionState() {
  std::lock_guard<std::mutex> lock(mutex_);
  execution_state_fault_ = true;
  execution_state_client_.reset();
  if (mission_request_ledger_) {
    mission_request_ledger_->Close();
    mission_request_ledger_.reset();
  }
  pending_typed_requests_.clear();
  request_identity_by_command_.clear();
}

bool MissionContext::SubmitTypedRequest(const MissionRequest& request,
                                        MissionRequestResult* result) {
  if (result == nullptr) {
    AERROR << "Typed Mission request requires a result output";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (mission_request_ledger_ == nullptr || execution_state_client_ == nullptr) {
    AERROR << "Typed Mission request services are not initialized";
    return false;
  }
  if (!execution_state_client_->Ready()) {
    result->Clear();
    result->mutable_identity()->CopyFrom(request.identity());
    result->set_outcome(MISSION_REQUEST_REJECTED);
    result->set_code(MISSION_REQUEST_STORAGE_UNAVAILABLE);
    result->set_retry_disposition(MISSION_REQUEST_RECONCILE);
    result->set_reason("Mission execution-state client is not ready");
    return true;
  }
  if (!request.has_identity() ||
      request.identity().client_epoch().empty() ||
      request.identity().request_id().empty() ||
      request.identity().client_epoch().size() > 64 ||
      request.identity().request_id().size() > 128 ||
      !request.has_contract_version() ||
      request.contract_version() != kMissionRequestContractVersion) {
    result->Clear();
    if (request.has_identity()) {
      result->mutable_identity()->CopyFrom(request.identity());
    }
    result->set_outcome(MISSION_REQUEST_REJECTED);
    result->set_code(request.has_contract_version() &&
                             request.contract_version() !=
                                 kMissionRequestContractVersion
                         ? MISSION_REQUEST_UNSUPPORTED_VERSION
                         : MISSION_REQUEST_INVALID_INPUT);
    result->set_retry_disposition(MISSION_REQUEST_DO_NOT_RETRY);
    result->set_reason("typed request requires a bounded identity and supported version");
    return true;
  }

  if (request.has_query()) {
    if (!request.query().has_request_identity()) {
      result->Clear();
      result->mutable_identity()->CopyFrom(request.identity());
      result->set_outcome(MISSION_REQUEST_REJECTED);
      result->set_code(MISSION_REQUEST_INVALID_INPUT);
      result->set_retry_disposition(MISSION_REQUEST_DO_NOT_RETRY);
      result->set_reason("query requires a request identity target");
      return true;
    }
    MissionRequestRecord record;
    const auto lookup = mission_request_ledger_->Get(
        request.query().request_identity(), &record);
    if (!lookup.ok()) {
      result->Clear();
      result->mutable_identity()->CopyFrom(request.identity());
      result->set_outcome(MISSION_REQUEST_REJECTED);
      result->set_code(lookup.code == MissionRequestLedgerCode::kNotFound
                           ? MISSION_REQUEST_RESULT_EXPIRED
                           : MISSION_REQUEST_STORAGE_UNAVAILABLE);
      result->set_retry_disposition(
          lookup.code == MissionRequestLedgerCode::kNotFound
              ? MISSION_REQUEST_DO_NOT_RETRY
              : MISSION_REQUEST_RECONCILE);
      result->set_reason(lookup.message);
      return true;
    }
    result->CopyFrom(record.result);
    return true;
  }

  MissionRequest comparable = request;
  comparable.clear_header();
  const std::string fingerprint = comparable.SerializeAsString();
  planning::MissionDirective directive;
  control::SafetyControlRequest safety_request;
  std::string reason;
  const bool is_safety_operation =
      request.has_safety_stop() || request.has_safety_reset();
  bool supported = false;
  if (request.has_safety_stop()) {
    const auto& source = request.safety_stop();
    const bool matching_identity =
        source.has_identity() &&
        source.identity().requester_epoch() ==
            request.identity().client_epoch() &&
        source.identity().request_id() == request.identity().request_id();
    if (matching_identity && source.has_policy() &&
        source.policy() == control::SAFETY_STOP_CONTROLLED &&
        !source.reason().empty()) {
      safety_request.mutable_stop()->CopyFrom(source);
      supported = true;
    } else {
      reason = "controlled safety stop requires matching identity, policy, and reason";
    }
  } else if (request.has_safety_reset()) {
    const auto& source = request.safety_reset();
    const bool matching_identity =
        source.has_identity() &&
        source.identity().requester_epoch() ==
            request.identity().client_epoch() &&
        source.identity().request_id() == request.identity().request_id();
    const auto& expected = source.expected_safety_identity();
    if (matching_identity && expected.has_control_epoch() &&
        !expected.control_epoch().empty() && expected.has_generation() &&
        expected.generation() > 0) {
      safety_request.mutable_reset()->CopyFrom(source);
      supported = true;
    } else {
      reason = "safety reset requires matching request and expected safety identities";
    }
  } else {
    supported =
        BuildTypedMissionDirective(request, &directive, &reason);
  }

  MissionRequestResult initial;
  initial.mutable_identity()->CopyFrom(request.identity());
  initial.set_requested_mission_id(request.mission_id());
  initial.set_durably_committed(false);
  if (supported) {
    initial.set_outcome(MISSION_REQUEST_PENDING);
    initial.set_code(MISSION_REQUEST_OK);
    initial.set_retry_disposition(MISSION_REQUEST_RECONCILE);
    if (!is_safety_operation) {
      initial.mutable_affected_task_identity()->CopyFrom(directive.identity());
    }
  } else {
    initial.set_outcome(MISSION_REQUEST_REJECTED);
    initial.set_code(request.has_safety_reset() ||
                             request.has_suspend_task() ||
                             request.has_resume_task() ||
                             request.has_acknowledge_recovery() ||
                             request.has_retry_task() ||
                             request.has_abort_task()
                         ? MISSION_REQUEST_UNSUPPORTED_OPERATION
                         : MISSION_REQUEST_INVALID_INPUT);
    initial.set_retry_disposition(MISSION_REQUEST_DO_NOT_RETRY);
    initial.set_reason(reason.empty() ? "typed operation is not supported"
                                      : reason);
  }

  MissionRequestRecord record;
  const auto begun = mission_request_ledger_->Begin(
      request.identity(), fingerprint, comparable.SerializeAsString(), initial,
      &record);
  if (!begun.ok()) {
    result->Clear();
    result->mutable_identity()->CopyFrom(request.identity());
    result->set_outcome(MISSION_REQUEST_REJECTED);
    result->set_code(begun.code == MissionRequestLedgerCode::kConflict
                         ? MISSION_REQUEST_CONFLICT
                         : MISSION_REQUEST_STORAGE_UNAVAILABLE);
    result->set_retry_disposition(MISSION_REQUEST_RECONCILE);
    result->set_reason(begun.message);
    return true;
  }
  if (begun.duplicate || !supported) {
    result->CopyFrom(record.result);
    return true;
  }

  const auto channel = is_safety_operation
                           ? execution_state_sync::Channel::kSafetyRequest
                           : execution_state_sync::Channel::kMission;
  const std::string payload = is_safety_operation
                                  ? safety_request.SerializeAsString()
                                  : directive.SerializeAsString();
  uint64_t ticket = 0;
  const auto submitted = execution_state_client_->Submit(
      channel, payload, {},
      !is_safety_operation && directive.has_cancel(), false, &ticket);
  if (!submitted.ok()) {
    MissionRequestResult rejected = initial;
    rejected.set_outcome(MISSION_REQUEST_REJECTED);
    rejected.set_code(MISSION_REQUEST_STORAGE_UNAVAILABLE);
    rejected.set_retry_disposition(MISSION_REQUEST_RECONCILE);
    rejected.set_reason("durable Mission operation admission failed: " +
                        submitted.message);
    MissionRequestRecord updated;
    const auto update = mission_request_ledger_->Update(
        request.identity(), fingerprint, record.version, rejected, &updated);
    if (!update.ok()) {
      AERROR << "Failed to persist rejected Mission request: "
             << update.message;
      return false;
    }
    result->CopyFrom(updated.result);
    return true;
  }
  pending_typed_requests_[ticket] = record;
  if (!is_safety_operation) {
    auto& identities =
        request_identity_by_command_[directive.identity().command_id()];
    const bool already_tracked =
        std::any_of(identities.begin(), identities.end(),
                    [&request](const MissionRequestIdentity& identity) {
                      return identity.SerializeAsString() ==
                             request.identity().SerializeAsString();
                    });
    if (!already_tracked) {
      identities.push_back(request.identity());
    }
  }
  result->CopyFrom(record.result);
  return true;
}

bool MissionContext::GetMissionRequestResult(
    const MissionRequestIdentity& identity, MissionRequestResult* result) const {
  if (result == nullptr) {
    AERROR << "Mission request result query requires an output";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (mission_request_ledger_ == nullptr) {
    return false;
  }
  MissionRequestRecord record;
  const auto lookup = mission_request_ledger_->Get(identity, &record);
  if (!lookup.ok()) {
    return false;
  }
  result->CopyFrom(record.result);
  return true;
}

void MissionContext::UpdateChassis(
    const std::shared_ptr<canbus::Chassis>& msg) {
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
  std::vector<planning::PlanningCommand> commands_to_publish;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg == nullptr) {
      return;
    }
    const bool admission_result =
        msg->has_admission_directive_identity() &&
        msg->mission_admission_outcome() != planning::MISSION_ADMISSION_UNKNOWN;
    const auto* identity =
        admission_result ? &msg->admission_directive_identity()
                         : msg->has_accepted_directive_identity()
                               ? &msg->accepted_directive_identity()
                               : nullptr;
    if (identity == nullptr) {
      AERROR << "Planning status lacks correlated Mission directive identity";
      return;
    }
    const auto known = mission_identities_.find(identity->command_id());
    const auto outstanding =
        pending_mission_directives_.find(identity->command_id());
    const bool matches_known =
        known != mission_identities_.end() &&
        known->second.SerializeAsString() == identity->SerializeAsString();
    const bool matches_pending =
        outstanding != pending_mission_directives_.end() &&
        outstanding->second.identity().SerializeAsString() ==
            identity->SerializeAsString();
    bool matches_typed_request = false;
    if (!matches_known && !matches_pending) {
      const auto requests =
          request_identity_by_command_.find(identity->command_id());
      if (requests != request_identity_by_command_.end()) {
        for (const auto& request_identity : requests->second) {
          MissionRequestRecord record;
          const auto lookup =
              mission_request_ledger_->Get(request_identity, &record);
          if (!lookup.ok()) {
            AERROR << "Failed to verify typed Mission directive identity: "
                   << lookup.message;
            continue;
          }
          if (record.result.has_affected_task_identity() &&
              record.result.affected_task_identity().SerializeAsString() ==
                  identity->SerializeAsString()) {
            matches_typed_request = true;
            break;
          }
        }
      }
    }
    if (!matches_known && !matches_pending && !matches_typed_request) {
      AWARN << "Ignoring Planning status for unrelated Mission identity";
      return;
    }
    planning_runtime_status_ = msg;
    if (!admission_result && msg->has_mission_id() &&
        !command_supervisor_.ShouldTrackMission(msg->mission_id())) {
      return;
    }
    if (admission_result) {
      const auto pending =
          pending_mission_directives_.find(identity->command_id());
      if (pending != pending_mission_directives_.end() &&
          pending->second.identity().command_id() ==
              identity->command_id() &&
          pending->second.identity().producer_epoch() ==
              identity->producer_epoch() &&
          pending->second.identity().aggregate_id() ==
              identity->aggregate_id() &&
          pending->second.identity().revision() ==
              identity->revision()) {
        if (msg->mission_admission_outcome() ==
                planning::MISSION_ADMISSION_ACCEPTED ||
            msg->mission_admission_outcome() ==
                planning::MISSION_ADMISSION_DUPLICATE) {
          mission_identities_[identity->command_id()].CopyFrom(
              pending->second.identity());
        }
        pending_mission_directives_.erase(pending);
      }
    }
    if (msg->has_mission_identity() &&
        msg->mission_admission_outcome() !=
            planning::MISSION_ADMISSION_REJECTED) {
      accepted_mission_identities_[msg->mission_identity().command_id()]
          .CopyFrom(msg->mission_identity());
    }
    command_supervisor_.UpdatePlanningRuntimeStatus(*msg, &commands_to_publish);
    if (msg->has_mission_identity()) {
      const auto request_identities =
          request_identity_by_command_.find(
              msg->mission_identity().command_id());
      if (request_identities != request_identity_by_command_.end()) {
        for (const auto& request_identity : request_identities->second) {
          MissionRequestRecord record;
          const auto lookup =
              mission_request_ledger_->Get(request_identity, &record);
        if (lookup.ok()) {
          MissionRequestResult next = record.result;
          bool update = false;
          if (msg->mission_session_state() == planning::MISSION_SESSION_ACCEPTED &&
              next.outcome() == MISSION_REQUEST_COMMITTED) {
            next.set_outcome(MISSION_REQUEST_ACCEPTED);
            update = true;
          } else if (msg->mission_session_state() ==
                     planning::MISSION_SESSION_COMPLETED) {
            next.set_outcome(MISSION_REQUEST_TERMINAL);
            next.set_terminal_disposition(MISSION_TASK_COMPLETED);
            update = true;
          } else if (msg->mission_session_state() ==
                     planning::MISSION_SESSION_CANCELLED) {
            next.set_outcome(MISSION_REQUEST_TERMINAL);
            next.set_terminal_disposition(MISSION_TASK_CANCELLED);
            update = true;
          } else if (msg->mission_session_state() ==
                     planning::MISSION_SESSION_FAILED) {
            next.set_outcome(MISSION_REQUEST_TERMINAL);
            next.set_terminal_disposition(MISSION_TASK_FAILED);
            update = true;
          }
          if (update) {
            if (msg->has_reason()) {
              next.set_reason(msg->reason());
            }
            MissionRequestRecord updated;
            const auto persisted = mission_request_ledger_->Update(
                request_identity, record.fingerprint, record.version, next,
                &updated);
            if (!persisted.ok()) {
              AERROR << "Failed to persist typed Mission lifecycle result: "
                     << persisted.message;
            }
          }
        } else {
          AERROR << "Failed to read typed Mission request result: "
                 << lookup.message;
        }
        }
      }
    }
  }
  PublishPlanningCommands(commands_to_publish);
}

void MissionContext::UpdateControlRuntimeStatus(
    const std::shared_ptr<control::ControlRuntimeStatus>& msg,
    bool motion_result) {
  std::vector<planning::PlanningCommand> commands_to_publish;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg == nullptr) {
      return;
    }
    if (!motion_result) {
      control_runtime_status_ = msg;
      return;
    }
    if (!msg->has_motion_execution() ||
        !msg->motion_execution().has_authorized_mission_identity()) {
      return;
    }
    const auto& parent = msg->motion_execution().authorized_mission_identity();
    const auto known = accepted_mission_identities_.find(parent.command_id());
    if (known == accepted_mission_identities_.end() ||
        !IsControlStatusForMission(*msg, known->second)) {
      AWARN << "Ignoring Control status for unrelated Mission identity";
      return;
    }
    control_runtime_status_ = msg;
    if (msg->has_mission_id() &&
        !command_supervisor_.ShouldTrackMission(msg->mission_id())) {
      return;
    }
    command_supervisor_.UpdateControlRuntimeStatus(*msg, &commands_to_publish);
  }
  PublishPlanningCommands(commands_to_publish);
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

bool MissionContext::SendPlanningCommand(
    const planning::PlanningCommand& command) {
  if (!command.has_command_id()) {
    AERROR << "Planning command must set command_id before publish.";
    return false;
  }

  planning::PlanningCommand command_copy = command;
  std::vector<planning::PlanningCommand> commands_to_publish;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (execution_state_client_ == nullptr) {
      AERROR << "Execution-state client is not ready.";
      return false;
    }
    if (!command_copy.has_mission_id() &&
        !command_supervisor_.GetCurrentMissionId().empty()) {
      command_copy.set_mission_id(command_supervisor_.GetCurrentMissionId());
    }
    command_supervisor_.EvaluatePlanningCommand(command_copy,
                                                &commands_to_publish);
  }
  return PublishPlanningCommands(commands_to_publish);
}

bool MissionContext::AcknowledgeRecovery() {
  std::lock_guard<std::mutex> lock(mutex_);
  return command_supervisor_.AcknowledgeRecovery();
}

bool MissionContext::ResumeRecovery() {
  std::vector<planning::PlanningCommand> commands_to_publish;
  bool resumed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    resumed = command_supervisor_.ResumeRecovery(&commands_to_publish);
  }
  PublishPlanningCommands(commands_to_publish);
  return resumed;
}

bool MissionContext::RetryRecovery() {
  std::vector<planning::PlanningCommand> commands_to_publish;
  bool retried = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    retried = command_supervisor_.RetryRecovery(&commands_to_publish);
  }
  PublishPlanningCommands(commands_to_publish);
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
  auto snapshot = command_supervisor_.GetSnapshot();
  const auto accepted =
      accepted_mission_identities_.find(snapshot.active_command_id);
  if (accepted != accepted_mission_identities_.end()) {
    snapshot.accepted_mission_identity.CopyFrom(accepted->second);
  }
  const auto directive = mission_identities_.find(snapshot.active_command_id);
  if (directive != mission_identities_.end()) {
    snapshot.accepted_directive_identity.CopyFrom(directive->second);
  }
  const auto pending =
      pending_mission_directives_.find(snapshot.active_command_id);
  if (pending != pending_mission_directives_.end()) {
    snapshot.pending_directive_identity.CopyFrom(pending->second.identity());
  }
  return snapshot;
}

bool MissionContext::PublishPlanningCommands(
    const std::vector<planning::PlanningCommand>& commands) {
  for (const auto& command : commands) {
    planning::MissionDirective directive;
    execution_state_sync::Client* client = nullptr;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!BuildMissionDirective(command, &directive)) {
        AERROR << "Failed to convert PlanningCommand to MissionDirective: "
               << command.command_id();
        return false;
      }
      client = execution_state_client_.get();
    }
    if (client == nullptr) {
      AERROR << "Execution-state client is not ready";
      return false;
    }
    uint64_t ticket = 0;
    const bool fenced = directive.has_cancel() &&
                        directive.cancel().postcondition() ==
                            planning::MISSION_CANCEL_CONTROLLED_STOP_THEN_HOLD;
    const auto result = client->Submit(execution_state_sync::Channel::kMission,
                                       directive.SerializeAsString(), {},
                                       fenced, false, &ticket);
    if (!result.ok()) {
      AERROR << "Mission directive admission failed: " << result.message;
      std::lock_guard<std::mutex> lock(mutex_);
      execution_state_fault_ = true;
      return false;
    }
    ADEBUG << "Mission directive admitted to execution-state queue, ticket "
           << ticket;
  }
  return true;
}

bool MissionContext::PollExecutionState() {
  execution_state_sync::Client* client = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    client = execution_state_client_.get();
  }
  if (client == nullptr) {
    AERROR << "Execution-state client is not initialized";
    return false;
  }
  std::vector<execution_state_sync::Event> events;
  const auto poll = client->Poll(&events);
  if (!poll.ok() && poll.code != execution_state_sync::Code::kBusy) {
    AERROR << "Execution-state polling failed: " << poll.message;
    return false;
  }
  for (const auto& event : events) {
    if (!ConsumeExecutionEvent(event)) {
      std::lock_guard<std::mutex> lock(mutex_);
      execution_state_fault_ = true;
      return false;
    }
  }
  const auto view = client->Latest();
  if (view && view->snapshot) {
    const auto& latest_safety =
        view->snapshot->latest[static_cast<size_t>(
            execution_state_sync::Channel::kSafetyStatus)];
    if (latest_safety &&
        latest_safety->sequence > last_safety_status_sequence_ &&
        !ConsumeExecutionEvent(*latest_safety)) {
      std::lock_guard<std::mutex> lock(mutex_);
      execution_state_fault_ = true;
      return false;
    }
  }
  if (!events.empty()) {
    const auto ack = client->Acknowledge(events.back().sequence);
    if (!ack.ok()) {
      AERROR << "Execution-state acknowledgement admission failed: "
             << ack.message;
      return false;
    }
  }
  execution_state_sync::Submission submission;
  while (client->TakeSubmission(&submission).ok()) {
    if (!submission.result.ok()) {
      AERROR << "Mission directive commit failed: "
             << submission.result.message;
      return false;
    }
    MissionRequestRecord typed_record;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto typed = pending_typed_requests_.find(submission.ticket);
      if (typed != pending_typed_requests_.end()) {
        typed_record = typed->second;
      }
    }
    if (submission.operation.channel ==
        execution_state_sync::Channel::kMission) {
      planning::MissionDirective directive;
      if (!directive.ParseFromString(submission.operation.payload)) {
        AERROR << "Committed mission payload cannot be parsed";
        return false;
      }
      std::shared_ptr<cyber::Writer<planning::MissionDirective>> writer;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        writer = mission_directive_writer_;
      }
      if (writer != nullptr && !writer->Write(directive)) {
        AERROR << "Failed to publish committed MissionDirective mirror";
      }
    } else if (submission.operation.channel ==
               execution_state_sync::Channel::kSafetyRequest) {
      control::SafetyControlRequest safety_request;
      if (!safety_request.ParseFromString(submission.operation.payload)) {
        AERROR << "Committed safety request payload cannot be parsed";
        return false;
      }
      if (safety_request.operation_case() ==
          control::SafetyControlRequest::OPERATION_NOT_SET) {
        AERROR << "Committed safety request has no operation";
        return false;
      }
    }
    if (typed_record.version != 0) {
      MissionRequestRecord latest_record;
      MissionRequestRecord updated;
      MissionRequestLedgerResult persisted;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mission_request_ledger_ == nullptr) {
          AERROR << "Mission request ledger shut down before commit result";
          return false;
        }
        const auto latest = mission_request_ledger_->Get(
            typed_record.identity, &latest_record);
        if (!latest.ok()) {
          AERROR << "Failed to reload typed Mission request before commit result: "
                 << latest.message;
          return false;
        }
        auto committed = latest_record.result;
        if (committed.outcome() == MISSION_REQUEST_PENDING) {
          committed.set_outcome(MISSION_REQUEST_COMMITTED);
          committed.set_code(MISSION_REQUEST_OK);
          committed.set_retry_disposition(MISSION_REQUEST_RECONCILE);
        }
        committed.set_durably_committed(true);
        committed.set_commit_sequence(submission.commit.sequence);
        persisted = mission_request_ledger_->Update(
            typed_record.identity, latest_record.fingerprint,
            latest_record.version, committed, &updated);
      }
      if (!persisted.ok()) {
        AERROR << "Failed to persist committed typed Mission request: "
               << persisted.message;
        return false;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      pending_typed_requests_.erase(submission.ticket);
    }
  }
  return ExecutionStateHealthy();
}

bool MissionContext::ExecutionStateHealthy() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return execution_state_client_ != nullptr &&
         execution_state_client_->Ready() && !execution_state_fault_;
}

bool MissionContext::BuildTypedMissionDirective(
    const MissionRequest& request, planning::MissionDirective* directive,
    std::string* reason) {
  const auto reject = [reason](const std::string& message) {
    if (reason != nullptr) {
      *reason = message;
    }
    return false;
  };
  if (directive == nullptr || request.has_task_name() ||
      request.has_enable_loop() || request.parameters_size() != 0 ||
      request.waypoints_size() != 0) {
    return reject("typed operations cannot include legacy BT fields");
  }
  const auto make_command_id = [&request]() {
    return request.identity().client_epoch() + ":" +
           request.identity().request_id();
  };
  if (request.has_submit_task()) {
    if (request.mission_id().empty() ||
        !request.submit_task().has_plan()) {
      return reject("submit_task requires mission_id and plan");
    }
    const auto& plan = request.submit_task().plan();
    if ((plan.has_preferred_mode() &&
         (!planning::PlanningMode_IsValid(plan.preferred_mode()) ||
          plan.preferred_mode() == planning::MODE_UNKNOWN ||
          plan.preferred_mode() == planning::MODE_SAFETY_HOLD)) ||
        plan.has_domain_policy() || plan.has_open_space() ||
        plan.has_budget_authorization_id()) {
      return reject("public mission plan contains internal authority fields");
    }
    auto* identity = directive->mutable_identity();
    identity->set_producer_epoch(producer_epoch_);
    identity->set_aggregate_id(request.mission_id());
    identity->set_command_id(make_command_id());
    identity->set_revision(1);
    directive->mutable_activate()->mutable_plan()->CopyFrom(plan);
  } else if (request.has_replace_task() || request.has_cancel_task()) {
    const auto& expected =
        request.has_replace_task()
            ? request.replace_task().expected_active_identity()
            : request.cancel_task().expected_active_identity();
    if (!expected.has_producer_epoch() || expected.producer_epoch().empty() ||
        !expected.has_aggregate_id() || expected.aggregate_id().empty() ||
        !expected.has_command_id() || expected.command_id().empty() ||
        !expected.has_revision() || expected.revision() == 0 ||
        expected.revision() == std::numeric_limits<uint64_t>::max() ||
        request.mission_id() != expected.aggregate_id()) {
      return reject(
          "replace/cancel requires the exact active Mission identity");
    }
    auto* identity = directive->mutable_identity();
    identity->CopyFrom(expected);
    identity->set_command_id(request.has_cancel_task()
                                 ? expected.command_id()
                                 : make_command_id());
    identity->set_revision(expected.revision() + 1);
    if (request.has_replace_task()) {
      if (!request.replace_task().has_plan()) {
        return reject("replace_task requires plan");
      }
      const auto& plan = request.replace_task().plan();
      if ((plan.has_preferred_mode() &&
           (!planning::PlanningMode_IsValid(plan.preferred_mode()) ||
            plan.preferred_mode() == planning::MODE_UNKNOWN ||
            plan.preferred_mode() == planning::MODE_SAFETY_HOLD)) ||
          plan.has_domain_policy() || plan.has_open_space() ||
          plan.has_budget_authorization_id()) {
        return reject("public mission plan contains internal authority fields");
      }
      directive->mutable_replace()->mutable_expected_active_identity()->CopyFrom(
          expected);
      directive->mutable_replace()->mutable_plan()->CopyFrom(plan);
    } else {
      directive->mutable_cancel()->mutable_expected_active_identity()->CopyFrom(
          expected);
      directive->mutable_cancel()->set_postcondition(
          planning::MISSION_CANCEL_CONTROLLED_STOP_THEN_HOLD);
      directive->mutable_cancel()->set_reason(
          request.cancel_task().reason().empty() ? "typed Mission cancellation"
                                                 : request.cancel_task().reason());
    }
  } else {
    return reject("typed operation requires an implemented Mission dispatcher");
  }
  common::util::FillHeader("mission", directive);
  return true;
}

bool MissionContext::ConsumeExecutionEvent(
    const execution_state_sync::Event& event) {
  if (event.operation.channel ==
      execution_state_sync::Channel::kPlanningStatus) {
    auto status = std::make_shared<planning::PlanningRuntimeStatus>();
    if (!status->ParseFromString(event.operation.payload)) {
      AERROR << "Invalid PlanningRuntimeStatus execution-state payload";
      return false;
    }
    UpdatePlanningRuntimeStatus(status);
  } else if (event.operation.channel ==
             execution_state_sync::Channel::kControlStatus) {
    auto status = std::make_shared<control::ControlRuntimeStatus>();
    if (!status->ParseFromString(event.operation.payload)) {
      AERROR << "Invalid ControlRuntimeStatus execution-state payload";
      return false;
    }
    UpdateControlRuntimeStatus(
        status, event.operation.control_status_kind ==
                    execution_state_sync::ControlStatusKind::kMotionResult);
  } else if (event.operation.channel ==
             execution_state_sync::Channel::kSafetyStatus) {
    if (event.sequence <= last_safety_status_sequence_) {
      return true;
    }
    if (event.owner != execution_state_sync::Role::kControl) {
      AERROR << "Safety observation was not committed by Control";
      return false;
    }
    control::SafetyStopObservation observation;
    if (!observation.ParseFromString(event.operation.payload) ||
        !observation.has_operation_identity() ||
        observation.operation_identity().requester_epoch().empty() ||
        observation.operation_identity().request_id().empty() ||
        !observation.has_operation_accepted() || !observation.has_enforced() ||
        !observation.has_safety_latched()) {
      AERROR << "Invalid committed Control safety observation";
      return false;
    }
    MissionRequestIdentity identity;
    identity.set_client_epoch(
        observation.operation_identity().requester_epoch());
    identity.set_request_id(observation.operation_identity().request_id());
    MissionRequestRecord record;
    MissionRequestLedgerResult lookup;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mission_request_ledger_ == nullptr) {
        AERROR << "Mission request ledger shut down during safety feedback";
        return false;
      }
      lookup = mission_request_ledger_->Get(identity, &record);
    }
    if (!lookup.ok()) {
      AWARN << "Ignoring safety observation for an unknown Mission request: "
            << lookup.message;
      last_safety_status_sequence_ = event.sequence;
      return true;
    }
    MissionRequest original_request;
    if (!original_request.ParseFromString(record.request_payload) ||
        (original_request.operation_case() !=
             MissionRequest::kSafetyStop &&
         original_request.operation_case() !=
             MissionRequest::kSafetyReset)) {
      AERROR << "Safety observation does not match a durable safety request";
      return false;
    }
    const auto& expected_identity =
        original_request.operation_case() == MissionRequest::kSafetyStop
            ? original_request.safety_stop().identity()
            : original_request.safety_reset().identity();
    if (expected_identity.requester_epoch() !=
            observation.operation_identity().requester_epoch() ||
        expected_identity.request_id() !=
            observation.operation_identity().request_id()) {
      AERROR << "Control safety observation identity does not match request";
      return false;
    }
    if (observation.operation_accepted() && !observation.enforced()) {
      AERROR << "Control accepted a safety operation without enforcing it";
      return false;
    }
    if (original_request.has_safety_stop() &&
        observation.operation_accepted() &&
        (!observation.safety_latched() ||
         !observation.has_safety_identity() ||
         observation.safety_identity().control_epoch().empty() ||
         observation.safety_identity().generation() == 0 ||
         observation.effective_policy() != control::SAFETY_STOP_CONTROLLED)) {
      AERROR << "Accepted safety stop lacks a valid Control latch identity";
      return false;
    }
    if (original_request.has_safety_reset() &&
        observation.operation_accepted() &&
        (observation.safety_latched() ||
         !observation.has_safety_identity() ||
         observation.safety_identity().SerializeAsString() !=
             original_request.safety_reset()
                 .expected_safety_identity()
                 .SerializeAsString())) {
      AERROR << "Control accepted a safety reset without clearing its identity";
      return false;
    }
    observation.set_durably_committed(true);
    observation.set_commit_sequence(event.sequence);
    auto updated_result = record.result;
    updated_result.mutable_safety_observation()->CopyFrom(observation);
    updated_result.set_outcome(observation.operation_accepted()
                                   ? MISSION_REQUEST_ACCEPTED
                                   : MISSION_REQUEST_REJECTED);
    updated_result.set_code(observation.operation_accepted()
                                ? MISSION_REQUEST_OK
                                : MISSION_REQUEST_SAFETY_RESTRICTED);
    updated_result.set_retry_disposition(
        observation.operation_accepted()
            ? MISSION_REQUEST_RECONCILE
            : MISSION_REQUEST_RETRY_AFTER_CHANGE);
    updated_result.set_reason(observation.reason());
    MissionRequestRecord updated_record;
    MissionRequestLedgerResult persisted;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mission_request_ledger_ == nullptr) {
        AERROR << "Mission request ledger shut down during safety feedback";
        return false;
      }
      persisted = mission_request_ledger_->Update(
          identity, record.fingerprint, record.version, updated_result,
          &updated_record);
    }
    if (!persisted.ok()) {
      AERROR << "Failed to persist Control safety outcome: "
             << persisted.message;
      return false;
    }
    last_safety_status_sequence_ = event.sequence;
  }
  return true;
}

planning::MissionPlan MissionContext::BuildMissionPlan(
    const planning::PlanningCommand& command) const {
  planning::MissionPlan plan;
  plan.set_task_type(MissionTaskForScene(command.requested_scene()));
  if (plan.task_type() == planning::MISSION_TASK_UNKNOWN) {
    AERROR << "Legacy scene has no ordinary Mission task mapping: "
           << command.requested_scene();
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
  const auto pending = pending_mission_directives_.find(command.command_id());
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
    directive->mutable_cancel()->mutable_expected_active_identity()->CopyFrom(
        expected);
    directive->mutable_cancel()->set_postcondition(
        planning::MISSION_CANCEL_CONTROLLED_STOP_THEN_HOLD);
    directive->mutable_cancel()->set_reason("Mission behavior halted");
    common::util::FillHeader("mission", directive);
    pending_mission_directives_[command.command_id()].CopyFrom(*directive);
    return true;
  }

  const auto plan = BuildMissionPlan(command);
  if (identity_it == mission_identities_.end()) {
    auto* identity = directive->mutable_identity();
    identity->set_producer_epoch(producer_epoch_);
    identity->set_aggregate_id(command.has_mission_id() ? command.mission_id()
                                                        : command.command_id());
    identity->set_command_id(command.command_id());
    identity->set_revision(1);
    directive->mutable_activate()->mutable_plan()->CopyFrom(plan);
  } else {
    const auto expected = identity_it->second;
    auto next = expected;
    next.set_revision(expected.revision() + 1);
    directive->mutable_identity()->CopyFrom(next);
    directive->mutable_replace()->mutable_expected_active_identity()->CopyFrom(
        expected);
    directive->mutable_replace()->mutable_plan()->CopyFrom(plan);
  }
  common::util::FillHeader("mission", directive);
  mission_plans_[command.command_id()].CopyFrom(plan);
  pending_mission_directives_[command.command_id()].CopyFrom(*directive);
  return true;
}

}  // namespace mission
}  // namespace apollo
