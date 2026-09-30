#include "modules/control/common/motion_execution_manager.h"

#include <cmath>
#include <utility>

namespace apollo {
namespace control {

namespace {

constexpr double kMaxVehicleStateAgeSec = 0.25;

}  // namespace

MotionExecutionManager::MotionExecutionManager(
    MotionExecutionValidator validator)
    : validator_(std::move(validator)) {}

bool MotionExecutionManager::IsTerminal(
    planning::MotionExecutionState state) const {
  return state == planning::MOTION_EXECUTION_REJECTED ||
         state == planning::MOTION_EXECUTION_SUCCEEDED ||
         state == planning::MOTION_EXECUTION_CANCELLED ||
         state == planning::MOTION_EXECUTION_TIMED_OUT ||
         state == planning::MOTION_EXECUTION_FAILED;
}

bool MotionExecutionManager::HasActiveCommand() const {
  return active_command_.has_identity() && status_.has_state() &&
         !IsTerminal(status_.state()) &&
         status_.state() != planning::MOTION_EXECUTION_HOLDING;
}

bool MotionExecutionManager::IsExecuting() const {
  return status_.state() ==
             planning::MOTION_EXECUTION_EXECUTING_TRAJECTORY ||
         status_.state() ==
             planning::MOTION_EXECUTION_EXECUTING_PRIMITIVE;
}

const planning::MotionExecutionCommand*
MotionExecutionManager::active_command() const {
  return HasActiveCommand() ? &active_command_ : nullptr;
}

planning::MotionExecutionStatus MotionExecutionManager::Reject(
    const planning::MotionExecutionCommand& command, double now_sec,
    planning::MotionCommandRejectReason reject_reason,
    const std::string& reason) const {
  planning::MotionExecutionStatus rejected;
  rejected.mutable_header()->set_timestamp_sec(now_sec);
  if (command.has_identity()) {
    rejected.mutable_identity()->CopyFrom(command.identity());
  }
  if (command.has_reference_frame_id()) {
    rejected.set_reference_frame_id(command.reference_frame_id());
  }
  rejected.set_state(planning::MOTION_EXECUTION_REJECTED);
  rejected.set_reject_reason(reject_reason);
  rejected.set_reason(reason);
  rejected.set_terminal_time_sec(now_sec);
  return rejected;
}

planning::MotionExecutionStatus MotionExecutionManager::InvalidTransition(
    double now_sec, const std::string& reason) const {
  auto rejected = status_;
  rejected.mutable_header()->set_timestamp_sec(now_sec);
  rejected.set_reject_reason(planning::MOTION_REJECT_INVALID_TRANSITION);
  rejected.set_reason(reason);
  return rejected;
}

void MotionExecutionManager::InitializeStatus(
    const planning::MotionExecutionCommand& command, double now_sec,
    planning::MotionExecutionType execution_type,
    const planning::MissionCommandIdentity* parent) {
  status_.Clear();
  status_.mutable_header()->set_timestamp_sec(now_sec);
  status_.mutable_identity()->CopyFrom(command.identity());
  status_.set_reference_frame_id(command.reference_frame_id());
  status_.set_execution_type(execution_type);
  if (parent != nullptr) {
    status_.mutable_parent_mission_identity()->CopyFrom(*parent);
  }
  status_.set_state(planning::MOTION_EXECUTION_VALIDATED);
  status_.set_reject_reason(planning::MOTION_REJECT_NONE);
  status_.set_reason("motion command validated");
  status_.set_accepted_time_sec(now_sec);
}

std::string MotionExecutionManager::CommandKey(
    const planning::MotionExecutionCommand& command) const {
  const auto& producer_epoch = command.identity().producer_epoch();
  const auto& aggregate_id = command.identity().aggregate_id();
  const auto& command_id = command.identity().command_id();
  return std::to_string(producer_epoch.size()) + ":" + producer_epoch +
         std::to_string(aggregate_id.size()) + ":" + aggregate_id +
         std::to_string(command_id.size()) + ":" + command_id;
}

std::string MotionExecutionManager::ParentKey(
    const planning::MissionCommandIdentity& identity) const {
  return std::to_string(identity.producer_epoch().size()) + ":" +
         identity.producer_epoch() +
         std::to_string(identity.aggregate_id().size()) + ":" +
         identity.aggregate_id() +
         std::to_string(identity.command_id().size()) + ":" +
         identity.command_id();
}

bool MotionExecutionManager::IsExactIdentity(
    const planning::MotionCommandIdentity& lhs,
    const planning::MotionCommandIdentity& rhs) const {
  return lhs.producer_epoch() == rhs.producer_epoch() &&
         lhs.aggregate_id() == rhs.aggregate_id() &&
         lhs.command_id() == rhs.command_id() &&
         lhs.revision() == rhs.revision();
}

bool MotionExecutionManager::IsExactParentIdentity(
    const planning::MissionCommandIdentity& lhs,
    const planning::MissionCommandIdentity& rhs) const {
  return lhs.producer_epoch() == rhs.producer_epoch() &&
         lhs.aggregate_id() == rhs.aggregate_id() &&
         lhs.command_id() == rhs.command_id() &&
         lhs.revision() == rhs.revision();
}

bool MotionExecutionManager::IsValidParentIdentity(
    const planning::MissionCommandIdentity& identity) const {
  return identity.has_producer_epoch() &&
         !identity.producer_epoch().empty() &&
         identity.has_aggregate_id() &&
         !identity.aggregate_id().empty() &&
         identity.has_command_id() &&
         !identity.command_id().empty() &&
         identity.has_revision() && identity.revision() > 0;
}

bool MotionExecutionManager::IsPlanningIdleHold(
    const planning::MotionExecutionCommand& command) const {
  return command.payload_case() ==
             planning::MotionExecutionCommand::kPrimitive &&
         command.primitive().type() ==
             planning::MOTION_PRIMITIVE_STANDSTILL_HOLD &&
         command.primitive().has_standstill_hold();
}

bool MotionExecutionManager::IsParentFenced(
    const planning::MissionCommandIdentity& identity) const {
  const auto it =
      highest_fenced_revision_by_parent_.find(ParentKey(identity));
  return it != highest_fenced_revision_by_parent_.end() &&
         identity.revision() <= it->second;
}

void MotionExecutionManager::RecordRevision(
    const planning::MotionExecutionCommand& command) {
  const auto key = CommandKey(command);
  auto& record = highest_revision_by_command_[key];
  record.revision = std::max(record.revision, command.identity().revision());
  record.expiry_time_sec =
      std::max(record.expiry_time_sec, command.expiry_time_sec());
}

void MotionExecutionManager::RemoveExpiredRevisionRecords(double now_sec) {
  for (auto it = highest_revision_by_command_.begin();
       it != highest_revision_by_command_.end();) {
    if (now_sec > it->second.expiry_time_sec) {
      it = highest_revision_by_command_.erase(it);
    } else {
      ++it;
    }
  }
}

bool MotionExecutionManager::AdvanceDeadlines(double now_sec) {
  if (!std::isfinite(now_sec) ||
      (last_event_time_sec_ >= 0.0 && now_sec < last_event_time_sec_)) {
    return false;
  }
  last_event_time_sec_ = now_sec;
  RemoveExpiredRevisionRecords(now_sec);
  if (!HasActiveCommand()) {
    return true;
  }
  if (now_sec > active_command_.expiry_time_sec()) {
    SetState(planning::MOTION_EXECUTION_TIMED_OUT, now_sec,
             "motion command expired during execution");
    return true;
  }
  if (active_command_.payload_case() ==
          planning::MotionExecutionCommand::kPrimitive &&
      active_command_.primitive().type() ==
          planning::MOTION_PRIMITIVE_STANDSTILL_HOLD &&
      active_command_.primitive().has_standstill_hold() &&
      status_.has_accepted_time_sec() &&
      now_sec - status_.accepted_time_sec() >
          active_command_.primitive()
              .standstill_hold()
              .reauthorization_period_sec()) {
    SetState(planning::MOTION_EXECUTION_TIMED_OUT, now_sec,
             "standstill hold reauthorization timeout");
    return true;
  }
  if (IsExecuting() && status_.has_execution_start_time_sec() &&
      now_sec - status_.execution_start_time_sec() >
          active_command_.completion().execution_timeout_sec()) {
    SetState(planning::MOTION_EXECUTION_TIMED_OUT, now_sec,
             "motion command execution timeout");
  }
  return true;
}

void MotionExecutionManager::SetState(
    planning::MotionExecutionState state, double now_sec,
    const std::string& reason) {
  status_.mutable_header()->set_timestamp_sec(now_sec);
  status_.set_state(state);
  status_.set_reject_reason(planning::MOTION_REJECT_NONE);
  status_.set_reason(reason);
  if (state == planning::MOTION_EXECUTION_EXECUTING_TRAJECTORY ||
      state == planning::MOTION_EXECUTION_EXECUTING_PRIMITIVE) {
    status_.set_execution_start_time_sec(now_sec);
  }
  if (IsTerminal(state)) {
    if ((state == planning::MOTION_EXECUTION_TIMED_OUT ||
         state == planning::MOTION_EXECUTION_FAILED) &&
        active_command_.has_identity()) {
      status_.set_requested_fallback(active_command_.failure_fallback());
    }
    status_.set_terminal_time_sec(now_sec);
    last_terminal_status_ = status_;
  }
}

planning::MotionExecutionStatus MotionExecutionManager::Submit(
    const planning::MotionExecutionCommand& command, double now_sec) {
  return SubmitCommand(command, nullptr, now_sec);
}

planning::MotionExecutionStatus MotionExecutionManager::Apply(
    const planning::MotionDirective& directive, double now_sec) {
  if (!directive.has_scope() ||
      directive.scope() == planning::MOTION_SCOPE_UNKNOWN) {
    planning::MotionExecutionCommand empty;
    return Reject(empty, now_sec, planning::MOTION_REJECT_MISSING_CONTEXT,
                  "motion directive requires an explicit ownership scope");
  }
  const bool mission_scoped =
      directive.scope() == planning::MOTION_SCOPE_MISSION_DESCENDANT;
  if (mission_scoped &&
      (!directive.has_parent_mission_identity() ||
       !IsValidParentIdentity(directive.parent_mission_identity()))) {
    planning::MotionExecutionCommand empty;
    return Reject(empty, now_sec, planning::MOTION_REJECT_MISSING_CONTEXT,
                  "Mission descendant requires a parent Mission identity");
  }
  if (!mission_scoped && directive.has_parent_mission_identity()) {
    planning::MotionExecutionCommand empty;
    return Reject(empty, now_sec, planning::MOTION_REJECT_MISSING_CONTEXT,
                  "Planning idle hold must not carry Mission ownership");
  }
  const auto* parent = mission_scoped
                           ? &directive.parent_mission_identity()
                           : nullptr;
  switch (directive.operation_case()) {
    case planning::MotionDirective::kExecute:
      if (!directive.execute().has_command()) {
        planning::MotionExecutionCommand empty;
        return Reject(empty, now_sec, planning::MOTION_REJECT_INVALID_PAYLOAD,
                      "execute requires a complete motion command");
      }
      if (!mission_scoped &&
          !IsPlanningIdleHold(directive.execute().command())) {
        return Reject(directive.execute().command(), now_sec,
                      planning::MOTION_REJECT_INVALID_PAYLOAD,
                      "Planning-owned scope permits only standstill hold");
      }
      if (parent != nullptr && IsParentFenced(*parent)) {
        return Reject(directive.execute().command(), now_sec,
                      planning::MOTION_REJECT_PARENT_FENCED,
                      "parent Mission revision is cancelled");
      }
      return SubmitCommand(directive.execute().command(), parent, now_sec);
    case planning::MotionDirective::kReplace: {
      if (!directive.replace().has_command() ||
          !directive.replace().has_expected_active_identity()) {
        planning::MotionExecutionCommand empty;
        return Reject(empty, now_sec, planning::MOTION_REJECT_INVALID_PAYLOAD,
                      "replace requires expected identity and complete command");
      }
      if (!HasActiveCommand() ||
          !IsExactIdentity(directive.replace().expected_active_identity(),
                           active_command_.identity()) ||
          (mission_scoped
               ? !IsExactParentIdentity(*parent, active_parent_identity_)
               : active_parent_identity_.has_revision())) {
        return Reject(directive.replace().command(), now_sec,
                      planning::MOTION_REJECT_CAS_MISMATCH,
                      "replace target does not match active motion ownership");
      }
      if (!active_command_.preemptible()) {
        return Reject(directive.replace().command(), now_sec,
                      planning::MOTION_REJECT_BUSY,
                      "active motion does not permit replacement");
      }
      if (!mission_scoped &&
          (!IsPlanningIdleHold(active_command_) ||
           !IsPlanningIdleHold(directive.replace().command()))) {
        return Reject(directive.replace().command(), now_sec,
                      planning::MOTION_REJECT_INVALID_PAYLOAD,
                      "Planning-owned replacement is limited to idle-hold renewal");
      }
      if (parent != nullptr && IsParentFenced(*parent)) {
        return Reject(directive.replace().command(), now_sec,
                      planning::MOTION_REJECT_PARENT_FENCED,
                      "parent Mission revision is cancelled");
      }
      if (!AdvanceDeadlines(now_sec)) {
        return Reject(directive.replace().command(), now_sec,
                      planning::MOTION_REJECT_INVALID_TIME,
                      "manager time must be finite and monotonic");
      }
      if (!HasActiveCommand() ||
          !IsExactIdentity(directive.replace().expected_active_identity(),
                           active_command_.identity())) {
        return Reject(directive.replace().command(), now_sec,
                      planning::MOTION_REJECT_CAS_MISMATCH,
                      "active motion expired before replacement admission");
      }
      const auto replacement_validation =
          validator_.Validate(directive.replace().command(), now_sec);
      if (!replacement_validation.accepted) {
        return Reject(directive.replace().command(), now_sec,
                      replacement_validation.reject_reason,
                      replacement_validation.reason);
      }
      const auto replacement_revision =
          highest_revision_by_command_.find(
              CommandKey(directive.replace().command()));
      if (replacement_revision != highest_revision_by_command_.end() &&
          directive.replace().command().identity().revision() <=
              replacement_revision->second.revision) {
        return Reject(directive.replace().command(), now_sec,
                      planning::MOTION_REJECT_REPLAY,
                      "replacement revision is stale or duplicated");
      }
      pending_replacement_.CopyFrom(directive.replace().command());
      pending_parent_identity_.Clear();
      if (parent != nullptr) {
        pending_parent_identity_.CopyFrom(*parent);
      }
      pending_execution_type_ = replacement_validation.execution_type;
      RecordRevision(pending_replacement_);
      SetState(planning::MOTION_EXECUTION_CANCELLING, now_sec,
               "exact replacement admitted; awaiting executor revocation");
      return status_;
    }
    case planning::MotionDirective::kCancel: {
      if (!mission_scoped) {
        planning::MotionExecutionCommand empty;
        return Reject(empty, now_sec, planning::MOTION_REJECT_INVALID_PAYLOAD,
                      "Planning idle hold cannot issue cancellation");
      }
      planning::MotionExecutionCommand target;
      if (directive.cancel().has_target_identity()) {
        target.mutable_identity()->CopyFrom(
            directive.cancel().target_identity());
      }
      if (!AdvanceDeadlines(now_sec)) {
        return Reject(target, now_sec, planning::MOTION_REJECT_INVALID_TIME,
                      "manager time must be finite and monotonic");
      }
      if (directive.cancel().fence_parent_mission()) {
        auto& highest =
            highest_fenced_revision_by_parent_[ParentKey(*parent)];
        highest = std::max(highest, parent->revision());
      }
      if (!directive.cancel().has_target_identity() && !HasActiveCommand() &&
          directive.cancel().fence_parent_mission()) {
        planning::MotionExecutionStatus fenced;
        fenced.mutable_header()->set_timestamp_sec(now_sec);
        fenced.mutable_parent_mission_identity()->CopyFrom(*parent);
        fenced.set_state(planning::MOTION_EXECUTION_CANCELLED);
        fenced.set_reject_reason(planning::MOTION_REJECT_NONE);
        fenced.set_reason("parent Mission fenced with no active descendant");
        fenced.set_terminal_time_sec(now_sec);
        return fenced;
      }
      if (!directive.cancel().has_target_identity() || !HasActiveCommand() ||
          !IsExactIdentity(directive.cancel().target_identity(),
                           active_command_.identity()) ||
          !IsExactParentIdentity(*parent, active_parent_identity_)) {
        return Reject(target, now_sec, planning::MOTION_REJECT_CAS_MISMATCH,
                      "cancel target does not match active motion ownership");
      }
      pending_replacement_.Clear();
      pending_parent_identity_.Clear();
      pending_execution_type_ = planning::MOTION_EXECUTION_TYPE_UNKNOWN;
      SetState(planning::MOTION_EXECUTION_CANCELLING, now_sec,
               directive.cancel().reason().empty()
                   ? "cancellation admitted; awaiting executor revocation"
                   : directive.cancel().reason());
      return status_;
    }
    case planning::MotionDirective::OPERATION_NOT_SET:
    default: {
      planning::MotionExecutionCommand empty;
      return Reject(empty, now_sec, planning::MOTION_REJECT_INVALID_PAYLOAD,
                    "motion directive requires exactly one operation");
    }
  }
}

planning::MotionExecutionStatus
MotionExecutionManager::ConfirmExecutorRevoked(
    double now_sec, const std::string& reason) {
  if (!AdvanceDeadlines(now_sec)) {
    return InvalidTransition(now_sec,
                             "manager time must be finite and monotonic");
  }
  if (status_.state() != planning::MOTION_EXECUTION_CANCELLING) {
    return InvalidTransition(
        now_sec, "executor revocation requires a cancelling command");
  }
  SetState(planning::MOTION_EXECUTION_CANCELLED, now_sec,
           reason.empty() ? "executor ownership revoked" : reason);
  if (!pending_replacement_.has_identity()) {
    return status_;
  }
  active_command_.Swap(&pending_replacement_);
  active_parent_identity_.Swap(&pending_parent_identity_);
  const auto execution_type = pending_execution_type_;
  pending_execution_type_ = planning::MOTION_EXECUTION_TYPE_UNKNOWN;
  InitializeStatus(
      active_command_, now_sec, execution_type,
      active_parent_identity_.has_revision() ? &active_parent_identity_
                                             : nullptr);
  return status_;
}

planning::MotionExecutionStatus MotionExecutionManager::SubmitCommand(
    const planning::MotionExecutionCommand& command,
    const planning::MissionCommandIdentity* parent, double now_sec) {
  if (!AdvanceDeadlines(now_sec)) {
    return Reject(command, now_sec, planning::MOTION_REJECT_INVALID_TIME,
                  "manager time must be finite and monotonic");
  }
  const auto validation = validator_.Validate(command, now_sec);
  if (!validation.accepted) {
    return Reject(command, now_sec, validation.reject_reason,
                  validation.reason);
  }

  const auto revision_it =
      highest_revision_by_command_.find(CommandKey(command));
  if (revision_it != highest_revision_by_command_.end() &&
      command.identity().revision() <= revision_it->second.revision) {
    return Reject(command, now_sec, planning::MOTION_REJECT_REPLAY,
                  "command revision is stale or duplicated");
  }

  if (HasActiveCommand()) {
    return Reject(command, now_sec, planning::MOTION_REJECT_BUSY,
                  "active motion requires explicit replace or cancel");
  }

  active_command_ = command;
  active_parent_identity_.Clear();
  if (parent != nullptr) {
    active_parent_identity_.CopyFrom(*parent);
  }
  RecordRevision(command);
  InitializeStatus(command, now_sec, validation.execution_type, parent);
  return status_;
}

planning::MotionExecutionStatus MotionExecutionManager::Arm(double now_sec) {
  if (!AdvanceDeadlines(now_sec)) {
    return InvalidTransition(now_sec,
                             "manager time must be finite and monotonic");
  }
  if (IsTerminal(status_.state())) {
    return status_;
  }
  if (status_.state() != planning::MOTION_EXECUTION_VALIDATED) {
    return InvalidTransition(now_sec,
                             "only a validated command can be armed");
  }
  SetState(planning::MOTION_EXECUTION_ARMED, now_sec, "motion command armed");
  return status_;
}

planning::MotionExecutionStatus MotionExecutionManager::Start(
    const MotionExecutionVehicleState& vehicle_state, double now_sec) {
  if (!AdvanceDeadlines(now_sec)) {
    return InvalidTransition(now_sec,
                             "manager time must be finite and monotonic");
  }
  if (IsTerminal(status_.state())) {
    return status_;
  }
  if (status_.state() != planning::MOTION_EXECUTION_ARMED) {
    return InvalidTransition(now_sec,
                             "only an armed command can start execution");
  }
  std::string mismatch_reason;
  if (!StartConditionMatches(active_command_, vehicle_state, now_sec,
                             &mismatch_reason)) {
    return Reject(active_command_, now_sec,
                  planning::MOTION_REJECT_INVALID_START_CONDITION,
                  mismatch_reason);
  }
  if (active_command_.payload_case() ==
      planning::MotionExecutionCommand::kTrajectory) {
    SetState(planning::MOTION_EXECUTION_EXECUTING_TRAJECTORY, now_sec,
             "trajectory execution started");
  } else if (active_command_.payload_case() ==
             planning::MotionExecutionCommand::kPrimitive) {
    SetState(planning::MOTION_EXECUTION_EXECUTING_PRIMITIVE, now_sec,
             "primitive execution started");
  } else {
    return InvalidTransition(now_sec, "active command has no motion payload");
  }
  return status_;
}

bool MotionExecutionManager::StartConditionMatches(
    const planning::MotionExecutionCommand& command,
    const MotionExecutionVehicleState& vehicle_state,
    double now_sec, std::string* reason) const {
  const auto& start = command.start_condition();
  const bool finite_state =
      vehicle_state.position.has_x() &&
      vehicle_state.position.has_y() &&
      std::isfinite(vehicle_state.position.x()) &&
      std::isfinite(vehicle_state.position.y()) &&
      std::isfinite(vehicle_state.heading) &&
      std::isfinite(vehicle_state.speed_mps) &&
      std::isfinite(vehicle_state.timestamp_sec) &&
      vehicle_state.timestamp_sec <= now_sec &&
      vehicle_state.timestamp_sec >= command.effective_time_sec() &&
      now_sec - vehicle_state.timestamp_sec <= kMaxVehicleStateAgeSec;
  const double dx =
      vehicle_state.position.x() - start.expected_position().x();
  const double dy =
      vehicle_state.position.y() - start.expected_position().y();
  const double heading_error =
      std::abs(std::atan2(std::sin(vehicle_state.heading -
                                  start.expected_heading()),
                         std::cos(vehicle_state.heading -
                                  start.expected_heading())));
  if (!finite_state ||
      vehicle_state.reference_frame_id != start.reference_frame_id() ||
      std::hypot(dx, dy) > start.max_position_error_m() ||
      heading_error > start.max_heading_error_rad() ||
      vehicle_state.gear != start.expected_gear() ||
      std::abs(vehicle_state.speed_mps) > start.max_abs_speed_mps()) {
    if (reason != nullptr) {
      *reason = "vehicle state does not satisfy the admitted start condition";
    }
    return false;
  }
  return true;
}

planning::MotionExecutionStatus MotionExecutionManager::Succeed(
    double now_sec, const std::string& reason) {
  if (!AdvanceDeadlines(now_sec)) {
    return InvalidTransition(now_sec,
                             "manager time must be finite and monotonic");
  }
  if (IsTerminal(status_.state())) {
    return status_;
  }
  if (!IsExecuting()) {
    return InvalidTransition(now_sec,
                             "only an executing command can succeed");
  }
  SetState(planning::MOTION_EXECUTION_SUCCEEDED, now_sec, reason);
  return status_;
}

planning::MotionExecutionStatus MotionExecutionManager::Fail(
    double now_sec, const std::string& reason) {
  if (!AdvanceDeadlines(now_sec)) {
    return InvalidTransition(now_sec,
                             "manager time must be finite and monotonic");
  }
  if (IsTerminal(status_.state())) {
    return status_;
  }
  if (!HasActiveCommand()) {
    return InvalidTransition(now_sec, "there is no active command to fail");
  }
  pending_replacement_.Clear();
  pending_parent_identity_.Clear();
  pending_execution_type_ = planning::MOTION_EXECUTION_TYPE_UNKNOWN;
  SetState(planning::MOTION_EXECUTION_FAILED, now_sec, reason);
  return status_;
}

planning::MotionExecutionStatus MotionExecutionManager::EnterHolding(
    double now_sec, const std::string& reason) {
  if (!AdvanceDeadlines(now_sec)) {
    return InvalidTransition(now_sec,
                             "manager time must be finite and monotonic");
  }
  if (!IsTerminal(status_.state()) &&
      status_.state() != planning::MOTION_EXECUTION_HOLDING) {
    return InvalidTransition(
        now_sec, "holding is allowed only after a terminal outcome");
  }
  SetState(planning::MOTION_EXECUTION_HOLDING, now_sec, reason);
  active_command_.Clear();
  active_parent_identity_.Clear();
  return status_;
}

planning::MotionExecutionStatus MotionExecutionManager::Tick(double now_sec) {
  if (!AdvanceDeadlines(now_sec)) {
    return InvalidTransition(now_sec,
                             "manager time must be finite and monotonic");
  }
  return status_;
}

}  // namespace control
}  // namespace apollo
