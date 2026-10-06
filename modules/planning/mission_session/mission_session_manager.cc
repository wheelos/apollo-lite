#include "modules/planning/mission_session/mission_session_manager.h"

#include <cmath>

namespace apollo {
namespace planning {

namespace {

bool IsFinite(double value) { return std::isfinite(value); }

bool HasFinitePoint(const common::PointENU& point) {
  return point.has_x() && point.has_y() && IsFinite(point.x()) &&
         IsFinite(point.y()) && (!point.has_z() || IsFinite(point.z()));
}

double NormalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

MissionAdmissionResult Reject(MissionAdmissionCode code,
                              const std::string& reason) {
  return {false, code, reason};
}

MissionAdmissionResult Accept(MissionAdmissionCode code,
                              const std::string& reason) {
  return {true, code, reason};
}

}  // namespace

PlanningCommand MissionSessionManager::BuildPlanningCommand() const {
  PlanningCommand command;
  if (!guidance_.identity.has_revision()) {
    return command;
  }
  command.set_mission_id(guidance_.identity.aggregate_id());
  command.set_command_id(guidance_.identity.command_id());
  command.set_action(guidance_.state == MISSION_SESSION_CANCELLING
                         ? COMMAND_CANCEL
                         : COMMAND_ACTIVATE);
  switch (guidance_.plan.task_type()) {
    case MISSION_TASK_A_TO_B:
      command.set_requested_scene(SCENE_LANE_CRUISE);
      break;
    case MISSION_TASK_PARK_IN:
      command.set_requested_scene(SCENE_PARK_IN);
      break;
    case MISSION_TASK_PULL_OVER:
      command.set_requested_scene(SCENE_PULL_OVER);
      break;
    case MISSION_TASK_DOCK:
      command.set_requested_scene(SCENE_DOCK);
      break;
    case MISSION_TASK_SUMMON:
      command.set_requested_scene(SCENE_SUMMON);
      break;
    case MISSION_TASK_UNKNOWN:
    default:
      command.set_requested_scene(SCENE_UNKNOWN);
      break;
  }
  const auto& plan = guidance_.plan;
  if (plan.has_preferred_mode())
    command.set_preferred_mode(plan.preferred_mode());
  if (plan.has_goal()) command.mutable_goal()->CopyFrom(plan.goal());
  if (plan.has_route_hint())
    command.mutable_route_hint()->CopyFrom(plan.route_hint());
  if (plan.has_priority()) command.set_priority(plan.priority());
  command.set_preemptible(plan.preemptible());
  if (plan.has_completion())
    command.mutable_completion()->CopyFrom(plan.completion());
  if (plan.has_fallback())
    command.mutable_fallback()->CopyFrom(plan.fallback());
  if (plan.has_recovery())
    command.mutable_recovery()->CopyFrom(plan.recovery());
  if (plan.has_domain_policy())
    command.mutable_domain_policy()->CopyFrom(plan.domain_policy());
  if (plan.has_open_space())
    command.mutable_open_space()->CopyFrom(plan.open_space());
  for (const auto& tag : plan.tags()) command.add_tags(tag);
  return command;
}

MissionAdmissionResult MissionSessionManager::Apply(
    const MissionDirective& directive,
    const localization::LocalizationEstimate& localization, double now_sec) {
  if (!IsFinite(now_sec) || !directive.has_identity()) {
    return Reject(MissionAdmissionCode::kInvalidDirective,
                  "mission directive requires identity and finite time");
  }
  const auto identity_result = ValidateDirectiveIdentity(directive.identity());
  if (!identity_result.accepted) {
    return identity_result;
  }
  if (directive.has_replace() &&
      (guidance_.cancellation_fenced ||
       guidance_.state == MISSION_SESSION_CANCELLING)) {
    return Reject(MissionAdmissionCode::kInvalidTransition,
                  "a cancelling mission cannot be replaced");
  }
  if (last_accepted_directive_.has_identity() &&
      IsExactIdentity(directive.identity(),
                      last_accepted_directive_.identity())) {
    if (directive.SerializeAsString() !=
        last_accepted_directive_.SerializeAsString()) {
      return Reject(MissionAdmissionCode::kInvalidDirective,
                    "accepted directive identity cannot change its operation");
    }
    return Accept(MissionAdmissionCode::kDuplicate,
                  "directive already accepted");
  }
  MissionAdmissionResult result;
  switch (directive.operation_case()) {
    case MissionDirective::kActivate:
      result = ApplyActivate(directive, localization, now_sec);
      break;
    case MissionDirective::kReplace:
      result = ApplyReplace(directive, localization, now_sec);
      break;
    case MissionDirective::kCancel:
      result = ApplyCancel(directive);
      break;
    case MissionDirective::kSuspend:
    case MissionDirective::kResume:
      return Reject(MissionAdmissionCode::kUnsupportedOperation,
                    "external suspend/resume requires a stop/hold and "
                    "reauthorization implementation");
    case MissionDirective::OPERATION_NOT_SET:
    default:
      return Reject(MissionAdmissionCode::kInvalidDirective,
                    "mission directive requires exactly one operation");
  }
  if (result.accepted) {
    last_accepted_directive_identity_.CopyFrom(directive.identity());
    last_accepted_directive_.CopyFrom(directive);
  }
  return result;
}

MissionAdmissionResult MissionSessionManager::ApplyActivate(
    const MissionDirective& directive,
    const localization::LocalizationEstimate& localization, double now_sec) {
  if (!directive.activate().has_plan()) {
    return Reject(MissionAdmissionCode::kInvalidPlan,
                  "activate requires a complete mission plan");
  }
  if (HasActiveSession()) {
    if (IsExactIdentity(directive.identity(), guidance_.identity)) {
      if (directive.activate().plan().SerializeAsString() !=
          guidance_.plan.SerializeAsString()) {
        return Reject(MissionAdmissionCode::kInvalidDirective,
                      "duplicate mission identity has different plan");
      }
      return Accept(MissionAdmissionCode::kDuplicate,
                    "duplicate active mission directive");
    }
    return Reject(MissionAdmissionCode::kBusy,
                  "activate cannot replace an active mission session");
  }
  return AcceptPlan(directive.identity(), directive.activate().plan(),
                    localization, now_sec);
}

MissionAdmissionResult MissionSessionManager::ApplyReplace(
    const MissionDirective& directive,
    const localization::LocalizationEstimate& localization, double now_sec) {
  if (guidance_.cancellation_fenced ||
      guidance_.state == MISSION_SESSION_CANCELLING) {
    return Reject(MissionAdmissionCode::kInvalidTransition,
                  "a cancelling mission cannot be replaced");
  }
  if (!HasActiveSession() ||
      !directive.replace().has_expected_active_identity() ||
      !IsExactIdentity(directive.replace().expected_active_identity(),
                       guidance_.identity)) {
    return Reject(MissionAdmissionCode::kCasMismatch,
                  "replace target does not match the active mission");
  }
  if (!guidance_.plan.preemptible()) {
    return Reject(MissionAdmissionCode::kBusy,
                  "active mission does not permit replacement");
  }
  if (!directive.replace().has_plan()) {
    return Reject(MissionAdmissionCode::kInvalidPlan,
                  "replace requires a complete mission plan");
  }
  if (directive.identity().producer_epoch() !=
          guidance_.identity.producer_epoch() ||
      directive.identity().aggregate_id() !=
          guidance_.identity.aggregate_id() ||
      directive.identity().command_id() != guidance_.identity.command_id() ||
      directive.identity().revision() <= guidance_.identity.revision()) {
    return Reject(
        MissionAdmissionCode::kReplay,
        "replace identity must be a newer revision of the active command");
  }
  return AcceptPlan(directive.identity(), directive.replace().plan(),
                    localization, now_sec);
}

MissionAdmissionResult MissionSessionManager::ApplyCancel(
    const MissionDirective& directive) {
  if (!HasActiveSession() ||
      !directive.cancel().has_expected_active_identity() ||
      !IsExactIdentity(directive.cancel().expected_active_identity(),
                       guidance_.identity)) {
    return Reject(MissionAdmissionCode::kCasMismatch,
                  "cancel target does not match the active mission");
  }
  if (directive.identity().producer_epoch() !=
          guidance_.identity.producer_epoch() ||
      directive.identity().aggregate_id() !=
          guidance_.identity.aggregate_id() ||
      directive.identity().command_id() != guidance_.identity.command_id() ||
      directive.identity().revision() <= guidance_.identity.revision()) {
    return Reject(
        MissionAdmissionCode::kReplay,
        "cancel identity must be a newer revision of the active command");
  }
  if (directive.cancel().postcondition() !=
      MISSION_CANCEL_CONTROLLED_STOP_THEN_HOLD) {
    return Reject(MissionAdmissionCode::kInvalidDirective,
                  "normal cancel requires controlled-stop-then-hold");
  }
  if (guidance_.cancellation_fenced &&
      guidance_.state == MISSION_SESSION_CANCELLING) {
    return Accept(MissionAdmissionCode::kDuplicate,
                  "duplicate mission cancellation directive");
  }
  guidance_.cancellation_fenced = true;
  guidance_.state = MISSION_SESSION_CANCELLING;
  highest_revision_by_command_[CommandKey(directive.identity())] =
      directive.identity().revision();
  reason_ = directive.cancel().reason().empty()
                ? "mission cancellation accepted"
                : directive.cancel().reason();
  return Accept(MissionAdmissionCode::kAccepted, reason_);
}

MissionAdmissionResult MissionSessionManager::AcceptPlan(
    const MissionCommandIdentity& identity, const MissionPlan& plan,
    const localization::LocalizationEstimate& localization, double now_sec) {
  const auto plan_result = ValidatePlan(plan);
  if (!plan_result.accepted) {
    return plan_result;
  }
  const auto key = CommandKey(identity);
  const auto revision = highest_revision_by_command_.find(key);
  if (revision != highest_revision_by_command_.end() &&
      identity.revision() <= revision->second) {
    return Reject(MissionAdmissionCode::kReplay,
                  "mission revision is stale or replayed");
  }

  MissionStartSnapshot snapshot;
  const auto snapshot_result =
      BuildStartSnapshot(plan.start(), localization, now_sec, &snapshot);
  if (!snapshot_result.accepted) {
    return snapshot_result;
  }

  guidance_.identity.CopyFrom(identity);
  guidance_.plan.CopyFrom(plan);
  guidance_.accepted_start.CopyFrom(snapshot);
  guidance_.phase = MISSION_PHASE_UNKNOWN;
  guidance_.route.Clear();
  guidance_.state = MISSION_SESSION_ACCEPTED;
  guidance_.cancellation_fenced = false;
  reason_ = "mission plan accepted";
  highest_revision_by_command_[key] = identity.revision();
  return Accept(MissionAdmissionCode::kAccepted, reason_);
}

MissionAdmissionResult MissionSessionManager::BuildStartSnapshot(
    const MissionStart& start,
    const localization::LocalizationEstimate& localization, double now_sec,
    MissionStartSnapshot* snapshot) const {
  if (snapshot == nullptr || !localization.has_pose() ||
      !localization.pose().has_position() ||
      !HasFinitePoint(localization.pose().position()) ||
      !localization.pose().has_heading() ||
      !IsFinite(localization.pose().heading()) || !localization.has_header() ||
      !localization.header().has_frame_id() ||
      localization.header().frame_id().empty()) {
    return Reject(
        MissionAdmissionCode::kInvalidStart,
        "acceptance requires a timestamped localization pose and frame");
  }

  if (!localization.has_measurement_time() &&
      !localization.header().has_timestamp_sec()) {
    return Reject(MissionAdmissionCode::kInvalidStart,
                  "acceptance requires an explicit observation timestamp");
  }
  const double observed_at = localization.has_measurement_time()
                                 ? localization.measurement_time()
                                 : localization.header().timestamp_sec();
  if (!IsFinite(observed_at) || observed_at > now_sec) {
    return Reject(MissionAdmissionCode::kInvalidStart,
                  "acceptance observation time must be finite and not future");
  }

  switch (start.source_case()) {
    case MissionStart::kCurrentPoseAtAcceptance:
      if (!start.current_pose_at_acceptance()) {
        return Reject(MissionAdmissionCode::kInvalidStart,
                      "current-pose start must be explicitly enabled");
      }
      break;
    case MissionStart::kExplicitStart: {
      const auto& explicit_start = start.explicit_start();
      if (!explicit_start.has_position() ||
          !HasFinitePoint(explicit_start.position()) ||
          !explicit_start.has_heading() ||
          !IsFinite(explicit_start.heading()) ||
          !explicit_start.has_reference_frame_id() ||
          explicit_start.reference_frame_id().empty() ||
          explicit_start.reference_frame_id() !=
              localization.header().frame_id() ||
          !explicit_start.has_max_position_error_m() ||
          !IsFinite(explicit_start.max_position_error_m()) ||
          explicit_start.max_position_error_m() < 0.0 ||
          !explicit_start.has_max_heading_error_rad() ||
          !IsFinite(explicit_start.max_heading_error_rad()) ||
          explicit_start.max_heading_error_rad() < 0.0) {
        return Reject(MissionAdmissionCode::kInvalidStart,
                      "explicit start pose or tolerances are incomplete");
      }
      const double dx =
          explicit_start.position().x() - localization.pose().position().x();
      const double dy =
          explicit_start.position().y() - localization.pose().position().y();
      if (std::hypot(dx, dy) > explicit_start.max_position_error_m() ||
          std::abs(NormalizeAngle(explicit_start.heading() -
                                  localization.pose().heading())) >
              explicit_start.max_heading_error_rad()) {
        return Reject(
            MissionAdmissionCode::kInvalidStart,
            "vehicle state does not match the explicit mission start");
      }
      break;
    }
    case MissionStart::SOURCE_NOT_SET:
    default:
      return Reject(MissionAdmissionCode::kInvalidStart,
                    "mission start policy is required");
  }

  snapshot->mutable_position()->CopyFrom(localization.pose().position());
  snapshot->set_heading(localization.pose().heading());
  snapshot->set_reference_frame_id(localization.header().frame_id());
  snapshot->set_snapshot_time_sec(observed_at);
  return Accept(MissionAdmissionCode::kAccepted, "start snapshot accepted");
}

MissionAdmissionResult MissionSessionManager::ValidateDirectiveIdentity(
    const MissionCommandIdentity& identity) const {
  if (!identity.has_producer_epoch() || identity.producer_epoch().empty() ||
      !identity.has_aggregate_id() || identity.aggregate_id().empty() ||
      !identity.has_command_id() || identity.command_id().empty() ||
      !identity.has_revision() || identity.revision() == 0) {
    return Reject(MissionAdmissionCode::kInvalidIdentity,
                  "mission identity fields and revision are required");
  }
  return Accept(MissionAdmissionCode::kAccepted, "identity accepted");
}

MissionAdmissionResult MissionSessionManager::ValidatePlan(
    const MissionPlan& plan) const {
  if (plan.task_type() == MISSION_TASK_PARK_OUT ||
      plan.task_type() == MISSION_TASK_ESCAPE ||
      plan.task_type() == MISSION_TASK_ROTATE_IN_PLACE ||
      plan.task_type() == MISSION_TASK_REACH_POSE ||
      plan.task_type() == MISSION_TASK_STANDSTILL_WAIT ||
      plan.has_travel_permission() || plan.has_budget_authorization_id() ||
      plan.task_policy_case() != MissionPlan::TASK_POLICY_NOT_SET ||
      plan.goal().has_reference_frame_id() ||
      plan.goal().has_vehicle_reference_point_id() ||
      plan.goal().has_resolver_version() ||
      plan.goal().has_transform_version() ||
      plan.completion().has_speed_tolerance_mps() ||
      plan.completion().has_angular_speed_tolerance_radps() ||
      plan.completion().has_settle_time_sec()) {
    return Reject(MissionAdmissionCode::kUnsupportedOperation,
                  "declared task type/policy requires an implemented "
                  "capability-gated task dispatcher");
  }
  if (!plan.has_task_type() || plan.task_type() == MISSION_TASK_UNKNOWN ||
      !plan.has_start() || !plan.has_goal() ||
      plan.goal().target_case() == GoalSpec::TARGET_NOT_SET ||
      !plan.has_completion() || !plan.completion().has_timeout_sec() ||
      !IsFinite(plan.completion().timeout_sec()) ||
      plan.completion().timeout_sec() <= 0.0) {
    return Reject(MissionAdmissionCode::kInvalidPlan,
                  "mission plan requires task, start, goal, and timeout");
  }
  return Accept(MissionAdmissionCode::kAccepted, "mission plan accepted");
}

MissionAdmissionResult MissionSessionManager::MarkExecuting() {
  if (guidance_.state != MISSION_SESSION_ACCEPTED) {
    return Reject(MissionAdmissionCode::kInvalidTransition,
                  "only an accepted session can execute");
  }
  return Transition(MISSION_SESSION_EXECUTING, "mission execution started");
}

MissionAdmissionResult MissionSessionManager::UpdateRoute(
    const MissionCommandIdentity& expected_identity,
    const MissionRouteContext& route) {
  if (!HasActiveSession() ||
      !IsExactIdentity(expected_identity, guidance_.identity)) {
    return Reject(MissionAdmissionCode::kCasMismatch,
                  "route update does not match the active mission");
  }
  if (!route.has_request_id() || route.request_id().empty() ||
      !route.has_state() || route.state() == MISSION_ROUTE_NONE ||
      (route.state() == MISSION_ROUTE_READY &&
       (!route.has_map_version() || route.map_version().empty() ||
        !route.has_route_id() || route.route_id().empty())) ||
      (route.state() == MISSION_ROUTE_FAILED &&
       (!route.has_reason() || route.reason().empty()))) {
    return Reject(MissionAdmissionCode::kInvalidPlan,
                  "route context identity, state, and provenance are required");
  }
  guidance_.route.CopyFrom(route);
  if (route.state() == MISSION_ROUTE_REQUESTED &&
      guidance_.phase == MISSION_PHASE_UNKNOWN) {
    guidance_.phase = MISSION_PHASE_ROUTING;
  } else if (route.state() == MISSION_ROUTE_READY &&
             (guidance_.phase == MISSION_PHASE_UNKNOWN ||
              guidance_.phase == MISSION_PHASE_ROUTING)) {
    guidance_.phase = MISSION_PHASE_ENROUTE;
  }
  if (route.state() == MISSION_ROUTE_FAILED) {
    reason_ = route.reason();
  }
  return Accept(MissionAdmissionCode::kAccepted, "mission route updated");
}

MissionAdmissionResult MissionSessionManager::AdvancePhase(
    const MissionCommandIdentity& expected_identity,
    MissionSessionPhase next_phase) {
  if (!HasActiveSession() ||
      !IsExactIdentity(expected_identity, guidance_.identity)) {
    return Reject(MissionAdmissionCode::kCasMismatch,
                  "phase update does not match the active mission");
  }
  if (!IsPhaseTransitionAllowed(guidance_.phase, next_phase)) {
    return Reject(MissionAdmissionCode::kInvalidTransition,
                  "mission phase transition is not allowed");
  }
  guidance_.phase = next_phase;
  return Accept(MissionAdmissionCode::kAccepted, "mission phase advanced");
}

MissionAdmissionResult MissionSessionManager::Suspend(
    const std::string& reason) {
  if (guidance_.state != MISSION_SESSION_EXECUTING) {
    return Reject(MissionAdmissionCode::kInvalidTransition,
                  "only an executing session can be suspended");
  }
  return Transition(MISSION_SESSION_SUSPENDED, reason);
}

MissionAdmissionResult MissionSessionManager::Resume() {
  if (guidance_.state != MISSION_SESSION_SUSPENDED) {
    return Reject(MissionAdmissionCode::kInvalidTransition,
                  "only a suspended session can resume");
  }
  return Transition(MISSION_SESSION_EXECUTING, "mission execution resumed");
}

MissionAdmissionResult MissionSessionManager::BeginCompleting() {
  if (guidance_.state != MISSION_SESSION_EXECUTING) {
    return Reject(MissionAdmissionCode::kInvalidTransition,
                  "only an executing session can begin completion");
  }
  return Transition(MISSION_SESSION_COMPLETING,
                    "mission terminal conditions observed");
}

MissionAdmissionResult MissionSessionManager::Complete(
    const MotionTerminalEvidence& terminal_evidence) {
  if (guidance_.state != MISSION_SESSION_COMPLETING) {
    return Reject(MissionAdmissionCode::kInvalidTransition,
                  "completion requires correlated terminal evidence");
  }
  std::string evidence_error;
  if (!ValidateHoldEvidence(terminal_evidence, &evidence_error)) {
    return Reject(MissionAdmissionCode::kInvalidTransition,
                  "completion evidence rejected: " + evidence_error);
  }
  return Transition(MISSION_SESSION_COMPLETED, "mission completed");
}

MissionAdmissionResult MissionSessionManager::ConfirmCancellation(
    const MotionTerminalEvidence& terminal_evidence) {
  if (guidance_.state != MISSION_SESSION_CANCELLING) {
    return Reject(
        MissionAdmissionCode::kInvalidTransition,
        "cancellation requires terminal motion and idle-hold confirmation");
  }
  std::string evidence_error;
  if (!ValidateHoldEvidence(terminal_evidence, &evidence_error)) {
    return Reject(MissionAdmissionCode::kInvalidTransition,
                  "cancellation evidence rejected: " + evidence_error);
  }
  return Transition(MISSION_SESSION_CANCELLED, "mission cancelled");
}

bool MissionSessionManager::ValidateHoldEvidence(
    const MotionTerminalEvidence& evidence, std::string* reason) const {
  const auto reject = [reason](const std::string& message) {
    if (reason != nullptr) {
      *reason = message;
    }
    return false;
  };
  if (evidence.has_absolute_angular_speed_radps() ||
      evidence.has_rotation_progress()) {
    return reject("angular hold evidence contract is unavailable");
  }
  if (!evidence.has_contract_version() || evidence.contract_version() != 1 ||
      !evidence.has_kind() ||
      evidence.kind() != MOTION_TERMINAL_EVIDENCE_STANDSTILL_HOLD ||
      !evidence.has_motion_identity() ||
      !evidence.motion_identity().has_producer_epoch() ||
      evidence.motion_identity().producer_epoch().empty() ||
      !evidence.motion_identity().has_aggregate_id() ||
      evidence.motion_identity().aggregate_id().empty() ||
      !evidence.motion_identity().has_command_id() ||
      evidence.motion_identity().command_id().empty() ||
      !evidence.motion_identity().has_revision() ||
      evidence.motion_identity().revision() == 0 ||
      !evidence.has_parent_mission_identity() ||
      !IsExactIdentity(evidence.parent_mission_identity(),
                       guidance_.identity) ||
      !evidence.has_authority_generation() ||
      evidence.authority_generation() != guidance_.identity.revision() ||
      !evidence.has_observed_at_sec() ||
      !IsFinite(evidence.observed_at_sec()) ||
      !evidence.has_reference_frame_id() ||
      evidence.reference_frame_id().empty() ||
      evidence.reference_frame_id() !=
          guidance_.accepted_start.reference_frame_id() ||
      !evidence.has_position_error_m() ||
      !IsFinite(evidence.position_error_m()) ||
      evidence.position_error_m() < 0.0 ||
      !evidence.has_heading_error_rad() ||
      !IsFinite(evidence.heading_error_rad()) ||
      evidence.heading_error_rad() < 0.0 ||
      !evidence.has_absolute_speed_mps() ||
      !IsFinite(evidence.absolute_speed_mps()) ||
      evidence.absolute_speed_mps() < 0.0 ||
      !evidence.has_settled_duration_sec() ||
      !IsFinite(evidence.settled_duration_sec()) ||
      evidence.settled_duration_sec() <= 0.0 ||
      !evidence.has_executor_ownership() ||
      evidence.executor_ownership() != MOTION_EXECUTOR_OWNERSHIP_ACTIVE ||
      !evidence.has_executor_type() ||
      evidence.executor_type() == MOTION_EXECUTION_TYPE_UNKNOWN ||
      !evidence.has_safety_state() ||
      evidence.safety_state() != MOTION_EVIDENCE_SAFETY_NORMAL) {
    return reject("evidence is incomplete, stale in identity, or unsafe");
  }
  return true;
}

MissionAdmissionResult MissionSessionManager::Fail(const std::string& reason) {
  if (!HasActiveSession()) {
    return Reject(MissionAdmissionCode::kInvalidTransition,
                  "there is no active mission to fail");
  }
  return Transition(MISSION_SESSION_FAILED, reason);
}

MissionAdmissionResult MissionSessionManager::Transition(
    MissionSessionState state, const std::string& reason) {
  guidance_.state = state;
  reason_ = reason;
  return Accept(MissionAdmissionCode::kAccepted, reason_);
}

bool MissionSessionManager::IsExactIdentity(
    const MissionCommandIdentity& lhs,
    const MissionCommandIdentity& rhs) const {
  return lhs.producer_epoch() == rhs.producer_epoch() &&
         lhs.aggregate_id() == rhs.aggregate_id() &&
         lhs.command_id() == rhs.command_id() &&
         lhs.revision() == rhs.revision();
}

std::string MissionSessionManager::CommandKey(
    const MissionCommandIdentity& identity) const {
  return identity.producer_epoch() + "\n" + identity.aggregate_id() + "\n" +
         identity.command_id();
}

bool MissionSessionManager::IsTerminal(MissionSessionState state) const {
  return state == MISSION_SESSION_COMPLETED ||
         state == MISSION_SESSION_CANCELLED || state == MISSION_SESSION_FAILED;
}

bool MissionSessionManager::IsPhaseTransitionAllowed(
    MissionSessionPhase from, MissionSessionPhase to) const {
  if (to == MISSION_PHASE_UNKNOWN || from == to) {
    return false;
  }
  switch (from) {
    case MISSION_PHASE_UNKNOWN:
      return to == MISSION_PHASE_ROUTING || to == MISSION_PHASE_APPROACH;
    case MISSION_PHASE_ROUTING:
      return to == MISSION_PHASE_ENROUTE;
    case MISSION_PHASE_ENROUTE:
      return to == MISSION_PHASE_APPROACH;
    case MISSION_PHASE_APPROACH:
      return to == MISSION_PHASE_HANDOFF || to == MISSION_PHASE_SETTLING;
    case MISSION_PHASE_HANDOFF:
      return to == MISSION_PHASE_LOCAL_MANEUVER;
    case MISSION_PHASE_LOCAL_MANEUVER:
      return to == MISSION_PHASE_SETTLING;
    case MISSION_PHASE_SETTLING:
    default:
      return false;
  }
}

bool MissionSessionManager::HasActiveSession() const {
  return guidance_.identity.has_revision() && !IsTerminal(guidance_.state);
}

}  // namespace planning
}  // namespace apollo
