#include "modules/planning/common/motion_plan_builder.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

namespace {

constexpr double kCommandValiditySec = 1.0;
constexpr double kStoppedSpeedMps = 0.1;

bool HasPose(const localization::LocalizationEstimate& localization) {
  return localization.has_pose() && localization.pose().has_position() &&
         localization.pose().position().has_x() &&
         localization.pose().position().has_y() &&
         localization.pose().has_heading() &&
         std::isfinite(localization.pose().position().x()) &&
         std::isfinite(localization.pose().position().y()) &&
         std::isfinite(localization.pose().heading()) &&
         localization.has_header() && !localization.header().frame_id().empty();
}

std::string ReferenceFrame(
    const localization::LocalizationEstimate& localization) {
  if (localization.has_header() && localization.header().has_frame_id() &&
      !localization.header().frame_id().empty()) {
    return localization.header().frame_id();
  }
  return "";
}

}  // namespace

MotionPlanBuilder::MotionPlanBuilder(std::string producer_epoch)
    : producer_epoch_(std::move(producer_epoch)) {}

void MotionPlanBuilder::SetProducerEpoch(std::string producer_epoch) {
  producer_epoch_ = std::move(producer_epoch);
}

void MotionPlanBuilder::PopulateCommonCommand(
    const canbus::Chassis& chassis,
    const localization::LocalizationEstimate& localization, double now_sec,
    MotionExecutionCommand* command) const {
  const auto frame = ReferenceFrame(localization);
  const double snapshot_time = localization.has_measurement_time()
                                   ? localization.measurement_time()
                                   : localization.header().timestamp_sec();
  command->mutable_header()->set_timestamp_sec(snapshot_time);
  command->mutable_header()->set_module_name("planning");
  command->mutable_header()->set_frame_id(frame);
  command->mutable_identity()->set_producer_epoch(producer_epoch_);
  command->mutable_identity()->set_aggregate_id("planning-motion");
  command->mutable_identity()->set_command_id("active-motion");
  command->set_reference_frame_id(frame);
  command->set_effective_time_sec(snapshot_time);
  command->set_expiry_time_sec(now_sec + kCommandValiditySec);
  command->set_priority(0);
  command->set_preemptible(true);
  command->set_failure_fallback(MOTION_FALLBACK_HOLD);

  auto* start = command->mutable_start_condition();
  start->mutable_expected_position()->CopyFrom(localization.pose().position());
  start->set_expected_heading(localization.pose().heading());
  start->set_expected_gear(chassis.has_gear_location()
                               ? chassis.gear_location()
                               : canbus::Chassis::GEAR_DRIVE);
  start->set_max_position_error_m(0.5);
  start->set_max_heading_error_rad(0.25);
  start->set_max_abs_speed_mps(
      std::max(0.5, std::abs(static_cast<double>(chassis.speed_mps())) + 0.5));
  start->set_snapshot_time_sec(snapshot_time);
  start->set_reference_frame_id(frame);
}

bool MotionPlanBuilder::BuildCommand(
    const PlanningCoordinatorState& state,
    const PlanningSemanticSummary& semantics, const canbus::Chassis& chassis,
    const localization::LocalizationEstimate& localization,
    const ADCTrajectory& trajectory, double now_sec,
    MotionExecutionCommand* command, std::string* reason) {
  if (command == nullptr || !HasPose(localization)) {
    if (reason != nullptr) {
      *reason = "motion plan requires a finite localization pose";
    }
    return false;
  }
  if (trajectory.trajectory_point_size() < 2) {
    if (semantics.runtime_state == RUNTIME_HOLDING ||
        semantics.full_stop_reached) {
      return BuildIdleHold(state, chassis, localization, now_sec, command,
                           reason);
    }
    if (reason != nullptr) {
      *reason = "trajectory motion requires at least two points";
    }
    return false;
  }

  PopulateCommonCommand(chassis, localization, now_sec, command);
  if (!PopulateAuthority(state, command, reason)) {
    return false;
  }
  if (!trajectory.has_control_intent() ||
      !trajectory.control_intent().has_tracking_mode() ||
      !TrackingMode_IsValid(trajectory.control_intent().tracking_mode()) ||
      trajectory.control_intent().tracking_mode() ==
          TRACKING_MODE_UNKNOWN ||
      !trajectory.control_intent().has_longitudinal_intent() ||
      !LongitudinalIntent_IsValid(
          trajectory.control_intent().longitudinal_intent()) ||
      trajectory.control_intent().longitudinal_intent() ==
          LON_INTENT_UNKNOWN ||
      !trajectory.control_intent().has_lateral_intent() ||
      !LateralIntent_IsValid(trajectory.control_intent().lateral_intent()) ||
      trajectory.control_intent().lateral_intent() == LAT_INTENT_UNKNOWN ||
      !trajectory.control_intent().has_execution_channel() ||
      !ControlExecutionChannel_IsValid(
          trajectory.control_intent().execution_channel()) ||
      trajectory.control_intent().execution_channel() ==
          EXECUTION_CHANNEL_UNKNOWN ||
      !trajectory.control_intent().has_primitive_type() ||
      !ControlPrimitiveType_IsValid(
          trajectory.control_intent().primitive_type())) {
    if (reason != nullptr) {
      *reason = "Planning output lacks explicit Control execution semantics";
    }
    return false;
  }
  command->mutable_control_intent()->CopyFrom(trajectory.control_intent());
  if (trajectory.has_execution()) {
    command->mutable_execution()->CopyFrom(trajectory.execution());
  }
  command->mutable_identity()->set_revision(next_revision_++);
  command->mutable_identity()->set_aggregate_id(
      state.mission_identity.has_aggregate_id()
          ? state.mission_identity.aggregate_id()
          : "planning-motion");
  command->add_required_capability(MOTION_CAPABILITY_TRAJECTORY_TRACKING);

  if (authorized_envelope_.boundary_size() < 3) {
    if (reason != nullptr) {
      *reason = "moving motion requires a planning-owned spatial boundary";
    }
    return false;
  }
  command->mutable_spatial_envelope()->CopyFrom(authorized_envelope_);
  command->mutable_spatial_envelope()->clear_reference_centerline();

  auto* payload = command->mutable_trajectory();
  payload->set_trajectory_id(command->identity().aggregate_id() + "-" +
                             std::to_string(command->identity().revision()));
  payload->set_gear(trajectory.has_gear() ? trajectory.gear()
                                          : canbus::Chassis::GEAR_DRIVE);
  double first_time = -1.0;
  double previous_time = -1.0;
  for (const auto& point : trajectory.trajectory_point()) {
    if (!point.has_path_point()) {
      if (reason != nullptr) {
        *reason = "legacy trajectory point is missing geometry";
      }
      return false;
    }
    if (point.relative_time() < 0.0) {
      continue;
    }
    if (first_time < 0.0) {
      first_time = point.relative_time();
    }
    const double normalized_time = point.relative_time() - first_time;
    if (previous_time >= 0.0 && normalized_time <= previous_time + 1e-6) {
      continue;
    }
    auto* output = payload->add_point();
    output->mutable_path_point()->CopyFrom(point.path_point());
    output->set_speed_mps(point.v());
    output->set_acceleration_mps2(point.a());
    output->set_relative_time_sec(normalized_time);
    if (point.has_da()) {
      output->set_jerk_mps3(point.da());
    }
    auto* centerline =
        command->mutable_spatial_envelope()->add_reference_centerline();
    centerline->set_x(point.path_point().x());
    centerline->set_y(point.path_point().y());
    if (point.path_point().has_z()) {
      centerline->set_z(point.path_point().z());
    }
    previous_time = normalized_time;
  }
  if (payload->point_size() < 2) {
    if (reason != nullptr) {
      *reason = "trajectory has fewer than two future ordered points";
    }
    return false;
  }

  auto* constraints = command->mutable_constraints();
  constraints->set_max_speed_mps(FLAGS_speed_upper_bound);
  constraints->set_max_acceleration_mps2(
      FLAGS_longitudinal_acceleration_upper_bound);
  constraints->set_max_deceleration_mps2(
      -FLAGS_longitudinal_acceleration_lower_bound);
  constraints->set_max_jerk_mps3(
      std::min(FLAGS_longitudinal_jerk_upper_bound,
               -FLAGS_longitudinal_jerk_lower_bound));
  constraints->set_max_abs_curvature_per_m(FLAGS_kappa_bound);
  constraints->set_max_abs_curvature_derivative_per_m2(
      FLAGS_motion_curvature_derivative_bound);

  auto* completion = command->mutable_completion();
  completion->set_position_tolerance_m(
      semantics.has_position_tolerance ? semantics.terminal_position_tolerance_m
                                       : 0.3);
  completion->set_heading_tolerance_rad(
      semantics.has_heading_tolerance ? semantics.terminal_heading_tolerance_rad
                                      : 0.2);
  completion->set_speed_tolerance_mps(semantics.max_terminal_speed_mps);
  completion->set_settle_time_sec(0.2);
  completion->set_execution_timeout_sec(
      std::max(0.5, trajectory.total_path_time() + 1.0));
  return true;
}

bool MotionPlanBuilder::PopulateAuthority(
    const PlanningCoordinatorState& state, MotionExecutionCommand* command,
    std::string* reason) const {
  if (command == nullptr || !state.mission_identity.has_revision() ||
      state.mission_identity.revision() == 0) {
    if (reason != nullptr) {
      *reason = "motion requires a live accepted Mission authority";
    }
    return false;
  }
  command->set_authority_generation(state.mission_identity.revision());
  command->mutable_authorized_mission_identity()->CopyFrom(
      state.mission_identity);
  return true;
}

bool MotionPlanBuilder::BuildStoppingCommand(
    const PlanningCoordinatorState& state,
    const canbus::Chassis& chassis,
    const localization::LocalizationEstimate& localization, double now_sec,
    const ADCTrajectory& trajectory, MotionExecutionCommand* command,
    std::string* reason) {
  PlanningSemanticSummary semantics;
  semantics.max_terminal_speed_mps = kStoppedSpeedMps;
  if (!BuildCommand(state, semantics, chassis, localization, trajectory,
                    now_sec, command, reason)) {
    return false;
  }
  command->mutable_identity()->set_command_id("controlled-stop");
  if (!trajectory.has_gear() || trajectory.gear() != chassis.gear_location()) {
    if (reason != nullptr) {
      *reason = "controlled stop cannot change the current travel direction";
    }
    return false;
  }
  // Keep the planner-validated lateral path; braking is an explicit execution
  // requirement, not a newly invented straight-line trajectory through space.
  auto* intent = command->mutable_control_intent();
  intent->set_tracking_mode(TRACKING_MODE_TRAJECTORY);
  intent->set_execution_channel(EXECUTION_CHANNEL_TRAJECTORY);
  intent->set_primitive_type(CONTROL_PRIMITIVE_NONE);
  intent->set_longitudinal_intent(LON_INTENT_MRM_STOP);
  intent->set_require_full_stop(true);
  return true;
}

bool MotionPlanBuilder::BuildIdleHold(
    const PlanningCoordinatorState& state,
    const canbus::Chassis& chassis,
    const localization::LocalizationEstimate& localization, double now_sec,
    MotionExecutionCommand* command, std::string* reason) {
  if (command == nullptr || !HasPose(localization) ||
      std::abs(static_cast<double>(chassis.speed_mps())) > kStoppedSpeedMps) {
    if (reason != nullptr) {
      *reason = "idle hold requires a valid stopped vehicle snapshot";
    }
    return false;
  }
  PopulateCommonCommand(chassis, localization, now_sec, command);
  if (!PopulateAuthority(state, command, reason)) {
    return false;
  }
  command->mutable_identity()->set_revision(next_revision_++);
  command->mutable_identity()->set_aggregate_id("planning-idle-hold");
  command->mutable_identity()->set_command_id("idle-hold");
  command->add_required_capability(MOTION_CAPABILITY_STANDSTILL_HOLD);
  auto* constraints = command->mutable_constraints();
  constraints->set_max_speed_mps(0.0);
  constraints->set_max_acceleration_mps2(1.0);
  constraints->set_max_deceleration_mps2(1.0);
  constraints->set_max_jerk_mps3(1.0);
  auto* completion = command->mutable_completion();
  completion->set_position_tolerance_m(0.1);
  completion->set_heading_tolerance_rad(0.1);
  completion->set_speed_tolerance_mps(kStoppedSpeedMps);
  completion->set_settle_time_sec(0.2);
  completion->set_execution_timeout_sec(2.0);
  auto* intent = command->mutable_control_intent();
  intent->set_tracking_mode(TRACKING_MODE_STANDSTILL_HOLD);
  intent->set_longitudinal_intent(LON_INTENT_HOLD_STOP);
  intent->set_lateral_intent(LAT_INTENT_MINIMIZE_STEER);
  intent->set_execution_channel(EXECUTION_CHANNEL_PRIMITIVE);
  intent->set_primitive_type(CONTROL_PRIMITIVE_STANDSTILL_HOLD);
  intent->set_require_full_stop(true);
  auto* envelope = command->mutable_spatial_envelope();
  const auto& position = localization.pose().position();
  for (const auto& offset : {std::pair<double, double>{-0.2, -0.2},
                             {0.2, -0.2},
                             {0.2, 0.2},
                             {-0.2, 0.2}}) {
    auto* corner = envelope->add_boundary();
    corner->set_x(position.x() + offset.first);
    corner->set_y(position.y() + offset.second);
  }
  auto* primitive = command->mutable_primitive();
  primitive->set_type(MOTION_PRIMITIVE_STANDSTILL_HOLD);
  primitive->mutable_standstill_hold()->set_reauthorization_period_sec(1.0);
  return true;
}

MotionPlanBuildResult MotionPlanBuilder::Build(
    const PlanningCoordinatorState& state,
    const PlanningSemanticSummary& semantics, const canbus::Chassis& chassis,
    const localization::LocalizationEstimate& localization,
    const ADCTrajectory& trajectory, double now_sec) {
  MotionPlanBuildResult result;
  result.directive.mutable_header()->set_module_name("planning");
  result.directive.mutable_header()->set_timestamp_sec(now_sec);
  if (state.mission_session_state == MISSION_SESSION_FAILED) {
    result.reason = "failed Mission requires a new Mission directive";
    return result;
  }
  if (!std::isfinite(now_sec) || !HasPose(localization) ||
      !chassis.has_speed_mps() || !std::isfinite(chassis.speed_mps()) ||
      !chassis.has_gear_location() ||
      (!localization.has_measurement_time() &&
       !localization.header().has_timestamp_sec())) {
    result.reason =
        "motion requires explicit finite pose, frame, time, speed and gear";
    return result;
  }
  if (pending_identity_.has_revision() || pending_cancel_) {
    const double pending_expiry =
        pending_directive_.has_execute()
            ? pending_directive_.execute().command().expiry_time_sec()
            : pending_directive_.replace().command().expiry_time_sec();
    if (pending_cancel_ || now_sec <= pending_expiry) {
      result.reason = "awaiting correlated Control acknowledgement";
      if (now_sec - pending_since_sec_ >= 0.25) {
        result.has_directive = true;
        result.directive.CopyFrom(pending_directive_);
        pending_since_sec_ = now_sec;
      }
      return result;
    }
    pending_identity_.Clear();
    pending_directive_.Clear();
    stopping_requested_ = false;
  }
  const bool cancelling =
      state.mission_session_state == MISSION_SESSION_CANCELLING;
  const bool completing =
      state.mission_session_state == MISSION_SESSION_COMPLETING;
  const bool retiring = cancelling || completing;
  if (!retiring && cancellation_fenced_ &&
      (!fenced_parent_.has_revision() ||
       fenced_parent_.SerializeAsString() !=
           state.mission_identity.SerializeAsString())) {
    cancellation_fenced_ = false;
    fenced_parent_.Clear();
  }
  const bool stopped =
      std::abs(static_cast<double>(chassis.speed_mps())) <= kStoppedSpeedMps;

  if (retiring && !stopped) {
    MotionExecutionCommand stop;
    if (!BuildStoppingCommand(state, chassis, localization, now_sec, trajectory,
                              &stop, &result.reason)) {
      return result;
    }
    result.has_directive = true;
    result.directive.set_scope(MOTION_SCOPE_MISSION_DESCENDANT);
    result.directive.mutable_parent_mission_identity()->CopyFrom(
        state.mission_identity);
    if (active_identity_.has_revision()) {
      result.directive.mutable_replace()
          ->mutable_expected_active_identity()
          ->CopyFrom(active_identity_);
      result.directive.mutable_replace()->mutable_command()->CopyFrom(stop);
    } else {
      result.directive.mutable_execute()->mutable_command()->CopyFrom(stop);
    }
    pending_identity_.CopyFrom(stop.identity());
    pending_directive_.CopyFrom(result.directive);
    pending_since_sec_ = now_sec;
    stopping_requested_ = true;
    return result;
  }

  if (retiring && stopped && active_identity_.has_revision() &&
      !cancellation_fenced_) {
    result.has_directive = true;
    result.directive.set_scope(MOTION_SCOPE_MISSION_DESCENDANT);
    result.directive.mutable_parent_mission_identity()->CopyFrom(
        state.mission_identity);
    result.directive.mutable_cancel()->mutable_target_identity()->CopyFrom(
        active_identity_);
    result.directive.mutable_cancel()->set_fence_parent_mission(true);
    result.directive.mutable_cancel()->set_reason(
        "Mission cancelled after controlled stop");
    pending_cancel_ = true;
    pending_directive_.CopyFrom(result.directive);
    pending_since_sec_ = now_sec;
    return result;
  }

  MotionExecutionCommand command;
  const bool needs_idle_hold =
      (retiring || state.mission_session_state == MISSION_SESSION_CANCELLED ||
       state.mission_session_state == MISSION_SESSION_COMPLETED) &&
      stopped && cancellation_fenced_;
  if (needs_idle_hold && (!active_identity_.has_revision() ||
                          active_scope_ == MOTION_SCOPE_PLANNING_IDLE_HOLD)) {
    if (!BuildIdleHold(state, chassis, localization, now_sec, &command,
                       &result.reason)) {
      return result;
    }
    result.directive.set_scope(MOTION_SCOPE_PLANNING_IDLE_HOLD);
  } else {
    if (!state.mission_identity.has_revision()) {
      result.reason = "Mission-scoped motion requires active Mission identity";
      return result;
    }
    if (!BuildCommand(state, semantics, chassis, localization, trajectory,
                      now_sec, &command, &result.reason)) {
      return result;
    }
    result.directive.set_scope(MOTION_SCOPE_MISSION_DESCENDANT);
    result.directive.mutable_parent_mission_identity()->CopyFrom(
        state.mission_identity);
  }

  result.has_directive = true;
  if (active_identity_.has_revision()) {
    result.directive.mutable_replace()
        ->mutable_expected_active_identity()
        ->CopyFrom(active_identity_);
    result.directive.mutable_replace()->mutable_command()->CopyFrom(command);
  } else {
    result.directive.mutable_execute()->mutable_command()->CopyFrom(command);
  }
  pending_identity_.CopyFrom(command.identity());
  pending_directive_.CopyFrom(result.directive);
  pending_since_sec_ = now_sec;
  return result;
}

bool MotionPlanBuilder::IsCorrelatedStatus(
    const MotionExecutionStatus& status) const {
  return status.has_identity() &&
         ((pending_identity_.has_revision() &&
           pending_identity_.SerializeAsString() ==
               status.identity().SerializeAsString()) ||
          (active_identity_.has_revision() &&
           active_identity_.SerializeAsString() ==
               status.identity().SerializeAsString()));
}

void MotionPlanBuilder::ObserveControlStatus(
    const MotionExecutionStatus& status, MotionDirectiveScope scope) {
  if (!IsCorrelatedStatus(status)) {
    return;
  }
  if (pending_identity_.has_revision() &&
      pending_identity_.SerializeAsString() ==
          status.identity().SerializeAsString()) {
    if (status.state() == MOTION_EXECUTION_REJECTED ||
        status.state() == MOTION_EXECUTION_FAILED ||
        status.state() == MOTION_EXECUTION_TIMED_OUT) {
      pending_identity_.Clear();
      stopping_requested_ = false;
      return;
    }
    active_identity_.CopyFrom(status.identity());
    pending_identity_.Clear();
    active_scope_ = scope;
    if (status.has_parent_mission_identity()) {
      active_parent_.CopyFrom(status.parent_mission_identity());
    } else {
      active_parent_.Clear();
    }
  }
  if (status.state() == MOTION_EXECUTION_CANCELLED ||
      status.state() == MOTION_EXECUTION_FAILED ||
      status.state() == MOTION_EXECUTION_TIMED_OUT ||
      status.state() == MOTION_EXECUTION_SUCCEEDED) {
    if (active_identity_.SerializeAsString() ==
        status.identity().SerializeAsString()) {
      if (pending_cancel_ && status.has_parent_mission_identity()) {
        fenced_parent_.CopyFrom(status.parent_mission_identity());
      }
      active_identity_.Clear();
      active_parent_.Clear();
      active_scope_ = MOTION_SCOPE_UNKNOWN;
      pending_cancel_ = false;
      cancellation_fenced_ = true;
      stopping_requested_ = false;
    }
    return;
  }
  active_identity_.CopyFrom(status.identity());
  active_scope_ = scope;
}

}  // namespace planning
}  // namespace apollo
