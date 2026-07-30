#include "modules/planning/common/motion_plan_builder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace apollo {
namespace planning {

namespace {

constexpr double kCommandValiditySec = 1.0;
constexpr double kEnvelopeMarginM = 1.0;
constexpr double kStoppedSpeedMps = 0.1;

bool HasPose(const localization::LocalizationEstimate& localization) {
  return localization.has_pose() && localization.pose().has_position() &&
         localization.pose().position().has_x() &&
         localization.pose().position().has_y() &&
         localization.pose().has_heading();
}

std::string ReferenceFrame(
    const localization::LocalizationEstimate& localization) {
  if (localization.has_header() && localization.header().has_frame_id() &&
      !localization.header().frame_id().empty()) {
    return localization.header().frame_id();
  }
  return "map";
}

}  // namespace

MotionPlanBuilder::MotionPlanBuilder(std::string producer_epoch)
    : producer_epoch_(std::move(producer_epoch)) {}

void MotionPlanBuilder::SetProducerEpoch(std::string producer_epoch) {
  producer_epoch_ = std::move(producer_epoch);
}

void MotionPlanBuilder::PopulateCommonCommand(
    const canbus::Chassis& chassis,
    const localization::LocalizationEstimate& localization,
    double now_sec, MotionExecutionCommand* command) const {
  const auto frame = ReferenceFrame(localization);
  const double snapshot_time =
      localization.has_measurement_time() ? localization.measurement_time()
                                          : now_sec;
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
  start->mutable_expected_position()->CopyFrom(
      localization.pose().position());
  start->set_expected_heading(localization.pose().heading());
  start->set_expected_gear(
      chassis.has_gear_location() ? chassis.gear_location()
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
    const PlanningSemanticSummary& semantics,
    const canbus::Chassis& chassis,
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
      return BuildIdleHold(chassis, localization, now_sec, command, reason);
    }
    if (reason != nullptr) {
      *reason = "trajectory motion requires at least two points";
    }
    return false;
  }

  PopulateCommonCommand(chassis, localization, now_sec, command);
  command->mutable_identity()->set_revision(next_revision_++);
  command->mutable_identity()->set_aggregate_id(
      state.mission_identity.has_aggregate_id()
          ? state.mission_identity.aggregate_id()
          : "planning-motion");
  command->add_required_capability(MOTION_CAPABILITY_TRAJECTORY_TRACKING);

  double max_speed = 0.1;
  double max_acceleration = 0.1;
  double max_deceleration = 0.1;
  double max_jerk = 0.1;
  double max_curvature = 0.01;
  double max_curvature_derivative = 0.01;
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

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
    if (previous_time >= 0.0 &&
        normalized_time <= previous_time + 1e-6) {
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
    max_speed = std::max(max_speed, std::abs(point.v()));
    max_acceleration = std::max(max_acceleration, point.a());
    max_deceleration = std::max(max_deceleration, -point.a());
    max_jerk = std::max(max_jerk, std::abs(point.da()));
    max_curvature =
        std::max(max_curvature, std::abs(point.path_point().kappa()));
    max_curvature_derivative =
        std::max(max_curvature_derivative,
                 std::abs(point.path_point().dkappa()));
    min_x = std::min(min_x, point.path_point().x());
    min_y = std::min(min_y, point.path_point().y());
    max_x = std::max(max_x, point.path_point().x());
    max_y = std::max(max_y, point.path_point().y());
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
  constraints->set_max_speed_mps(max_speed + 0.1);
  constraints->set_max_acceleration_mps2(max_acceleration + 0.1);
  constraints->set_max_deceleration_mps2(max_deceleration + 0.1);
  constraints->set_max_jerk_mps3(max_jerk + 0.1);
  constraints->set_max_abs_curvature_per_m(max_curvature + 0.01);
  constraints->set_max_abs_curvature_derivative_per_m2(
      max_curvature_derivative + 0.01);

  auto* envelope = command->mutable_spatial_envelope();
  auto add_corner = [envelope](double x, double y) {
    auto* point = envelope->add_boundary();
    point->set_x(x);
    point->set_y(y);
  };
  add_corner(min_x - kEnvelopeMarginM, min_y - kEnvelopeMarginM);
  add_corner(max_x + kEnvelopeMarginM, min_y - kEnvelopeMarginM);
  add_corner(max_x + kEnvelopeMarginM, max_y + kEnvelopeMarginM);
  add_corner(min_x - kEnvelopeMarginM, max_y + kEnvelopeMarginM);
  envelope->set_max_lateral_deviation_m(kEnvelopeMarginM);

  auto* completion = command->mutable_completion();
  completion->set_position_tolerance_m(
      semantics.has_position_tolerance
          ? std::max(0.01, semantics.terminal_position_tolerance_m)
          : 0.3);
  completion->set_heading_tolerance_rad(
      semantics.has_heading_tolerance
          ? std::max(0.01, semantics.terminal_heading_tolerance_rad)
          : 0.2);
  completion->set_speed_tolerance_mps(
      std::max(0.05, semantics.max_terminal_speed_mps));
  completion->set_settle_time_sec(0.2);
  completion->set_execution_timeout_sec(
      std::max(0.5, trajectory.total_path_time() + 1.0));
  return true;
}

bool MotionPlanBuilder::BuildStoppingCommand(
    const canbus::Chassis& chassis,
    const localization::LocalizationEstimate& localization,
    double now_sec, MotionExecutionCommand* command,
    std::string* reason) {
  if (command == nullptr || !HasPose(localization)) {
    if (reason != nullptr) {
      *reason = "stopping motion requires a valid vehicle snapshot";
    }
    return false;
  }
  PopulateCommonCommand(chassis, localization, now_sec, command);
  command->mutable_identity()->set_revision(next_revision_++);
  command->mutable_identity()->set_command_id("controlled-stop");
  command->add_required_capability(MOTION_CAPABILITY_TRAJECTORY_TRACKING);
  const double speed =
      std::abs(static_cast<double>(chassis.speed_mps()));
  const double deceleration = 1.0;
  const double stop_time = std::max(0.5, speed / deceleration);
  const double stop_distance =
      std::max(0.05, speed * stop_time * 0.5);
  const bool reverse =
      chassis.gear_location() == canbus::Chassis::GEAR_REVERSE;
  const double direction = reverse ? -1.0 : 1.0;
  const auto& pose = localization.pose();
  auto* payload = command->mutable_trajectory();
  payload->set_trajectory_id("controlled-stop-" +
                             std::to_string(command->identity().revision()));
  payload->set_gear(chassis.gear_location());
  for (int i = 0; i < 2; ++i) {
    const double ratio = static_cast<double>(i);
    auto* point = payload->add_point();
    point->mutable_path_point()->set_x(
        pose.position().x() +
        direction * std::cos(pose.heading()) * stop_distance * ratio);
    point->mutable_path_point()->set_y(
        pose.position().y() +
        direction * std::sin(pose.heading()) * stop_distance * ratio);
    point->mutable_path_point()->set_z(pose.position().z());
    point->mutable_path_point()->set_theta(pose.heading());
    point->mutable_path_point()->set_s(
        direction * stop_distance * ratio);
    point->mutable_path_point()->set_kappa(0.0);
    point->mutable_path_point()->set_dkappa(0.0);
    point->set_speed_mps(i == 0 ? direction * speed : 0.0);
    point->set_acceleration_mps2(
        i == 0 ? -direction * deceleration : 0.0);
    point->set_relative_time_sec(stop_time * ratio);
    auto* centerline =
        command->mutable_spatial_envelope()->add_reference_centerline();
    centerline->set_x(point->path_point().x());
    centerline->set_y(point->path_point().y());
  }
  auto* constraints = command->mutable_constraints();
  constraints->set_max_speed_mps(std::max(0.1, speed + 0.1));
  constraints->set_max_acceleration_mps2(1.0);
  constraints->set_max_deceleration_mps2(1.1);
  constraints->set_max_jerk_mps3(5.0);
  constraints->set_max_abs_curvature_per_m(0.1);
  constraints->set_max_abs_curvature_derivative_per_m2(0.1);
  auto* completion = command->mutable_completion();
  completion->set_position_tolerance_m(0.2);
  completion->set_heading_tolerance_rad(0.2);
  completion->set_speed_tolerance_mps(kStoppedSpeedMps);
  completion->set_settle_time_sec(0.2);
  completion->set_execution_timeout_sec(stop_time + 1.0);
  auto* envelope = command->mutable_spatial_envelope();
  const double min_x =
      std::min(payload->point(0).path_point().x(),
               payload->point(1).path_point().x()) -
      kEnvelopeMarginM;
  const double max_x =
      std::max(payload->point(0).path_point().x(),
               payload->point(1).path_point().x()) +
      kEnvelopeMarginM;
  const double min_y =
      std::min(payload->point(0).path_point().y(),
               payload->point(1).path_point().y()) -
      kEnvelopeMarginM;
  const double max_y =
      std::max(payload->point(0).path_point().y(),
               payload->point(1).path_point().y()) +
      kEnvelopeMarginM;
  for (const auto& corner :
       {std::pair<double, double>{min_x, min_y}, {max_x, min_y},
        {max_x, max_y}, {min_x, max_y}}) {
    auto* boundary = envelope->add_boundary();
    boundary->set_x(corner.first);
    boundary->set_y(corner.second);
  }
  envelope->set_max_lateral_deviation_m(kEnvelopeMarginM);
  return true;
}

bool MotionPlanBuilder::BuildIdleHold(
    const canbus::Chassis& chassis,
    const localization::LocalizationEstimate& localization,
    double now_sec, MotionExecutionCommand* command,
    std::string* reason) {
  if (command == nullptr || !HasPose(localization) ||
      std::abs(static_cast<double>(chassis.speed_mps())) >
          kStoppedSpeedMps) {
    if (reason != nullptr) {
      *reason = "idle hold requires a valid stopped vehicle snapshot";
    }
    return false;
  }
  PopulateCommonCommand(chassis, localization, now_sec, command);
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
  auto* envelope = command->mutable_spatial_envelope();
  const auto& position = localization.pose().position();
  for (const auto& offset :
       {std::pair<double, double>{-0.2, -0.2}, {0.2, -0.2},
        {0.2, 0.2}, {-0.2, 0.2}}) {
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
    const PlanningSemanticSummary& semantics,
    const canbus::Chassis& chassis,
    const localization::LocalizationEstimate& localization,
    const ADCTrajectory& trajectory, double now_sec) {
  MotionPlanBuildResult result;
  if (pending_identity_.has_revision() || pending_cancel_) {
    result.reason = "awaiting correlated Control acknowledgement";
    return result;
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
    if (stopping_requested_) {
      result.reason = "controlled stopping motion is active";
      return result;
    }
    MotionExecutionCommand stop;
    if (!BuildStoppingCommand(chassis, localization, now_sec, &stop,
                              &result.reason)) {
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
    return result;
  }

  MotionExecutionCommand command;
  const bool needs_idle_hold =
      (retiring ||
       state.mission_session_state == MISSION_SESSION_CANCELLED ||
       state.mission_session_state == MISSION_SESSION_COMPLETED) &&
      stopped && cancellation_fenced_;
  if (needs_idle_hold &&
      (!active_identity_.has_revision() ||
       active_scope_ == MOTION_SCOPE_PLANNING_IDLE_HOLD)) {
    if (!BuildIdleHold(chassis, localization, now_sec, &command,
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
  return result;
}

void MotionPlanBuilder::ObserveControlStatus(
    const MotionExecutionStatus& status, MotionDirectiveScope scope) {
  if (!status.has_identity()) {
    if (pending_cancel_ &&
        status.state() == MOTION_EXECUTION_CANCELLED) {
      active_identity_.Clear();
      active_parent_.Clear();
      active_scope_ = MOTION_SCOPE_UNKNOWN;
      pending_cancel_ = false;
      cancellation_fenced_ = true;
    }
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
