#include "modules/control/common/motion_execution_validator.h"

#include <cmath>
#include <utility>

namespace apollo {
namespace control {

namespace {

constexpr double kMaxCommandValiditySec = 60.0;

bool IsFinite(double value) { return std::isfinite(value); }

bool IsPositiveFinite(double value) {
  return IsFinite(value) && value > 0.0;
}

bool IsNonNegativeFinite(double value) {
  return IsFinite(value) && value >= 0.0;
}

bool HasFinitePoint(const common::PointENU& point) {
  return point.has_x() && point.has_y() && IsFinite(point.x()) &&
         IsFinite(point.y()) && (!point.has_z() || IsFinite(point.z()));
}

double SquaredDistance(const common::PointENU& lhs,
                       const common::PointENU& rhs) {
  const double dx = lhs.x() - rhs.x();
  const double dy = lhs.y() - rhs.y();
  return dx * dx + dy * dy;
}

double CenterlineLength(const planning::MotionSpatialEnvelope& envelope) {
  double length = 0.0;
  for (int i = 1; i < envelope.reference_centerline_size(); ++i) {
    length += std::sqrt(SquaredDistance(
        envelope.reference_centerline(i - 1),
        envelope.reference_centerline(i)));
  }
  return length;
}

bool IsInsideBoundary(const common::PointENU& point,
                      const planning::MotionSpatialEnvelope& envelope) {
  bool inside = false;
  for (int i = 0, j = envelope.boundary_size() - 1;
       i < envelope.boundary_size(); j = i++) {
    const auto& current = envelope.boundary(i);
    const auto& previous = envelope.boundary(j);
    const bool crosses =
        (current.y() > point.y()) != (previous.y() > point.y());
    if (crosses) {
      const double intersection_x =
          (previous.x() - current.x()) * (point.y() - current.y()) /
              (previous.y() - current.y()) +
          current.x();
      if (point.x() < intersection_x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

bool IsInsideBoundary(const common::PathPoint& point,
                      const planning::MotionSpatialEnvelope& envelope) {
  common::PointENU position;
  position.set_x(point.x());
  position.set_y(point.y());
  return IsInsideBoundary(position, envelope);
}

bool HasValidSpatialEnvelope(
    const planning::MotionExecutionCommand& command) {
  if (!command.has_spatial_envelope()) {
    return false;
  }
  const auto& envelope = command.spatial_envelope();
  if (envelope.boundary_size() < 3) {
    return false;
  }
  double twice_area = 0.0;
  for (const auto& point : envelope.boundary()) {
    if (!HasFinitePoint(point)) {
      return false;
    }
  }
  for (int i = 0; i < envelope.boundary_size(); ++i) {
    const auto& current = envelope.boundary(i);
    const auto& next =
        envelope.boundary((i + 1) % envelope.boundary_size());
    twice_area += current.x() * next.y() - next.x() * current.y();
  }
  if (!IsFinite(twice_area) || std::abs(twice_area) <= 1e-6) {
    return false;
  }
  if (envelope.has_max_lateral_deviation_m() &&
      !IsPositiveFinite(envelope.max_lateral_deviation_m())) {
    return false;
  }
  for (const auto& point : envelope.reference_centerline()) {
    if (!HasFinitePoint(point) || !IsInsideBoundary(point, envelope)) {
      return false;
    }
  }
  return true;
}

bool HasSettledPoseCompletion(
    const planning::MotionCompletionCondition& completion) {
  return completion.has_position_tolerance_m() &&
         IsPositiveFinite(completion.position_tolerance_m()) &&
         completion.has_heading_tolerance_rad() &&
         IsPositiveFinite(completion.heading_tolerance_rad()) &&
         completion.has_speed_tolerance_mps() &&
         IsNonNegativeFinite(completion.speed_tolerance_mps()) &&
         completion.has_settle_time_sec() &&
         IsPositiveFinite(completion.settle_time_sec());
}

planning::MotionCapability CapabilityForPrimitive(
    planning::MotionPrimitiveType type) {
  switch (type) {
    case planning::MOTION_PRIMITIVE_STANDSTILL_HOLD:
      return planning::MOTION_CAPABILITY_STANDSTILL_HOLD;
    case planning::MOTION_PRIMITIVE_POSE_SERVO:
      return planning::MOTION_CAPABILITY_POSE_SERVO;
    case planning::MOTION_PRIMITIVE_CORRIDOR_SERVO:
      return planning::MOTION_CAPABILITY_CORRIDOR_SERVO;
    case planning::MOTION_PRIMITIVE_ROTATE_IN_PLACE:
      return planning::MOTION_CAPABILITY_ROTATE_IN_PLACE;
    case planning::MOTION_PRIMITIVE_UNKNOWN:
    default:
      return planning::MOTION_CAPABILITY_UNKNOWN;
  }
}

}  // namespace

MotionExecutionValidator::MotionExecutionValidator(
    MotionExecutionCapabilities capabilities)
    : capabilities_(std::move(capabilities)) {}

MotionValidationResult MotionExecutionValidator::Reject(
    planning::MotionCommandRejectReason reject_reason,
    const std::string& reason) const {
  MotionValidationResult result;
  result.reject_reason = reject_reason;
  result.reason = reason;
  return result;
}

MotionValidationResult MotionExecutionValidator::Validate(
    const planning::MotionExecutionCommand& command, double now_sec) const {
  if (!IsFinite(now_sec) || !command.has_header() ||
      !command.header().has_timestamp_sec() ||
      !IsFinite(command.header().timestamp_sec()) ||
      !command.header().has_module_name() ||
      command.header().module_name().empty() ||
      !command.has_identity() ||
      !command.identity().has_producer_epoch() ||
      command.identity().producer_epoch().empty() ||
      !command.identity().has_aggregate_id() ||
      command.identity().aggregate_id().empty() ||
      !command.identity().has_command_id() ||
      command.identity().command_id().empty() ||
      !command.identity().has_revision() || command.identity().revision() == 0) {
    return Reject(planning::MOTION_REJECT_MISSING_CONTEXT,
                  "missing command identity, revision, or source timestamp");
  }
  if (!command.has_reference_frame_id() ||
      command.reference_frame_id().empty() ||
      (command.header().has_frame_id() &&
       command.header().frame_id() != command.reference_frame_id())) {
    return Reject(planning::MOTION_REJECT_INVALID_FRAME,
                  "missing or conflicting command reference frame");
  }
  const auto start_condition_result = ValidateStartCondition(command);
  if (!start_condition_result.accepted) {
    return start_condition_result;
  }
  if (!command.has_effective_time_sec() || !command.has_expiry_time_sec() ||
      !IsFinite(command.effective_time_sec()) ||
      !IsFinite(command.expiry_time_sec()) ||
      command.expiry_time_sec() <= command.effective_time_sec() ||
      command.expiry_time_sec() - command.effective_time_sec() >
          kMaxCommandValiditySec) {
    return Reject(planning::MOTION_REJECT_INVALID_TIME,
                  "invalid command effective/expiry interval");
  }
  if (command.header().timestamp_sec() > command.effective_time_sec()) {
    return Reject(planning::MOTION_REJECT_INVALID_TIME,
                  "source timestamp is later than effective time");
  }
  if (now_sec < command.effective_time_sec()) {
    return Reject(planning::MOTION_REJECT_INVALID_TIME,
                  "command is not effective yet");
  }
  if (now_sec > command.expiry_time_sec()) {
    return Reject(planning::MOTION_REJECT_STALE, "command has expired");
  }
  if (!command.has_completion() ||
      !command.completion().has_execution_timeout_sec() ||
      !IsPositiveFinite(command.completion().execution_timeout_sec()) ||
      (command.completion().has_settle_time_sec() &&
       !IsNonNegativeFinite(command.completion().settle_time_sec())) ||
      (command.completion().has_position_tolerance_m() &&
       !IsNonNegativeFinite(command.completion().position_tolerance_m())) ||
      (command.completion().has_heading_tolerance_rad() &&
       !IsNonNegativeFinite(command.completion().heading_tolerance_rad())) ||
      (command.completion().has_speed_tolerance_mps() &&
       !IsNonNegativeFinite(command.completion().speed_tolerance_mps()))) {
    return Reject(planning::MOTION_REJECT_INVALID_COMPLETION,
                  "missing or invalid completion timeout");
  }
  for (const auto capability_value : command.required_capability()) {
    const auto capability =
        static_cast<planning::MotionCapability>(capability_value);
    if (capability == planning::MOTION_CAPABILITY_UNKNOWN ||
        !capabilities_.Supports(capability)) {
      return Reject(planning::MOTION_REJECT_UNSUPPORTED_CAPABILITY,
                    "required motion capability is unavailable");
    }
  }

  const auto constraints_result = ValidateConstraints(command);
  if (!constraints_result.accepted) {
    return constraints_result;
  }

  if (command.payload_case() ==
      planning::MotionExecutionCommand::kTrajectory) {
    return ValidateTrajectory(command);
  }
  if (command.payload_case() ==
      planning::MotionExecutionCommand::kPrimitive) {
    return ValidatePrimitive(command);
  }
  return Reject(planning::MOTION_REJECT_INVALID_PAYLOAD,
                "motion command must contain exactly one payload");
}

MotionValidationResult MotionExecutionValidator::ValidateStartCondition(
    const planning::MotionExecutionCommand& command) const {
  if (!command.has_start_condition()) {
    return Reject(planning::MOTION_REJECT_INVALID_START_CONDITION,
                  "motion start condition is required");
  }
  const auto& start = command.start_condition();
  if (!start.has_expected_position() ||
      !HasFinitePoint(start.expected_position()) ||
      !start.has_expected_heading() ||
      !IsFinite(start.expected_heading()) ||
      !start.has_expected_gear() ||
      start.expected_gear() == canbus::Chassis::GEAR_NONE ||
      !start.has_max_position_error_m() ||
      !IsNonNegativeFinite(start.max_position_error_m()) ||
      !start.has_max_heading_error_rad() ||
      !IsNonNegativeFinite(start.max_heading_error_rad()) ||
      !start.has_max_abs_speed_mps() ||
      !IsNonNegativeFinite(start.max_abs_speed_mps()) ||
      !start.has_snapshot_time_sec() ||
      !IsFinite(start.snapshot_time_sec()) ||
      start.snapshot_time_sec() > command.effective_time_sec() ||
      !start.has_reference_frame_id() ||
      start.reference_frame_id().empty() ||
      start.reference_frame_id() != command.reference_frame_id()) {
    return Reject(planning::MOTION_REJECT_INVALID_START_CONDITION,
                  "motion start pose, tolerances, gear, speed, time, and frame are required");
  }
  MotionValidationResult result;
  result.accepted = true;
  return result;
}

MotionValidationResult MotionExecutionValidator::ValidateConstraints(
    const planning::MotionExecutionCommand& command) const {
  if (!command.has_constraints()) {
    return Reject(planning::MOTION_REJECT_INVALID_CONSTRAINTS,
                  "motion constraints are required");
  }
  const auto& constraints = command.constraints();
  if (!constraints.has_max_speed_mps() ||
      !IsNonNegativeFinite(constraints.max_speed_mps()) ||
      !constraints.has_max_acceleration_mps2() ||
      !IsPositiveFinite(constraints.max_acceleration_mps2()) ||
      !constraints.has_max_deceleration_mps2() ||
      !IsPositiveFinite(constraints.max_deceleration_mps2()) ||
      !constraints.has_max_jerk_mps3() ||
      !IsPositiveFinite(constraints.max_jerk_mps3())) {
    return Reject(planning::MOTION_REJECT_INVALID_CONSTRAINTS,
                  "motion dynamic constraints are incomplete or invalid");
  }
  MotionValidationResult result;
  result.accepted = true;
  return result;
}

MotionValidationResult MotionExecutionValidator::ValidateTrajectory(
    const planning::MotionExecutionCommand& command) const {
  if (!capabilities_.Supports(
          planning::MOTION_CAPABILITY_TRAJECTORY_TRACKING)) {
    return Reject(planning::MOTION_REJECT_UNSUPPORTED_CAPABILITY,
                  "trajectory tracking capability is unavailable");
  }
  const auto& trajectory = command.trajectory();
  if (!HasValidSpatialEnvelope(command) ||
      !HasSettledPoseCompletion(command.completion())) {
    return Reject(planning::MOTION_REJECT_INVALID_COMPLETION,
                  "trajectory requires a hard envelope and settled completion");
  }
  if (!trajectory.has_trajectory_id() || trajectory.trajectory_id().empty() ||
      trajectory.point_size() < 2) {
    return Reject(planning::MOTION_REJECT_INVALID_PAYLOAD,
                  "trajectory payload requires identity and at least two points");
  }
  if (!trajectory.has_gear() ||
      (trajectory.gear() != canbus::Chassis::GEAR_DRIVE &&
       trajectory.gear() != canbus::Chassis::GEAR_REVERSE &&
       trajectory.gear() != canbus::Chassis::GEAR_LOW)) {
    return Reject(planning::MOTION_REJECT_INVALID_PAYLOAD,
                  "trajectory requires a valid directional gear");
  }
  const bool reverse = trajectory.gear() == canbus::Chassis::GEAR_REVERSE;
  double previous_time = -1.0;
  double previous_s = 0.0;
  double previous_acceleration = 0.0;
  bool has_previous_point = false;
  const auto& constraints = command.constraints();
  if (!constraints.has_max_abs_curvature_per_m() ||
      !IsPositiveFinite(constraints.max_abs_curvature_per_m()) ||
      !constraints.has_max_abs_curvature_derivative_per_m2() ||
      !IsPositiveFinite(
          constraints.max_abs_curvature_derivative_per_m2())) {
    return Reject(planning::MOTION_REJECT_INVALID_CONSTRAINTS,
                  "trajectory curvature constraints are required");
  }
  for (const auto& point : trajectory.point()) {
    const double delta_time =
        previous_time >= 0.0
            ? point.relative_time_sec() - previous_time
            : 0.0;
    if (!point.has_path_point() || !point.path_point().has_x() ||
        !point.path_point().has_y() || !point.path_point().has_theta() ||
        !point.path_point().has_s() || !point.path_point().has_kappa() ||
        !point.path_point().has_dkappa() || !point.has_speed_mps() ||
        !point.has_acceleration_mps2() ||
        !point.has_relative_time_sec() ||
        !IsFinite(point.path_point().x()) ||
        !IsFinite(point.path_point().y()) ||
        (!point.path_point().has_z() ? false
                                    : !IsFinite(point.path_point().z())) ||
        !IsFinite(point.path_point().theta()) ||
        !IsFinite(point.path_point().s()) ||
        !IsFinite(point.path_point().kappa()) ||
        !IsFinite(point.path_point().dkappa()) ||
        (point.path_point().has_ddkappa() &&
         !IsFinite(point.path_point().ddkappa())) ||
        (point.path_point().has_x_derivative() &&
         !IsFinite(point.path_point().x_derivative())) ||
        (point.path_point().has_y_derivative() &&
         !IsFinite(point.path_point().y_derivative())) ||
        !IsFinite(point.speed_mps()) ||
        !IsFinite(point.acceleration_mps2()) ||
        !IsFinite(point.relative_time_sec()) ||
        point.relative_time_sec() < 0.0 ||
        (previous_time >= 0.0 &&
         point.relative_time_sec() <= previous_time) ||
        (has_previous_point &&
         ((!reverse && point.path_point().s() < previous_s) ||
          (reverse && point.path_point().s() > previous_s))) ||
        std::abs(point.path_point().kappa()) >
            command.constraints().max_abs_curvature_per_m() ||
        std::abs(point.path_point().dkappa()) >
            command.constraints()
                .max_abs_curvature_derivative_per_m2() ||
        std::abs(point.speed_mps()) > constraints.max_speed_mps() ||
        point.acceleration_mps2() > constraints.max_acceleration_mps2() ||
        point.acceleration_mps2() < -constraints.max_deceleration_mps2() ||
        (point.has_jerk_mps3() &&
         (!IsFinite(point.jerk_mps3()) ||
          std::abs(point.jerk_mps3()) > constraints.max_jerk_mps3())) ||
        (has_previous_point &&
         std::abs(point.acceleration_mps2() - previous_acceleration) /
                 delta_time >
             constraints.max_jerk_mps3()) ||
        !IsInsideBoundary(point.path_point(),
                          command.spatial_envelope())) {
      return Reject(planning::MOTION_REJECT_INVALID_PAYLOAD,
                    "trajectory violates ordering, finiteness, or motion bounds");
    }
    previous_time = point.relative_time_sec();
    previous_s = point.path_point().s();
    previous_acceleration = point.acceleration_mps2();
    has_previous_point = true;
  }
  MotionValidationResult result;
  result.accepted = true;
  result.execution_type = planning::MOTION_EXECUTION_TYPE_TRAJECTORY;
  return result;
}

MotionValidationResult MotionExecutionValidator::ValidatePrimitive(
    const planning::MotionExecutionCommand& command) const {
  const auto& primitive = command.primitive();
  if (!primitive.has_type() ||
      primitive.type() == planning::MOTION_PRIMITIVE_UNKNOWN) {
    return Reject(planning::MOTION_REJECT_INVALID_PAYLOAD,
                  "primitive type is required");
  }
  const auto required_capability = CapabilityForPrimitive(primitive.type());
  if (required_capability == planning::MOTION_CAPABILITY_UNKNOWN ||
      !capabilities_.Supports(required_capability)) {
    return Reject(planning::MOTION_REJECT_UNSUPPORTED_CAPABILITY,
                  "primitive is not supported by this vehicle");
  }
  if (primitive.type() != planning::MOTION_PRIMITIVE_STANDSTILL_HOLD &&
      !HasValidSpatialEnvelope(command)) {
    return Reject(planning::MOTION_REJECT_INVALID_CONSTRAINTS,
                  "moving primitive requires a bounded spatial envelope");
  }
  const auto& constraints = command.constraints();
  const auto& completion = command.completion();
  switch (primitive.type()) {
    case planning::MOTION_PRIMITIVE_STANDSTILL_HOLD:
      if (!primitive.has_standstill_hold() ||
          !HasValidSpatialEnvelope(command) ||
          constraints.max_speed_mps() != 0.0 ||
          !HasSettledPoseCompletion(completion) ||
          !primitive.standstill_hold().has_reauthorization_period_sec() ||
          !IsPositiveFinite(
              primitive.standstill_hold().reauthorization_period_sec()) ||
          primitive.standstill_hold().reauthorization_period_sec() >
              completion.execution_timeout_sec()) {
        return Reject(planning::MOTION_REJECT_INVALID_PAYLOAD,
                      "standstill hold requires zero speed and bounded reauthorization");
      }
      break;
    case planning::MOTION_PRIMITIVE_POSE_SERVO:
      if (!primitive.has_pose_servo() ||
          !primitive.pose_servo().has_target_position() ||
          !HasFinitePoint(primitive.pose_servo().target_position()) ||
          !IsInsideBoundary(primitive.pose_servo().target_position(),
                            command.spatial_envelope()) ||
          !primitive.pose_servo().has_target_heading() ||
          !IsFinite(primitive.pose_servo().target_heading()) ||
          !primitive.pose_servo().has_direction() ||
          primitive.pose_servo().direction() ==
              planning::MOTION_DIRECTION_UNKNOWN ||
          !constraints.has_max_distance_m() ||
          !IsPositiveFinite(constraints.max_distance_m()) ||
          constraints.max_speed_mps() <= 0.0 ||
          !completion.has_position_tolerance_m() ||
          !IsPositiveFinite(completion.position_tolerance_m()) ||
          !HasSettledPoseCompletion(completion)) {
        return Reject(planning::MOTION_REJECT_INVALID_PAYLOAD,
                      "pose servo target or bounds are incomplete");
      }
      break;
    case planning::MOTION_PRIMITIVE_CORRIDOR_SERVO:
      if (!primitive.has_corridor_servo() ||
          !primitive.corridor_servo().has_target_position() ||
          !HasFinitePoint(primitive.corridor_servo().target_position()) ||
          !IsInsideBoundary(primitive.corridor_servo().target_position(),
                            command.spatial_envelope()) ||
          !primitive.corridor_servo().has_target_heading() ||
          !IsFinite(primitive.corridor_servo().target_heading()) ||
          !primitive.corridor_servo().has_direction() ||
          primitive.corridor_servo().direction() ==
              planning::MOTION_DIRECTION_UNKNOWN ||
          command.spatial_envelope().reference_centerline_size() < 2 ||
          !command.spatial_envelope().has_max_lateral_deviation_m() ||
          SquaredDistance(
              command.spatial_envelope().reference_centerline(
                  command.spatial_envelope().reference_centerline_size() - 1),
              primitive.corridor_servo().target_position()) > 1e-4 ||
          !constraints.has_max_distance_m() ||
          !IsPositiveFinite(constraints.max_distance_m()) ||
          !IsPositiveFinite(
              CenterlineLength(command.spatial_envelope())) ||
          CenterlineLength(command.spatial_envelope()) >
              constraints.max_distance_m() ||
          constraints.max_speed_mps() <= 0.0 ||
          !HasSettledPoseCompletion(completion)) {
        return Reject(planning::MOTION_REJECT_INVALID_PAYLOAD,
                      "corridor servo target or bounds are incomplete");
      }
      break;
    case planning::MOTION_PRIMITIVE_ROTATE_IN_PLACE:
      if (!primitive.has_rotate_in_place() ||
          !primitive.rotate_in_place().has_target_heading() ||
          !IsFinite(primitive.rotate_in_place().target_heading()) ||
          !primitive.rotate_in_place().has_pivot_position() ||
          !HasFinitePoint(primitive.rotate_in_place().pivot_position()) ||
          !IsInsideBoundary(primitive.rotate_in_place().pivot_position(),
                            command.spatial_envelope()) ||
          !primitive.rotate_in_place().has_max_position_deviation_m() ||
          !IsPositiveFinite(
              primitive.rotate_in_place().max_position_deviation_m()) ||
          !primitive.rotate_in_place().has_direction() ||
          primitive.rotate_in_place().direction() ==
              planning::ROTATION_DIRECTION_UNKNOWN ||
          !constraints.has_max_yaw_rate_radps() ||
          !IsPositiveFinite(constraints.max_yaw_rate_radps()) ||
          !constraints.has_max_yaw_acceleration_radps2() ||
          !IsPositiveFinite(constraints.max_yaw_acceleration_radps2()) ||
          !completion.has_heading_tolerance_rad() ||
          !IsPositiveFinite(completion.heading_tolerance_rad()) ||
          !completion.has_speed_tolerance_mps() ||
          !IsNonNegativeFinite(completion.speed_tolerance_mps()) ||
          !completion.has_settle_time_sec() ||
          !IsPositiveFinite(completion.settle_time_sec())) {
        return Reject(planning::MOTION_REJECT_INVALID_PAYLOAD,
                      "rotate-in-place target or angular bounds are incomplete");
      }
      break;
    case planning::MOTION_PRIMITIVE_UNKNOWN:
    default:
      return Reject(planning::MOTION_REJECT_INVALID_PAYLOAD,
                    "unknown primitive type");
  }

  MotionValidationResult result;
  result.accepted = true;
  result.execution_type = planning::MOTION_EXECUTION_TYPE_PRIMITIVE;
  return result;
}

}  // namespace control
}  // namespace apollo
