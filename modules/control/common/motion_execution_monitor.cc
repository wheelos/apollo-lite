#include "modules/control/common/motion_execution_monitor.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace apollo {
namespace control {

MotionExecutionMonitor::MotionExecutionMonitor(
    double footprint_radius_m, double spatial_step_m, double max_state_age_sec)
    : radius_(footprint_radius_m), step_(spatial_step_m),
      max_age_(max_state_age_sec) {}

void MotionExecutionMonitor::Reset() {
  identity_.clear();
  settled_since_ = -1.0;
  holding_ = false;
}

MotionPrimitiveResult MotionExecutionMonitor::Update(
    const planning::MotionExecutionCommand& command,
    const common::VehicleState& state, const std::string& frame_id,
    double now_sec) {
  MotionPrimitiveResult result;
  const auto reject = [&](const std::string& reason) {
    settled_since_ = -1.0;
    result.reason = reason;
    result.reject_reason = planning::MOTION_REJECT_INVALID_CONSTRAINTS;
    return result;
  };
  const bool hold = command.has_primitive() &&
      command.primitive().type() == planning::MOTION_PRIMITIVE_STANDSTILL_HOLD;
  if ((!hold && (!command.has_trajectory() ||
                 command.trajectory().point_size() < 2)) ||
      !std::isfinite(now_sec) || !std::isfinite(max_age_) || max_age_ <= 0.0 ||
      !state.has_x() || !state.has_y() || !state.has_heading() ||
      !state.has_linear_velocity() || !state.has_timestamp() ||
      !std::isfinite(state.x()) || !std::isfinite(state.y()) ||
      !std::isfinite(state.heading()) ||
      !std::isfinite(state.linear_velocity()) ||
      !std::isfinite(state.timestamp()) || state.timestamp() > now_sec ||
      now_sec - state.timestamp() > max_age_ ||
      state.timestamp() < command.start_condition().snapshot_time_sec() ||
      frame_id.empty() || frame_id != command.reference_frame_id()) {
    return reject("execution monitor lacks a fresh finite same-frame state");
  }
  const auto identity = command.SerializeAsString();
  if (identity != identity_) {
    Reset();
    identity_ = identity;
    previous_x_ = state.x();
    previous_y_ = state.y();
    previous_stamp_ = state.timestamp();
    previous_time_ = now_sec;
    admitted_at_ = now_sec;
  } else if (state.timestamp() < previous_stamp_ ||
             now_sec < previous_time_ || now_sec - previous_time_ > max_age_) {
    return reject("execution evidence timestamp regressed or monitoring gap");
  }
  if (!IsMotionSweptFootprintInside(
          previous_x_, previous_y_, state.x(), state.y(),
          command.spatial_envelope(), radius_, step_)) {
    return reject("live footprint left the authorized spatial envelope");
  }
  const auto& completion = command.completion();
  if (std::abs(state.linear_velocity()) >
      (hold ? completion.speed_tolerance_mps()
            : command.constraints().max_speed_mps())) {
    return reject("live speed exceeds authorized motion bound");
  }
  if (!hold) {
    double distance = std::numeric_limits<double>::infinity();
    double path_heading = 0.0;
    for (int i = 1; i < command.trajectory().point_size(); ++i) {
      const auto& a = command.trajectory().point(i - 1).path_point();
      const auto& b = command.trajectory().point(i).path_point();
      const double dx = b.x() - a.x();
      const double dy = b.y() - a.y();
      const double length2 = dx * dx + dy * dy;
      const double t = length2 > 0.0 ? std::max(0.0, std::min(1.0,
          ((state.x() - a.x()) * dx + (state.y() - a.y()) * dy) / length2)) : 0.0;
      const double d = std::hypot(state.x() - a.x() - t * dx,
                                  state.y() - a.y() - t * dy);
      if (d < distance) {
        distance = d;
        path_heading = a.theta() + t * std::atan2(
            std::sin(b.theta() - a.theta()), std::cos(b.theta() - a.theta()));
      }
    }
    const double heading_error = std::abs(std::atan2(
        std::sin(state.heading() - path_heading),
        std::cos(state.heading() - path_heading)));
    const double lateral_bound = command.spatial_envelope().has_max_lateral_deviation_m()
        ? command.spatial_envelope().max_lateral_deviation_m()
        : command.start_condition().max_position_error_m();
    const bool reverse = command.trajectory().gear() == canbus::Chassis::GEAR_REVERSE;
    if (distance > lateral_bound ||
        heading_error > command.start_condition().max_heading_error_rad() ||
        (reverse ? state.linear_velocity() : -state.linear_velocity()) >
            completion.speed_tolerance_mps()) {
      return reject("live trajectory corridor, heading or direction bound violated");
    }
  }
  double x = command.start_condition().expected_position().x();
  double y = command.start_condition().expected_position().y();
  double heading = command.start_condition().expected_heading();
  bool reference_elapsed = true;
  if (!hold) {
    const auto& endpoint = command.trajectory().point(
        command.trajectory().point_size() - 1);
    x = endpoint.path_point().x();
    y = endpoint.path_point().y();
    heading = endpoint.path_point().theta();
    reference_elapsed = now_sec >= command.header().timestamp_sec() +
                                      endpoint.relative_time_sec();
  }
  result.position_error_m = std::hypot(state.x() - x, state.y() - y);
  result.heading_error_rad = std::abs(std::atan2(
      std::sin(state.heading() - heading), std::cos(state.heading() - heading)));
  const bool at_rest =
      result.position_error_m <= completion.position_tolerance_m() &&
      result.heading_error_rad <= completion.heading_tolerance_rad() &&
      std::abs(state.linear_velocity()) <= completion.speed_tolerance_mps();
  if (hold && holding_ && !at_rest) {
    return reject("measured standstill hold no longer satisfies terminal bounds");
  }
  if (at_rest && reference_elapsed) {
    if (settled_since_ < 0.0) {
      settled_since_ = std::max(admitted_at_, state.timestamp());
    }
    const bool settled =
        state.timestamp() - settled_since_ >= completion.settle_time_sec();
    result.completed = !hold && settled;
    result.hold_confirmed = hold && settled;
    holding_ = holding_ || result.hold_confirmed;
  } else {
    settled_since_ = -1.0;
  }
  previous_x_ = state.x();
  previous_y_ = state.y();
  previous_stamp_ = state.timestamp();
  previous_time_ = now_sec;
  result.observed_at_sec = state.timestamp();
  result.reference_frame_id = frame_id;
  if (settled_since_ >= 0.0) {
    result.settled_duration_sec =
        std::max(0.0, state.timestamp() - settled_since_);
  }
  result.accepted = true;
  return result;
}

}  // namespace control
}  // namespace apollo
