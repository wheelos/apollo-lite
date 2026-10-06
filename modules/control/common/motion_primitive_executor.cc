#include "modules/control/common/motion_primitive_executor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "modules/control/common/motion_execution_validator.h"

namespace apollo {
namespace control {
namespace {

constexpr double kGeometryTolerance = 1e-4;
constexpr double kPi = 3.14159265358979323846;
constexpr int kMaxSamples = 10000;

double Angle(double value) { return std::remainder(value, 2.0 * kPi); }
double Square(double value) { return value * value; }
bool Positive(double value) { return std::isfinite(value) && value > 0.0; }

double SegmentDistance(double x, double y, double ax, double ay, double bx,
                       double by) {
  const double length2 = Square(bx - ax) + Square(by - ay);
  const double t = length2 > 0.0
                       ? std::max(0.0, std::min(1.0,
                             ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) /
                                 length2))
                       : 0.0;
  return std::hypot(x - ax - t * (bx - ax), y - ay - t * (by - ay));
}

bool Inside(double x, double y,
            const planning::MotionSpatialEnvelope& envelope, double margin) {
  bool inside = false;
  for (int i = 0, j = envelope.boundary_size() - 1;
       i < envelope.boundary_size(); j = i++) {
    const auto& a = envelope.boundary(i);
    const auto& b = envelope.boundary(j);
    if (SegmentDistance(x, y, a.x(), a.y(), b.x(), b.y()) <= margin) {
      return false;
    }
    if ((a.y() > y) != (b.y() > y) &&
        x < (b.x() - a.x()) * (y - a.y()) / (b.y() - a.y()) + a.x()) {
      inside = !inside;
    }
  }
  return inside;
}

// Endpoint clearance exceeds half the segment length, covering the entire
// swept disk, including concave boundary crossings between samples.
bool SafeSegment(double ax, double ay, double bx, double by,
                 const planning::MotionSpatialEnvelope& envelope,
                 double radius, double step) {
  const double length = std::hypot(bx - ax, by - ay);
  if (!std::isfinite(length) || length / step > kMaxSamples) {
    return false;
  }
  const int count = std::max(1, static_cast<int>(std::ceil(length / step)));
  for (int i = 0; i <= count; ++i) {
    const double t = static_cast<double>(i) / count;
    if (!Inside(ax + t * (bx - ax), ay + t * (by - ay), envelope,
                radius + length / count * 0.5)) {
      return false;
    }
  }
  return true;
}

common::PathPoint Advance(const common::PathPoint& from, double ds) {
  common::PathPoint point = from;
  const auto theta = [&](double s) {
    return from.theta() + from.kappa() * s + 0.5 * from.dkappa() * s * s;
  };
  // Composite Simpson quadrature integrates prescribed linear curvature;
  // geometry is never synthesized from the target pose.
  double dx = 0.0;
  double dy = 0.0;
  constexpr int kIntegrationSteps = 8;
  for (int i = 0; i <= kIntegrationSteps; ++i) {
    const double angle = theta(ds * i / kIntegrationSteps);
    const double weight = (i == 0 || i == kIntegrationSteps) ? 1.0
                           : (i % 2 == 0 ? 2.0 : 4.0);
    dx += weight * std::cos(angle);
    dy += weight * std::sin(angle);
  }
  point.set_x(from.x() + dx * ds / (3.0 * kIntegrationSteps));
  point.set_y(from.y() + dy * ds / (3.0 * kIntegrationSteps));
  point.set_s(from.s() + ds);
  point.set_theta(Angle(theta(ds)));
  point.set_kappa(from.kappa() + from.dkappa() * ds);
  return point;
}

using Polynomial = std::vector<double>;

double Evaluate(const Polynomial& p, double t) {
  double value = 0.0;
  for (auto it = p.rbegin(); it != p.rend(); ++it) {
    value = value * t + *it;
  }
  return value;
}

Polynomial Derivative(const Polynomial& p) {
  Polynomial d;
  for (size_t i = 1; i < p.size(); ++i) {
    d.push_back(i * p[i]);
  }
  return d;
}

// Roots on a bounded interval are isolated by derivative roots; checking these
// extrema gives continuous-time limits, not merely sampled-point limits.
std::vector<double> Roots(const Polynomial& p, double end) {
  if (p.size() <= 1) {
    return {};
  }
  auto partitions = Roots(Derivative(p), end);
  partitions.insert(partitions.begin(), 0.0);
  partitions.push_back(end);
  std::vector<double> roots;
  for (size_t i = 1; i < partitions.size(); ++i) {
    double left = partitions[i - 1];
    double right = partitions[i];
    if (std::abs(Evaluate(p, left)) < 1e-10) {
      roots.push_back(left);
    }
    if ((Evaluate(p, left) < 0.0) == (Evaluate(p, right) < 0.0)) {
      continue;
    }
    for (int iteration = 0; iteration < 60; ++iteration) {
      const double middle = (left + right) * 0.5;
      if ((Evaluate(p, left) < 0.0) == (Evaluate(p, middle) < 0.0)) {
        left = middle;
      } else {
        right = middle;
      }
    }
    roots.push_back((left + right) * 0.5);
  }
  return roots;
}

bool Bounded(const Polynomial& p, double end, double low, double high) {
  auto times = Roots(Derivative(p), end);
  times.push_back(0.0);
  times.push_back(end);
  for (double t : times) {
    const double value = Evaluate(p, t);
    if (!std::isfinite(value) || value < low - 1e-8 || value > high + 1e-8) {
      return false;
    }
  }
  return true;
}

bool TimeLaw(double distance, double speed, double acceleration,
             double max_speed, double max_acceleration, double max_deceleration,
             double max_jerk, double period, double timeout, double preferred,
             Polynomial* position, double* duration) {
  // Quintic scalar time scaling of planning's fixed spatial guidance. The
  // measured initial velocity/acceleration and stopped endpoint are exact.
  std::vector<double> candidates;
  if (preferred >= period && preferred <= timeout) {
    candidates.push_back(preferred);
  }
  for (double t = 2.0 * period; t <= timeout; t *= 1.1) {
    candidates.push_back(t);
  }
  for (double t : candidates) {
    const double c0 = distance - speed * t - 0.5 * acceleration * t * t;
    const double c1 = -speed - acceleration * t;
    const double c2 = -acceleration;
    Polynomial p{0.0, speed, 0.5 * acceleration,
                 10.0 * c0 / std::pow(t, 3) - 4.0 * c1 / Square(t) +
                     0.5 * c2 / t,
                 -15.0 * c0 / std::pow(t, 4) + 7.0 * c1 / std::pow(t, 3) -
                     c2 / Square(t),
                 6.0 * c0 / std::pow(t, 5) - 3.0 * c1 / std::pow(t, 4) +
                     0.5 * c2 / std::pow(t, 3)};
    const auto v = Derivative(p);
    const auto a = Derivative(v);
    if (Bounded(v, t, 0.0, max_speed) &&
        Bounded(a, t, -max_deceleration, max_acceleration) &&
        Bounded(Derivative(a), t, -max_jerk, max_jerk)) {
      *position = std::move(p);
      *duration = t;
      return true;
    }
  }
  return false;
}

bool ModelValid(const MotionPrimitiveModel& model) {
  for (double value : {
           model.max_speed_mps, model.max_acceleration_mps2,
           model.max_deceleration_mps2, model.max_jerk_mps3,
           model.max_curvature_per_m, model.max_curvature_derivative_per_m2,
           model.max_lateral_acceleration_mps2, model.footprint_radius_m,
           model.sample_period_sec, model.spatial_step_m,
           model.max_state_age_sec, model.max_tracking_heading_error_rad}) {
    if (!Positive(value)) {
      return false;
    }
  }
  return model.max_tracking_heading_error_rad < kPi * 0.5 &&
         model.max_curvature_per_m * model.spatial_step_m < 0.1;
}

}  // namespace

bool IsMotionSweptFootprintInside(
    double from_x, double from_y, double to_x, double to_y,
    const planning::MotionSpatialEnvelope& envelope, double radius,
    double spatial_step) {
  return Positive(radius) && Positive(spatial_step) &&
         envelope.boundary_size() >= 3 &&
         SafeSegment(from_x, from_y, to_x, to_y, envelope, radius, spatial_step);
}

MotionPrimitiveExecutor::MotionPrimitiveExecutor(
    const MotionPrimitiveModel& model) : model_(model) {}

bool MotionPrimitiveExecutor::IsAvailable() const { return ModelValid(model_); }

void MotionPrimitiveExecutor::Reset() {
  command_bytes_.clear();
  command_identity_.clear();
  path_.clear();
  settled_since_ = -1.0;
  reference_duration_ = 0.0;
  failed_ = false;
}

bool MotionPrimitiveExecutor::BuildPath(
    const planning::MotionExecutionCommand& command, std::string* reason) {
  const auto& primitive = command.primitive();
  const bool pose = primitive.type() == planning::MOTION_PRIMITIVE_POSE_SERVO;
  const auto& target = pose ? primitive.pose_servo().target_position()
                            : primitive.corridor_servo().target_position();
  const double heading = pose ? primitive.pose_servo().target_heading()
                              : primitive.corridor_servo().target_heading();
  const auto direction = pose ? primitive.pose_servo().direction()
                              : primitive.corridor_servo().direction();
  if (direction != planning::MOTION_DIRECTION_FORWARD &&
      direction != planning::MOTION_DIRECTION_REVERSE) {
    *reason = "unsupported motion direction";
    return false;
  }
  direction_ = direction == planning::MOTION_DIRECTION_REVERSE ? -1.0 : 1.0;
  const auto gear = command.start_condition().expected_gear();
  if ((direction_ < 0.0 && gear != canbus::Chassis::GEAR_REVERSE) ||
      (direction_ > 0.0 && gear != canbus::Chassis::GEAR_DRIVE &&
       gear != canbus::Chassis::GEAR_LOW)) {
    *reason = "primitive direction conflicts with start gear";
    return false;
  }
  const auto& envelope = command.spatial_envelope();
  if (!envelope.has_max_lateral_deviation_m() ||
      !Positive(envelope.max_lateral_deviation_m())) {
    *reason = "spatial tracking requires an explicit lateral deviation bound";
    return false;
  }
  std::vector<common::PathPoint> guidance(envelope.reference_path().begin(),
                                          envelope.reference_path().end());
  if (guidance.size() > kMaxSamples) {
    *reason = "spatial guidance exceeds bounded executor capacity";
    return false;
  }
  if (guidance.empty()) {
    const auto& start = command.start_condition().expected_position();
    if (!pose && (envelope.reference_centerline_size() < 2 ||
        std::hypot(envelope.reference_centerline(0).x() - start.x(),
                   envelope.reference_centerline(0).y() - start.y()) >
            kGeometryTolerance)) {
      *reason = "corridor centerline does not begin at the declared start";
      return false;
    }
    const double distance = std::hypot(target.x() - start.x(),
                                       target.y() - start.y());
    const double body_heading = distance > kGeometryTolerance
        ? Angle(std::atan2(target.y() - start.y(), target.x() - start.x()) +
                (direction_ < 0.0 ? kPi : 0.0))
        : heading;
    if (std::abs(Angle(body_heading - heading)) > kGeometryTolerance ||
        std::abs(Angle(body_heading -
                       command.start_condition().expected_heading())) >
            kGeometryTolerance) {
      *reason = "nonholonomic pose alignment requires feasible spatial guidance";
      return false;
    }
    double previous_station = -kGeometryTolerance;
    for (const auto& point : envelope.reference_centerline()) {
      const double station = direction_ *
          ((point.x() - start.x()) * std::cos(body_heading) +
           (point.y() - start.y()) * std::sin(body_heading));
      if (SegmentDistance(point.x(), point.y(), start.x(), start.y(),
                          target.x(), target.y()) > kGeometryTolerance ||
          station < previous_station) {
        *reason = "bent centerline requires heading and curvature guidance";
        return false;
      }
      previous_station = station;
    }
    common::PathPoint first;
    first.set_x(start.x());
    first.set_y(start.y());
    first.set_z(start.z());
    first.set_theta(body_heading);
    first.set_s(0.0);
    first.set_kappa(0.0);
    first.set_dkappa(0.0);
    guidance.push_back(first);
    if (distance > kGeometryTolerance) {
      guidance.push_back(Advance(first, direction_ * distance));
    }
  }
  const auto& constraints = command.constraints();
  if (!constraints.has_max_abs_curvature_per_m() ||
      !Positive(constraints.max_abs_curvature_per_m()) ||
      !constraints.has_max_abs_curvature_derivative_per_m2() ||
      !Positive(constraints.max_abs_curvature_derivative_per_m2())) {
    *reason = "spatial tracking requires curvature and curvature-rate bounds";
    return false;
  }
  const double curvature = std::min(model_.max_curvature_per_m,
                                    constraints.max_abs_curvature_per_m());
  const double derivative = std::min(
      model_.max_curvature_derivative_per_m2,
      constraints.max_abs_curvature_derivative_per_m2());
  path_.clear();
  for (const auto& point : guidance) {
    if (!point.has_x() || !point.has_y() || !point.has_theta() ||
        !point.has_s() || !point.has_kappa() || !point.has_dkappa() ||
        !std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z()) || !std::isfinite(point.theta()) ||
        !std::isfinite(point.s()) || !std::isfinite(point.kappa()) ||
        !std::isfinite(point.dkappa()) || std::abs(point.kappa()) > curvature ||
        std::abs(point.dkappa()) > derivative ||
        (point.has_ddkappa() &&
         (!std::isfinite(point.ddkappa()) || point.ddkappa() != 0.0))) {
      *reason = "invalid or platform-infeasible spatial geometry";
      return false;
    }
    if (path_.empty()) {
      path_.push_back(point);
      continue;
    }
    const auto previous = path_.back();
    const double ds = point.s() - previous.s();
    if (direction_ * ds <= 0.0 ||
        std::abs(ds) / model_.spatial_step_m > kMaxSamples) {
      *reason = "spatial station must advance in the commanded direction";
      return false;
    }
    const int count = std::max(1, static_cast<int>(
        std::ceil(std::abs(ds) / model_.spatial_step_m)));
    auto integrated = previous;
    for (int i = 1; i <= count; ++i) {
      integrated = Advance(integrated, ds / count);
      if (std::abs(integrated.kappa()) > curvature + 1e-8) {
        *reason = "integrated spatial curvature exceeds the authorized bound";
        return false;
      }
      path_.push_back(integrated);
    }
    if (std::hypot(integrated.x() - point.x(), integrated.y() - point.y()) >
            kGeometryTolerance ||
        std::abs(Angle(integrated.theta() - point.theta())) >
            kGeometryTolerance ||
        std::abs(integrated.kappa() - point.kappa()) > kGeometryTolerance ||
        path_.size() > kMaxSamples) {
      *reason = "spatial guidance is discontinuous or kinematically inconsistent";
      return false;
    }
    path_.back() = point;
  }
  const auto& start = command.start_condition().expected_position();
  if (std::hypot(path_.front().x() - start.x(),
                 path_.front().y() - start.y()) > kGeometryTolerance ||
      std::abs(Angle(path_.front().theta() -
                     command.start_condition().expected_heading())) >
          kGeometryTolerance ||
      std::hypot(path_.back().x() - target.x(),
                 path_.back().y() - target.y()) > kGeometryTolerance ||
      std::abs(Angle(path_.back().theta() - heading)) > kGeometryTolerance ||
      std::abs(path_.back().s() - path_.front().s()) >
          constraints.max_distance_m()) {
    *reason = "spatial guidance does not match start, terminal pose or distance";
    return false;
  }
  const double margin = model_.footprint_radius_m +
      envelope.max_lateral_deviation_m() +
      curvature * Square(model_.spatial_step_m);
  for (size_t i = 0; i < path_.size(); ++i) {
    const auto& a = path_[i == 0 ? 0 : i - 1];
    const auto& b = path_[i];
    if (!SafeSegment(a.x(), a.y(), b.x(), b.y(), envelope, margin,
                     model_.spatial_step_m)) {
      *reason = "spatial guidance cannot contain the swept footprint and tracking tube";
      return false;
    }
  }
  return true;
}

MotionPrimitiveResult MotionPrimitiveExecutor::Update(
    const planning::MotionExecutionCommand& command,
    const common::VehicleState& state, const std::string& state_frame_id,
    double now_sec) {
  MotionPrimitiveResult result;
  const auto reject = [&](planning::MotionCommandRejectReason code,
                          const std::string& reason) {
    failed_ = true;
    settled_since_ = -1.0;
    result.accepted = false;
    result.completed = false;
    result.hold_confirmed = false;
    result.reject_reason = code;
    result.reason = reason;
    result.reference.Clear();
    return result;
  };
  if (!ModelValid(model_)) {
    return reject(planning::MOTION_REJECT_UNSUPPORTED_CAPABILITY,
                  "primitive platform model is incomplete or invalid");
  }
  MotionExecutionCapabilities capabilities;
  capabilities.supported = {planning::MOTION_CAPABILITY_POSE_SERVO,
                            planning::MOTION_CAPABILITY_CORRIDOR_SERVO};
  const auto validation = MotionExecutionValidator(capabilities).Validate(
      command, now_sec);
  if (!validation.accepted) {
    return reject(validation.reject_reason, validation.reason);
  }
  if (state_frame_id != command.reference_frame_id()) {
    return reject(planning::MOTION_REJECT_INVALID_FRAME,
                  "current vehicle state and primitive frames differ");
  }
  if (!state.has_x() || !state.has_y() || !state.has_heading() ||
      !state.has_timestamp() || !state.has_linear_velocity() ||
      !state.has_linear_acceleration() || !state.has_kappa() ||
      !state.has_angular_velocity() || !std::isfinite(state.x()) ||
      !std::isfinite(state.y()) || !std::isfinite(state.heading()) ||
      (state.has_z() && !std::isfinite(state.z())) ||
      (state.has_kappa() && !std::isfinite(state.kappa())) ||
      (state.has_angular_velocity() &&
       !std::isfinite(state.angular_velocity())) ||
      !std::isfinite(state.linear_velocity()) ||
      !std::isfinite(state.linear_acceleration()) ||
      !std::isfinite(state.timestamp()) || state.timestamp() > now_sec ||
      state.timestamp() < command.start_condition().snapshot_time_sec() ||
      now_sec - state.timestamp() > model_.max_state_age_sec) {
    return reject(planning::MOTION_REJECT_INVALID_START_CONDITION,
                  "vehicle state is missing, nonfinite, future-dated or stale");
  }
  const std::string bytes = command.SerializeAsString();
  const bool new_command = bytes != command_bytes_;
  if (new_command) {
    if (command.identity().SerializeAsString() == command_identity_) {
      return reject(planning::MOTION_REJECT_INVALID_TRANSITION,
                    "primitive payload changed without a new identity");
    }
    Reset();
    std::string reason;
    if (!BuildPath(command, &reason)) {
      return reject(planning::MOTION_REJECT_INVALID_PAYLOAD, reason);
    }
    const auto& start = command.start_condition();
    if (std::hypot(state.x() - start.expected_position().x(),
                   state.y() - start.expected_position().y()) >
            start.max_position_error_m() ||
        std::abs(Angle(state.heading() - start.expected_heading())) >
            start.max_heading_error_rad() ||
        std::abs(state.linear_velocity()) > start.max_abs_speed_mps()) {
      return reject(planning::MOTION_REJECT_INVALID_START_CONDITION,
                    "live start pose or speed violates the admitted start condition");
    }
    command_bytes_ = bytes;
    command_identity_ = command.identity().SerializeAsString();
    started_at_ = now_sec;
    last_time_ = now_sec;
    last_state_time_ = state.timestamp();
    last_progress_ = 0.0;
    traveled_distance_ = 0.0;
    previous_x_ = state.x();
    previous_y_ = state.y();
  } else if (failed_) {
    return reject(planning::MOTION_REJECT_INVALID_TRANSITION,
                  "failed primitive requires explicit reset/re-admission");
  } else if (now_sec <= last_time_ ||
             state.timestamp() < last_state_time_ ||
             now_sec - last_time_ > model_.max_state_age_sec) {
    return reject(planning::MOTION_REJECT_STALE,
                  "primitive requires advancing cycles and nonregressing fresh state");
  }
  const auto& completion = command.completion();
  if (now_sec - started_at_ > completion.execution_timeout_sec()) {
    return reject(planning::MOTION_REJECT_INVALID_TIME,
                  "primitive execution timeout");
  }
  const auto& envelope = command.spatial_envelope();
  const double displacement = std::hypot(state.x() - previous_x_,
                                         state.y() - previous_y_);
  if (!new_command && displacement >
      std::min(model_.max_speed_mps, command.constraints().max_speed_mps()) *
          (state.timestamp() - last_state_time_) +
          2.0 * completion.position_tolerance_m()) {
    return reject(planning::MOTION_REJECT_INVALID_CONSTRAINTS,
                  "live displacement is incompatible with authorized speed");
  }
  if (!SafeSegment(previous_x_, previous_y_, state.x(), state.y(), envelope,
                   model_.footprint_radius_m, model_.spatial_step_m)) {
    return reject(planning::MOTION_REJECT_INVALID_CONSTRAINTS,
                  "live swept vehicle footprint leaves the hard envelope");
  }
  traveled_distance_ += displacement;
  if (traveled_distance_ > command.constraints().max_distance_m()) {
    return reject(planning::MOTION_REJECT_INVALID_CONSTRAINTS,
                  "actual traveled distance exceeds primitive authorization");
  }
  double distance = std::numeric_limits<double>::infinity();
  double progress = 0.0;
  double reference_heading = path_.front().theta();
  for (size_t i = 0; i < path_.size(); ++i) {
    const auto& a = path_[i == 0 ? 0 : i - 1];
    const auto& b = path_[i];
    const double length2 = Square(b.x() - a.x()) + Square(b.y() - a.y());
    const double t = length2 > 0.0 ? std::max(0.0, std::min(1.0,
        ((state.x() - a.x()) * (b.x() - a.x()) +
         (state.y() - a.y()) * (b.y() - a.y())) / length2)) : 0.0;
    const double d = std::hypot(state.x() - a.x() - t * (b.x() - a.x()),
                                state.y() - a.y() - t * (b.y() - a.y()));
    if (d < distance) {
      distance = d;
      progress = direction_ * (a.s() + t * (b.s() - a.s()) - path_.front().s());
      reference_heading = Angle(a.theta() + t * Angle(b.theta() - a.theta()));
    }
  }
  if (distance > envelope.max_lateral_deviation_m() ||
      std::abs(Angle(state.heading() - reference_heading)) >
          model_.max_tracking_heading_error_rad ||
      progress + completion.position_tolerance_m() < last_progress_ ||
      direction_ * state.linear_velocity() < -completion.speed_tolerance_mps()) {
    return reject(planning::MOTION_REJECT_INVALID_CONSTRAINTS,
                  "live direction, progress or spatial tracking bound violated");
  }
  result.position_error_m = std::hypot(state.x() - path_.back().x(),
                                       state.y() - path_.back().y());
  result.heading_error_rad = std::abs(Angle(state.heading() - path_.back().theta()));
  const bool at_pose =
      result.position_error_m <= completion.position_tolerance_m() &&
      result.heading_error_rad <= completion.heading_tolerance_rad();
  const bool stopped =
      std::abs(state.linear_velocity()) <= completion.speed_tolerance_mps();
  if (at_pose && stopped) {
    if (settled_since_ < 0.0) {
      settled_since_ = std::max(started_at_, state.timestamp());
    }
    result.hold_confirmed =
        state.timestamp() - settled_since_ >= completion.settle_time_sec();
    result.completed = result.hold_confirmed;
  } else {
    settled_since_ = -1.0;
  }
  result.observed_at_sec = state.timestamp();
  result.settled_duration_sec =
      settled_since_ >= 0.0
          ? std::max(0.0, state.timestamp() - settled_since_)
          : 0.0;
  result.reference_frame_id = state_frame_id;
  auto* reference = &result.reference;
  reference->mutable_header()->CopyFrom(command.header());
  reference->mutable_header()->set_timestamp_sec(now_sec);
  reference->mutable_header()->set_frame_id(command.reference_frame_id());
  reference->mutable_header()->set_sequence_num(++sequence_);
  reference->set_gear(command.start_condition().expected_gear());
  reference->set_trajectory_type(planning::ADCTrajectory::NORMAL);
  if (command.has_execution()) {
    reference->mutable_execution()->CopyFrom(command.execution());
    reference->mutable_execution()->set_execution_channel(
        planning::EXECUTION_CHANNEL_PRIMITIVE);
  }
  auto* intent = reference->mutable_control_intent();
  if (command.has_control_intent()) {
    intent->CopyFrom(command.control_intent());
  }
  intent->set_require_full_stop(false);
  intent->set_suppress_large_steer(false);
  intent->set_execution_channel(planning::EXECUTION_CHANNEL_PRIMITIVE);
  intent->set_tracking_mode(planning::TRACKING_MODE_PATH_SPEED);
  intent->set_primitive_type(planning::CONTROL_PRIMITIVE_NONE);
  intent->set_longitudinal_intent(planning::LON_INTENT_PRECISE_STOP);
  intent->set_lateral_intent(planning::LAT_INTENT_TRACK_PATH);
  intent->mutable_target_stop_point()->set_x(path_.back().x());
  intent->mutable_target_stop_point()->set_y(path_.back().y());
  intent->mutable_target_stop_point()->set_z(path_.back().z());
  intent->set_target_stop_heading(path_.back().theta());
  intent->set_terminal_position_tolerance_m(completion.position_tolerance_m());
  intent->set_terminal_heading_tolerance_rad(completion.heading_tolerance_rad());
  intent->set_terminal_servo_timeout_sec(completion.execution_timeout_sec());
  intent->set_reason("validated spatial primitive reference");

  const auto& constraints = command.constraints();
  double max_speed = std::min(model_.max_speed_mps, constraints.max_speed_mps());
  double max_curvature = 0.0;
  double max_derivative = 0.0;
  for (const auto& point : path_) {
    max_curvature = std::max(max_curvature, std::abs(point.kappa()));
    max_derivative = std::max(max_derivative, std::abs(point.dkappa()));
    if (std::abs(point.kappa()) > 0.0) {
      max_speed = std::min(max_speed, std::sqrt(
          model_.max_lateral_acceleration_mps2 / std::abs(point.kappa())));
    }
  }
  double max_acceleration = std::min(model_.max_acceleration_mps2,
                                     constraints.max_acceleration_mps2());
  double max_deceleration = std::min(model_.max_deceleration_mps2,
                                     constraints.max_deceleration_mps2());
  if ((constraints.has_max_yaw_rate_radps() &&
       !Positive(constraints.max_yaw_rate_radps())) ||
      (constraints.has_max_yaw_acceleration_radps2() &&
       !Positive(constraints.max_yaw_acceleration_radps2()))) {
    return reject(planning::MOTION_REJECT_INVALID_CONSTRAINTS,
                  "invalid optional angular constraints");
  }
  if (constraints.has_max_yaw_rate_radps() && max_curvature > 0.0) {
    max_speed = std::min(max_speed,
                        constraints.max_yaw_rate_radps() / max_curvature);
  }
  if (constraints.has_max_yaw_acceleration_radps2()) {
    const double budget = constraints.max_yaw_acceleration_radps2();
    if (max_curvature > 0.0) {
      const double bound = budget / max_curvature *
                           (max_derivative > 0.0 ? 0.5 : 1.0);
      max_acceleration = std::min(max_acceleration, bound);
      max_deceleration = std::min(max_deceleration, bound);
    }
    if (max_derivative > 0.0) {
      max_speed = std::min(max_speed, std::sqrt(
          budget / max_derivative * (max_curvature > 0.0 ? 0.5 : 1.0)));
    }
  }
  if (std::abs(state.linear_velocity()) > max_speed + 1e-8 ||
      direction_ * state.linear_acceleration() > max_acceleration + 1e-8 ||
      direction_ * state.linear_acceleration() < -max_deceleration - 1e-8 ||
      (state.has_kappa() &&
       (std::abs(state.kappa()) >
            std::min(model_.max_curvature_per_m,
                     constraints.max_abs_curvature_per_m()) ||
        Square(state.linear_velocity()) * std::abs(state.kappa()) >
            model_.max_lateral_acceleration_mps2)) ||
      (state.has_angular_velocity() && constraints.has_max_yaw_rate_radps() &&
       std::abs(state.angular_velocity()) > constraints.max_yaw_rate_radps())) {
    return reject(planning::MOTION_REJECT_INVALID_CONSTRAINTS,
                  "live dynamics exceed the primitive or platform bounds");
  }
  intent->set_max_terminal_speed_mps(max_speed);
  const double remaining = direction_ * (path_.back().s() - path_.front().s()) -
                           progress;
  Polynomial position;
  double duration = model_.sample_period_sec;
  if (!at_pose || !stopped) {
    const double available_time = std::min(
        command.expiry_time_sec() - now_sec,
        completion.execution_timeout_sec() - (now_sec - started_at_)) -
        completion.settle_time_sec();
    if (remaining <= kGeometryTolerance ||
        !TimeLaw(remaining, direction_ * state.linear_velocity(),
                 direction_ * state.linear_acceleration(), max_speed,
                 max_acceleration, max_deceleration,
                 std::min(model_.max_jerk_mps3, constraints.max_jerk_mps3()),
                 model_.sample_period_sec, available_time,
                 reference_duration_ - (now_sec - last_time_),
                 &position, &duration)) {
      return reject(planning::MOTION_REJECT_INVALID_CONSTRAINTS,
                    "no bounded stopping time law; replan or safety stop required");
    }
  } else {
    position = {0.0};
    intent->set_tracking_mode(planning::TRACKING_MODE_STANDSTILL_HOLD);
    intent->set_primitive_type(planning::CONTROL_PRIMITIVE_STANDSTILL_HOLD);
    intent->set_longitudinal_intent(planning::LON_INTENT_HOLD_STOP);
    intent->set_lateral_intent(planning::LAT_INTENT_MINIMIZE_STEER);
    intent->set_require_full_stop(true);
  }
  if (duration / model_.sample_period_sec > kMaxSamples) {
    return reject(planning::MOTION_REJECT_INVALID_CONSTRAINTS,
                  "reference sampling exceeds bounded executor capacity");
  }
  const auto velocity = Derivative(position);
  const auto acceleration = Derivative(velocity);
  const auto jerk = Derivative(acceleration);
  const int count = std::max(1, static_cast<int>(
      std::ceil(duration / model_.sample_period_sec)));
  size_t index = 0;
  for (int i = 0; i <= count; ++i) {
    const double t = duration * i / count;
    const double station = std::min(
        direction_ * (path_.back().s() - path_.front().s()),
        progress + std::max(0.0, Evaluate(position, t)));
    while (index + 1 < path_.size() &&
           direction_ * (path_[index + 1].s() - path_.front().s()) < station) {
      ++index;
    }
    const double ds = path_.front().s() + direction_ * station - path_[index].s();
    auto* point = reference->add_trajectory_point();
    point->mutable_path_point()->CopyFrom(Advance(path_[index], ds));
    point->set_relative_time(t);
    point->set_v(direction_ * std::max(0.0, Evaluate(velocity, t)));
    point->set_a(direction_ * Evaluate(acceleration, t));
    point->set_da(direction_ * Evaluate(jerk, t));
  }
  reference->set_total_path_length(remaining);
  reference->set_total_path_time(duration);
  result.accepted = true;
  result.reason = result.completed ? "measured terminal pose settled"
                                   : "spatial primitive executing";
  last_time_ = now_sec;
  last_state_time_ = state.timestamp();
  last_progress_ = progress;
  reference_duration_ = duration;
  previous_x_ = state.x();
  previous_y_ = state.y();
  return result;
}

}  // namespace control
}  // namespace apollo
