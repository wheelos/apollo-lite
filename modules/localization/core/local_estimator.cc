// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/core/local_estimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

#include "Eigen/Cholesky"

namespace apollo {
namespace localization {
namespace unified {
namespace {

bool Positive(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool Nonnegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

Result CheckStamp(const Stamp& stamp, const LocalConfig& config,
                  const SourceCounters& health, uint64_t last_sequence,
                  Reason stale_reason) {
  if (stamp.clock_id != config.clock_id) {
    return {Reason::CLOCK_INVALID, "Measurement clock does not match"};
  }
  if (!Positive(stamp.time) || !Positive(stamp.receive_time) ||
      stamp.sequence == 0) {
    return {Reason::INVALID_INPUT, "Missing or nonfinite measurement stamp"};
  }
  if (stamp.time - stamp.receive_time > config.future_tolerance) {
    return {Reason::INVALID_INPUT, "Measurement is in the future"};
  }
  if (stamp.receive_time - stamp.time > config.max_sample_age) {
    return {stale_reason, "Measurement exceeded the receive-age limit"};
  }
  if (health.accepted != 0) {
    if (stamp.sequence == last_sequence) {
      return {Reason::DUPLICATE, "Sequence already consumed"};
    }
    if (stamp.sequence < last_sequence || stamp.time < health.last_measurement ||
        stamp.receive_time < health.last_receive) {
      return {Reason::TIMESTAMP_REGRESSION, "Source time or sequence regressed"};
    }
    if (stamp.time == health.last_measurement) {
      return {Reason::DUPLICATE, "Measurement timestamp already consumed"};
    }
  }
  return {};
}

void Accept(const Stamp& stamp, SourceCounters* health) {
  ++health->accepted;
  health->last_measurement = stamp.time;
  health->last_receive = stamp.receive_time;
  health->last_sequence = stamp.sequence;
  health->reason = Reason::NONE;
}

ImuSample InterpolateImu(const ImuSample& start, const ImuSample& end,
                         double time) {
  if (time <= start.stamp.time) {
    return start;
  }
  if (time >= end.stamp.time) {
    return end;
  }
  const double alpha =
      (time - start.stamp.time) / (end.stamp.time - start.stamp.time);
  ImuSample sample;
  sample.stamp.time = time;
  sample.stamp.receive_time =
      start.stamp.receive_time +
      alpha * (end.stamp.receive_time - start.stamp.receive_time);
  sample.stamp.sequence = end.stamp.sequence;
  sample.stamp.clock_id = end.stamp.clock_id;
  sample.acceleration =
      ((1.0 - alpha) * start.acceleration + alpha * end.acceleration).eval();
  sample.angular_velocity =
      ((1.0 - alpha) * start.angular_velocity +
       alpha * end.angular_velocity)
          .eval();
  return sample;
}

bool FiniteState(const LocalState& state) {
  if (!state.position.allFinite() || !state.velocity.allFinite() ||
      !state.orientation.coeffs().allFinite() ||
      std::abs(state.orientation.norm() - 1.0) > 1e-9 ||
      !state.gyro_bias.allFinite() || !state.accel_bias.allFinite() ||
      !state.angular_velocity.allFinite() || !state.covariance.allFinite() ||
      !state.covariance.isApprox(state.covariance.transpose(), 1e-9)) {
    return false;
  }
  Eigen::LLT<Matrix15d> factor(state.covariance);
  return factor.info() == Eigen::Success;
}

}  // namespace

LocalEstimator::LocalEstimator(const LocalConfig& config)
    : config_(config), lio_(config), continuous_output_(config) {
  state_.epoch.session = config.session;
  state_.epoch.generation = 1;
  output_state_.epoch = state_.epoch;
}

Result LocalEstimator::ValidateConfig() const {
  if (config_.session.empty() || config_.clock_id.empty() ||
      config_.stationary_samples == 0 || !Positive(config_.gravity) ||
      !Positive(config_.max_sample_age) ||
      !Nonnegative(config_.future_tolerance) || !Positive(config_.max_imu_gap) ||
      !Positive(config_.wheel_timeout) || !Nonnegative(config_.stationary_speed) ||
      !Positive(config_.stationary_gyro) ||
      !Positive(config_.stationary_accel_tolerance) ||
      !Nonnegative(config_.gyro_noise) || !Nonnegative(config_.accel_noise) ||
      !Nonnegative(config_.gyro_bias_noise) ||
      !Nonnegative(config_.accel_bias_noise) ||
      !Positive(config_.initial_velocity_std) ||
      !Positive(config_.initial_attitude_std) ||
      !Positive(config_.initial_gyro_bias_std) ||
      !Positive(config_.initial_accel_bias_std) ||
      !Positive(config_.nonholonomic_variance) ||
      !Positive(config_.innovation_gate) || !Positive(config_.max_position_std) ||
      !Positive(config_.max_velocity_std) ||
      !Positive(config_.max_attitude_std) || !Positive(config_.history_duration) ||
      config_.max_history_states < 2 ||
      config_.stationary_accel_tolerance >= config_.gravity ||
      config_.initial_velocity_std > config_.max_velocity_std ||
      config_.initial_attitude_std > config_.max_attitude_std) {
    return {Reason::CONFIG_INVALID, "Invalid local estimator configuration"};
  }
  if (config_.enable_lidar &&
      (config_.lidar_frame.empty() ||
       config_.lidar_calibration_id.empty() ||
       !Positive(config_.lidar_point_noise_std) ||
       !Positive(config_.lidar_plane_noise_std) ||
       !Positive(config_.lidar_voxel_size) ||
       config_.lidar_max_voxels == 0 || config_.lidar_max_scans == 0 ||
       config_.lidar_min_points_per_plane < 3 ||
       config_.lidar_min_correspondences == 0 ||
       config_.lidar_max_points < config_.lidar_min_correspondences ||
       !Positive(config_.lidar_max_correspondence_distance) ||
       !Positive(config_.lidar_observed_singular_value) ||
       !Positive(config_.lidar_innovation_gate) ||
       !Positive(config_.lidar_max_scan_age) ||
       !Nonnegative(config_.lidar_future_tolerance) ||
       config_.lidar_time_tolerance != 0.0 ||
       !Positive(config_.lidar_max_translation_correction_rate) ||
       !Positive(config_.lidar_max_rotation_correction_rate) ||
       !Positive(config_.lidar_max_velocity_correction_acceleration) ||
       !Positive(config_.lidar_anchor_variance_scale) ||
       !Positive(config_.lidar_correction_variance_scale) ||
       !Positive(config_.lidar_max_translation_correction) ||
       !Positive(config_.lidar_max_rotation_correction) ||
       !Positive(config_.lidar_max_velocity_correction) ||
       config_.lidar_max_update_iterations == 0 ||
       !Nonnegative(config_.lidar_process_noise_psd_tolerance) ||
       !Positive(config_.lidar_rotation_length))) {
    return {Reason::CONFIG_INVALID,
            "LiDAR needs explicit calibration, noise, storage and dynamics budgets"};
  }
  if (config_.enable_lidar &&
      (!std::isfinite(config_.lidar_point_noise_std *
                      config_.lidar_point_noise_std) ||
       !std::isfinite(config_.lidar_plane_noise_std *
                      config_.lidar_plane_noise_std) ||
       !std::isfinite(config_.lidar_voxel_size *
                      config_.lidar_voxel_size))) {
    return {Reason::CONFIG_INVALID,
            "LiDAR noise or geometry scale is unrepresentable"};
  }
  const double standard_deviations[] = {
      config_.gyro_noise, config_.accel_noise, config_.gyro_bias_noise,
      config_.accel_bias_noise, config_.initial_velocity_std,
      config_.initial_attitude_std, config_.initial_gyro_bias_std,
      config_.initial_accel_bias_std, config_.max_position_std,
      config_.max_velocity_std, config_.max_attitude_std,
      std::min(1e-6, config_.max_position_std * 0.5)};
  for (double value : standard_deviations) {
    if (!std::isfinite(value * value) ||
        (value > 0.0 && value * value == 0.0)) {
      return {Reason::CONFIG_INVALID, "Unrepresentable covariance scale"};
    }
  }
  return {};
}

Result LocalEstimator::Reject(Reason reason, const std::string& message,
                              SourceCounters* counters) {
  ++counters->rejected;
  counters->reason = reason;
  if (reason == Reason::DUPLICATE) {
    ++counters->duplicates;
  } else if (reason == Reason::TIMESTAMP_REGRESSION) {
    ++counters->regressions;
  }
  return {reason, message};
}

Result LocalEstimator::AddWheel(const WheelSample& sample) {
  Result result = ValidateConfig();
  if (result.ok()) {
    result = CheckStamp(sample.stamp, config_, wheel_health_,
                        last_wheel_sequence_, Reason::WHEEL_STALE);
  }
  if (!result.ok()) {
    return Reject(result.reason, result.message, &wheel_health_);
  }
  if (!std::isfinite(sample.speed) || !Positive(sample.variance)) {
    return Reject(Reason::INVALID_INPUT, "Invalid wheel speed or variance",
                  &wheel_health_);
  }
  if (sample.stamp.receive_time - sample.stamp.time > config_.wheel_timeout) {
    return Reject(Reason::WHEEL_STALE, "Wheel sample exceeded wheel freshness",
                  &wheel_health_);
  }
  // Only the next forward IMU interval is supported, not delayed replay.
  if (has_imu_ && sample.stamp.time < previous_imu_.stamp.time) {
    return Reject(Reason::HISTORY_UNAVAILABLE,
                  "Wheel sample predates the committed IMU frontier",
                  &wheel_health_);
  }
  if (has_imu_ &&
      !faulted_ &&
      sample.stamp.time - previous_imu_.stamp.time > config_.max_imu_gap) {
    return Reject(Reason::INVALID_INPUT,
                  "Wheel sample exceeds the next bounded IMU interval",
                  &wheel_health_);
  }
  if (pending_wheels_.size() >= kMaxPendingWheelSamples) {
    ++wheel_health_.evictions;
    return Reject(Reason::INVALID_INPUT,
                  "Pending wheel queue exceeded its fixed capacity",
                  &wheel_health_);
  }
  pending_wheels_.push_back(sample);
  last_received_wheel_ = sample;
  has_received_wheel_ = true;
  last_wheel_sequence_ = sample.stamp.sequence;
  Accept(sample.stamp, &wheel_health_);
  return {};
}

Result LocalEstimator::AddImu(const ImuSample& sample) {
  Result result = ValidateConfig();
  if (result.ok()) {
    result = CheckStamp(sample.stamp, config_, imu_health_, last_imu_sequence_,
                        Reason::IMU_STALE);
  }
  if (!result.ok()) {
    return Reject(result.reason, result.message, &imu_health_);
  }
  if (!sample.acceleration.allFinite() ||
      !sample.angular_velocity.allFinite() ||
      !std::isfinite(sample.acceleration.norm()) ||
      !std::isfinite(sample.angular_velocity.norm())) {
    return Reject(Reason::INVALID_INPUT, "Nonfinite IMU measurement",
                  &imu_health_);
  }
  const bool gap =
      has_imu_ &&
      sample.stamp.time - previous_imu_.stamp.time > config_.max_imu_gap;
  const ImuSample interval_start = previous_imu_;
  const bool had_imu = has_imu_;
  if (gap) {
    faulted_ = true;
    has_motion_ = false;
    motion_status_ = {Reason::IMU_GAP, "Motion cannot span an IMU gap"};
  }
  ConsumeStationaryWheels(sample.stamp.time);
  AccumulateStationary(sample, gap);
  previous_imu_ = sample;
  has_imu_ = true;
  last_imu_sequence_ = sample.stamp.sequence;
  Accept(sample.stamp, &imu_health_);

  if (gap) {
    state_.valid = false;
    output_state_.valid = false;
    imu_health_.reason = Reason::IMU_GAP;
    return {Reason::IMU_GAP, "IMU gap requires explicit stopped reset"};
  }
  if (faulted_) {
    imu_health_.reason = Reason::IMU_GAP;
    return {Reason::IMU_GAP, "Only stopped reset can recover the IMU fault"};
  }
  if (!initialized_) {
    if (stationary_count_ < config_.stationary_samples) {
      state_.stamp = sample.stamp;
      state_.valid = false;
      output_state_.stamp = sample.stamp;
      output_state_.valid = false;
      RecordHistory();
      return {};
    }
    result = Initialize(sample);
    if (!result.ok()) {
      stationary_count_ = 0;
      stationary_acceleration_.setZero();
      stationary_rotation_.setZero();
      return Reject(result.reason, result.message, &imu_health_);
    }
  } else if (had_imu) {
    const LocalState start_state = state_;
    const LocalState start_output = output_state_;
    LocalState working = state_;
    Matrix15d error_transition = Matrix15d::Identity();
    std::vector<SourceSampleId> sources{
        {"imu", interval_start.stamp.sequence}, {"imu", sample.stamp.sequence}};
    sources.insert(sources.end(), pending_lidar_sources_.begin(),
                   pending_lidar_sources_.end());
    ImuSample cursor = interval_start;
    Result wheel_result;
    while (!pending_wheels_.empty() &&
           pending_wheels_.front().stamp.time <= sample.stamp.time) {
      const WheelSample wheel = pending_wheels_.front();
      pending_wheels_.pop_front();
      const ImuSample at_wheel =
          InterpolateImu(interval_start, sample, wheel.stamp.time);
      result = Propagate(cursor, at_wheel, &working, &error_transition);
      if (!result.ok()) {
        faulted_ = true;
        state_.valid = false;
        has_motion_ = false;
        motion_status_ = result;
        imu_health_.reason = result.reason;
        return result;
      }
      cursor = at_wheel;
      if (sample.stamp.receive_time - wheel.stamp.time >
          config_.wheel_timeout) {
        result = Reject(Reason::WHEEL_STALE,
                        "Buffered wheel exceeded the causal freshness window",
                        &wheel_health_);
      } else {
        result = UpdateWheel(wheel, at_wheel, &working, &error_transition);
        if (result.ok()) {
          sources.push_back({"wheel", wheel.stamp.sequence});
        }
      }
      if (!result.ok() && wheel_result.ok()) {
        wheel_result = result;
      }
    }
    result = Propagate(cursor, sample, &working, &error_transition);
    if (!result.ok()) {
      faulted_ = true;
      state_.valid = false;
      has_motion_ = false;
      motion_status_ = result;
      imu_health_.reason = result.reason;
      return result;
    }
    working.stamp = sample.stamp;
    working.angular_velocity =
        sample.angular_velocity - working.gyro_bias;
    state_ = working;
    state_.valid = true;
    if (lio_active_) {
      const Result output = continuous_output_.Advance(
          start_state, state_, error_transition,
          sample.stamp.time - start_state.stamp.time);
      if (!output.ok()) {
        state_.valid = false;
        faulted_ = true;
        return output;
      }
      output_state_ = continuous_output_.state();
    } else {
      output_state_ = state_;
      continuous_output_.Reset(output_state_);
    }
    state_.valid = EvaluateIntegrity(sample.stamp.receive_time).ok();
    output_state_.valid = state_.valid;
    if (!start_output.covariance_model_valid ||
        !output_state_.covariance_model_valid) {
      has_motion_ = false;
      motion_status_ = {
          Reason::CORRELATION_UNQUALIFIED,
          "Local covariance model is invalid for joint motion export"};
    } else {
      motion_status_ = ComputeMotionIncrement(
          start_output, output_state_,
          start_output.covariance * error_transition.transpose(),
          sources, &last_motion_);
      has_motion_ = motion_status_.ok();
    }
    RecordHistory(error_transition, sources,
                  start_output.covariance_model_valid &&
                      output_state_.covariance_model_valid);
    RecordInternalHistory();
    pending_lidar_sources_.clear();
    if (!wheel_result.ok()) {
      return wheel_result;
    }
    return Evaluate(sample.stamp.receive_time);
  }

  state_.valid = initialized_;
  output_state_ = state_;
  continuous_output_.Reset(output_state_);
  state_.valid = EvaluateIntegrity(sample.stamp.receive_time).ok();
  output_state_.valid = state_.valid;
  RecordHistory();
  RecordInternalHistory();
  return Evaluate(sample.stamp.receive_time);
}

void LocalEstimator::ConsumeStationaryWheels(double time) {
  if (initialized_ && !faulted_) {
    return;
  }
  while (!pending_wheels_.empty() &&
         pending_wheels_.front().stamp.time <= time) {
    stationary_wheel_ = pending_wheels_.front();
    has_stationary_wheel_ = true;
    pending_wheels_.pop_front();
  }
}

bool LocalEstimator::IsStationary(const ImuSample& sample) const {
  return has_stationary_wheel_ &&
         stationary_wheel_.stamp.time <= sample.stamp.time &&
         sample.stamp.time - stationary_wheel_.stamp.time <=
             config_.wheel_timeout &&
         sample.stamp.receive_time - stationary_wheel_.stamp.time <=
             config_.wheel_timeout &&
         std::abs(stationary_wheel_.speed) <= config_.stationary_speed &&
         sample.angular_velocity.norm() <= config_.stationary_gyro &&
         std::abs(sample.acceleration.norm() - config_.gravity) <=
             config_.stationary_accel_tolerance;
}

void LocalEstimator::AccumulateStationary(const ImuSample& sample, bool reset) {
  if (reset || !IsStationary(sample)) {
    stationary_count_ = 0;
    stationary_acceleration_.setZero();
    stationary_rotation_.setZero();
  }
  if (IsStationary(sample) &&
      stationary_count_ < config_.stationary_samples) {
    ++stationary_count_;
    stationary_acceleration_ += sample.acceleration;
    stationary_rotation_ += sample.angular_velocity;
  }
}

Result LocalEstimator::Initialize(const ImuSample& sample) {
  LocalState initial;
  initial.epoch = state_.epoch;
  initial.stamp = sample.stamp;
  const Eigen::Vector3d acceleration =
      stationary_acceleration_ / stationary_count_;
  // Gravity constrains tilt only. Yaw is a local gauge, not an absolute heading.
  const double roll = std::atan2(acceleration.y(), acceleration.z());
  const double pitch =
      std::atan2(-acceleration.x(),
                 std::hypot(acceleration.y(), acceleration.z()));
  initial.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  initial.gyro_bias = stationary_rotation_ / stationary_count_;
  initial.angular_velocity = sample.angular_velocity - initial.gyro_bias;
  // Position is an arbitrary origin. A positive gauge covariance keeps P SPD.
  const double gauge_std = std::min(1e-6, config_.max_position_std * 0.5);
  initial.covariance.diagonal().segment<3>(0).setConstant(gauge_std * gauge_std);
  initial.covariance.diagonal().segment<3>(3).setConstant(
      config_.initial_velocity_std * config_.initial_velocity_std);
  initial.covariance.diagonal().segment<3>(6).setConstant(
      config_.initial_attitude_std * config_.initial_attitude_std);
  initial.covariance.diagonal().segment<3>(9).setConstant(
      config_.initial_gyro_bias_std * config_.initial_gyro_bias_std);
  initial.covariance.diagonal().segment<3>(12).setConstant(
      config_.initial_accel_bias_std * config_.initial_accel_bias_std);
  if (!FiniteState(initial)) {
    return {Reason::INVALID_INPUT, "Stationary initialization is not finite SPD"};
  }
  state_ = initial;
  output_state_ = initial;
  continuous_output_.Reset(output_state_);
  initialized_ = true;
  last_trusted_wheel_ = stationary_wheel_;
  has_trusted_wheel_ = true;
  return {};
}

Result LocalEstimator::Propagate(const ImuSample& start,
                                 const ImuSample& end,
                                 LocalState* state,
                                 Matrix15d* error_transition) const {
  // Inputs are already in body axes at the motion reference; the adapter owns
  // sensor rotation and lever-arm compensation.
  const double dt = end.stamp.time - start.stamp.time;
  if (dt < 0.0 || dt > config_.max_imu_gap) {
    return {Reason::INVALID_INPUT, "Invalid bounded IMU propagation interval"};
  }
  if (dt == 0.0) {
    state->angular_velocity = end.angular_velocity - state->gyro_bias;
    return {};
  }
  const Eigen::Vector3d acceleration =
      (0.5 * (start.acceleration + end.acceleration)).eval() -
      state->accel_bias;
  const Eigen::Vector3d rotation =
      (0.5 * (start.angular_velocity + end.angular_velocity)).eval() -
      state->gyro_bias;
  const Eigen::Quaterniond midpoint =
      (state->orientation * ExpRotation(rotation * (0.5 * dt))).normalized();
  const Eigen::Matrix3d body_to_local = midpoint.toRotationMatrix();
  const Eigen::Vector3d local_acceleration =
      (body_to_local * acceleration).eval() -
      Eigen::Vector3d(0.0, 0.0, config_.gravity);
  LocalState next = *state;
  next.stamp = end.stamp;
  next.position +=
      state->velocity * dt + 0.5 * local_acceleration * dt * dt;
  next.velocity += local_acceleration * dt;
  next.orientation =
      (state->orientation * ExpRotation(rotation * dt)).normalized();
  next.angular_velocity = end.angular_velocity - state->gyro_bias;

  // Right-multiplicative body-frame attitude error: [p, v, theta, bg, ba].
  Matrix15d dynamics = Matrix15d::Zero();
  dynamics.block<3, 3>(0, 3).setIdentity();
  dynamics.block<3, 3>(3, 6) = -body_to_local * Skew(acceleration);
  dynamics.block<3, 3>(3, 12) = -body_to_local;
  dynamics.block<3, 3>(6, 6) = -Skew(rotation);
  dynamics.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity();
  const Matrix15d squared = dynamics * dynamics;
  const Matrix15d transition =
      Matrix15d::Identity() + dynamics * dt + 0.5 * squared * dt * dt;
  const Matrix15d half_transition =
      Matrix15d::Identity() + dynamics * (0.5 * dt) +
      squared * (0.125 * dt * dt);
  Matrix15d noise = Matrix15d::Zero();
  noise.block<3, 3>(3, 3) =
      body_to_local * body_to_local.transpose() *
      (config_.accel_noise * config_.accel_noise);
  noise.block<3, 3>(6, 6).diagonal().setConstant(
      config_.gyro_noise * config_.gyro_noise);
  noise.block<3, 3>(9, 9).diagonal().setConstant(
      config_.gyro_bias_noise * config_.gyro_bias_noise);
  noise.block<3, 3>(12, 12).diagonal().setConstant(
      config_.accel_bias_noise * config_.accel_bias_noise);
  const Matrix15d discrete_noise =
      (dt / 6.0) *
      (noise + 4.0 * half_transition * noise * half_transition.transpose() +
       transition * noise * transition.transpose());
  next.covariance =
      transition * state->covariance * transition.transpose() + discrete_noise;
  next.covariance =
      (0.5 * (next.covariance + next.covariance.transpose())).eval();
  if (!FiniteState(next)) {
    return {Reason::INVALID_INPUT, "IMU propagation is not finite SPD"};
  }
  *state = next;
  *error_transition = (transition * *error_transition).eval();
  return {};
}

Result LocalEstimator::UpdateWheel(const WheelSample& sample,
                                   const ImuSample& imu,
                                   LocalState* state,
                                   Matrix15d* error_transition) {
  const int dimension = config_.use_nonholonomic_constraint ? 3 : 1;
  const Eigen::Matrix3d local_to_body =
      state->orientation.toRotationMatrix().transpose();
  const Eigen::Vector3d body_velocity =
      (local_to_body * state->velocity).eval();
  Eigen::MatrixXd observation = Eigen::MatrixXd::Zero(dimension, 15);
  observation.block(0, 3, dimension, 3) = local_to_body.topRows(dimension);
  observation.block(0, 6, dimension, 3) =
      Skew(body_velocity).topRows(dimension);
  Eigen::VectorXd innovation = -body_velocity.head(dimension);
  innovation(0) += sample.speed;
  Eigen::MatrixXd measurement_noise =
      Eigen::MatrixXd::Identity(dimension, dimension) *
      config_.nonholonomic_variance;
  measurement_noise(0, 0) = sample.variance;
  const Eigen::MatrixXd innovation_covariance =
      observation * state->covariance * observation.transpose() +
      measurement_noise;
  Eigen::LLT<Eigen::MatrixXd> factor(innovation_covariance);
  if (factor.info() != Eigen::Success) {
    return Reject(Reason::INVALID_INPUT, "Wheel innovation covariance is invalid",
                  &wheel_health_);
  }
  const double distance = innovation.dot(factor.solve(innovation));
  if (!std::isfinite(distance) || distance > config_.innovation_gate) {
    return Reject(Reason::INNOVATION_REJECTED, "Wheel innovation exceeded gate",
                  &wheel_health_);
  }
  Eigen::MatrixXd gain =
      factor.solve(observation * state->covariance).transpose();
  // A constrained gain preserves the local pose exactly, including orientation.
  gain.block(0, 0, 3, dimension).setZero();
  gain.block(6, 0, 3, dimension).setZero();
  const Eigen::Matrix<double, 15, 1> correction = gain * innovation;
  LocalState next = *state;
  next.velocity += correction.segment<3>(3);
  next.gyro_bias += correction.segment<3>(9);
  next.accel_bias += correction.segment<3>(12);
  next.angular_velocity = imu.angular_velocity - next.gyro_bias;
  const Matrix15d residual = Matrix15d::Identity() - gain * observation;
  next.covariance = residual * state->covariance * residual.transpose() +
                    gain * measurement_noise * gain.transpose();
  next.covariance =
      (0.5 * (next.covariance + next.covariance.transpose())).eval();
  if (!FiniteState(next)) {
    return Reject(Reason::INVALID_INPUT, "Wheel update is not finite SPD",
                  &wheel_health_);
  }
  *state = next;
  *error_transition = (residual * *error_transition).eval();
  last_trusted_wheel_ = sample;
  has_trusted_wheel_ = true;
  wheel_health_.reason = Reason::NONE;
  return {};
}

Result LocalEstimator::EvaluateIntegrity(double now) const {
  const Result config = ValidateConfig();
  if (!config.ok()) {
    return config;
  }
  if (!Positive(now)) {
    return {Reason::INVALID_INPUT, "Evaluation time is invalid"};
  }
  if (faulted_) {
    return {Reason::IMU_GAP, "Explicit stopped reset is required"};
  }
  if (!has_imu_) {
    return {Reason::WAITING_FOR_IMU, "No actual IMU evidence"};
  }
  if (!initialized_) {
    return {Reason::WAITING_FOR_STATIONARY, "Stationary alignment is incomplete"};
  }
  if (previous_imu_.stamp.clock_id != config_.clock_id ||
      state_.stamp.clock_id != config_.clock_id) {
    return {Reason::CLOCK_INVALID, "State evidence clock does not match"};
  }
  if (previous_imu_.stamp.time - now > config_.future_tolerance ||
      previous_imu_.stamp.receive_time - now > config_.future_tolerance) {
    return {Reason::INVALID_INPUT, "Evaluation precedes available evidence"};
  }
  if (now - previous_imu_.stamp.time > config_.max_sample_age) {
    return {Reason::IMU_STALE, "IMU evidence expired"};
  }
  if (state_.stamp.time != previous_imu_.stamp.time ||
      state_.stamp.sequence != previous_imu_.stamp.sequence ||
      output_state_.stamp.time != previous_imu_.stamp.time ||
      output_state_.stamp.sequence != previous_imu_.stamp.sequence) {
    return {Reason::INVALID_INPUT, "State is not committed at the IMU frontier"};
  }
  if (!FiniteState(state_) || !FiniteState(output_state_)) {
    return {Reason::INVALID_INPUT, "State or full error covariance is invalid"};
  }
  return {};
}

Result LocalEstimator::Evaluate(double now) const {
  const Result integrity = EvaluateIntegrity(now);
  if (!integrity.ok()) {
    return integrity;
  }
  if (lio_active_ && continuous_output_.correction_budget_exceeded()) {
    return {Reason::CORRECTION_BUDGET_EXCEEDED,
            "Local geometry correction exceeds the declared output envelope"};
  }
  if (!output_state_.covariance_model_valid) {
    return {Reason::CORRELATION_UNQUALIFIED,
            "Local covariance does not follow a qualified declared model"};
  }
  // IMU and wheel dead reckoning preserves a continuous local gauge, but
  // cannot guarantee long-duration position or heading accuracy by itself.
  if (!has_trusted_wheel_ ||
      last_trusted_wheel_.stamp.time - now > config_.future_tolerance ||
      last_trusted_wheel_.stamp.receive_time - now > config_.future_tolerance) {
    return {Reason::WHEEL_STALE, "No trusted causal wheel evidence"};
  }
  if (now - last_trusted_wheel_.stamp.time > config_.wheel_timeout) {
    return {Reason::WHEEL_STALE, "No fresh causal wheel evidence"};
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (std::sqrt(output_state_.covariance(axis, axis)) >
            config_.max_position_std ||
        std::sqrt(output_state_.covariance(axis + 3, axis + 3)) >
            config_.max_velocity_std ||
        std::sqrt(output_state_.covariance(axis + 6, axis + 6)) >
            config_.max_attitude_std) {
      return {Reason::COVARIANCE_EXCEEDED, "Per-axis local sigma budget exceeded"};
    }
  }
  return {};
}

Result LocalEstimator::ResetStopped(double now) {
  const Result config = ValidateConfig();
  if (!config.ok()) {
    return config;
  }
  if (!Positive(now)) {
    return {Reason::INVALID_INPUT, "Reset time is invalid"};
  }
  WheelSample reset_wheel = stationary_wheel_;
  bool has_reset_wheel = has_stationary_wheel_;
  if (has_received_wheel_ &&
      (!has_reset_wheel ||
       last_received_wheel_.stamp.time >= reset_wheel.stamp.time)) {
    reset_wheel = last_received_wheel_;
    has_reset_wheel = true;
  }
  if (!has_imu_ || !has_stationary_wheel_ ||
      stationary_count_ < config_.stationary_samples ||
      now - previous_imu_.stamp.time > config_.max_sample_age ||
      !has_reset_wheel ||
      now - reset_wheel.stamp.time > config_.wheel_timeout ||
      previous_imu_.stamp.time > now || reset_wheel.stamp.time > now ||
      previous_imu_.stamp.receive_time > now ||
      reset_wheel.stamp.receive_time > now ||
      std::abs(reset_wheel.speed) > config_.stationary_speed ||
      previous_imu_.angular_velocity.norm() > config_.stationary_gyro) {
    return {Reason::RESET_WHILE_MOVING,
            "Reset needs fresh independent stopped wheel and stationary IMU"};
  }
  if (state_.epoch.generation == std::numeric_limits<uint64_t>::max()) {
    return {Reason::EPOCH_MISMATCH, "Epoch generation is exhausted"};
  }
  const Epoch epoch{state_.epoch.session, state_.epoch.generation + 1};
  const Stamp reset_stamp = previous_imu_.stamp;
  state_ = LocalState();
  state_.epoch = epoch;
  state_.stamp = reset_stamp;
  output_state_ = state_;
  initialized_ = false;
  faulted_ = false;
  has_imu_ = false;
  has_received_wheel_ = false;
  has_stationary_wheel_ = false;
  has_trusted_wheel_ = false;
  previous_imu_ = ImuSample();
  last_received_wheel_ = WheelSample();
  stationary_wheel_ = WheelSample();
  last_trusted_wheel_ = WheelSample();
  stationary_count_ = 0;
  stationary_acceleration_.setZero();
  stationary_rotation_.setZero();
  pending_wheels_.clear();
  history_.clear();
  internal_history_.clear();
  lio_.Reset(epoch);
  continuous_output_.Reset(output_state_);
  lio_active_ = false;
  pending_lidar_sources_.clear();
  has_motion_ = false;
  last_motion_ = MotionIncrement();
  motion_status_ = {Reason::HISTORY_UNAVAILABLE, "ODOM reset clears motion"};
  imu_health_.reason = Reason::NONE;
  wheel_health_.reason = Reason::NONE;
  return {};
}

void LocalEstimator::RecordHistory(
    const Matrix15d& transition, const std::vector<SourceSampleId>& sources,
    bool motion_qualified) {
  if (!state_.valid) {
    return;
  }
  history_.push_back(
      {output_state_, transition, sources, motion_qualified});
  while (!history_.empty() &&
         (output_state_.stamp.time - history_.front().state.stamp.time >
              config_.history_duration ||
          history_.size() > config_.max_history_states)) {
    history_.pop_front();
  }
}

void LocalEstimator::RecordInternalHistory() {
  if (!state_.valid) {
    return;
  }
  if (!internal_history_.empty() &&
      internal_history_.back().stamp.time == state_.stamp.time) {
    internal_history_.back() = state_;
  } else {
    internal_history_.push_back(state_);
  }
  while (!internal_history_.empty() &&
         (state_.stamp.time - internal_history_.front().stamp.time >
              config_.history_duration ||
          internal_history_.size() > config_.max_history_states)) {
    internal_history_.pop_front();
  }
}

Result LocalEstimator::LookupInternal(double time, LocalState* state) const {
  if (state == nullptr || !std::isfinite(time) || time <= 0.0 ||
      internal_history_.empty()) {
    return {Reason::HISTORY_UNAVAILABLE, "Invalid internal history request"};
  }
  const auto upper = std::lower_bound(
      internal_history_.begin(), internal_history_.end(), time,
      [](const LocalState& item, double stamp) {
        return item.stamp.time < stamp;
      });
  if (upper != internal_history_.end() && upper->stamp.time == time) {
    *state = *upper;
    return {};
  }
  if (upper == internal_history_.begin() || upper == internal_history_.end()) {
    return {Reason::HISTORY_UNAVAILABLE,
            "LiDAR point is outside bounded IMU history"};
  }
  const auto lower = std::prev(upper);
  const double dt = upper->stamp.time - lower->stamp.time;
  if (dt <= 0.0 || dt > config_.max_imu_gap) {
    return {Reason::IMU_GAP, "LiDAR point brackets an IMU gap"};
  }
  const double alpha = (time - lower->stamp.time) / dt;
  LocalState interpolated = *lower;
  interpolated.stamp.time = time;
  interpolated.position =
      (1.0 - alpha) * lower->position + alpha * upper->position;
  interpolated.velocity =
      (1.0 - alpha) * lower->velocity + alpha * upper->velocity;
  interpolated.orientation =
      lower->orientation.slerp(alpha, upper->orientation).normalized();
  interpolated.gyro_bias =
      (1.0 - alpha) * lower->gyro_bias + alpha * upper->gyro_bias;
  interpolated.accel_bias =
      (1.0 - alpha) * lower->accel_bias + alpha * upper->accel_bias;
  interpolated.angular_velocity =
      (1.0 - alpha) * lower->angular_velocity +
      alpha * upper->angular_velocity;
  interpolated.covariance =
      (1.0 - alpha) * lower->covariance + alpha * upper->covariance;
  interpolated.covariance_model_valid =
      lower->covariance_model_valid && upper->covariance_model_valid;
  *state = interpolated;
  return {};
}

Result LocalEstimator::AddLidar(const LidarScan& scan) {
  if (!config_.enable_lidar) {
    return Reject(Reason::CONFIG_INVALID, "Local LiDAR is disabled",
                  &lidar_health_);
  }
  if (scan.stamp.clock_id != config_.clock_id ||
      !Positive(scan.stamp.time) || !Positive(scan.stamp.receive_time) ||
      scan.stamp.sequence == 0) {
    return Reject(Reason::INVALID_INPUT, "Invalid LiDAR measurement stamp",
                  &lidar_health_);
  }
  if (scan.stamp.time - scan.stamp.receive_time >
      config_.lidar_future_tolerance) {
    return Reject(Reason::INVALID_INPUT, "LiDAR scan is in the future",
                  &lidar_health_);
  }
  if (scan.stamp.receive_time - scan.stamp.time >
      config_.lidar_max_scan_age) {
    return Reject(Reason::LIDAR_STALE,
                  "LiDAR scan exceeded the causal age budget",
                  &lidar_health_);
  }
  if (lidar_health_.accepted != 0) {
    if (scan.stamp.sequence == lidar_health_.last_sequence ||
        scan.stamp.time == lidar_health_.last_measurement) {
      return Reject(Reason::DUPLICATE,
                    "LiDAR scan frontier did not advance",
                    &lidar_health_);
    }
    if (scan.stamp.sequence < lidar_health_.last_sequence ||
        scan.stamp.time < lidar_health_.last_measurement ||
        scan.stamp.receive_time < lidar_health_.last_receive) {
      return Reject(Reason::TIMESTAMP_REGRESSION,
                    "LiDAR source time or sequence regressed",
                    &lidar_health_);
    }
  }
  if (!state_.valid || scan.epoch != state_.epoch) {
    return Reject(Reason::EPOCH_MISMATCH,
                  "LiDAR scan does not match the active local epoch",
                  &lidar_health_);
  }
  if (scan.calibration_id != config_.lidar_calibration_id) {
    return Reject(Reason::CONFIG_INVALID,
                  "LiDAR calibration identity does not match",
                  &lidar_health_);
  }
  if (scan.frame_id != config_.lidar_frame) {
    return Reject(Reason::CONFIG_INVALID,
                  "LiDAR sensor frame does not match",
                  &lidar_health_);
  }
  if (scan.points.size() > config_.lidar_max_points) {
    return Reject(Reason::INVALID_INPUT,
                  "LiDAR scan exceeds the configured point bound",
                  &lidar_health_);
  }
  if (scan.stamp.time != state_.stamp.time) {
    return Reject(Reason::LIDAR_STALE,
                  "LiDAR correction requires the exact committed IMU frontier",
                  &lidar_health_);
  }
  Result result = lio_.Observe(
      scan,
      [this](double time, LocalState* item) {
        return LookupInternal(time, item);
      },
      &state_);
  if (!result.ok() && result.reason != Reason::GEOMETRY_UNAVAILABLE) {
    return Reject(result.reason, result.message, &lidar_health_);
  }
  Accept(scan.stamp, &lidar_health_);
  if (result.ok()) {
    lio_active_ = true;
    state_.covariance_model_valid = false;
    output_state_.covariance_model_valid = false;
    has_motion_ = false;
    motion_status_ = {
        Reason::CORRELATION_UNQUALIFIED,
        "LIO correction invalidated the declared joint covariance model"};
    if (!history_.empty() &&
        history_.back().state.stamp.time == output_state_.stamp.time) {
      history_.back().state.covariance_model_valid = false;
      history_.back().motion_qualified = false;
    }
    pending_lidar_sources_.clear();
    for (uint64_t sequence : lio_.geometry_source_sequences()) {
      pending_lidar_sources_.push_back(
          {"lidar_local_geometry", sequence});
    }
  } else {
    lidar_health_.reason = result.reason;
  }
  RecordInternalHistory();
  return result;
}

Result LocalEstimator::Lookup(double time, LocalState* state) const {
  if (state == nullptr || !std::isfinite(time) || time <= 0.0) {
    return {Reason::INVALID_INPUT, "Invalid local history request"};
  }
  const auto found = std::lower_bound(
      history_.begin(), history_.end(), time,
      [](const HistoryEntry& entry, double stamp) {
        return entry.state.stamp.time < stamp;
      });
  if (found == history_.end() || found->state.stamp.time != time) {
    return {Reason::HISTORY_UNAVAILABLE, "No exact committed local state"};
  }
  *state = found->state;
  return {};
}

Result LocalEstimator::MotionBetween(double start_time, double end_time,
                                     MotionIncrement* motion) const {
  if (motion == nullptr || !std::isfinite(start_time) ||
      !std::isfinite(end_time) || start_time <= 0.0 || end_time <= start_time) {
    return {Reason::INVALID_INPUT, "Invalid local motion interval"};
  }
  LocalState start;
  LocalState end;
  auto status = Lookup(start_time, &start);
  if (!status.ok()) {
    return status;
  }
  status = Lookup(end_time, &end);
  if (!status.ok()) {
    return status;
  }
  if (!start.covariance_model_valid || !end.covariance_model_valid) {
    return {
        Reason::CORRELATION_UNQUALIFIED,
        "Motion endpoint covariance model is invalid"};
  }
  Matrix15d transition = Matrix15d::Identity();
  std::set<std::pair<std::string, uint64_t>> identities;
  std::vector<SourceSampleId> sources;
  for (const auto& entry : history_) {
    if (entry.state.stamp.time <= start_time) {
      continue;
    }
    if (entry.state.stamp.time > end_time) {
      break;
    }
    if (!entry.motion_qualified) {
      return {
          Reason::CORRELATION_UNQUALIFIED,
          "Motion interval crosses an unqualified LiDAR output update"};
    }
    transition = (entry.transition * transition).eval();
    for (const auto& sample : entry.sources) {
      if (identities.emplace(sample.source, sample.sequence).second) {
        sources.push_back(sample);
      }
    }
  }
  return ComputeMotionIncrement(
      start, end, start.covariance * transition.transpose(), sources, motion);
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
