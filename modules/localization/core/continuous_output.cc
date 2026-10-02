// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "modules/localization/core/continuous_output.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Eigen/Cholesky"
#include "Eigen/Eigenvalues"

namespace apollo {
namespace localization {
namespace unified {
namespace {

Eigen::Vector3d Limited(const Eigen::Vector3d& value, double limit) {
  const double norm = value.norm();
  if (norm <= limit || norm == 0.0) {
    return value;
  }
  return value * (limit / norm);
}

}  // namespace

void ContinuousOutput::Reset(const LocalState& internal) {
  state_ = internal;
  active_ = internal.valid;
  pose_correction_velocity_.setZero();
  correction_budget_exceeded_ = false;
}

Result ContinuousOutput::Advance(const LocalState& previous_internal,
                                 const LocalState& internal,
                                 const Matrix15d& error_transition,
                                 double dt) {
  if (!internal.valid || !std::isfinite(dt) || dt <= 0.0 ||
      internal.epoch != state_.epoch ||
      previous_internal.epoch != internal.epoch ||
      !error_transition.allFinite()) {
    return {Reason::INVALID_INPUT,
            "Continuous output needs a valid same-epoch forward state"};
  }
  if (!active_) {
    Reset(internal);
    return {};
  }

  LocalState next = state_;
  const Eigen::Vector3d previous_position = state_.position;
  const Eigen::Quaterniond previous_orientation = state_.orientation;
  const Matrix15d start_covariance = state_.covariance;
  next.stamp = internal.stamp;
  next.covariance_model_valid =
      state_.covariance_model_valid &&
      previous_internal.covariance_model_valid &&
      internal.covariance_model_valid;
  const Eigen::Isometry3d propagated =
      Pose(state_) * Pose(previous_internal).inverse() * Pose(internal);
  next.position = propagated.translation();
  next.orientation = Eigen::Quaterniond(propagated.linear()).normalized();
  const Eigen::Vector3d propagated_pose_velocity =
      (next.position - previous_position) / dt;
  next.velocity += internal.velocity - previous_internal.velocity;
  next.gyro_bias += internal.gyro_bias - previous_internal.gyro_bias;
  next.accel_bias += internal.accel_bias - previous_internal.accel_bias;
  const Eigen::Vector3d translation_error =
      internal.position - next.position;
  const Eigen::Vector3d rotation_error =
      LogRotation(next.orientation.conjugate() * internal.orientation);
  const Eigen::Vector3d velocity_error =
      internal.velocity - next.velocity;
  correction_budget_exceeded_ =
      translation_error.norm() >
          config_.lidar_max_translation_correction ||
      rotation_error.norm() > config_.lidar_max_rotation_correction ||
      velocity_error.norm() > config_.lidar_max_velocity_correction;
  const Eigen::Vector3d target_pose_correction_velocity = Limited(
      translation_error / dt,
      config_.lidar_max_translation_correction_rate);
  const Eigen::Vector3d pose_correction_acceleration_step = Limited(
      target_pose_correction_velocity - pose_correction_velocity_,
      config_.lidar_max_velocity_correction_acceleration * dt);
  const Eigen::Vector3d next_pose_correction_velocity =
      pose_correction_velocity_ + pose_correction_acceleration_step;
  const Eigen::Vector3d translation_correction =
      0.5 * (pose_correction_velocity_ +
             next_pose_correction_velocity) *
      dt;
  const Eigen::Vector3d rotation_correction = Limited(
      rotation_error,
      config_.lidar_max_rotation_correction_rate * dt);
  next.position += translation_correction;
  next.orientation =
      (next.orientation *
       ExpRotation(rotation_correction))
          .normalized();
  const double transfer = std::clamp(dt, 0.0, 1.0);
  next.gyro_bias += transfer * (internal.gyro_bias - next.gyro_bias);
  next.accel_bias += transfer * (internal.accel_bias - next.accel_bias);
  next.velocity = (next.position - previous_position) / dt;
  next.angular_velocity =
      LogRotation(previous_orientation.conjugate() * next.orientation) / dt;
  const Eigen::Vector3d applied_velocity_correction =
      next.velocity - propagated_pose_velocity;
  correction_budget_exceeded_ =
      correction_budget_exceeded_ ||
      pose_correction_acceleration_step.norm() >
          config_.lidar_max_velocity_correction_acceleration * dt +
              std::numeric_limits<double>::epsilon();
  next.valid = true;
  Matrix15d process_noise =
      internal.covariance -
      error_transition * previous_internal.covariance *
          error_transition.transpose();
  process_noise =
      (0.5 * (process_noise + process_noise.transpose())).eval();
  Eigen::SelfAdjointEigenSolver<Matrix15d> noise_spectrum(process_noise);
  if (noise_spectrum.info() != Eigen::Success) {
    return {Reason::INVALID_INPUT,
            "Continuous-output process covariance decomposition failed"};
  }
  if (noise_spectrum.eigenvalues().minCoeff() <
      -config_.lidar_process_noise_psd_tolerance) {
    return {Reason::INVALID_INPUT,
            "Local transition implies non-PSD process covariance"};
  }
  const Eigen::VectorXd eigenvalues =
      noise_spectrum.eigenvalues().cwiseMax(0.0);
  process_noise = noise_spectrum.eigenvectors() *
                  eigenvalues.asDiagonal() *
                  noise_spectrum.eigenvectors().transpose();
  next.covariance =
      error_transition * start_covariance * error_transition.transpose() +
      process_noise;
  const double scale = config_.lidar_correction_variance_scale;
  next.covariance.diagonal().segment<3>(0) +=
      scale * translation_correction.cwiseProduct(translation_correction);
  next.covariance.diagonal().segment<3>(3) +=
      scale * applied_velocity_correction.cwiseProduct(
                  applied_velocity_correction);
  next.covariance.diagonal().segment<3>(6) +=
      scale * rotation_correction.cwiseProduct(rotation_correction);
  next.covariance =
      (0.5 * (next.covariance + next.covariance.transpose())).eval();
  if (!next.position.allFinite() ||
      !next.velocity.allFinite() || !next.orientation.coeffs().allFinite()) {
    return {Reason::INVALID_INPUT,
            "Continuous output update is nonfinite"};
  }
  if (Eigen::LLT<Matrix15d>(next.covariance).info() != Eigen::Success) {
    return {Reason::INVALID_INPUT,
            "Continuous output covariance is not SPD"};
  }
  state_ = next;
  pose_correction_velocity_ = next_pose_correction_velocity;
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
