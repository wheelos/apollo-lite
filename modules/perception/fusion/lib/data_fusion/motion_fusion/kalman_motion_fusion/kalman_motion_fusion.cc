/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/fusion/lib/data_fusion/motion_fusion/kalman_motion_fusion/kalman_motion_fusion.h"

#include <cmath>

#include "cyber/common/log.h"
#include "modules/perception/fusion/base/observation_validation.h"
#include "modules/perception/fusion/base/track.h"

namespace apollo {
namespace perception {
namespace fusion {
bool KalmanMotionFusion::ValidateMeasurement(
    const SensorObjectConstPtr& measurement) const {
  if (!measurement || !std::isfinite(measurement->GetTimestamp())) {
    AERROR << "Invalid motion measurement timestamp.";
    return false;
  }
  const auto& object = measurement->GetBaseObject();
  if (!object || !ValidPlanarObservation(*object)) {
    AERROR << "Invalid motion measurement or covariance.";
    return false;
  }
  return true;
}

bool KalmanMotionFusion::Init() {
  if (!track_ref_ || !std::isfinite(jerk_variance_) || jerk_variance_ <= 0 ||
      !std::isfinite(covariance_floor_) || covariance_floor_ <= 0) {
    AERROR << "Invalid motion estimator configuration.";
    return false;
  }
  if (!std::isfinite(prior_weight_) || prior_weight_ <= 0 || prior_weight_ >= 1) {
    AERROR << "Covariance-intersection weight must be between zero and one.";
    return false;
  }
  SensorObjectConstPtr measurement = track_ref_->GetLatestLidarObject();
  if (!measurement) measurement = track_ref_->GetLatestRadarObject();
  if (!measurement) measurement = track_ref_->GetLatestCameraObject();
  if (!ValidateMeasurement(measurement)) return false;

  const auto& object = measurement->GetBaseObject();
  velocity_initialized_ = object->velocity_converged;
  Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
  state.head<2>() = object->center.head<2>();
  if (object->velocity_converged) {
    state.segment<2>(2) = object->velocity.head<2>().cast<double>();
  }
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(6, 6);
  covariance.topLeftCorner<2, 2>() =
      object->center_uncertainty.topLeftCorner<2, 2>().cast<double>();
  covariance.topLeftCorner<2, 2>().diagonal().array() += covariance_floor_;
  if (object->velocity_converged) {
    covariance.block<2, 2>(2, 2) =
        object->velocity_uncertainty.topLeftCorner<2, 2>().cast<double>();
    covariance.block<2, 2>(2, 2).diagonal().array() += covariance_floor_;
  } else {
    covariance.block<2, 2>(2, 2) *= 100.0;
  }
  covariance.bottomRightCorner<2, 2>() *= jerk_variance_;
  initialized_ = kalman_filter_.Init(state, covariance);
  if (!initialized_) return false;
  state_timestamp_ = measurement->GetTimestamp();
  track_ref_->RecordMotionObservation(state_timestamp_);
  return UpdateMotionState();
}

bool KalmanMotionFusion::PredictTo(double timestamp) {
  if (!initialized_ || !std::isfinite(timestamp) ||
      timestamp < state_timestamp_) {
    AERROR << "Cannot predict fusion state backwards: " << timestamp
           << " state_time=" << state_timestamp_;
    return false;
  }
  const double dt = timestamp - state_timestamp_;
  if (dt == 0.0) return true;
  Eigen::MatrixXd transition = Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd noise = Eigen::MatrixXd::Zero(6, 6);
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  const double dt5 = dt4 * dt;
  for (int axis = 0; axis < 2; ++axis) {
    transition(axis, axis + 2) = dt;
    transition(axis, axis + 4) = 0.5 * dt2;
    transition(axis + 2, axis + 4) = dt;
    noise(axis, axis) = dt5 / 20.0;
    noise(axis, axis + 2) = noise(axis + 2, axis) = dt4 / 8.0;
    noise(axis, axis + 4) = noise(axis + 4, axis) = dt3 / 6.0;
    noise(axis + 2, axis + 2) = dt3 / 3.0;
    noise(axis + 2, axis + 4) = noise(axis + 4, axis + 2) = dt2 / 2.0;
    noise(axis + 4, axis + 4) = dt;
  }
  if (!kalman_filter_.Predict(transition, noise * jerk_variance_)) {
    return false;
  }
  state_timestamp_ = timestamp;
  return UpdateMotionState();
}

bool KalmanMotionFusion::UpdateWithMeasurement(
    const SensorObjectConstPtr& measurement, double target_timestamp) {
  if (!ValidateMeasurement(measurement) ||
      target_timestamp != measurement->GetTimestamp() ||
      !PredictTo(measurement->GetTimestamp())) {
    AERROR << "Motion correction requires the measurement's state timestamp.";
    return false;
  }
  const auto& object = measurement->GetBaseObject();
  Eigen::VectorXd observation = Eigen::VectorXd::Zero(6);
  Eigen::MatrixXd projection = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(6, 6);
  observation.head<2>() = object->center.head<2>();
  projection.topLeftCorner<2, 2>().setIdentity();
  covariance.topLeftCorner<2, 2>() =
      object->center_uncertainty.topLeftCorner<2, 2>().cast<double>();
  covariance.topLeftCorner<2, 2>().diagonal().array() += covariance_floor_;
  if (object->velocity_converged) {
    observation.segment<2>(2) = object->velocity.head<2>().cast<double>();
    projection.block<2, 2>(2, 2).setIdentity();
    covariance.block<2, 2>(2, 2) =
        object->velocity_uncertainty.topLeftCorner<2, 2>().cast<double>();
    covariance.block<2, 2>(2, 2).diagonal().array() += covariance_floor_;
  }
  if (!kalman_filter_.SetControlMatrix(projection)) return false;
  const bool corrected = use_covariance_intersection_
      ? kalman_filter_.CorrectCorrelated(observation, covariance, prior_weight_)
      : kalman_filter_.Correct(observation, covariance);
  if (!corrected) {
    return false;
  }
  velocity_initialized_ = velocity_initialized_ || object->velocity_converged;
  if (IsLidar(measurement) ||
      (IsCamera(measurement) && !track_ref_->GetLatestLidarObject())) {
    auto fused = track_ref_->GetFusedObject()->GetBaseObject();
    const double displacement = object->center.z() - fused->center.z();
    fused->center.z() = object->center.z();
    fused->center_uncertainty(2, 2) = object->center_uncertainty(2, 2);
    for (auto& point : fused->polygon) point.z += displacement;
  }
  track_ref_->RecordMotionObservation(measurement->GetTimestamp());
  return UpdateMotionState();
}

bool KalmanMotionFusion::UpdateWithoutMeasurement(const std::string& sensor_id,
                                                  double measurement_timestamp,
                                                  double target_timestamp) {
  return PredictTo(target_timestamp);
}

bool KalmanMotionFusion::UpdateMotionState() {
  auto object = track_ref_->GetFusedObject()->GetBaseObject();
  const auto state = kalman_filter_.GetStates();
  const auto covariance = kalman_filter_.GetUncertainty();
  if (!state.allFinite() || !covariance.allFinite() ||
      !state.cast<float>().allFinite() ||
      !covariance.cast<float>().allFinite()) {
    AERROR << "Fusion motion posterior exceeds representable output limits.";
    return false;
  }
  Eigen::Vector3d center = object->center;
  center.head<2>() = state.head<2>();
  const Eigen::Vector3d displacement = center - object->center;
  for (auto& point : object->polygon) {
    point.x += displacement.x();
    point.y += displacement.y();
  }
  object->center = center;
  object->anchor_point = center;
  object->latest_tracked_time = state_timestamp_;
  object->velocity.head<2>() = state.segment<2>(2).cast<float>();
  object->velocity_converged = velocity_initialized_;
  object->acceleration.head<2>() = state.tail<2>().cast<float>();
  object->center_uncertainty.topLeftCorner<2, 2>() =
      covariance.topLeftCorner<2, 2>().cast<float>();
  object->velocity_uncertainty.topLeftCorner<2, 2>() =
      covariance.block<2, 2>(2, 2).cast<float>();
  object->acceleration_uncertainty.topLeftCorner<2, 2>() =
      covariance.bottomRightCorner<2, 2>().cast<float>();
  object->center_uncertainty.block<2, 1>(0, 2).setZero();
  object->center_uncertainty.block<1, 2>(2, 0).setZero();
  object->velocity_uncertainty.block<2, 1>(0, 2).setZero();
  object->velocity_uncertainty.block<1, 2>(2, 0).setZero();
  object->acceleration_uncertainty.block<2, 1>(0, 2).setZero();
  object->acceleration_uncertainty.block<1, 2>(2, 0).setZero();
  return true;
}

void KalmanMotionFusion::GetStates(Eigen::Vector3d* center,
                                   Eigen::Vector3d* velocity) {
  *center = track_ref_->GetFusedObject()->GetBaseObject()->center;
  *velocity =
      track_ref_->GetFusedObject()->GetBaseObject()->velocity.cast<double>();
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
