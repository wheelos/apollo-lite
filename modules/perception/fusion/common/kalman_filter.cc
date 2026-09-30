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
#include "modules/perception/fusion/common/kalman_filter.h"

#include <cmath>

#include "Eigen/Cholesky"
#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace fusion {

KalmanFilter::KalmanFilter() : BaseFilter("KalmanFilter") {}

bool KalmanFilter::Init(const Eigen::VectorXd &initial_belief_states,
                        const Eigen::MatrixXd &initial_uncertainty) {
  if (initial_uncertainty.rows() != initial_uncertainty.cols()) {
    AERROR << "the cols and rows of uncertainty martix should be equal";
    return false;
  }
  const int dimension = static_cast<int>(initial_uncertainty.rows());

  if (dimension <= 0) {
    AERROR << "state_num should be greater than zero";
    return false;
  }

  if (dimension != initial_belief_states.rows()) {
    AERROR << "the rows of state should be equal to state_num";
    return false;
  }
  if (!initial_belief_states.allFinite() || !initial_uncertainty.allFinite() ||
      !initial_uncertainty.isApprox(initial_uncertainty.transpose(), 1e-8) ||
      !Eigen::LDLT<Eigen::MatrixXd>(initial_uncertainty).isPositive()) {
    AERROR << "Invalid initial filter state or covariance.";
    return false;
  }

  states_num_ = dimension;
  global_states_ = initial_belief_states;
  global_uncertainty_ = initial_uncertainty;
  prior_global_states_ = global_states_;

  transform_matrix_.setIdentity(states_num_, states_num_);
  cur_observation_.setZero(states_num_, 1);
  cur_observation_uncertainty_.setIdentity(states_num_, states_num_);

  c_matrix_.setIdentity(states_num_, states_num_);
  env_uncertainty_.setZero(states_num_, states_num_);

  gain_break_down_.setZero(states_num_, 1);
  value_break_down_.setZero(states_num_, 1);

  kalman_gain_.setZero(states_num_, states_num_);
  init_ = true;
  return true;
}

bool KalmanFilter::Predict(const Eigen::MatrixXd &transform_matrix,
                           const Eigen::MatrixXd &env_uncertainty_matrix) {
  if (!init_) {
    AERROR << "Predict: Kalman Filter initialize not successfully";
    return false;
  }
  if (transform_matrix.rows() != states_num_) {
    AERROR << "the rows of transform matrix should be equal to state_num";
    return false;
  }
  if (transform_matrix.cols() != states_num_) {
    AERROR << "the cols of transform matrix should be equal to state_num";
    return false;
  }
  if (env_uncertainty_matrix.rows() != states_num_) {
    AERROR << "the rows of env uncertainty should be equal to state_num";
    return false;
  }
  if (env_uncertainty_matrix.cols() != states_num_) {
    AERROR << "the cols of env uncertainty should be equal to state_num";
    return false;
  }
  if (!transform_matrix.allFinite() || !env_uncertainty_matrix.allFinite()) {
    AERROR << "Non-finite motion prediction matrices.";
    return false;
  }
  if (!env_uncertainty_matrix.isApprox(
      env_uncertainty_matrix.transpose(), 1e-8) ||
      !Eigen::LDLT<Eigen::MatrixXd>(env_uncertainty_matrix).isPositive()) {
    AERROR << "Invalid motion process covariance.";
    return false;
  }
  const Eigen::VectorXd state = transform_matrix * global_states_;
  const Eigen::MatrixXd covariance =
      transform_matrix * global_uncertainty_ * transform_matrix.transpose() +
      env_uncertainty_matrix;
  if (!state.allFinite() || !covariance.allFinite()) {
    AERROR << "Non-finite motion prediction result.";
    return false;
  }
  transform_matrix_ = transform_matrix;
  env_uncertainty_ = env_uncertainty_matrix;
  global_states_ = state;
  global_uncertainty_ = 0.5 * (covariance + covariance.transpose());
  return true;
}

bool KalmanFilter::Correct(const Eigen::VectorXd &cur_observation,
                           const Eigen::MatrixXd &cur_observation_uncertainty) {
  if (!init_) {
    AERROR << "Correct: Kalman Filter initialize not successfully";
    return false;
  }
  if (cur_observation.rows() != states_num_) {
    AERROR << "the rows of current observation should be equal to state_num";
    return false;
  }
  if (cur_observation_uncertainty.rows() != states_num_) {
    AERROR << "the rows of current observation uncertainty "
              "should be equal to state_num";
    return false;
  }
  if (cur_observation_uncertainty.cols() != states_num_) {
    AERROR << "the cols of current observation uncertainty "
              "should be equal to state_num";
    return false;
  }

  if (!cur_observation.allFinite() ||
      !cur_observation_uncertainty.allFinite() ||
      !cur_observation_uncertainty.isApprox(
          cur_observation_uncertainty.transpose(), 1e-8) ||
      !Eigen::LDLT<Eigen::MatrixXd>(
          cur_observation_uncertainty).isPositive()) {
    AERROR << "Non-finite filter observation.";
    return false;
  }
  const Eigen::MatrixXd innovation =
      c_matrix_ * global_uncertainty_ * c_matrix_.transpose() +
      cur_observation_uncertainty;
  Eigen::LDLT<Eigen::MatrixXd> solver(innovation);
  if (solver.info() != Eigen::Success || !solver.isPositive()) {
    AERROR << "Filter innovation covariance is not positive definite.";
    return false;
  }
  const Eigen::MatrixXd gain =
      solver.solve(c_matrix_ * global_uncertainty_).transpose();
  const Eigen::VectorXd state = global_states_ +
      gain * (cur_observation - c_matrix_ * global_states_);
  Eigen::MatrixXd tmp_identity;
  tmp_identity.setIdentity(states_num_, states_num_);
  const Eigen::MatrixXd covariance =
      (tmp_identity - gain * c_matrix_) * global_uncertainty_ *
          (tmp_identity - gain * c_matrix_).transpose() +
      gain * cur_observation_uncertainty * gain.transpose();
  if (!state.allFinite() || !covariance.allFinite()) {
    AERROR << "Non-finite filter correction result.";
    return false;
  }
  cur_observation_ = cur_observation;
  cur_observation_uncertainty_ = cur_observation_uncertainty;
  kalman_gain_ = gain;
  global_states_ = state;
  global_uncertainty_ = 0.5 * (covariance + covariance.transpose());
  return true;
}

bool KalmanFilter::CorrectCorrelated(const Eigen::VectorXd& observation,
                                     const Eigen::MatrixXd& covariance,
                                     double prior_weight) {
  if (!init_ || observation.size() != states_num_ ||
      covariance.rows() != states_num_ || covariance.cols() != states_num_ ||
      !observation.allFinite() || !covariance.allFinite() ||
      !covariance.isApprox(covariance.transpose(), 1e-8) ||
      !std::isfinite(prior_weight) || prior_weight <= 0 || prior_weight >= 1) {
    AERROR << "Invalid covariance-intersection correction.";
    return false;
  }
  std::vector<int> observed_rows;
  for (int row = 0; row < states_num_; ++row) {
    if (c_matrix_.row(row).squaredNorm() > 0) observed_rows.push_back(row);
  }
  if (observed_rows.empty()) {
    AERROR << "Covariance intersection requires an observed state subspace.";
    return false;
  }
  const int dimension = static_cast<int>(observed_rows.size());
  Eigen::MatrixXd projection(dimension, states_num_);
  Eigen::MatrixXd measurement_covariance(dimension, dimension);
  Eigen::VectorXd observed(dimension);
  for (int row = 0; row < dimension; ++row) {
    projection.row(row) = c_matrix_.row(observed_rows[row]);
    observed(row) = observation(observed_rows[row]);
    for (int col = 0; col < dimension; ++col) {
      measurement_covariance(row, col) =
          covariance(observed_rows[row], observed_rows[col]);
    }
  }
  const Eigen::MatrixXd projected_covariance =
      projection * global_uncertainty_ * projection.transpose();
  const Eigen::MatrixXd identity =
      Eigen::MatrixXd::Identity(dimension, dimension);
  Eigen::LLT<Eigen::MatrixXd> prior(projected_covariance);
  Eigen::LLT<Eigen::MatrixXd> measurement(measurement_covariance);
  if (prior.info() != Eigen::Success ||
      measurement.info() != Eigen::Success) {
    AERROR << "Covariance intersection requires positive definite inputs.";
    return false;
  }
  const Eigen::MatrixXd prior_information = prior.solve(identity);
  const Eigen::MatrixXd measurement_information = measurement.solve(identity);
  const Eigen::MatrixXd information =
      prior_weight * prior_information +
      (1.0 - prior_weight) * measurement_information;
  Eigen::LLT<Eigen::MatrixXd> posterior(information);
  if (posterior.info() != Eigen::Success) {
    AERROR << "Invalid covariance-intersection posterior information.";
    return false;
  }
  const Eigen::VectorXd projected_state = projection * global_states_;
  const Eigen::VectorXd fused_state = posterior.solve(
      prior_weight * prior_information * projected_state +
      (1.0 - prior_weight) * measurement_information * observed);
  const Eigen::MatrixXd fused_covariance = posterior.solve(identity);
  // Lift the conservative observed marginal through the prior conditional;
  // do not repeatedly inflate states that this sensor never observes.
  const Eigen::MatrixXd gain =
      prior.solve(projection * global_uncertainty_).transpose();
  const Eigen::VectorXd state =
      global_states_ + gain * (fused_state - projected_state);
  const Eigen::MatrixXd result =
      global_uncertainty_ +
      gain * (fused_covariance - projected_covariance) * gain.transpose();
  if (!state.allFinite() || !result.allFinite()) {
    AERROR << "Non-finite covariance-intersection posterior.";
    return false;
  }
  global_states_ = state;
  global_uncertainty_ = 0.5 * (result + result.transpose());
  return true;
}

bool KalmanFilter::SetControlMatrix(const Eigen::MatrixXd &control_matrix) {
  if (!init_) {
    AERROR << "SetControlMatrix: Kalman Filter initialize not successfully";
    return false;
  }
  if (control_matrix.rows() != states_num_ ||
      control_matrix.cols() != states_num_) {
    AERROR << "the rows/cols of control matrix should be equal to state_num";
    return false;
  }
  if (!control_matrix.allFinite()) {
    AERROR << "Non-finite filter observation projection.";
    return false;
  }
  c_matrix_ = control_matrix;
  return true;
}

Eigen::VectorXd KalmanFilter::GetStates() const { return global_states_; }

Eigen::MatrixXd KalmanFilter::GetUncertainty() const {
  return global_uncertainty_;
}

bool KalmanFilter::SetGainBreakdownThresh(const std::vector<bool> &break_down,
                                          const float threshold) {
  if (static_cast<int>(break_down.size()) != states_num_) {
    return false;
  }
  for (int i = 0; i < states_num_; i++) {
    if (break_down[i]) {
      gain_break_down_(i) = 1;
    }
  }
  gain_break_down_threshold_ = threshold;
  return true;
}

bool KalmanFilter::SetValueBreakdownThresh(const std::vector<bool> &break_down,
                                           const float threshold) {
  if (static_cast<int>(break_down.size()) != states_num_) {
    return false;
  }
  for (int i = 0; i < states_num_; i++) {
    if (break_down[i]) {
      value_break_down_(i) = 1;
    }
  }
  value_break_down_threshold_ = threshold;
  return true;
}
void KalmanFilter::CorrectionBreakdown() {
  Eigen::VectorXd states_gain = global_states_ - prior_global_states_;
  Eigen::VectorXd breakdown_diff = states_gain.cwiseProduct(gain_break_down_);
  global_states_ -= breakdown_diff;
  if (breakdown_diff.norm() > gain_break_down_threshold_) {
    breakdown_diff.normalize();
    breakdown_diff *= gain_break_down_threshold_;
  }
  global_states_ += breakdown_diff;

  Eigen::VectorXd temp;
  temp.setOnes(states_num_, 1);
  if ((global_states_.cwiseProduct(value_break_down_)).norm() <
      value_break_down_threshold_) {
    global_states_ = global_states_.cwiseProduct(temp - value_break_down_);
  }
  prior_global_states_ = global_states_;
}

bool KalmanFilter::DeCorrelation(int x, int y, int x_len, int y_len) {
  if (x >= states_num_ || y >= states_num_ || x + x_len >= states_num_ ||
      y + y_len >= states_num_) {
    return false;
  }
  for (int i = 0; i < x_len; i++) {
    for (int j = 0; j < y_len; j++) {
      global_uncertainty_(x + i, y + j) = 0;
    }
  }
  return true;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
