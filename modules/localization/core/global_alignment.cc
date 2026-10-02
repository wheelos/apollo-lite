// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/core/global_alignment.h"
#include "modules/localization/core/fixed_lag_graph.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "Eigen/Cholesky"
#include "Eigen/LU"

namespace apollo {
namespace localization {
namespace unified {
namespace {

Eigen::Matrix<double, 6, 1> Difference(const Eigen::Isometry3d& a,
                                      const Eigen::Isometry3d& b) {
  Eigen::Matrix<double, 6, 1> difference;
  difference.head<3>() = b.translation() - a.translation();
  difference.tail<3>() = LogRotation(
      Eigen::Quaterniond(b.linear() * a.linear().transpose()));
  return difference;
}

bool ValidPose(const Eigen::Isometry3d& pose) {
  return pose.matrix().allFinite() &&
         (pose.linear().transpose() * pose.linear())
             .isApprox(Eigen::Matrix3d::Identity(), 1e-6) &&
         std::abs(pose.linear().determinant() - 1.0) < 1e-6 &&
         pose.matrix().row(3).isApprox(
             Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), 1e-9);
}

bool ValidPartial(const GlobalObservation& observation) {
  if (observation.kind == GlobalObservation::Kind::POSITION) {
    const Eigen::Matrix3d covariance =
        observation.covariance.topLeftCorner<3, 3>();
    return observation.pose.translation().allFinite() &&
           observation.point_in_base.allFinite() && covariance.allFinite() &&
           covariance.isApprox(covariance.transpose(), 1e-9) &&
           covariance.llt().info() == Eigen::Success;
  }
  const auto& h = observation.projection;
  const auto& r = observation.projected_covariance;
  return ValidPose(observation.pose) && h.cols() == 6 &&
         h.rows() > 0 && h.rows() < 6 && r.rows() == h.rows() &&
         r.cols() == h.rows() && h.allFinite() && r.allFinite() &&
         h.fullPivLu().rank() == h.rows() &&
         r.isApprox(r.transpose(), 1e-9) &&
         r.llt().info() == Eigen::Success;
}

}  // namespace

GlobalAlignment::GlobalAlignment(const GlobalConfig& config) : config_(config) {}
GlobalAlignment::~GlobalAlignment() = default;

Result GlobalAlignment::ValidateConfig() const {
  if (config_.enable_fixed_lag_graph &&
      (!config_.motion_error_model_qualified ||
       !std::isfinite(config_.graph_solve_budget_ms) ||
       config_.graph_solve_budget_ms <= 0.0 ||
       !std::isfinite(config_.graph_robust_threshold) ||
       config_.graph_robust_threshold <= 0.0 ||
       config_.graph_max_states < 2 || config_.graph_max_observations < 1 ||
       config_.graph_max_iterations == 0 || config_.graph_max_iterations > 100 ||
       !std::isfinite(config_.graph_max_linearization_angle) ||
       config_.graph_max_linearization_angle <= 0.0 ||
       config_.graph_max_linearization_angle >= 1.0)) {
    return {Reason::CONFIG_INVALID,
            "Fixed-lag graph requires qualified motion model and finite budgets"};
  }
  if (config_.clock_id.empty() || config_.map_id.empty() ||
      config_.map_version.empty() || config_.calibration_id.empty() ||
      config_.verification_frames < 2) {
    return {Reason::CONFIG_INVALID, "missing global identity or verification"};
  }
  for (double value : {config_.history_duration, config_.max_interpolation_gap,
                       config_.max_observation_age, config_.local_timeout,
                       config_.global_timeout, config_.innovation_gate,
                       config_.max_position_std, config_.max_attitude_std,
                       config_.verification_position,
                       config_.verification_angle,
                       config_.position_drift_variance_rate,
                       config_.attitude_drift_variance_rate}) {
    if (!std::isfinite(value) || value <= 0.0) {
      return {Reason::CONFIG_INVALID, "invalid global budget"};
    }
  }
  if (!std::isfinite(config_.future_tolerance) ||
      config_.future_tolerance < 0.0 ||
      config_.history_duration < config_.max_observation_age) {
    return {Reason::CONFIG_INVALID, "invalid global history/time policy"};
  }
  return {};
}

Result GlobalAlignment::Reject(Reason reason, const std::string& message,
                                SourceCounters* source) {
  if (source != nullptr) {
    ++source->rejected;
    source->reason = reason;
    source->duplicates += reason == Reason::DUPLICATE;
    source->regressions += reason == Reason::TIMESTAMP_REGRESSION;
  }
  reason_ = reason;
  return {reason, message};
}

void GlobalAlignment::ClearEpoch(const Epoch& epoch) {
  graph_.reset();
  motion_history_.clear();
  history_.clear();
  sources_.clear();
  evidence_.clear();
  state_ = GlobalState();
  state_.epoch = epoch;
  candidate_frames_ = 0;
  candidate_time_ = 0.0;
  reason_ = Reason::GLOBAL_UNAVAILABLE;
}

Result GlobalAlignment::AddLocal(const LocalState& local,
                                 const MotionIncrement* motion) {
  if (!local.valid || local.epoch.session.empty() ||
      local.epoch.generation == 0 || local.stamp.sequence == 0 ||
      local.stamp.clock_id != config_.clock_id ||
      !std::isfinite(local.stamp.time) || local.stamp.time <= 0.0 ||
      !std::isfinite(local.stamp.receive_time) ||
      local.stamp.receive_time - local.stamp.time > config_.local_timeout ||
      local.stamp.time - local.stamp.receive_time > config_.future_tolerance ||
      !ValidPose(Pose(local)) || !ValidCovariance(PoseCovariance(local))) {
    Invalidate(Reason::INVALID_INPUT);
    return {Reason::INVALID_INPUT, "invalid local snapshot"};
  }
  if (retired_sessions_.count(local.epoch.session) != 0) {
    return {Reason::EPOCH_MISMATCH, "retired producer session"};
  }
  if (!history_.empty()) {
    const auto& last = history_.back();
    if (local.epoch.session == last.epoch.session &&
        local.epoch.generation < last.epoch.generation) {
      return {Reason::EPOCH_MISMATCH, "old local epoch"};
    }
    if (local.stamp.time <= last.stamp.time) {
      return {Reason::TIMESTAMP_REGRESSION, "local time did not advance"};
    }
    if (local.epoch == last.epoch &&
        local.stamp.sequence <= last.stamp.sequence) {
      return {Reason::DUPLICATE, "local sequence did not advance"};
    }
  }
  if (history_.empty() || local.epoch != state_.epoch) {
    if (!history_.empty() && state_.epoch.session != local.epoch.session) {
      if (retired_sessions_.size() >= 128) {
        return {Reason::EPOCH_MISMATCH, "retired-session capacity exhausted"};
      }
      retired_sessions_.insert(state_.epoch.session);
    }
    ClearEpoch(local.epoch);
  }
  if (!local.covariance_model_valid) {
    Invalidate(Reason::INVALID_INPUT);
    history_.clear();
    motion_history_.clear();
    history_.push_back(local);
    return {Reason::INVALID_INPUT, "Local covariance error model is unavailable"};
  }
  if (!history_.empty() && !history_.back().covariance_model_valid) {
    history_.clear();
    motion_history_.clear();
  }
  if (config_.enable_fixed_lag_graph && !history_.empty()) {
    if (motion == nullptr || motion->epoch != local.epoch ||
        motion->start.time != history_.back().stamp.time ||
        motion->start.sequence != history_.back().stamp.sequence ||
        motion->end.time != local.stamp.time ||
        motion->end.sequence != local.stamp.sequence) {
      Invalidate(Reason::HISTORY_UNAVAILABLE);
      history_.clear();
      motion_history_.clear();
      history_.push_back(local);
      return {Reason::HISTORY_UNAVAILABLE,
              "Motion gap revoked global alignment; fresh local history retained"};
    }
    motion_history_[local.stamp.time] = *motion;
  }
  history_.push_back(local);
  while (history_.size() > 1 &&
         (local.stamp.time - history_.front().stamp.time >
              config_.history_duration || history_.size() > 5000)) {
    history_.pop_front();
  }
  while (!motion_history_.empty() &&
         motion_history_.begin()->first <= history_.front().stamp.time) {
    motion_history_.erase(motion_history_.begin());
  }
  if (graph_ && graph_->initialized() && motion != nullptr) {
    const auto added = graph_->AddLocal(local, *motion);
    if (!added.ok()) {
      Invalidate(added.reason);
      return added;
    }
    AdoptGraphState();
  }
  return {};
}

void GlobalAlignment::AdoptGraphState() {
  const uint64_t correction = state_.correction_id + 1;
  state_ = graph_->state();
  state_.correction_id = correction;
}

Result GlobalAlignment::InitializeGraph(const GlobalObservation& observation,
                                        const LocalState& local) {
  FixedLagConfig policy;
  policy.lag = config_.history_duration;
  policy.solve_budget_ms = config_.graph_solve_budget_ms;
  policy.innovation_gate = config_.innovation_gate;
  policy.robust_threshold = config_.graph_robust_threshold;
  policy.max_states = config_.graph_max_states;
  policy.max_observations = config_.graph_max_observations;
  policy.max_iterations = config_.graph_max_iterations;
  policy.max_linearization_angle = config_.graph_max_linearization_angle;
  std::unique_ptr<FixedLagGraph> candidate(new FixedLagGraph(policy));
  auto result = candidate->Initialize(local, observation);
  if (!result.ok()) {
    return result;
  }
  for (const auto& snapshot : history_) {
    if (snapshot.stamp.time <= local.stamp.time) {
      continue;
    }
    const auto motion = motion_history_.find(snapshot.stamp.time);
    if (motion == motion_history_.end()) {
      return {Reason::HISTORY_UNAVAILABLE, "Graph bootstrap motion is missing"};
    }
    result = candidate->AddLocal(snapshot, motion->second);
    if (!result.ok()) {
      return result;
    }
  }
  graph_.swap(candidate);
  AdoptGraphState();
  return {};
}

const LocalState* GlobalAlignment::latest_local() const {
  return history_.empty() ? nullptr : &history_.back();
}

Result GlobalAlignment::Lookup(double time, LocalState* local) const {
  if (local == nullptr || !std::isfinite(time) || history_.empty() ||
      time < history_.front().stamp.time || time > history_.back().stamp.time) {
    return {Reason::HISTORY_UNAVAILABLE, "no retained local bracket"};
  }
  if (!history_.back().covariance_model_valid) {
    return {Reason::INVALID_INPUT, "Local covariance error model is unavailable"};
  }
  auto upper = std::lower_bound(
      history_.begin(), history_.end(), time,
      [](const LocalState& state, double stamp) { return state.stamp.time < stamp; });
  if (upper->stamp.time == time) {
    *local = *upper;
    return {};
  }
  const auto& before = *std::prev(upper);
  const double duration = upper->stamp.time - before.stamp.time;
  if (duration > config_.max_interpolation_gap) {
    return {Reason::HISTORY_UNAVAILABLE, "local interpolation gap exceeded"};
  }
  const double weight = (time - before.stamp.time) / duration;
  *local = before;
  local->stamp.time = time;
  local->position =
      (1.0 - weight) * before.position + weight * upper->position;
  local->velocity =
      (1.0 - weight) * before.velocity + weight * upper->velocity;
  local->orientation = before.orientation.slerp(weight, upper->orientation);
  // Adjacent states are correlated. This is a conservative interpolation bound.
  local->covariance =
      2.0 * ((1.0 - weight) * before.covariance + weight * upper->covariance);
  return {};
}

Result GlobalAlignment::Observe(const GlobalObservation& observation) {
  const auto existing = evidence_.find(observation.source);
  if (!config_.enable_fixed_lag_graph || existing == evidence_.end() ||
      observation.stamp.time >= existing->second.observation.stamp.time) {
    evidence_.erase(observation.source);
  }
  if (sources_.size() >= 32 && sources_.count(observation.source) == 0) {
    return {Reason::INVALID_INPUT, "too many global sources"};
  }
  auto& source = sources_[observation.source];
  const auto valid_config = ValidateConfig();
  if (!valid_config.ok()) {
    return Reject(valid_config.reason, valid_config.message, &source);
  }
  if (observation.source.empty() || !observation.quality_valid ||
      (config_.enable_fixed_lag_graph && !observation.independent_of_local) ||
      (observation.kind == GlobalObservation::Kind::POSE
           ? (!ValidPose(observation.pose) ||
              !ValidCovariance(observation.covariance))
           : !ValidPartial(observation)) ||
      observation.stamp.sequence == 0 ||
      !std::isfinite(observation.stamp.time) || observation.stamp.time <= 0.0 ||
      !std::isfinite(observation.stamp.receive_time)) {
    return Reject(Reason::INVALID_INPUT, "invalid global evidence", &source);
  }
  if (observation.stamp.clock_id != config_.clock_id ||
      observation.stamp.time - observation.stamp.receive_time >
          config_.future_tolerance ||
      observation.stamp.receive_time - observation.stamp.time >
          config_.max_observation_age) {
    return Reject(Reason::CLOCK_INVALID, "global observation time invalid", &source);
  }
  if (observation.map_id != config_.map_id ||
      observation.map_version != config_.map_version ||
      observation.calibration_id != config_.calibration_id) {
    return Reject(Reason::MAP_MISMATCH, "map/calibration identity mismatch", &source);
  }
  if (observation.epoch != state_.epoch) {
    return Reject(Reason::EPOCH_MISMATCH, "global observation epoch mismatch", &source);
  }
  if (config_.enable_fixed_lag_graph && graph_ && state_.valid &&
      observation.stamp.receive_time - state_.last_full_observation >
          config_.global_timeout) {
    Invalidate(Reason::GLOBAL_STALE);
  }
  const bool graph_update = graph_ && graph_->initialized() && state_.valid &&
                            !observation.recovery;
  if (!graph_update && (observation.stamp.time <= source.last_measurement ||
                       observation.stamp.sequence <= source.last_sequence)) {
    return Reject(observation.stamp.time == source.last_measurement
                      ? Reason::DUPLICATE : Reason::TIMESTAMP_REGRESSION,
                  "global source frontier did not advance", &source);
  }
  source.last_measurement = std::max(source.last_measurement, observation.stamp.time);
  source.last_receive = std::max(source.last_receive, observation.stamp.receive_time);
  source.last_sequence = std::max(source.last_sequence, observation.stamp.sequence);
  if (observation.ambiguous) {
    evidence_.erase(observation.source);
    candidate_frames_ = 0;
    state_.verifying = false;
    return Reject(Reason::RELOCALIZATION_AMBIGUOUS, "ambiguous map hypothesis", &source);
  }
  LocalState local;
  const auto lookup = Lookup(observation.stamp.time, &local);
  if (!lookup.ok()) {
    return Reject(lookup.reason, lookup.message, &source);
  }
  if (graph_update) {
    const auto observed = graph_->Observe(observation);
    if (!observed.ok()) {
      return Reject(observed.reason, observed.message, &source);
    }
    AdoptGraphState();
    const auto latest = evidence_.find(observation.source);
    if (latest == evidence_.end() ||
        observation.stamp.time >= latest->second.observation.stamp.time) {
      evidence_[observation.source] = {observation, local};
    }
    ++source.accepted;
    source.reason = Reason::NONE;
    reason_ = observation.kind == GlobalObservation::Kind::POSE
                  ? Reason::NONE : Reason::DEGENERATE;
    return {};
  }
  if (state_.valid && observation.stamp.time < state_.last_observation) {
    return Reject(Reason::TIMESTAMP_REGRESSION,
                  "observation older than committed global frontier", &source);
  }
  if (state_.valid) {
    state_ = Predict(observation.stamp.time);
  }
  if (observation.kind != GlobalObservation::Kind::POSE) {
    return ObservePartial(observation, local, &source);
  }
  const Eigen::Isometry3d measured = observation.pose * Pose(local).inverse();
  const Matrix6d covariance =
      TransformCovarianceBound(observation.pose, local, observation.covariance);
  if (!ValidCovariance(covariance)) {
    return Reject(Reason::INVALID_INPUT, "invalid alignment covariance", &source);
  }
  if (state_.valid &&
      observation.stamp.time - state_.last_full_observation > config_.global_timeout) {
    Invalidate(Reason::GLOBAL_STALE);
  }
  if (!state_.valid || observation.recovery) {
    graph_.reset();
    if (candidate_frames_ > 0 && observation.stamp.time <= candidate_time_) {
      return Reject(Reason::TIMESTAMP_REGRESSION,
                    "recovery needs distinct advancing measurement frames", &source);
    }
    if (candidate_frames_ == 0 ||
        observation.stamp.time - candidate_time_ > config_.global_timeout ||
        candidate_georeferenced_ != observation.georeferenced ||
        Difference(candidate_, measured).head<3>().norm() >
            config_.verification_position ||
        Difference(candidate_, measured).tail<3>().norm() >
            config_.verification_angle) {
      candidate_ = measured;
      candidate_covariance_ = covariance;
      candidate_frames_ = 1;
      candidate_georeferenced_ = observation.georeferenced;
    } else {
      ++candidate_frames_;
      candidate_ = measured;
      candidate_covariance_ = covariance;
    }
    candidate_time_ = observation.stamp.time;
    state_.verifying = true;
    if (observation.recovery) {
      state_.valid = false;
    }
    if (candidate_frames_ < config_.verification_frames) {
      return Reject(Reason::RELOCALIZATION_VERIFYING,
                    "waiting for distinct consistent frames", &source);
    }
    state_.map_to_odom = candidate_;
    state_.covariance = candidate_covariance_;
    state_.covariance_time = observation.stamp.time;
    state_.georeferenced = candidate_georeferenced_;
    state_.valid = true;
    state_.verifying = false;
    candidate_frames_ = 0;
    ++state_.correction_id;
    if (config_.enable_fixed_lag_graph) {
      const auto initialized = InitializeGraph(observation, local);
      if (!initialized.ok()) {
        state_.valid = false;
        return Reject(initialized.reason, initialized.message, &source);
      }
    }
  } else {
    if (observation.stamp.time < state_.last_observation) {
      return Reject(Reason::TIMESTAMP_REGRESSION,
                    "global fusion frontier did not advance", &source);
    }
    const auto innovation = Difference(state_.map_to_odom, measured);
    // The same local trajectory can occur in both estimates.
    const Matrix6d innovation_covariance = 2.0 * (state_.covariance + covariance);
    const double distance = innovation.dot(
        innovation_covariance.ldlt().solve(innovation));
    if (!std::isfinite(distance) || distance > config_.innovation_gate) {
      return Reject(Reason::INNOVATION_REJECTED, "global innovation rejected", &source);
    }
    const Matrix6d previous_information =
        state_.covariance.ldlt().solve(Matrix6d::Identity());
    const Matrix6d measured_information =
        covariance.ldlt().solve(Matrix6d::Identity());
    // Fixed-weight covariance intersection prevents correlated double counting.
    const Matrix6d fused =
        (0.5 * previous_information + 0.5 * measured_information)
            .ldlt().solve(Matrix6d::Identity());
    const Eigen::Matrix<double, 6, 1> correction =
        fused * (0.5 * measured_information) * innovation;
    state_.map_to_odom.translation() += correction.head<3>();
    state_.map_to_odom.linear() =
        ExpRotation(correction.tail<3>()).toRotationMatrix() *
        state_.map_to_odom.linear();
    state_.covariance = 0.5 * (fused + fused.transpose());
    state_.georeferenced = observation.georeferenced;
    ++state_.correction_id;
  }
  state_.last_observation = observation.stamp.time;
  state_.last_full_observation = observation.stamp.time;
  state_.last_georeference =
      observation.georeferenced ? observation.stamp.time : 0.0;
  evidence_[observation.source] = {observation, local};
  ++source.accepted;
  source.reason = Reason::NONE;
  reason_ = Reason::NONE;
  return {};
}

Result GlobalAlignment::ObservePartial(const GlobalObservation& observation,
                                       const LocalState& local,
                                       SourceCounters* source) {
  if (observation.recovery) {
    return Reject(Reason::DEGENERATE,
                  "partial evidence cannot complete full-pose recovery", source);
  }
  if (!state_.valid) {
    if (observation.kind != GlobalObservation::Kind::POSITION) {
      return Reject(Reason::GLOBAL_UNAVAILABLE,
                    "projected pose requires a verified alignment basin", source);
    }
    // A directly measured point is useful before global rotation is known.
    evidence_[observation.source] = {observation, local};
    ++source->accepted;
    source->reason = Reason::NONE;
    return {};
  }
  if (observation.stamp.time < state_.last_observation) {
    return Reject(Reason::TIMESTAMP_REGRESSION,
                  "partial observation older than global fusion frontier", source);
  }
  const auto predicted = state_.map_to_odom * Pose(local);
  Eigen::MatrixXd h;
  Eigen::MatrixXd noise;
  Eigen::VectorXd residual;
  if (observation.kind == GlobalObservation::Kind::POSITION) {
    const Eigen::Vector3d local_point =
        local.position + local.orientation * observation.point_in_base;
    const Eigen::Vector3d rotated = state_.map_to_odom.linear() * local_point;
    residual = observation.pose.translation() -
               (rotated + state_.map_to_odom.translation());
    h = Eigen::Matrix<double, 3, 6>::Zero();
    h.leftCols(3).setIdentity();
    h.rightCols(3) = -Skew(rotated);
    Eigen::Matrix<double, 3, 6> local_jacobian;
    local_jacobian.leftCols<3>() = state_.map_to_odom.linear();
    local_jacobian.rightCols<3>() =
        -predicted.linear() * Skew(observation.point_in_base);
    noise = 2.0 * (observation.covariance.topLeftCorner<3, 3>() +
                    local_jacobian * PoseCovariance(local) *
                        local_jacobian.transpose());
  } else {
    Matrix6d alignment_jacobian = Matrix6d::Identity();
    alignment_jacobian.topRightCorner<3, 3>() =
        -Skew(state_.map_to_odom.linear() * local.position);
    Matrix6d local_jacobian = Matrix6d::Zero();
    local_jacobian.topLeftCorner<3, 3>() = state_.map_to_odom.linear();
    local_jacobian.bottomRightCorner<3, 3>() = predicted.linear();
    h = observation.projection * alignment_jacobian;
    residual = observation.projection * Difference(predicted, observation.pose);
    noise = 2.0 * (observation.projected_covariance +
                    observation.projection * local_jacobian *
                        PoseCovariance(local) * local_jacobian.transpose() *
                        observation.projection.transpose());
  }
  const Eigen::MatrixXd innovation =
      2.0 * (h * state_.covariance * h.transpose() + noise);
  const double distance = residual.dot(innovation.ldlt().solve(residual));
  if (!std::isfinite(distance) || distance > config_.innovation_gate) {
    evidence_.erase(observation.source);
    return Reject(Reason::INNOVATION_REJECTED,
                  "partial innovation rejected in observed subspace", source);
  }
  // Generalized covariance intersection: unknown shared trajectory correlation.
  // Null directions receive no information and may become less certain.
  const Matrix6d prior_information =
      state_.covariance.ldlt().solve(Matrix6d::Identity());
  const Eigen::MatrixXd weighted = noise.ldlt().solve(h);
  const Matrix6d information =
      0.5 * prior_information + 0.5 * h.transpose() * weighted;
  const Matrix6d covariance = information.ldlt().solve(Matrix6d::Identity());
  const Eigen::Matrix<double, 6, 1> correction =
      covariance * (0.5 * weighted.transpose()) * residual;
  if (!ValidCovariance(covariance) || !correction.allFinite()) {
    return Reject(Reason::INVALID_INPUT, "partial update is not finite/positive", source);
  }
  state_.map_to_odom.translation() += correction.head<3>();
  state_.map_to_odom.linear() =
      ExpRotation(correction.tail<3>()).toRotationMatrix() *
      state_.map_to_odom.linear();
  state_.covariance = 0.5 * (covariance + covariance.transpose());
  state_.last_observation = observation.stamp.time;
  // Do not refresh the full-pose lease or claim previously missing directions.
  if (observation.georeferenced) {
    state_.last_georeference = observation.stamp.time;
  }
  ++state_.correction_id;
  evidence_[observation.source] = {observation, local};
  ++source->accepted;
  source->reason = Reason::NONE;
  reason_ = Reason::DEGENERATE;
  return {};
}

void GlobalAlignment::Invalidate(Reason reason) {
  graph_.reset();
  state_.valid = false;
  state_.verifying = false;
  candidate_frames_ = 0;
  evidence_.clear();
  reason_ = reason;
}

Result GlobalAlignment::Evaluate(double now) const {
  if (!std::isfinite(now) || now <= 0.0) {
    return {Reason::CLOCK_INVALID, "invalid evaluation clock"};
  }
  if (history_.empty()) {
    return {Reason::HISTORY_UNAVAILABLE, "local history empty"};
  }
  if (now - history_.back().stamp.time > config_.local_timeout ||
      history_.back().stamp.time - now > config_.future_tolerance) {
    return {Reason::IMU_STALE, "local snapshot stale"};
  }
  if (!state_.valid) {
    return {reason_, "global alignment unavailable"};
  }
  if (now - state_.last_full_observation > config_.global_timeout ||
      state_.last_full_observation - now > config_.future_tolerance) {
    return {Reason::GLOBAL_STALE, "no fresh global evidence"};
  }
  if (!ValidCovariance(state_.covariance)) {
    return {Reason::INVALID_INPUT, "alignment covariance invalid"};
  }
  const Matrix6d combined =
      GlobalPoseCovarianceBound(Predict(history_.back().stamp.time), history_.back());
  if (combined.topLeftCorner<3, 3>().diagonal().maxCoeff() >
          config_.max_position_std * config_.max_position_std ||
      combined.bottomRightCorner<3, 3>().diagonal().maxCoeff() >
          config_.max_attitude_std * config_.max_attitude_std) {
    return {Reason::COVARIANCE_EXCEEDED, "global uncertainty budget exceeded"};
  }
  return {};
}

GlobalState GlobalAlignment::Predict(double time) const {
  GlobalState predicted = state_;
  if (predicted.valid && std::isfinite(time) && time > state_.covariance_time) {
    const double elapsed = time - state_.covariance_time;
    predicted.covariance.topLeftCorner<3, 3>().diagonal().array() +=
        elapsed * config_.position_drift_variance_rate;
    predicted.covariance.bottomRightCorner<3, 3>().diagonal().array() +=
        elapsed * config_.attitude_drift_variance_rate;
    predicted.covariance_time = time;
  }
  return predicted;
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
