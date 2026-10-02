// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/core/fixed_lag_graph.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <set>
#include <utility>

#include "Eigen/Cholesky"
#include "Eigen/LU"
#include "gtsam/base/numericalDerivative.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/linear/linearExceptions.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/slam/PriorFactor.h"
#include "gtsam/nonlinear/IncrementalFixedLagSmoother.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

using Clock = std::chrono::steady_clock;
using OptionalJacobian = gtsam::OptionalMatrixType;

gtsam::Key PoseKey(uint64_t index) { return gtsam::Symbol('x', index); }
gtsam::Key ErrorKey(uint64_t index) { return gtsam::Symbol('e', index); }

gtsam::ISAM2Params Parameters() {
  gtsam::ISAM2Params params;
  params.factorization = gtsam::ISAM2Params::QR;
  params.findUnusedFactorSlots = true;
  params.relinearizeSkip = 1;
  params.relinearizeThreshold = 1e-6;
  return params;
}

gtsam::Pose3 ToPose(const Eigen::Isometry3d& pose) {
  return gtsam::Pose3(pose.matrix());
}

Eigen::Isometry3d FromPose(const gtsam::Pose3& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.matrix() = pose.matrix();
  return result;
}

class ErrorChainFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector> {
 public:
  ErrorChainFactor(gtsam::Key first, gtsam::Key second,
                   const Matrix15d& transition, const Matrix15d& noise)
      : gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector>(
            gtsam::noiseModel::Gaussian::Covariance(noise), first, second),
        transition_(transition) {}

  gtsam::Vector evaluateError(const gtsam::Vector& first,
                             const gtsam::Vector& second,
                             OptionalJacobian h1 = OptionalNone,
                             OptionalJacobian h2 = OptionalNone) const override {
    if (h1) {
      *h1 = -transition_;
    }
    if (h2) {
      *h2 = Matrix15d::Identity();
    }
    return second - transition_ * first;
  }

 private:
  Matrix15d transition_;
};

class MotionFactor final
    : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector,
                                     gtsam::Pose3, gtsam::Vector> {
 public:
  MotionFactor(uint64_t first, uint64_t second, const MotionIncrement& motion)
      : gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector,
                                 gtsam::Pose3, gtsam::Vector>(
            gtsam::noiseModel::Constrained::All(6), PoseKey(first),
            ErrorKey(first), PoseKey(second), ErrorKey(second)),
        delta_(motion.delta) {
    RelativeMotionJacobians(motion.start_pose, motion.delta, &first_, &second_);
  }

  gtsam::Vector RelativeError(const gtsam::Pose3& first,
                             const gtsam::Pose3& second) const {
    const Eigen::Isometry3d relative = FromPose(first.between(second));
    gtsam::Vector error(6);
    error.head<3>() = relative.translation() - delta_.translation();
    error.tail<3>() = LogRotation(
        Eigen::Quaterniond(delta_.linear().transpose() * relative.linear()));
    return error;
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& first,
                             const gtsam::Vector& first_error,
                             const gtsam::Pose3& second,
                             const gtsam::Vector& second_error,
                             OptionalJacobian h1 = OptionalNone,
                             OptionalJacobian h2 = OptionalNone,
                             OptionalJacobian h3 = OptionalNone,
                             OptionalJacobian h4 = OptionalNone) const override {
    if (h1) {
      const std::function<gtsam::Vector(const gtsam::Pose3&)> function =
          [&](const gtsam::Pose3& value) { return RelativeError(value, second); };
      *h1 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
          function, first);
    }
    if (h2) {
      *h2 = -first_;
    }
    if (h3) {
      const std::function<gtsam::Vector(const gtsam::Pose3&)> function =
          [&](const gtsam::Pose3& value) { return RelativeError(first, value); };
      *h3 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
          function, second);
    }
    if (h4) {
      *h4 = -second_;
    }
    return RelativeError(first, second) - first_ * first_error -
           second_ * second_error;
  }

 private:
  Eigen::Isometry3d delta_;
  Matrix6x15d first_;
  Matrix6x15d second_;
};

Eigen::MatrixXd ObservationNoise(const GlobalObservation& observation) {
  if (observation.kind == GlobalObservation::Kind::POSITION) {
    return observation.covariance.topLeftCorner<3, 3>();
  }
  if (observation.kind == GlobalObservation::Kind::PROJECTED_POSE) {
    return observation.projected_covariance;
  }
  return observation.covariance;
}

class ObservationFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  ObservationFactor(gtsam::Key key, const GlobalObservation& observation,
                    double robust_threshold)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(
            gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Huber::Create(robust_threshold),
                gtsam::noiseModel::Gaussian::Covariance(
                    ObservationNoise(observation))), key),
        observation_(observation) {}

  gtsam::Vector Residual(const gtsam::Pose3& pose) const {
    const Eigen::Isometry3d estimate = FromPose(pose);
    if (observation_.kind == GlobalObservation::Kind::POSITION) {
      return estimate * observation_.point_in_base -
             observation_.pose.translation();
    }
    gtsam::Vector residual(6);
    residual.head<3>() =
        estimate.translation() - observation_.pose.translation();
    residual.tail<3>() = LogRotation(Eigen::Quaterniond(
        estimate.linear() * observation_.pose.linear().transpose()));
    if (observation_.kind == GlobalObservation::Kind::PROJECTED_POSE) {
      return observation_.projection * residual;
    }
    return residual;
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                             OptionalJacobian h = OptionalNone) const override {
    if (h) {
      const std::function<gtsam::Vector(const gtsam::Pose3&)> function =
          [&](const gtsam::Pose3& value) { return Residual(value); };
      *h = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
          function, pose);
    }
    return Residual(pose);
  }

 private:
  GlobalObservation observation_;
};

Result CheckObservation(const GlobalObservation& observation) {
  const Eigen::MatrixXd noise = ObservationNoise(observation);
  const auto& pose = observation.pose;
  const bool projected =
      observation.kind == GlobalObservation::Kind::PROJECTED_POSE;
  if ((observation.kind != GlobalObservation::Kind::POSE &&
       observation.kind != GlobalObservation::Kind::POSITION && !projected) ||
      !observation.quality_valid || observation.ambiguous ||
      !observation.independent_of_local || observation.source.empty() ||
      observation.map_id.empty() || observation.map_version.empty() ||
      observation.calibration_id.empty() || observation.stamp.sequence == 0 ||
      !std::isfinite(observation.stamp.time) || observation.stamp.time <= 0.0 ||
      !std::isfinite(observation.stamp.receive_time) ||
      observation.stamp.receive_time <= 0.0 ||
      !pose.matrix().allFinite() || !observation.point_in_base.allFinite() ||
      !(pose.linear().transpose() * pose.linear())
           .isApprox(Eigen::Matrix3d::Identity(), 1e-9) ||
      std::abs(pose.linear().determinant() - 1.0) > 1e-9 ||
      noise.rows() < 1 || noise.rows() > 6 || noise.cols() != noise.rows() ||
      !noise.allFinite() || !noise.isApprox(noise.transpose(), 1e-9) ||
      (projected &&
       (observation.projection.rows() != noise.rows() ||
        observation.projection.rows() > 5 ||
        observation.projection.cols() != 6 ||
        !observation.projection.allFinite() ||
        observation.projection.fullPivLu().rank() != noise.rows())) ||
      Eigen::LLT<Eigen::MatrixXd>(noise).info() != Eigen::Success) {
    return {Reason::INVALID_INPUT,
            "Graph needs valid, explicitly independent observed information"};
  }
  return {};
}

}  // namespace

struct FixedLagGraph::Impl {
  struct Node {
    uint64_t key;
    LocalState local;
  };
  explicit Impl(double lag) : smoother(lag, Parameters()) {}
  gtsam::IncrementalFixedLagSmoother smoother;
  std::map<double, Node> nodes;
  std::map<std::pair<std::string, uint64_t>, double> observations;
  std::map<std::string, uint64_t> retired_sequences;
  std::set<std::string> sources;
  GlobalObservation anchor;
  GlobalState output;
  uint64_t next_key = 1;

  Result Finalize(const FixedLagConfig& config, Clock::time_point started) {
    for (uint32_t iteration = 0; iteration < config.max_iterations; ++iteration) {
      const gtsam::Values estimate = smoother.calculateEstimate();
      for (const auto& node : nodes) {
        const auto& error =
            estimate.at<gtsam::Vector>(ErrorKey(node.second.key));
        if (error.size() != 15 || !error.allFinite() ||
            error.segment<3>(6).norm() > config.max_linearization_angle) {
          return {Reason::INNOVATION_REJECTED,
                  "Global nuisance error exceeded its linearization domain"};
        }
      }
      bool consistent = true;
      for (const auto& factor : smoother.getFactors()) {
        const auto* motion = dynamic_cast<const MotionFactor*>(factor.get());
        if (motion == nullptr) {
          continue;
        }
        const auto& keys = motion->keys();
        const gtsam::Vector residual = motion->evaluateError(
            estimate.at<gtsam::Pose3>(keys[0]),
            estimate.at<gtsam::Vector>(keys[1]),
            estimate.at<gtsam::Pose3>(keys[2]),
            estimate.at<gtsam::Vector>(keys[3]));
        if (!residual.allFinite()) {
          return {Reason::INVALID_INPUT, "Nonfinite graph motion constraint"};
        }
        consistent = consistent && residual.cwiseAbs().maxCoeff() <= 1e-6;
      }
      if (consistent) {
        return Snapshot(config.solve_budget_ms, started);
      }
      if (std::chrono::duration<double, std::milli>(Clock::now() - started).count() >
          config.solve_budget_ms) {
        return {Reason::GLOBAL_STALE, "Graph exhausted its solve budget"};
      }
      if (iteration + 1 < config.max_iterations) {
        smoother.update();
      }
    }
    return {Reason::MATCH_FAILED, "Graph motion constraints did not converge"};
  }

  Result Snapshot(double budget_ms, Clock::time_point started) {
    const auto& latest = nodes.rbegin()->second;
    const auto pose =
        smoother.calculateEstimate<gtsam::Pose3>(PoseKey(latest.key));
    const Eigen::Isometry3d map_base = FromPose(pose);
    const gtsam::Matrix tangent =
        smoother.marginalCovariance(PoseKey(latest.key));
    if (tangent.rows() != 6 || tangent.cols() != 6 ||
        !map_base.matrix().allFinite() || !tangent.allFinite()) {
      return {Reason::INVALID_INPUT, "Nonfinite graph pose/marginal"};
    }
    // GTSAM Pose3 tangent is [rotation_body, translation_body].
    Matrix6d axes = Matrix6d::Zero();
    axes.topRightCorner<3, 3>() = map_base.linear();
    axes.bottomLeftCorner<3, 3>() = map_base.linear();
    const Matrix6d covariance = axes * tangent * axes.transpose();
    if (!ValidCovariance(covariance)) {
      return {Reason::DEGENERATE, "Graph marginal is not full-rank"};
    }
    if (std::chrono::duration<double, std::milli>(Clock::now() - started).count() >
        budget_ms) {
      return {Reason::GLOBAL_STALE, "Graph solve exceeded its publication budget"};
    }
    const Eigen::Isometry3d alignment = map_base * Pose(latest.local).inverse();
    const Matrix6d alignment_covariance =
        TransformCovarianceBound(map_base, latest.local, covariance);
    if (!alignment.matrix().allFinite() ||
        !ValidCovariance(alignment_covariance)) {
      return {Reason::INVALID_INPUT, "Graph alignment composition is invalid"};
    }
    output.map_to_odom = alignment;
    output.covariance = alignment_covariance;
    output.covariance_time = latest.local.stamp.time;
    output.epoch = latest.local.epoch;
    output.valid = true;
    ++output.correction_id;
    return {};
  }

  void Prune(double cutoff) {
    while (!nodes.empty() && nodes.begin()->first < cutoff) {
      nodes.erase(nodes.begin());
    }
    for (auto it = observations.begin(); it != observations.end();) {
      if (it->second < cutoff) {
        auto& frontier = retired_sequences[it->first.first];
        frontier = std::max(frontier, it->first.second);
        it = observations.erase(it);
      } else {
        ++it;
      }
    }
  }
};

FixedLagGraph::FixedLagGraph(const FixedLagConfig& config)
    : config_(config), impl_(new Impl(config.lag)) {}
FixedLagGraph::~FixedLagGraph() = default;

Result FixedLagGraph::ValidateConfig() const {
  for (double value : {config_.lag, config_.solve_budget_ms,
                       config_.innovation_gate, config_.robust_threshold,
                       config_.max_linearization_angle}) {
    if (!std::isfinite(value) || value <= 0.0) {
      return {Reason::CONFIG_INVALID, "Invalid fixed-lag graph budget"};
    }
  }
  if (config_.max_states < 2 || config_.max_observations < 1 ||
      config_.max_iterations == 0 || config_.max_iterations > 100 ||
      config_.max_linearization_angle >= 1.0) {
    return {Reason::CONFIG_INVALID, "Invalid fixed-lag graph capacity"};
  }
  return {};
}

bool FixedLagGraph::initialized() const { return !impl_->nodes.empty(); }
const GlobalState& FixedLagGraph::state() const { return impl_->output; }

Result FixedLagGraph::Initialize(const LocalState& local,
                                const GlobalObservation& anchor) {
  Result valid = ValidateConfig();
  if (!valid.ok()) {
    return valid;
  }
  valid = CheckObservation(anchor);
  if (!valid.ok()) {
    return valid;
  }
  if (anchor.kind != GlobalObservation::Kind::POSE || !local.valid ||
      !local.covariance_model_valid ||
      anchor.epoch != local.epoch || local.epoch.generation == 0 ||
      local.epoch.session.empty() || local.stamp.sequence == 0 ||
      local.stamp.clock_id.empty() || anchor.stamp.time != local.stamp.time ||
      anchor.stamp.clock_id != local.stamp.clock_id ||
      !local.covariance.allFinite() ||
      !local.covariance.isApprox(local.covariance.transpose(), 1e-9) ||
      Eigen::LLT<Matrix15d>(local.covariance).info() != Eigen::Success ||
      !Pose(local).matrix().allFinite() ||
      !local.orientation.coeffs().allFinite() ||
      std::abs(local.orientation.norm() - 1.0) > 1e-9) {
    return {Reason::GLOBAL_UNAVAILABLE,
            "Graph initialization requires a verified full-pose same-time anchor"};
  }
  const auto started = Clock::now();
  std::unique_ptr<Impl> candidate(new Impl(config_.lag));
  candidate->anchor = anchor;
  candidate->nodes.emplace(local.stamp.time, Impl::Node{1, local});
  candidate->next_key = 2;
  candidate->output.last_observation = anchor.stamp.time;
  candidate->output.last_full_observation = anchor.stamp.time;
  candidate->output.last_georeference =
      anchor.georeferenced ? anchor.stamp.time : 0.0;
  candidate->output.georeferenced = anchor.georeferenced;
  candidate->observations[{anchor.source, anchor.stamp.sequence}] =
      anchor.stamp.time;
  candidate->sources.insert(anchor.source);
  gtsam::NonlinearFactorGraph factors;
  factors.emplace_shared<ObservationFactor>(PoseKey(1), anchor,
                                            config_.robust_threshold);
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector>>(
      ErrorKey(1), gtsam::Vector::Zero(15),
      gtsam::noiseModel::Gaussian::Covariance(local.covariance));
  gtsam::Values values;
  values.insert(PoseKey(1), ToPose(anchor.pose));
  const gtsam::Vector zero = gtsam::Vector::Zero(15);
  values.insert(ErrorKey(1), zero);
  try {
    candidate->smoother.update(factors, values,
                              {{PoseKey(1), local.stamp.time},
                               {ErrorKey(1), local.stamp.time}});
    valid = candidate->Finalize(config_, started);
  } catch (const gtsam::IndeterminantLinearSystemException& error) {
    return {Reason::DEGENERATE, error.what()};
  }
  if (!valid.ok()) {
    return valid;
  }
  impl_.swap(candidate);
  return {};
}

Result FixedLagGraph::AddLocal(const LocalState& local,
                              const MotionIncrement& motion) {
  if (!initialized()) {
    return {Reason::GLOBAL_UNAVAILABLE, "Graph has no verified anchor"};
  }
  const auto& previous = impl_->nodes.rbegin()->second;
  if (motion.epoch != previous.local.epoch || local.epoch != motion.epoch) {
    return {Reason::EPOCH_MISMATCH, "Graph motion changed ODOM epoch"};
  }
  if (motion.start.time != previous.local.stamp.time ||
      motion.end.time != local.stamp.time ||
      motion.start.sequence != previous.local.stamp.sequence ||
      motion.end.sequence != local.stamp.sequence ||
      !motion.start_covariance.isApprox(previous.local.covariance, 1e-9) ||
      !motion.end_covariance.isApprox(local.covariance, 1e-9)) {
    return {Reason::HISTORY_UNAVAILABLE, "Graph requires a contiguous local error chain"};
  }
  MotionIncrement verified;
  auto valid = ComputeMotionIncrement(previous.local, local,
                                     motion.cross_covariance, motion.sources,
                                     &verified);
  if (!valid.ok()) {
    return valid;
  }
  if (!motion.delta.matrix().isApprox(verified.delta.matrix(), 1e-9) ||
      motion.correlation_group != verified.correlation_group) {
    return {Reason::INVALID_INPUT, "Motion identity or relative pose mismatch"};
  }
  const Matrix15d transition =
      previous.local.covariance.llt().solve(motion.cross_covariance).transpose();
  Matrix15d noise =
      local.covariance -
      transition * previous.local.covariance * transition.transpose();
  noise = (0.5 * (noise + noise.transpose())).eval();
  if (!transition.allFinite() || !noise.allFinite() ||
      Eigen::LLT<Matrix15d>(noise).info() != Eigen::Success) {
    return {Reason::DEGENERATE,
            "Conditional motion noise is singular; no artificial prior is added"};
  }
  if (impl_->nodes.size() >= config_.max_states &&
      impl_->nodes.begin()->first >= local.stamp.time - config_.lag) {
    return {Reason::INVALID_INPUT, "Fixed-lag state capacity exceeded"};
  }
  const auto started = Clock::now();
  std::unique_ptr<Impl> candidate(new Impl(*impl_));
  const uint64_t key = candidate->next_key++;
  candidate->nodes.emplace(local.stamp.time, Impl::Node{key, local});
  gtsam::NonlinearFactorGraph factors;
  factors.emplace_shared<ErrorChainFactor>(
      ErrorKey(previous.key), ErrorKey(key), transition, noise);
  factors.emplace_shared<MotionFactor>(previous.key, key, verified);
  gtsam::Values values;
  const auto previous_pose =
      candidate->smoother.calculateEstimate<gtsam::Pose3>(PoseKey(previous.key));
  const gtsam::Vector previous_error =
      candidate->smoother.calculateEstimate<gtsam::Vector>(ErrorKey(previous.key));
  const gtsam::Vector next_error = transition * previous_error;
  Matrix6x15d first;
  Matrix6x15d second;
  RelativeMotionJacobians(verified.start_pose, verified.delta, &first, &second);
  const Eigen::Matrix<double, 6, 1> relative_error =
      first * previous_error + second * next_error;
  Eigen::Isometry3d initial_delta = verified.delta;
  initial_delta.translation() += relative_error.head<3>();
  initial_delta.linear() =
      (initial_delta.linear() *
       ExpRotation(relative_error.tail<3>()).toRotationMatrix()).eval();
  values.insert(PoseKey(key), previous_pose.compose(ToPose(initial_delta)));
  values.insert(ErrorKey(key), next_error);
  try {
    candidate->smoother.update(factors, values,
                              {{PoseKey(key), local.stamp.time},
                               {ErrorKey(key), local.stamp.time}});
    candidate->Prune(local.stamp.time - config_.lag);
    valid = candidate->Finalize(config_, started);
  } catch (const gtsam::IndeterminantLinearSystemException& error) {
    return {Reason::DEGENERATE, error.what()};
  }
  if (!valid.ok()) {
    return valid;
  }
  impl_.swap(candidate);
  return {};
}

Result FixedLagGraph::Observe(const GlobalObservation& observation) {
  if (!initialized()) {
    return {Reason::GLOBAL_UNAVAILABLE, "Graph has no verified anchor"};
  }
  auto valid = CheckObservation(observation);
  if (!valid.ok()) {
    return valid;
  }
  if (observation.epoch != impl_->output.epoch) {
    return {Reason::EPOCH_MISMATCH, "Observation ODOM epoch differs from graph"};
  }
  const auto& anchor = impl_->anchor;
  if (observation.map_id != anchor.map_id ||
      observation.map_version != anchor.map_version ||
      observation.calibration_id != anchor.calibration_id ||
      observation.stamp.clock_id != anchor.stamp.clock_id) {
    return {Reason::MAP_MISMATCH, "Observation graph/map/clock identity mismatch"};
  }
  const auto found = impl_->nodes.find(observation.stamp.time);
  if (found == impl_->nodes.end()) {
    return {Reason::HISTORY_UNAVAILABLE,
            "Observation needs an exact retained graph state"};
  }
  const auto identity = std::make_pair(observation.source,
                                     observation.stamp.sequence);
  const auto retired = impl_->retired_sequences.find(observation.source);
  if (impl_->observations.count(identity) != 0 ||
      (retired != impl_->retired_sequences.end() &&
       observation.stamp.sequence <= retired->second)) {
    return {Reason::DUPLICATE, "Observation identity already used or retired"};
  }
  for (const auto& accepted : impl_->observations) {
    if (accepted.first.first == observation.source &&
        accepted.second == observation.stamp.time) {
      return {Reason::DUPLICATE, "Source measurement time already consumed"};
    }
  }
  if (impl_->observations.size() >= config_.max_observations) {
    return {Reason::INVALID_INPUT, "Fixed-lag observation capacity exceeded"};
  }
  if (impl_->sources.count(observation.source) == 0 &&
      impl_->sources.size() >= 32) {
    return {Reason::INVALID_INPUT, "Fixed-lag source capacity exceeded"};
  }
  const auto started = Clock::now();
  std::unique_ptr<Impl> candidate(new Impl(*impl_));
  ObservationFactor factor(PoseKey(found->second.key), observation,
                           config_.robust_threshold);
  const auto predicted = candidate->smoother.calculateEstimate<gtsam::Pose3>(
      PoseKey(found->second.key));
  gtsam::Matrix jacobian;
  const gtsam::Vector residual = factor.evaluateError(predicted, &jacobian);
  const gtsam::Matrix innovation =
      jacobian * candidate->smoother.marginalCovariance(PoseKey(found->second.key)) *
      jacobian.transpose() + ObservationNoise(observation);
  Eigen::LLT<Eigen::MatrixXd> whiten(innovation);
  if (whiten.info() != Eigen::Success || !residual.allFinite()) {
    return {Reason::INVALID_INPUT, "Invalid graph innovation"};
  }
  const double distance = residual.dot(whiten.solve(residual));
  if (!std::isfinite(distance) || distance > config_.innovation_gate) {
    return {Reason::INNOVATION_REJECTED, "Graph observed-subspace innovation rejected"};
  }
  gtsam::NonlinearFactorGraph factors;
  factors.emplace_shared<ObservationFactor>(PoseKey(found->second.key),
                                            observation, config_.robust_threshold);
  try {
    candidate->smoother.update(factors);
    valid = candidate->Finalize(config_, started);
  } catch (const gtsam::IndeterminantLinearSystemException& error) {
    return {Reason::DEGENERATE, error.what()};
  }
  if (!valid.ok()) {
    return valid;
  }
  candidate->observations[identity] = observation.stamp.time;
  candidate->sources.insert(observation.source);
  candidate->output.last_observation =
      std::max(candidate->output.last_observation, observation.stamp.time);
  if (observation.kind == GlobalObservation::Kind::POSE) {
    candidate->output.last_full_observation =
        std::max(candidate->output.last_full_observation, observation.stamp.time);
  }
  if (observation.georeferenced) {
    candidate->output.georeferenced = true;
    candidate->output.last_georeference =
        std::max(candidate->output.last_georeference, observation.stamp.time);
  }
  impl_.swap(candidate);
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
