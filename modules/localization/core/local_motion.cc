// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/core/local_motion.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

#include "Eigen/Cholesky"
#include "Eigen/Eigenvalues"

namespace apollo {
namespace localization {
namespace unified {
namespace {

bool ValidEndpoint(const LocalState& state) {
  return state.valid && state.covariance_model_valid &&
         !state.epoch.session.empty() &&
         state.epoch.generation != 0 && !state.stamp.clock_id.empty() &&
         state.stamp.sequence != 0 && std::isfinite(state.stamp.time) &&
         state.stamp.time > 0.0 && state.position.allFinite() &&
         state.orientation.coeffs().allFinite() &&
         std::abs(state.orientation.norm() - 1.0) < 1e-9 &&
         state.covariance.allFinite() &&
         state.covariance.isApprox(state.covariance.transpose(), 1e-9) &&
         Eigen::LLT<Matrix15d>(state.covariance).info() == Eigen::Success;
}

}  // namespace

void RelativeMotionJacobians(const Eigen::Isometry3d& start_pose,
                             const Eigen::Isometry3d& delta,
                             Matrix6x15d* start, Matrix6x15d* end) {
  start->setZero();
  end->setZero();
  start->block<3, 3>(0, 0) = -start_pose.linear().transpose();
  start->block<3, 3>(0, 6) = Skew(delta.translation());
  end->block<3, 3>(0, 0) = start_pose.linear().transpose();
  start->block<3, 3>(3, 6) = -delta.linear().transpose();
  end->block<3, 3>(3, 6).setIdentity();
}

Result ComputeMotionIncrement(const LocalState& start, const LocalState& end,
                              const Matrix15d& cross_covariance,
                              const std::vector<SourceSampleId>& sources,
                              MotionIncrement* output) {
  if (output == nullptr || !ValidEndpoint(start) || !ValidEndpoint(end) ||
      !cross_covariance.allFinite()) {
    return {Reason::INVALID_INPUT, "Invalid local motion endpoint/covariance"};
  }
  if (start.epoch != end.epoch) {
    return {Reason::EPOCH_MISMATCH, "Motion cannot cross an ODOM reset"};
  }
  if (start.stamp.clock_id != end.stamp.clock_id) {
    return {Reason::CLOCK_INVALID, "Motion endpoints use different clocks"};
  }
  if (end.stamp.time <= start.stamp.time ||
      end.stamp.sequence <= start.stamp.sequence) {
    return {Reason::TIMESTAMP_REGRESSION, "Motion endpoints must advance"};
  }
  bool has_start_imu = false;
  bool has_end_imu = false;
  std::set<std::pair<std::string, uint64_t>> identities;
  for (const auto& source : sources) {
    if (source.source.empty() || source.sequence == 0 ||
        !identities.emplace(source.source, source.sequence).second) {
      return {Reason::INVALID_INPUT, "Invalid/duplicate motion source sample"};
    }
    if (source.source == "imu") {
      has_start_imu = has_start_imu || source.sequence == start.stamp.sequence;
      has_end_imu = has_end_imu || source.sequence == end.stamp.sequence;
    }
  }
  if (!has_start_imu || !has_end_imu) {
    return {Reason::INVALID_INPUT, "Motion lacks its actual endpoint IMU samples"};
  }

  Eigen::Matrix<double, 30, 30> joint;
  joint.topLeftCorner<15, 15>() = start.covariance;
  joint.bottomRightCorner<15, 15>() = end.covariance;
  joint.topRightCorner<15, 15>() = cross_covariance;
  joint.bottomLeftCorner<15, 15>() = cross_covariance.transpose();
  // Check the joint distribution, not merely its two positive marginals.
  const Eigen::Matrix<double, 30, 1> scale =
      joint.diagonal().cwiseSqrt().cwiseInverse();
  const Eigen::Matrix<double, 30, 30> correlation =
      scale.asDiagonal() * joint * scale.asDiagonal();
  if (!correlation.allFinite()) {
    return {Reason::INVALID_INPUT, "Local joint covariance scale overflow"};
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 30, 30>> spectrum(
      correlation, Eigen::EigenvaluesOnly);
  if (spectrum.info() != Eigen::Success ||
      spectrum.eigenvalues().minCoeff() < -1e-9) {
    return {Reason::INVALID_INPUT, "Local cross-covariance is not jointly PSD"};
  }

  MotionIncrement next;
  next.epoch = end.epoch;
  next.start = start.stamp;
  next.end = end.stamp;
  next.start_pose = Pose(start);
  next.delta = Pose(start).inverse() * Pose(end);
  next.start_covariance = start.covariance;
  next.end_covariance = end.covariance;
  next.cross_covariance = cross_covariance;
  next.sources = sources;
  next.correlation_group =
      end.epoch.session + ":" + std::to_string(end.epoch.generation);

  Eigen::Matrix<double, 6, 30> jacobian =
      Eigen::Matrix<double, 6, 30>::Zero();
  Matrix6x15d first;
  Matrix6x15d second;
  RelativeMotionJacobians(next.start_pose, next.delta, &first, &second);
  jacobian.leftCols<15>() = first;
  jacobian.rightCols<15>() = second;
  next.covariance = jacobian * joint * jacobian.transpose();
  next.covariance =
      (0.5 * (next.covariance + next.covariance.transpose())).eval();
  Eigen::SelfAdjointEigenSolver<Matrix6d> relative(next.covariance);
  if (!next.delta.matrix().allFinite() || !next.covariance.allFinite() ||
      relative.info() != Eigen::Success ||
      relative.eigenvalues().minCoeff() <
          -1e-9 * std::max(1.0, next.covariance.norm())) {
    return {Reason::INVALID_INPUT, "Relative motion covariance is not PSD"};
  }
  *output = next;
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
