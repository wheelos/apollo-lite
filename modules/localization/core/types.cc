// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/core/types.h"

#include <cmath>

#include "Eigen/Cholesky"

namespace apollo {
namespace localization {
namespace unified {

Eigen::Isometry3d Pose(const LocalState& state) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = state.orientation.toRotationMatrix();
  pose.translation() = state.position;
  return pose;
}

Matrix6d PoseCovariance(const LocalState& state) {
  Matrix6d covariance;
  covariance.topLeftCorner<3, 3>() = state.covariance.block<3, 3>(0, 0);
  covariance.topRightCorner<3, 3>() = state.covariance.block<3, 3>(0, 6);
  covariance.bottomLeftCorner<3, 3>() = state.covariance.block<3, 3>(6, 0);
  covariance.bottomRightCorner<3, 3>() = state.covariance.block<3, 3>(6, 6);
  return covariance;
}

Eigen::Matrix3d Skew(const Eigen::Vector3d& value) {
  Eigen::Matrix3d result;
  result << 0.0, -value.z(), value.y(), value.z(), 0.0, -value.x(),
      -value.y(), value.x(), 0.0;
  return result;
}

Eigen::Quaterniond ExpRotation(const Eigen::Vector3d& value) {
  const double angle = value.norm();
  if (angle < 1e-12) {
    return Eigen::Quaterniond(1.0, value.x() / 2.0, value.y() / 2.0,
                              value.z() / 2.0).normalized();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, value / angle));
}

Eigen::Vector3d LogRotation(const Eigen::Quaterniond& value) {
  Eigen::Quaterniond q = value.normalized();
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }
  const Eigen::AngleAxisd angle(q);
  return angle.angle() * angle.axis();
}

bool ValidCovariance(const Matrix6d& covariance) {
  if (!covariance.allFinite() ||
      !covariance.isApprox(covariance.transpose(), 1e-9)) {
    return false;
  }
  Eigen::LLT<Matrix6d> decomposition(covariance);
  return decomposition.info() == Eigen::Success;
}

const char* ReasonName(Reason reason) {
  switch (reason) {
    case Reason::NONE: return "NONE";
    case Reason::CONFIG_INVALID: return "CONFIG_INVALID";
    case Reason::WAITING_FOR_STATIONARY: return "WAITING_FOR_STATIONARY";
    case Reason::WAITING_FOR_IMU: return "WAITING_FOR_IMU";
    case Reason::IMU_STALE: return "IMU_STALE";
    case Reason::WHEEL_STALE: return "WHEEL_STALE";
    case Reason::INVALID_INPUT: return "INVALID_INPUT";
    case Reason::DUPLICATE: return "DUPLICATE";
    case Reason::TIMESTAMP_REGRESSION: return "TIMESTAMP_REGRESSION";
    case Reason::CLOCK_INVALID: return "CLOCK_INVALID";
    case Reason::IMU_GAP: return "IMU_GAP";
    case Reason::COVARIANCE_EXCEEDED: return "COVARIANCE_EXCEEDED";
    case Reason::INNOVATION_REJECTED: return "INNOVATION_REJECTED";
    case Reason::HISTORY_UNAVAILABLE: return "HISTORY_UNAVAILABLE";
    case Reason::EPOCH_MISMATCH: return "EPOCH_MISMATCH";
    case Reason::RESET_WHILE_MOVING: return "RESET_WHILE_MOVING";
    case Reason::GLOBAL_UNAVAILABLE: return "GLOBAL_UNAVAILABLE";
    case Reason::GLOBAL_STALE: return "GLOBAL_STALE";
    case Reason::MAP_MISMATCH: return "MAP_MISMATCH";
    case Reason::MAP_NOT_READY: return "MAP_NOT_READY";
    case Reason::MATCH_FAILED: return "MATCH_FAILED";
    case Reason::DEGENERATE: return "DEGENERATE";
    case Reason::RELOCALIZATION_AMBIGUOUS: return "RELOCALIZATION_AMBIGUOUS";
    case Reason::RELOCALIZATION_VERIFYING: return "RELOCALIZATION_VERIFYING";
    case Reason::LIDAR_STALE: return "LIDAR_STALE";
    case Reason::POINT_TIME_MISSING: return "POINT_TIME_MISSING";
    case Reason::GEOMETRY_UNAVAILABLE: return "GEOMETRY_UNAVAILABLE";
    case Reason::CORRECTION_BUDGET_EXCEEDED:
      return "CORRECTION_BUDGET_EXCEEDED";
    case Reason::CORRELATION_UNQUALIFIED:
      return "CORRELATION_UNQUALIFIED";
  }
  return "UNKNOWN";
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
