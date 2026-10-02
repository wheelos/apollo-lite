// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/core/global_types.h"

namespace apollo {
namespace localization {
namespace unified {

Matrix6d TransformCovarianceBound(const Eigen::Isometry3d& map_base,
                                  const LocalState& local,
                                  const Matrix6d& observation_covariance) {
  const Eigen::Matrix3d rotation =
      map_base.linear() * local.orientation.toRotationMatrix().transpose();
  Matrix6d observation_jacobian = Matrix6d::Identity();
  observation_jacobian.topRightCorner<3, 3>() =
      Skew(rotation * local.position);
  Matrix6d local_jacobian = Matrix6d::Zero();
  local_jacobian.topLeftCorner<3, 3>() = -rotation;
  local_jacobian.topRightCorner<3, 3>() =
      -rotation * Skew(local.position) *
      local.orientation.toRotationMatrix();
  local_jacobian.bottomRightCorner<3, 3>() = -map_base.linear();
  // 2(Pa + Pb) bounds unknown cross-correlation; independence is not assumed.
  return 2.0 * (observation_jacobian * observation_covariance *
                    observation_jacobian.transpose() +
                local_jacobian * PoseCovariance(local) *
                    local_jacobian.transpose());
}

Matrix6d GlobalPoseCovarianceBound(const GlobalState& global,
                                   const LocalState& local) {
  Matrix6d alignment_jacobian = Matrix6d::Identity();
  alignment_jacobian.topRightCorner<3, 3>() =
      -Skew(global.map_to_odom.linear() * local.position);
  Matrix6d local_jacobian = Matrix6d::Zero();
  local_jacobian.topLeftCorner<3, 3>() = global.map_to_odom.linear();
  local_jacobian.bottomRightCorner<3, 3>() =
      global.map_to_odom.linear() * local.orientation.toRotationMatrix();
  return 2.0 * (alignment_jacobian * global.covariance *
                    alignment_jacobian.transpose() +
                local_jacobian * PoseCovariance(local) *
                    local_jacobian.transpose());
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
