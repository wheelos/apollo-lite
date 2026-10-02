// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <vector>

#include "modules/localization/core/types.h"

namespace apollo {
namespace localization {
namespace unified {

using Matrix6x15d = Eigen::Matrix<double, 6, 15>;

void RelativeMotionJacobians(const Eigen::Isometry3d& start_pose,
                             const Eigen::Isometry3d& delta,
                             Matrix6x15d* start, Matrix6x15d* end);

struct SourceSampleId {
  std::string source;
  uint64_t sequence = 0;
};

struct MotionIncrement {
  Epoch epoch;
  Stamp start;
  Stamp end;
  Eigen::Isometry3d start_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
  // [translation in start body, right attitude error in end body].
  Matrix6d covariance = Matrix6d::Zero();
  Matrix15d start_covariance = Matrix15d::Zero();
  Matrix15d end_covariance = Matrix15d::Zero();
  // E[error_start * error_end^T], with frozen, unsmoothed start state.
  Matrix15d cross_covariance = Matrix15d::Zero();
  std::vector<SourceSampleId> sources;
  // All increments from one local filter share bias/history information.
  // This joint marginal is NOT an independent between-factor covariance.
  std::string correlation_group;
};

Result ComputeMotionIncrement(const LocalState& start, const LocalState& end,
                              const Matrix15d& cross_covariance,
                              const std::vector<SourceSampleId>& sources,
                              MotionIncrement* output);

}  // namespace unified
}  // namespace localization
}  // namespace apollo
