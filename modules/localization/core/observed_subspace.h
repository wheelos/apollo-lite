// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "modules/localization/core/types.h"

namespace apollo {
namespace localization {
namespace unified {

// Preserve the residual information while removing arbitrary eigenbasis signs,
// permutations and rotations. Metric is [translation, rotation_length * angle].
Result CanonicalizeObservedSubspace(const Eigen::MatrixXd& projection,
                                   const Eigen::MatrixXd& covariance,
                                   double rotation_length,
                                   Eigen::MatrixXd* canonical_projection,
                                   Eigen::MatrixXd* canonical_covariance);

}  // namespace unified
}  // namespace localization
}  // namespace apollo
