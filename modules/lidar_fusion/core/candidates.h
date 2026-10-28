// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <vector>

#include "modules/lidar_fusion/proto/lidar_fusion.pb.h"

namespace apollo {
namespace lidar_fusion {

struct CandidateWork {
  uint64_t pairs = 0;
  uint64_t point_transforms = 0;
};

bool ValidateCandidates(const FusionConfig& config);
std::vector<int> CandidateComponents(const FusionConfig& config,
                                     const FusionOutput& frame);
void RefineCandidates(const FusionConfig& config, FusionOutput* frame,
                      CandidateWork* work = nullptr);

}  // namespace lidar_fusion
}  // namespace apollo
