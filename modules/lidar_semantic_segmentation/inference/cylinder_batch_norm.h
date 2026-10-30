// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

#include <ATen/core/Tensor.h>

namespace apollo {
namespace lidar_semantic_segmentation {
namespace cylinder {

enum class BatchNormActivation {
  kNone,
  kReluAfter,
  kLeakyBefore,
  kSigmoidAfter
};

at::Tensor BatchNorm(const at::Tensor& input, const at::Tensor& mean,
                     const at::Tensor& scale, const at::Tensor& bias,
                     BatchNormActivation activation);

#ifdef USE_GPU
void LaunchBatchNorm(const float* input, const float* mean, const float* scale,
                     const float* bias, float* output, int64_t count,
                     int64_t channels, BatchNormActivation activation,
                     std::uintptr_t stream);
#endif

}  // namespace cylinder
}  // namespace lidar_semantic_segmentation
}  // namespace apollo
