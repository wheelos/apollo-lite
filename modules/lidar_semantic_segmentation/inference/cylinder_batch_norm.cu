// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include <cuda_runtime.h>

#include "modules/lidar_semantic_segmentation/inference/cylinder_batch_norm.h"

namespace apollo {
namespace lidar_semantic_segmentation {
namespace cylinder {
namespace {

__global__ void Normalize(const float* input, const float* mean,
                          const float* scale, const float* bias, float* output,
                          int64_t count, int64_t channels,
                          BatchNormActivation activation) {
  for (int64_t index =
           static_cast<int64_t>(blockIdx.x) * blockDim.x + threadIdx.x;
       index < count; index += static_cast<int64_t>(blockDim.x) * gridDim.x) {
    const auto channel = index % channels;
    float value = input[index];
    if (activation == BatchNormActivation::kLeakyBefore && !(value > 0)) {
      value = __fmul_rn(value, 0.01F);
    }
    // Keep the reference's three FP32 roundings; do not contract into FMA.
    value =
        __fadd_rn(__fmul_rn(__fsub_rn(value, mean[channel]), scale[channel]),
                  bias[channel]);
    if (activation == BatchNormActivation::kReluAfter && value < 0) {
      value = 0;
    } else if (activation == BatchNormActivation::kSigmoidAfter) {
      value = 1.0F / (1.0F + expf(-value));
    }
    output[index] = value;
  }
}

}  // namespace

void LaunchBatchNorm(const float* input, const float* mean, const float* scale,
                     const float* bias, float* output, int64_t count,
                     int64_t channels, BatchNormActivation activation,
                     std::uintptr_t stream) {
  constexpr int kThreads = 256;
  const int blocks = static_cast<int>((count + kThreads - 1) / kThreads > 4096
                                          ? 4096
                                          : (count + kThreads - 1) / kThreads);
  Normalize<<<blocks, kThreads, 0, reinterpret_cast<cudaStream_t>(stream)>>>(
      input, mean, scale, bias, output, count, channels, activation);
}

}  // namespace cylinder
}  // namespace lidar_semantic_segmentation
}  // namespace apollo
