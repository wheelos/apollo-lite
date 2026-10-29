// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_semantic_segmentation/inference/cylinder_batch_norm.h"

#include <ATen/ATen.h>
#ifdef USE_GPU
#include <c10/cuda/CUDAException.h>
#include <c10/cuda/CUDAGuard.h>
#include <c10/cuda/CUDAStream.h>
#endif

namespace apollo {
namespace lidar_semantic_segmentation {
namespace cylinder {

at::Tensor BatchNorm(const at::Tensor& input, const at::Tensor& mean,
                     const at::Tensor& scale, const at::Tensor& bias,
                     BatchNormActivation activation) {
  TORCH_CHECK(input.dim() == 2 && input.size(1) > 0 &&
                  input.scalar_type() == at::kFloat && input.is_contiguous(),
              "Cylinder3D normalization requires contiguous FP32 rows");
  for (const auto& parameter : {mean, scale, bias}) {
    TORCH_CHECK(parameter.dim() == 1 && parameter.size(0) == input.size(1) &&
                    parameter.scalar_type() == at::kFloat &&
                    parameter.is_contiguous() &&
                    parameter.device() == input.device(),
                "Cylinder3D normalization parameter mismatch");
  }
#ifdef USE_GPU
  if (input.is_cuda()) {
    c10::cuda::CUDAGuard guard(input.device());
    auto output = at::empty_like(input);
    if (input.numel() != 0) {
      LaunchBatchNorm(input.data_ptr<float>(), mean.data_ptr<float>(),
                      scale.data_ptr<float>(), bias.data_ptr<float>(),
                      output.data_ptr<float>(), input.numel(), input.size(1),
                      activation,
                      reinterpret_cast<std::uintptr_t>(
                          c10::cuda::getCurrentCUDAStream().stream()));
      C10_CUDA_KERNEL_LAUNCH_CHECK();
    }
    return output;
  }
#endif
  auto values = activation == BatchNormActivation::kLeakyBefore
                    ? at::leaky_relu(input, 0.01)
                    : input;
  auto output = (values - mean) * scale + bias;
  if (activation == BatchNormActivation::kReluAfter) {
    output = at::relu(output);
  } else if (activation == BatchNormActivation::kSigmoidAfter) {
    output = at::sigmoid(output);
  }
  return output;
}

}  // namespace cylinder
}  // namespace lidar_semantic_segmentation
}  // namespace apollo
