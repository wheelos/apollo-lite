// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "centerpoint_runtime_api.h"

#include <algorithm>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <cuda_runtime.h>

#include "centerpoint.h"

namespace {

constexpr std::uint32_t kFeatureCount = 5U;
constexpr std::uint32_t kMaxInputPoints = 300000U;
constexpr std::size_t kMaxOutputDetections = 1000U;

void SetError(const std::string& message, char* error,
              std::size_t error_capacity) {
  if (error == nullptr || error_capacity == 0U) {
    return;
  }
  const std::size_t copy_size = std::min(error_capacity - 1U, message.size());
  std::memcpy(error, message.data(), copy_size);
  error[copy_size] = '\0';
}

struct Runtime {
  explicit Runtime(int device) : gpu_device_id(device) {}

  ~Runtime() {
    cudaSetDevice(gpu_device_id);
    centerpoint.reset();
    if (device_points != nullptr) {
      cudaFree(device_points);
    }
    if (stream != nullptr) {
      cudaStreamDestroy(stream);
    }
  }

  int gpu_device_id;
  cudaStream_t stream = nullptr;
  float* device_points = nullptr;
  std::unique_ptr<CenterPoint> centerpoint;
  std::mutex mutex;
};

}  // namespace

extern "C" void* centerpoint_runtime_create(
    const char* rpn_centerhead_engine_path, const char* scn_onnx_path,
    int gpu_device_id, char* error, std::size_t error_capacity) {
  if (rpn_centerhead_engine_path == nullptr ||
      rpn_centerhead_engine_path[0] == '\0' || scn_onnx_path == nullptr ||
      scn_onnx_path[0] == '\0') {
    SetError("CenterPoint model paths must not be empty", error,
             error_capacity);
    return nullptr;
  }
  try {
    auto runtime = std::make_unique<Runtime>(gpu_device_id);
    cudaError_t status = cudaSetDevice(gpu_device_id);
    if (status != cudaSuccess) {
      SetError(cudaGetErrorString(status), error, error_capacity);
      return nullptr;
    }
    status = cudaStreamCreate(&runtime->stream);
    if (status != cudaSuccess) {
      SetError(cudaGetErrorString(status), error, error_capacity);
      return nullptr;
    }
    status = cudaMalloc(reinterpret_cast<void**>(&runtime->device_points),
                        static_cast<std::size_t>(kMaxInputPoints) *
                            kFeatureCount * sizeof(float));
    if (status != cudaSuccess) {
      SetError(cudaGetErrorString(status), error, error_capacity);
      return nullptr;
    }
    runtime->centerpoint = std::make_unique<CenterPoint>(
        rpn_centerhead_engine_path, scn_onnx_path, false);
    if (runtime->centerpoint->prepare() != 0) {
      SetError("CenterPoint preprocessing initialization failed", error,
               error_capacity);
      return nullptr;
    }
    return runtime.release();
  } catch (const std::exception& exception) {
    SetError(exception.what(), error, error_capacity);
    return nullptr;
  }
}

extern "C" void centerpoint_runtime_destroy(void* handle) {
  delete static_cast<Runtime*>(handle);
}

extern "C" bool centerpoint_runtime_infer(
    void* handle, const float* points, std::uint32_t point_count,
    CenterPointRuntimeDetection* detections, std::size_t detection_capacity,
    std::size_t* detection_count, CenterPointRuntimeTimings* timings,
    char* error, std::size_t error_capacity) {
  auto* runtime = static_cast<Runtime*>(handle);
  if (runtime == nullptr || points == nullptr || point_count == 0U ||
      point_count > kMaxInputPoints || detections == nullptr ||
      detection_count == nullptr || timings == nullptr) {
    SetError("CenterPoint runtime received invalid inference arguments", error,
             error_capacity);
    return false;
  }

  std::lock_guard<std::mutex> lock(runtime->mutex);
  cudaError_t status = cudaSetDevice(runtime->gpu_device_id);
  if (status != cudaSuccess) {
    SetError(cudaGetErrorString(status), error, error_capacity);
    return false;
  }
  const std::size_t input_bytes =
      static_cast<std::size_t>(point_count) * kFeatureCount * sizeof(float);
  status = cudaMemcpyAsync(runtime->device_points, points, input_bytes,
                           cudaMemcpyHostToDevice, runtime->stream);
  if (status != cudaSuccess) {
    SetError(cudaGetErrorString(status), error, error_capacity);
    return false;
  }
  if (runtime->centerpoint->doinfer(runtime->device_points, point_count,
                                    runtime->stream) != 0) {
    SetError("CenterPoint inference execution failed", error,
             error_capacity);
    return false;
  }
  const auto& output = runtime->centerpoint->nms_pred_;
  if (output.size() > detection_capacity ||
      output.size() > kMaxOutputDetections) {
    SetError("CenterPoint output exceeds the configured detection capacity",
             error, error_capacity);
    return false;
  }
  for (std::size_t index = 0U; index < output.size(); ++index) {
    const auto& source = output[index];
    detections[index] = {source.x,  source.y,  source.z,    source.w,
                         source.l,  source.h,  source.vx,   source.vy,
                         source.rt, source.id, source.score};
  }
  *detection_count = output.size();
  timings->voxelization_ms = runtime->centerpoint->voxelization_ms();
  timings->sparse_backbone_ms = runtime->centerpoint->sparse_backbone_ms();
  timings->rpn_centerhead_ms = runtime->centerpoint->rpn_centerhead_ms();
  timings->decode_nms_ms = runtime->centerpoint->decode_nms_ms();
  return true;
}
