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

#include "modules/lidar_object_detection/inference/centerpoint_runtime_backend.h"

#include <dlfcn.h>

#include "modules/lidar_object_detection/inference/centerpoint_runtime_api.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/lidar_object_detection/proto/lidar_object_detection.pb.h"

namespace apollo {
namespace lidar_object_detection {

namespace {

constexpr std::size_t kMaxOutputDetections = 1000U;
constexpr std::uint32_t kCenterPointRuntimeAbiVersion = 1U;

using AbiVersionFunction = std::uint32_t (*)();
using StructSizeFunction = std::size_t (*)();
using CreateRuntimeFunction = void* (*)(const char*, const char*, int, char*,
                                        std::size_t);
using DestroyRuntimeFunction = void (*)(void*);
using InferFunction = bool (*)(void*, const float*, std::uint32_t,
                               CenterPointRuntimeDetection*, std::size_t,
                               std::size_t*, CenterPointRuntimeTimings*, char*,
                               std::size_t);

void SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

class CenterPointRuntimeBackend final : public CenterPointBackend {
 public:
  ~CenterPointRuntimeBackend() override {
    if (runtime_ != nullptr && destroy_runtime_ != nullptr) {
      destroy_runtime_(runtime_);
    }
    if (library_ != nullptr) {
      dlclose(library_);
    }
  }

  bool Init(const CenterPointModelConfig& config, std::string* error) override {
    if (config.runtime_library_path().empty()) {
      SetError("CenterPoint runtime_library_path is not configured", error);
      return false;
    }
    library_ =
        dlopen(config.runtime_library_path().c_str(), RTLD_NOW | RTLD_LOCAL);
    if (library_ == nullptr) {
      SetError("Unable to load CenterPoint runtime library " +
                   config.runtime_library_path() + ": " + dlerror(),
               error);
      return false;
    }
    create_runtime_ = reinterpret_cast<CreateRuntimeFunction>(
        dlsym(library_, "centerpoint_runtime_create"));
    destroy_runtime_ = reinterpret_cast<DestroyRuntimeFunction>(
        dlsym(library_, "centerpoint_runtime_destroy"));
    infer_ = reinterpret_cast<InferFunction>(
        dlsym(library_, "centerpoint_runtime_infer"));
    abi_version_ = reinterpret_cast<AbiVersionFunction>(
        dlsym(library_, "centerpoint_runtime_abi_version"));
    detection_size_ = reinterpret_cast<StructSizeFunction>(
        dlsym(library_, "centerpoint_runtime_detection_size"));
    timings_size_ = reinterpret_cast<StructSizeFunction>(
        dlsym(library_, "centerpoint_runtime_timings_size"));
    if (create_runtime_ == nullptr || destroy_runtime_ == nullptr ||
        infer_ == nullptr || abi_version_ == nullptr ||
        detection_size_ == nullptr || timings_size_ == nullptr) {
      SetError("CenterPoint runtime library is missing required C ABI symbols",
               error);
      return false;
    }
    if (abi_version_() != kCenterPointRuntimeAbiVersion ||
        detection_size_() != sizeof(CenterPointRuntimeDetection) ||
        timings_size_() != sizeof(CenterPointRuntimeTimings)) {
      SetError("CenterPoint runtime library has an incompatible C ABI", error);
      return false;
    }

    char error_buffer[1024] = {};
    runtime_ =
        create_runtime_(config.rpn_centerhead_engine_path().c_str(),
                        config.scn_onnx_path().c_str(), config.gpu_device_id(),
                        error_buffer, sizeof(error_buffer));
    if (runtime_ == nullptr) {
      SetError("Unable to initialize CenterPoint runtime: " +
                   std::string(error_buffer),
               error);
      return false;
    }
    return true;
  }

  bool Infer(const ModelInput& input,
             std::vector<CenterPointDetection>* detections,
             InferenceTimings* timings, std::string* error) override {
    if (runtime_ == nullptr || infer_ == nullptr || detections == nullptr ||
        timings == nullptr || input.points.empty() ||
        input.points.size() % 5U != 0U) {
      SetError("CenterPoint backend received invalid inference input", error);
      return false;
    }
    const std::size_t point_count = input.points.size() / 5U;
    if (point_count > std::numeric_limits<std::uint32_t>::max()) {
      SetError("CenterPoint input point count exceeds uint32 range", error);
      return false;
    }

    std::vector<CenterPointRuntimeDetection> runtime_detections(
        kMaxOutputDetections);
    std::size_t detection_count = 0U;
    CenterPointRuntimeTimings runtime_timings = {};
    char error_buffer[1024] = {};
    if (!infer_(runtime_, input.points.data(),
                static_cast<std::uint32_t>(point_count),
                runtime_detections.data(), runtime_detections.size(),
                &detection_count, &runtime_timings, error_buffer,
                sizeof(error_buffer))) {
      SetError("CenterPoint inference failed: " + std::string(error_buffer),
               error);
      return false;
    }
    if (detection_count > runtime_detections.size()) {
      SetError(
          "CenterPoint runtime returned an invalid detection "
          "count",
          error);
      return false;
    }

    std::vector<CenterPointDetection> output;
    output.reserve(detection_count);
    for (std::size_t index = 0U; index < detection_count; ++index) {
      const auto& source = runtime_detections[index];
      output.emplace_back();
      auto* detection = &output.back();
      detection->set_center_x(source.center_x);
      detection->set_center_y(source.center_y);
      detection->set_center_z(source.center_z);
      detection->set_width(source.width);
      detection->set_length(source.length);
      detection->set_height(source.height);
      detection->set_velocity_x(source.velocity_x);
      detection->set_velocity_y(source.velocity_y);
      detection->set_yaw(source.yaw);
      detection->set_class_id(source.class_id);
      detection->set_score(source.score);
    }

    timings->voxelization_ms = runtime_timings.voxelization_ms;
    timings->sparse_backbone_ms = runtime_timings.sparse_backbone_ms;
    timings->rpn_centerhead_ms = runtime_timings.rpn_centerhead_ms;
    timings->decode_nms_ms = runtime_timings.decode_nms_ms;
    timings->total_inference_ms =
        (runtime_timings.voxelization_ms + runtime_timings.sparse_backbone_ms +
         runtime_timings.rpn_centerhead_ms + runtime_timings.decode_nms_ms);
    if (!std::isfinite(timings->total_inference_ms) ||
        timings->total_inference_ms < 0.0) {
      SetError("CenterPoint runtime returned invalid timing values", error);
      return false;
    }
    *detections = std::move(output);
    return true;
  }

 private:
  void* library_ = nullptr;
  void* runtime_ = nullptr;
  CreateRuntimeFunction create_runtime_ = nullptr;
  DestroyRuntimeFunction destroy_runtime_ = nullptr;
  InferFunction infer_ = nullptr;
  AbiVersionFunction abi_version_ = nullptr;
  StructSizeFunction detection_size_ = nullptr;
  StructSizeFunction timings_size_ = nullptr;
};

}  // namespace

std::unique_ptr<CenterPointBackend> CreateCenterPointRuntimeBackend() {
  return std::make_unique<CenterPointRuntimeBackend>();
}

}  // namespace lidar_object_detection
}  // namespace apollo
