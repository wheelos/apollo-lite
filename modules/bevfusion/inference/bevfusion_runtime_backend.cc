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

#include "modules/bevfusion/inference/bevfusion_runtime_backend.h"

#include <dlfcn.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string>

namespace apollo {
namespace bevfusion {

namespace {

constexpr std::size_t kDetectionCapacity = 4096;
constexpr std::uint32_t kRgb8Hwc = 1;

void SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

bool IsFiniteDetection(const BevFusionRuntimeDetection& detection) {
  return std::isfinite(detection.center_x) &&
         std::isfinite(detection.center_y) &&
         std::isfinite(detection.center_z) && std::isfinite(detection.width) &&
         detection.width > 0.0F && std::isfinite(detection.length) &&
         detection.length > 0.0F && std::isfinite(detection.height) &&
         detection.height > 0.0F && std::isfinite(detection.velocity_x) &&
         std::isfinite(detection.velocity_y) && std::isfinite(detection.yaw) &&
         std::isfinite(detection.score) && detection.score >= 0.0F &&
         detection.score <= 1.0F;
}

}  // namespace

BevFusionRuntimeBackend::~BevFusionRuntimeBackend() { Reset(); }

void BevFusionRuntimeBackend::Reset() {
  if (runtime_ != nullptr && destroy_ != nullptr) {
    destroy_(runtime_);
  }
  runtime_ = nullptr;
  if (library_ != nullptr) {
    dlclose(library_);
  }
  library_ = nullptr;
  abi_version_ = nullptr;
  create_ = nullptr;
  destroy_ = nullptr;
  infer_ = nullptr;
}

bool BevFusionRuntimeBackend::LoadSymbol(const char* symbol, void** function,
                                         std::string* error) {
  dlerror();
  *function = dlsym(library_, symbol);
  const char* load_error = dlerror();
  if (load_error != nullptr || *function == nullptr) {
    SetError(std::string("BEVFusion runtime is missing symbol ") + symbol +
                 (load_error == nullptr ? "" : std::string(": ") + load_error),
             error);
    return false;
  }
  return true;
}

bool BevFusionRuntimeBackend::Init(const BevFusionModelConfig& config,
                                   std::uint64_t max_timestamp_skew_ns,
                                   std::string* error) {
  Reset();
  if (config.runtime_library_path().empty() ||
      config.camera_backbone_engine_path().empty() ||
      config.lidar_scn_onnx_path().empty() ||
      config.camera_vtransform_engine_path().empty() ||
      config.transfusion_engine_path().empty() ||
      config.bbox_engine_path().empty() || config.model_revision().empty()) {
    SetError(
        "BEVFusion runtime and all five ResNet50 FP16 model artifacts "
        "and model_revision must be configured",
        error);
    return false;
  }
  if (max_timestamp_skew_ns == 0 || !config.has_confidence_threshold() ||
      config.max_input_point_count() == 0 ||
      config.max_input_point_count() > kMaxPointCount ||
      !std::isfinite(config.confidence_threshold()) ||
      config.confidence_threshold() < 0.0F ||
      config.confidence_threshold() > 1.0F) {
    SetError(
        "invalid BEVFusion timestamp tolerance, point limit, or "
        "confidence threshold",
        error);
    return false;
  }
  library_ =
      dlopen(config.runtime_library_path().c_str(), RTLD_NOW | RTLD_LOCAL);
  if (library_ == nullptr) {
    const char* load_error = dlerror();
    SetError(std::string("unable to load BEVFusion runtime library: ") +
                 (load_error == nullptr ? "unknown dlopen error" : load_error),
             error);
    return false;
  }

  void* symbol = nullptr;
  if (!LoadSymbol("apollo_bevfusion_runtime_abi_version", &symbol, error)) {
    Reset();
    return false;
  }
  abi_version_ = reinterpret_cast<AbiVersionFunction>(symbol);
  if (!LoadSymbol("apollo_bevfusion_runtime_create", &symbol, error)) {
    Reset();
    return false;
  }
  create_ = reinterpret_cast<CreateFunction>(symbol);
  if (!LoadSymbol("apollo_bevfusion_runtime_destroy", &symbol, error)) {
    Reset();
    return false;
  }
  destroy_ = reinterpret_cast<DestroyFunction>(symbol);
  if (!LoadSymbol("apollo_bevfusion_runtime_infer", &symbol, error)) {
    Reset();
    return false;
  }
  infer_ = reinterpret_cast<InferFunction>(symbol);
  if (abi_version_() != APOLLO_BEVFUSION_RUNTIME_ABI_VERSION) {
    SetError("BEVFusion runtime ABI version mismatch", error);
    Reset();
    return false;
  }

  ApolloBevFusionRuntimeConfigV1 runtime_config{};
  runtime_config.struct_size = sizeof(runtime_config);
  runtime_config.abi_version = APOLLO_BEVFUSION_RUNTIME_ABI_VERSION;
  runtime_config.gpu_device_id = config.gpu_device_id();
  runtime_config.max_input_point_count = config.max_input_point_count();
  runtime_config.max_timestamp_skew_ns = max_timestamp_skew_ns;
  runtime_config.confidence_threshold = config.confidence_threshold();
  runtime_config.camera_backbone_engine_path =
      config.camera_backbone_engine_path().c_str();
  runtime_config.lidar_scn_onnx_path = config.lidar_scn_onnx_path().c_str();
  runtime_config.camera_vtransform_engine_path =
      config.camera_vtransform_engine_path().c_str();
  runtime_config.transfusion_engine_path =
      config.transfusion_engine_path().c_str();
  runtime_config.bbox_engine_path = config.bbox_engine_path().c_str();
  std::array<char, 1024> runtime_error{};
  if (create_(&runtime_config, &runtime_, runtime_error.data(),
              runtime_error.size()) != 0) {
    SetError(std::string("BEVFusion runtime initialization failed: ") +
                 runtime_error.data(),
             error);
    Reset();
    return false;
  }

  limits_.max_timestamp_skew_ns = max_timestamp_skew_ns;
  limits_.max_point_count = config.max_input_point_count();
  config_ = config;
  return true;
}

bool BevFusionRuntimeBackend::Infer(const BevFusionInputView& input,
                                    BevFusionInferenceResult* result,
                                    std::string* error) {
  if (runtime_ == nullptr || infer_ == nullptr) {
    SetError("BEVFusion runtime backend is not initialized", error);
    return false;
  }
  if (result == nullptr) {
    SetError("BEVFusion inference result output is null", error);
    return false;
  }
  if (!ValidateInput(input, limits_, error)) {
    return false;
  }

  ApolloBevFusionInputV1 runtime_input{};
  runtime_input.struct_size = sizeof(runtime_input);
  runtime_input.abi_version = APOLLO_BEVFUSION_RUNTIME_ABI_VERSION;
  runtime_input.timestamp_ns = input.timestamp_ns;
  for (std::size_t camera = 0; camera < kCameraCount; ++camera) {
    const auto& source = input.cameras[camera];
    runtime_input.camera_name[camera] = source.camera_name;
    runtime_input.camera_timestamp_ns[camera] = source.timestamp_ns;
    runtime_input.image_width[camera] = source.width;
    runtime_input.image_height[camera] = source.height;
    runtime_input.image_stride_bytes[camera] =
        static_cast<std::uint32_t>(source.stride_bytes);
    runtime_input.pixel_format[camera] =
        source.pixel_format == PixelFormat::kRgb8Hwc ? kRgb8Hwc : 0;
    runtime_input.image_data[camera] = source.data;
    runtime_input.image_byte_count[camera] = source.byte_count;
  }
  runtime_input.lidar_points = input.lidar_points;
  runtime_input.point_count = input.point_count;
  runtime_input.point_feature_count = input.point_feature_count;
  runtime_input.camera_to_lidar = input.camera_to_lidar.data();
  runtime_input.camera_intrinsics = input.camera_intrinsics.data();
  runtime_input.lidar_to_image = input.lidar_to_image.data();
  runtime_input.image_augmentation = input.image_augmentation.data();

  std::array<ApolloBevFusionDetectionV1, kDetectionCapacity> detections{};
  std::size_t detection_count = 0;
  std::array<char, 1024> runtime_error{};
  const auto start = std::chrono::steady_clock::now();
  if (infer_(runtime_, &runtime_input, detections.data(), detections.size(),
             &detection_count, runtime_error.data(),
             runtime_error.size()) != 0) {
    SetError(std::string("BEVFusion inference failed: ") + runtime_error.data(),
             error);
    return false;
  }
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - start)
                                .count();

  BevFusionInferenceResult output;
  output.timestamp_ns = input.timestamp_ns;
  output.input_point_count = input.point_count;
  output.inference_ms = elapsed_ms;
  output.detections.reserve(detection_count);
  for (std::size_t index = 0; index < detection_count; ++index) {
    const auto& source = detections[index];
    BevFusionRuntimeDetection detection;
    detection.center_x = source.center_x;
    detection.center_y = source.center_y;
    detection.center_z = source.center_z;
    detection.width = source.width;
    detection.length = source.length;
    detection.height = source.height;
    detection.velocity_x = source.velocity_x;
    detection.velocity_y = source.velocity_y;
    detection.yaw = source.yaw;
    detection.score = source.score;
    detection.class_id = source.class_id;
    if (!IsFiniteDetection(detection)) {
      SetError(
          "NVIDIA BEVFusion runtime returned a non-finite or invalid "
          "detection",
          error);
      return false;
    }
    output.detections.push_back(detection);
  }
  *result = std::move(output);
  return true;
}

}  // namespace bevfusion
}  // namespace apollo
