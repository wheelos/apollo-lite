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

#include "modules/bevfusion/inference/bevfusion_runtime_api.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <exception>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <cuda_fp16.h>
#include <cuda_runtime.h>

#include "modules/bevfusion/third_party/bevfusion/bevfusion/bevfusion.hpp"
#include "modules/bevfusion/types/bevfusion_input.h"

namespace {

constexpr size_t kDetectionCapacity = 4096;
constexpr uint32_t kRgb8Hwc = 1;

struct Runtime {
  std::shared_ptr<bevfusion::Core> core;
  cudaStream_t stream = nullptr;
  uint32_t max_points = 0;
  uint64_t max_timestamp_skew_ns = 0;

  ~Runtime() {
    if (stream != nullptr) {
      cudaStreamDestroy(stream);
    }
  }
};

void SetError(const std::string& message, char* output, size_t capacity) {
  if (output == nullptr || capacity == 0) {
    return;
  }
  std::strncpy(output, message.c_str(), capacity - 1);
  output[capacity - 1] = '\0';
}

bool IsConfiguredPath(const char* path) {
  return path != nullptr && path[0] != '\0';
}

bool IsReadableNonemptyFile(const char* path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return false;
  }
  input.seekg(0, std::ios::end);
  return input.tellg() > 0;
}

bevfusion::CoreParameter MakeCoreParameter(
    const ApolloBevFusionRuntimeConfigV1& config) {
  using namespace ::bevfusion;

  camera::NormalizationParameter normalization{};
  normalization.image_width = 1600;
  normalization.image_height = 900;
  normalization.output_width = 704;
  normalization.output_height = 256;
  normalization.num_camera = APOLLO_BEVFUSION_CAMERA_COUNT;
  normalization.resize_lim = 0.48F;
  normalization.interpolation = camera::Interpolation::Bilinear;
  const float mean[3] = {0.485F, 0.456F, 0.406F};
  const float standard_deviation[3] = {0.229F, 0.224F, 0.225F};
  normalization.method =
      camera::NormMethod::mean_std(mean, standard_deviation, 1.0F / 255.0F);

  lidar::VoxelizationParameter voxelization{};
  voxelization.min_range = nvtype::Float3(-54.0F, -54.0F, -5.0F);
  voxelization.max_range = nvtype::Float3(54.0F, 54.0F, 3.0F);
  voxelization.voxel_size = nvtype::Float3(0.075F, 0.075F, 0.2F);
  voxelization.grid_size = voxelization.compute_grid_size(
      voxelization.max_range, voxelization.min_range, voxelization.voxel_size);
  voxelization.max_points_per_voxel = 10;
  voxelization.max_points = config.max_input_point_count;
  voxelization.max_voxels = 160000;
  voxelization.num_feature = 5;

  lidar::SCNParameter scn{};
  scn.voxelization = voxelization;
  scn.model = config.lidar_scn_onnx_path;
  scn.order = lidar::CoordinateOrder::XYZ;
  scn.precision = lidar::Precision::Float16;

  camera::GeometryParameter geometry{};
  geometry.xbound = nvtype::Float3(-54.0F, 54.0F, 0.3F);
  geometry.ybound = nvtype::Float3(-54.0F, 54.0F, 0.3F);
  geometry.zbound = nvtype::Float3(-10.0F, 10.0F, 20.0F);
  geometry.dbound = nvtype::Float3(1.0F, 60.0F, 0.5F);
  geometry.image_width = 704;
  geometry.image_height = 256;
  geometry.feat_width = 88;
  geometry.feat_height = 32;
  geometry.num_camera = APOLLO_BEVFUSION_CAMERA_COUNT;
  geometry.geometry_dim = nvtype::Int3(360, 360, 80);

  head::transbbox::TransBBoxParameter bbox{};
  bbox.model = config.bbox_engine_path;
  bbox.out_size_factor = 8;
  bbox.pc_range = nvtype::Float2(-54.0F, -54.0F);
  bbox.post_center_range_start = nvtype::Float3(-61.2F, -61.2F, -10.0F);
  bbox.post_center_range_end = nvtype::Float3(61.2F, 61.2F, 10.0F);
  bbox.voxel_size = nvtype::Float2(0.075F, 0.075F);
  bbox.confidence_threshold = config.confidence_threshold;
  bbox.sorted_bboxes = true;

  CoreParameter parameter;
  parameter.camera_model = config.camera_backbone_engine_path;
  parameter.camera_vtransform = config.camera_vtransform_engine_path;
  parameter.geometry = geometry;
  parameter.normalize = normalization;
  parameter.lidar_scn = scn;
  parameter.transfusion = config.transfusion_engine_path;
  parameter.transbbox = bbox;
  return parameter;
}

bool ValidateApiInput(const ApolloBevFusionInputV1& source, uint32_t max_points,
                      uint64_t max_skew_ns, std::string* error) {
  if (source.struct_size != sizeof(ApolloBevFusionInputV1) ||
      source.abi_version != APOLLO_BEVFUSION_RUNTIME_ABI_VERSION) {
    *error = "BEVFusion input ABI version or structure size mismatch";
    return false;
  }
  if (source.camera_to_lidar == nullptr ||
      source.camera_intrinsics == nullptr || source.lidar_to_image == nullptr ||
      source.image_augmentation == nullptr) {
    *error = "BEVFusion calibration arrays are missing";
    return false;
  }
  apollo::bevfusion::BevFusionInputView input;
  input.timestamp_ns = source.timestamp_ns;
  input.point_count = source.point_count;
  input.point_feature_count = source.point_feature_count;
  input.lidar_points = source.lidar_points;
  for (size_t camera = 0; camera < APOLLO_BEVFUSION_CAMERA_COUNT; ++camera) {
    input.cameras[camera].camera_name = source.camera_name[camera];
    input.cameras[camera].timestamp_ns = source.camera_timestamp_ns[camera];
    input.cameras[camera].width = source.image_width[camera];
    input.cameras[camera].height = source.image_height[camera];
    input.cameras[camera].stride_bytes = source.image_stride_bytes[camera];
    input.cameras[camera].pixel_format =
        source.pixel_format[camera] == kRgb8Hwc
            ? apollo::bevfusion::PixelFormat::kRgb8Hwc
            : apollo::bevfusion::PixelFormat::kOther;
    input.cameras[camera].data = source.image_data[camera];
    input.cameras[camera].byte_count = source.image_byte_count[camera];
  }
  std::copy_n(source.camera_to_lidar, input.camera_to_lidar.size(),
              input.camera_to_lidar.begin());
  std::copy_n(source.camera_intrinsics, input.camera_intrinsics.size(),
              input.camera_intrinsics.begin());
  std::copy_n(source.lidar_to_image, input.lidar_to_image.size(),
              input.lidar_to_image.begin());
  std::copy_n(source.image_augmentation, input.image_augmentation.size(),
              input.image_augmentation.begin());
  apollo::bevfusion::InputLimits limits;
  limits.max_timestamp_skew_ns = max_skew_ns;
  limits.max_point_count = max_points;
  return apollo::bevfusion::ValidateInput(input, limits, error);
}

}  // namespace

extern "C" uint32_t apollo_bevfusion_runtime_abi_version(void) {
  return APOLLO_BEVFUSION_RUNTIME_ABI_VERSION;
}

extern "C" int apollo_bevfusion_runtime_create(
    const ApolloBevFusionRuntimeConfigV1* config, void** runtime, char* error,
    size_t error_capacity) {
  try {
    if (runtime == nullptr || config == nullptr ||
        config->struct_size != sizeof(ApolloBevFusionRuntimeConfigV1) ||
        config->abi_version != APOLLO_BEVFUSION_RUNTIME_ABI_VERSION) {
      SetError(
          "BEVFusion runtime config ABI version or structure size "
          "mismatch",
          error, error_capacity);
      return 1;
    }
    *runtime = nullptr;
    if (config->gpu_device_id < 0 || config->max_input_point_count == 0 ||
        config->max_input_point_count > apollo::bevfusion::kMaxPointCount ||
        config->max_timestamp_skew_ns == 0 ||
        !std::isfinite(config->confidence_threshold) ||
        config->confidence_threshold < 0.0F ||
        config->confidence_threshold > 1.0F ||
        !IsConfiguredPath(config->camera_backbone_engine_path) ||
        !IsConfiguredPath(config->lidar_scn_onnx_path) ||
        !IsConfiguredPath(config->camera_vtransform_engine_path) ||
        !IsConfiguredPath(config->transfusion_engine_path) ||
        !IsConfiguredPath(config->bbox_engine_path)) {
      SetError(
          "BEVFusion model artifacts, GPU, point limit, or confidence "
          "threshold are not configured",
          error, error_capacity);
      return 1;
    }
    const std::array<const char*, 5> artifacts = {
        config->camera_backbone_engine_path, config->lidar_scn_onnx_path,
        config->camera_vtransform_engine_path, config->transfusion_engine_path,
        config->bbox_engine_path};
    for (const char* artifact : artifacts) {
      if (!IsReadableNonemptyFile(artifact)) {
        SetError(std::string("BEVFusion model artifact is missing or empty: ") +
                     artifact,
                 error, error_capacity);
        return 1;
      }
    }

    if (cudaSetDevice(config->gpu_device_id) != cudaSuccess) {
      SetError("failed to select configured CUDA device", error,
               error_capacity);
      return 1;
    }
    std::unique_ptr<Runtime> context(new Runtime());
    context->max_points = config->max_input_point_count;
    context->max_timestamp_skew_ns = config->max_timestamp_skew_ns;
    if (cudaStreamCreate(&context->stream) != cudaSuccess) {
      SetError("failed to create BEVFusion CUDA stream", error, error_capacity);
      return 1;
    }
    context->core = ::bevfusion::create_core(MakeCoreParameter(*config));
    if (context->core == nullptr) {
      SetError(
          "NVIDIA BEVFusion core initialization failed; check model "
          "artifacts and TensorRT compatibility",
          error, error_capacity);
      return 1;
    }
    *runtime = context.release();
    return 0;
  } catch (const std::exception& exception) {
    SetError(exception.what(), error, error_capacity);
  } catch (...) {
    SetError("unknown exception during BEVFusion runtime initialization", error,
             error_capacity);
  }
  return 1;
}

extern "C" void apollo_bevfusion_runtime_destroy(void* runtime) {
  delete static_cast<Runtime*>(runtime);
}

extern "C" int apollo_bevfusion_runtime_infer(
    void* runtime, const ApolloBevFusionInputV1* input,
    ApolloBevFusionDetectionV1* detections, size_t detection_capacity,
    size_t* detection_count, char* error, size_t error_capacity) {
  try {
    auto* context = static_cast<Runtime*>(runtime);
    if (context == nullptr || input == nullptr || detection_count == nullptr ||
        (detection_capacity > 0 && detections == nullptr)) {
      SetError("invalid BEVFusion runtime inference arguments", error,
               error_capacity);
      return 1;
    }
    std::string validation_error;
    if (!ValidateApiInput(*input, context->max_points,
                          context->max_timestamp_skew_ns, &validation_error)) {
      SetError(validation_error, error, error_capacity);
      return 1;
    }

    std::array<const unsigned char*, APOLLO_BEVFUSION_CAMERA_COUNT> images{};
    for (size_t camera = 0; camera < APOLLO_BEVFUSION_CAMERA_COUNT; ++camera) {
      images[camera] = input->image_data[camera];
    }
    std::vector<nvtype::half> fp16_points(input->point_count *
                                          input->point_feature_count);
    for (size_t index = 0; index < fp16_points.size(); ++index) {
      if (std::abs(input->lidar_points[index]) > 65504.0F) {
        SetError("LiDAR feature exceeds the finite FP16 input range", error,
                 error_capacity);
        return 1;
      }
      fp16_points[index].__x =
          __half_as_ushort(__float2half_rn(input->lidar_points[index]));
    }

    context->core->update(input->camera_to_lidar, input->camera_intrinsics,
                          input->lidar_to_image, input->image_augmentation,
                          context->stream);
    const auto boxes = context->core->forward(
        images.data(), fp16_points.data(), static_cast<int>(input->point_count),
        context->stream);
    if (cudaStreamSynchronize(context->stream) != cudaSuccess) {
      SetError("CUDA stream synchronization failed after BEVFusion inference",
               error, error_capacity);
      return 1;
    }
    if (boxes.size() > detection_capacity ||
        boxes.size() > kDetectionCapacity) {
      *detection_count = boxes.size();
      SetError("BEVFusion detection output exceeds configured API capacity",
               error, error_capacity);
      return 1;
    }
    for (size_t index = 0; index < boxes.size(); ++index) {
      const auto& source = boxes[index];
      detections[index] = ApolloBevFusionDetectionV1{
          source.position.x,  source.position.y,  source.position.z,
          source.size.w,      source.size.l,      source.size.h,
          source.velocity.vx, source.velocity.vy, source.z_rotation,
          source.score,       source.id};
    }
    *detection_count = boxes.size();
    return 0;
  } catch (const std::exception& exception) {
    SetError(exception.what(), error, error_capacity);
  } catch (...) {
    SetError("unknown exception during BEVFusion inference", error,
             error_capacity);
  }
  return 1;
}
