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

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APOLLO_BEVFUSION_RUNTIME_ABI_VERSION 1
#define APOLLO_BEVFUSION_CAMERA_COUNT 6

typedef struct ApolloBevFusionRuntimeConfigV1 {
  uint32_t struct_size;
  uint32_t abi_version;
  int32_t gpu_device_id;
  uint32_t max_input_point_count;
  uint64_t max_timestamp_skew_ns;
  float confidence_threshold;
  const char* camera_backbone_engine_path;
  const char* lidar_scn_onnx_path;
  const char* camera_vtransform_engine_path;
  const char* transfusion_engine_path;
  const char* bbox_engine_path;
} ApolloBevFusionRuntimeConfigV1;

typedef struct ApolloBevFusionInputV1 {
  uint32_t struct_size;
  uint32_t abi_version;
  uint64_t timestamp_ns;
  const char* camera_name[APOLLO_BEVFUSION_CAMERA_COUNT];
  uint64_t camera_timestamp_ns[APOLLO_BEVFUSION_CAMERA_COUNT];
  uint32_t image_width[APOLLO_BEVFUSION_CAMERA_COUNT];
  uint32_t image_height[APOLLO_BEVFUSION_CAMERA_COUNT];
  uint32_t image_stride_bytes[APOLLO_BEVFUSION_CAMERA_COUNT];
  uint32_t pixel_format[APOLLO_BEVFUSION_CAMERA_COUNT];
  const uint8_t* image_data[APOLLO_BEVFUSION_CAMERA_COUNT];
  size_t image_byte_count[APOLLO_BEVFUSION_CAMERA_COUNT];
  const float* lidar_points;
  size_t point_count;
  size_t point_feature_count;
  const float* camera_to_lidar;
  const float* camera_intrinsics;
  const float* lidar_to_image;
  const float* image_augmentation;
} ApolloBevFusionInputV1;

typedef struct ApolloBevFusionDetectionV1 {
  float center_x;
  float center_y;
  float center_z;
  float width;
  float length;
  float height;
  float velocity_x;
  float velocity_y;
  float yaw;
  float score;
  int32_t class_id;
} ApolloBevFusionDetectionV1;

uint32_t apollo_bevfusion_runtime_abi_version(void);

int apollo_bevfusion_runtime_create(
    const ApolloBevFusionRuntimeConfigV1* config, void** runtime, char* error,
    size_t error_capacity);

void apollo_bevfusion_runtime_destroy(void* runtime);

int apollo_bevfusion_runtime_infer(void* runtime,
                                   const ApolloBevFusionInputV1* input,
                                   ApolloBevFusionDetectionV1* detections,
                                   size_t detection_capacity,
                                   size_t* detection_count, char* error,
                                   size_t error_capacity);

#ifdef __cplusplus
}
#endif
