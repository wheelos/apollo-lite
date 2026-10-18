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

#include <cstddef>
#include <cstdint>

extern "C" {

struct CenterPointRuntimeDetection {
  float center_x;
  float center_y;
  float center_z;
  float width;
  float length;
  float height;
  float velocity_x;
  float velocity_y;
  float yaw;
  int32_t class_id;
  float score;
};

struct CenterPointRuntimeTimings {
  double voxelization_ms;
  double sparse_backbone_ms;
  double rpn_centerhead_ms;
  double decode_nms_ms;
};

void* centerpoint_runtime_create(const char* rpn_centerhead_engine_path,
                                 const char* scn_onnx_path, int gpu_device_id,
                                 char* error, std::size_t error_capacity);

void centerpoint_runtime_destroy(void* runtime);

bool centerpoint_runtime_infer(void* runtime, const float* points,
                               std::uint32_t point_count,
                               CenterPointRuntimeDetection* detections,
                               std::size_t detection_capacity,
                               std::size_t* detection_count,
                               CenterPointRuntimeTimings* timings, char* error,
                               std::size_t error_capacity);

}  // extern "C"
