// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {

struct PointPillarsDetection {
  float x, y, z;
  float dx, dy, dz;
  float yaw;
  std::int32_t class_id;
  float score;
};

std::uint32_t pointpillars_runtime_abi_version();
std::size_t pointpillars_runtime_detection_size();
bool pointpillars_runtime_build_engine(const char* onnx_path,
                                       const char* engine_path, int gpu,
                                       char* error, std::size_t error_capacity);
void* pointpillars_runtime_create(const char* engine_path, int gpu, char* error,
                                  std::size_t error_capacity);
void pointpillars_runtime_destroy(void* runtime);
bool pointpillars_runtime_infer(void* runtime, const float* xyzi,
                                std::uint32_t point_count,
                                PointPillarsDetection* detections,
                                std::size_t capacity, std::size_t* count,
                                char* error, std::size_t error_capacity);
}
