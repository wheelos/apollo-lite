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

#include "modules/lidar_object_detection/inference/centerpoint_runner.h"

#include <chrono>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace apollo {
namespace lidar_object_detection {

namespace {

void SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

bool IsFiniteDetection(const CenterPointDetection& detection) {
  return std::isfinite(detection.center_x()) &&
         std::isfinite(detection.center_y()) &&
         std::isfinite(detection.center_z()) &&
         std::isfinite(detection.width()) && detection.width() > 0.0F &&
         std::isfinite(detection.length()) && detection.length() > 0.0F &&
         std::isfinite(detection.height()) && detection.height() > 0.0F &&
         std::isfinite(detection.velocity_x()) &&
         std::isfinite(detection.velocity_y()) &&
         std::isfinite(detection.yaw()) &&
         std::isfinite(detection.score());
}

bool IsValidTimings(const InferenceTimings& timings) {
  return std::isfinite(timings.voxelization_ms) &&
         timings.voxelization_ms >= 0.0 &&
         std::isfinite(timings.sparse_backbone_ms) &&
         timings.sparse_backbone_ms >= 0.0 &&
         std::isfinite(timings.rpn_centerhead_ms) &&
         timings.rpn_centerhead_ms >= 0.0 &&
         std::isfinite(timings.decode_nms_ms) &&
         timings.decode_nms_ms >= 0.0 &&
         std::isfinite(timings.total_inference_ms) &&
         timings.total_inference_ms >= 0.0;
}

}  // namespace

bool CenterPointRunner::Init(const NvidiaCenterPointConfig& config,
                             std::string* error) {
  if (backend_ == nullptr) {
    SetError("NVIDIA CenterPoint backend is null", error);
    return false;
  }
  if (config.scn_onnx_path().empty() ||
      config.rpn_centerhead_engine_path().empty()) {
    SetError("SCN ONNX and RPN/CenterHead engine paths must be configured",
             error);
    return false;
  }
  if (!input_builder_.Init(config, error)) {
    return false;
  }
  if (!backend_->Init(config, error)) {
    if (error != nullptr && error->empty()) {
      SetError("NVIDIA CenterPoint backend initialization failed", error);
    }
    return false;
  }
  config_ = config;
  initialized_ = true;
  return true;
}

bool CenterPointRunner::Run(const apollo::drivers::PointCloud& cloud,
                            const std::string& source_path,
                            const std::string& sensor_name,
                            CenterPointFrameResult* result,
                            std::string* error) const {
  if (!initialized_) {
    SetError("NVIDIA CenterPoint runner is not initialized", error);
    return false;
  }
  if (result == nullptr) {
    SetError("NVIDIA CenterPoint result output is null", error);
    return false;
  }

  ModelInput input;
  const auto preprocess_start = std::chrono::steady_clock::now();
  if (!input_builder_.Build(cloud, &input, error)) {
    return false;
  }
  const double preprocess_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - preprocess_start)
          .count();

  std::vector<CenterPointDetection> detections;
  InferenceTimings timings;
  if (!backend_->Infer(input, &detections, &timings, error)) {
    if (error != nullptr && error->empty()) {
      SetError("NVIDIA CenterPoint backend inference failed", error);
    }
    return false;
  }
  if (!IsValidTimings(timings)) {
    SetError("NVIDIA CenterPoint backend returned invalid stage timings",
             error);
    return false;
  }
  for (const auto& detection : detections) {
    if (!IsFiniteDetection(detection) || detection.class_id() < 0 ||
        detection.score() < 0.0F || detection.score() > 1.0F) {
      SetError("NVIDIA CenterPoint backend returned an invalid detection",
               error);
      return false;
    }
  }

  CenterPointFrameResult output;
  if (cloud.has_header()) {
    *output.mutable_header() = cloud.header();
  }
  output.set_source_path(source_path);
  output.set_sensor_name(sensor_name);
  output.set_model_revision(config_.model_revision());
  output.set_input_point_count(static_cast<uint32_t>(input.point_count()));
  for (const auto& detection : detections) {
    *output.add_detection() = detection;
  }
  output.set_preprocess_ms(preprocess_ms);
  output.set_voxelization_ms(timings.voxelization_ms);
  output.set_sparse_backbone_ms(timings.sparse_backbone_ms);
  output.set_rpn_centerhead_ms(timings.rpn_centerhead_ms);
  output.set_decode_nms_ms(timings.decode_nms_ms);
  output.set_total_inference_ms(timings.total_inference_ms);
  *result = std::move(output);
  return true;
}

}  // namespace lidar_object_detection
}  // namespace apollo
