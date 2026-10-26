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

#include "modules/lidar_object_detection/component/lidar_object_detection_component.h"

#include <dlfcn.h>

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "modules/lidar_object_detection/inference/centerpoint_runtime_backend.h"
#include "modules/lidar_object_detection/types/pointpillars_input.h"

namespace apollo {
namespace lidar_object_detection {

namespace {

template <typename Function>
Function LoadSymbol(void* library, const char* name) {
  dlerror();
  auto* symbol = dlsym(library, name);
  const char* error = dlerror();
  if (error != nullptr) {
    throw std::runtime_error(error);
  }
  return reinterpret_cast<Function>(symbol);
}

}  // namespace

LidarObjectDetectionComponent::~LidarObjectDetectionComponent() {
  if (pointpillars_runtime_ != nullptr && pointpillars_destroy_ != nullptr) {
    pointpillars_destroy_(pointpillars_runtime_);
  }
  if (pointpillars_library_ != nullptr) {
    dlclose(pointpillars_library_);
  }
}

bool LidarObjectDetectionComponent::Init() {
  if (!GetProtoConfig(&config_) || config_.output_channel().empty() ||
      config_.sensor_frame_id().empty()) {
    AERROR << "LiDAR object detection requires output channel and sensor frame "
              "configuration";
    return false;
  }

  std::string error;
  switch (config_.backend_config_case()) {
    case LidarObjectDetectionComponentConfig::kCenterpoint:
      if (config_.sensor_name().empty() ||
          config_.sensor_parent_frame_id().empty() ||
          config_.world_frame_id().empty() ||
          config_.vehicle_frame_id().empty()) {
        AERROR << "CenterPoint requires sensor name and transform frame "
                  "configuration";
        return false;
      }
      if (!InitCenterPoint(config_.centerpoint(), &error)) {
        AERROR << "Failed to initialize CenterPoint backend: " << error;
        return false;
      }
      break;
    case LidarObjectDetectionComponentConfig::kPointpillars:
      if (!InitPointPillars(config_.pointpillars(), &error)) {
        AERROR << "Failed to initialize PointPillars backend: " << error;
        return false;
      }
      break;
    case LidarObjectDetectionComponentConfig::BACKEND_CONFIG_NOT_SET:
      AERROR << "Select exactly one LiDAR detection backend in component "
                "configuration";
      return false;
  }

  writer_ = node_->CreateWriter<LidarObjectDetectionFrameResult>(
      config_.output_channel());
  if (writer_ == nullptr) {
    AERROR << "Failed to create LiDAR detection output writer on channel "
           << config_.output_channel();
    return false;
  }
  AINFO << "Initialized LiDAR object detection component, backend="
        << (config_.has_centerpoint() ? "CenterPoint" : "PointPillars")
        << ", output=" << config_.output_channel()
        << ", sensor_frame=" << config_.sensor_frame_id();
  return true;
}

bool LidarObjectDetectionComponent::InitCenterPoint(
    const CenterPointModelConfig& model, std::string* error) {
  if (model.scn_onnx_path().empty() ||
      model.rpn_centerhead_engine_path().empty() ||
      model.runtime_library_path().empty() || model.model_revision().empty()) {
    if (error != nullptr) {
      *error = "model paths and model revision are required";
    }
    return false;
  }
  if (config_.sweep_count() != 10U) {
    if (error != nullptr) {
      *error = "the configured CenterPoint model requires exactly 10 sweeps";
    }
    return false;
  }
  if (!sweep_fusion_.Init(config_.sweep_count(),
                          config_.max_input_point_count(), error)) {
    return false;
  }

  transform_ = std::make_unique<perception::onboard::TransformWrapper>();
  transform_->Init(config_.sensor_parent_frame_id(), config_.sensor_frame_id(),
                   config_.world_frame_id(), config_.vehicle_frame_id());

  centerpoint_backend_ = CreateCenterPointRuntimeBackend();
  if (centerpoint_backend_ == nullptr) {
    if (error != nullptr) {
      *error = "failed to create inference backend";
    }
    return false;
  }
  centerpoint_runner_ =
      std::make_unique<CenterPointRunner>(centerpoint_backend_.get());
  return centerpoint_runner_->Init(model, error);
}

bool LidarObjectDetectionComponent::InitPointPillars(
    const PointPillarsModelConfig& model, std::string* error) {
  if (model.engine_path().empty() || model.runtime_library_path().empty() ||
      model.model_revision().empty() || !model.has_intensity_scale() ||
      !std::isfinite(model.intensity_scale()) ||
      model.intensity_scale() <= 0.0F) {
    if (error != nullptr) {
      *error =
          "engine/runtime paths, model revision, and a positive explicit "
          "intensity scale are required";
    }
    return false;
  }
  pointpillars_library_ = dlopen(model.runtime_library_path().c_str(),
                                 RTLD_NOW | RTLD_LOCAL | RTLD_NODELETE);
  if (pointpillars_library_ == nullptr) {
    if (error != nullptr) {
      *error = dlerror();
    }
    return false;
  }
  try {
    const auto version =
        LoadSymbol<decltype(&pointpillars_runtime_abi_version)>(
            pointpillars_library_, "pointpillars_runtime_abi_version");
    const auto size =
        LoadSymbol<decltype(&pointpillars_runtime_detection_size)>(
            pointpillars_library_, "pointpillars_runtime_detection_size");
    if (version() != 1U || size() != sizeof(PointPillarsDetection)) {
      if (error != nullptr) {
        *error = "runtime ABI mismatch";
      }
      return false;
    }
    pointpillars_destroy_ = LoadSymbol<decltype(pointpillars_destroy_)>(
        pointpillars_library_, "pointpillars_runtime_destroy");
    pointpillars_infer_ = LoadSymbol<decltype(pointpillars_infer_)>(
        pointpillars_library_, "pointpillars_runtime_infer");
    const auto create = LoadSymbol<decltype(&pointpillars_runtime_create)>(
        pointpillars_library_, "pointpillars_runtime_create");
    char runtime_error[2048] = {};
    pointpillars_runtime_ =
        create(model.engine_path().c_str(), model.gpu_device_id(),
               runtime_error, sizeof(runtime_error));
    if (pointpillars_runtime_ == nullptr) {
      if (error != nullptr) {
        *error = runtime_error;
      }
      return false;
    }
  } catch (const std::runtime_error& symbol_error) {
    if (error != nullptr) {
      *error = symbol_error.what();
    }
    return false;
  }
  return true;
}

bool LidarObjectDetectionComponent::Proc(
    const std::shared_ptr<apollo::drivers::PointCloud>& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (message == nullptr) {
    AERROR << "LiDAR object detection received a null PointCloud";
    return false;
  }
  if (message->has_frame_id() && !message->frame_id().empty() &&
      message->frame_id() != config_.sensor_frame_id()) {
    AERROR << "PointCloud frame_id " << message->frame_id()
           << " does not match configured sensor frame "
           << config_.sensor_frame_id();
    return false;
  }

  const auto pipeline_start = std::chrono::steady_clock::now();
  auto result = std::make_shared<LidarObjectDetectionFrameResult>();
  std::string error;
  bool processed = false;
  switch (config_.backend_config_case()) {
    case LidarObjectDetectionComponentConfig::kCenterpoint:
      processed = ProcessCenterPoint(message, result.get(), &error);
      result->set_backend(LIDAR_OBJECT_DETECTION_BACKEND_CENTERPOINT);
      result->set_model_revision(config_.centerpoint().model_revision());
      break;
    case LidarObjectDetectionComponentConfig::kPointpillars:
      processed = ProcessPointPillars(message, result.get(), &error);
      result->set_backend(LIDAR_OBJECT_DETECTION_BACKEND_POINTPILLARS);
      result->set_model_revision(config_.pointpillars().model_revision());
      break;
    case LidarObjectDetectionComponentConfig::BACKEND_CONFIG_NOT_SET:
      AERROR << "No LiDAR detection backend is configured";
      return false;
  }
  if (!processed) {
    AERROR << "LiDAR object detection pipeline failed: " << error;
    return false;
  }
  result->set_sensor_frame_id(config_.sensor_frame_id());
  result->set_total_pipeline_ms(
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - pipeline_start)
          .count());
  if (!writer_->Write(result)) {
    AERROR << "Failed to publish LiDAR detection result";
    return false;
  }
  return true;
}

bool LidarObjectDetectionComponent::ProcessCenterPoint(
    const std::shared_ptr<const apollo::drivers::PointCloud>& message,
    LidarObjectDetectionFrameResult* result, std::string* error) {
  const double timestamp = message->measurement_time();
  Eigen::Affine3d sensor_to_world = Eigen::Affine3d::Identity();
  if (!transform_->GetSensor2worldTrans(timestamp, &sensor_to_world)) {
    if (error != nullptr) {
      *error = "failed to query sensor-to-world transform for " +
               config_.sensor_name();
    }
    return false;
  }
  apollo::drivers::PointCloud fused_cloud;
  if (!sweep_fusion_.AddFrame(message, sensor_to_world, &fused_cloud, error)) {
    return false;
  }
  CenterPointFrameResult centerpoint_result;
  if (!centerpoint_runner_->Run(fused_cloud, std::string(),
                                config_.sensor_name(), &centerpoint_result,
                                error)) {
    return false;
  }
  if (centerpoint_result.has_header()) {
    *result->mutable_header() = centerpoint_result.header();
  }
  result->set_input_point_count(centerpoint_result.input_point_count());
  result->set_preprocess_ms(centerpoint_result.preprocess_ms());
  result->set_inference_ms(centerpoint_result.total_inference_ms());
  for (const auto& detection : centerpoint_result.detection()) {
    auto* object = result->add_detection();
    object->set_center_x(detection.center_x());
    object->set_center_y(detection.center_y());
    object->set_center_z(detection.center_z());
    object->set_size_x(detection.width());
    object->set_size_y(detection.length());
    object->set_size_z(detection.height());
    object->set_velocity_x(detection.velocity_x());
    object->set_velocity_y(detection.velocity_y());
    object->set_yaw(detection.yaw());
    object->set_class_id(detection.class_id());
    object->set_score(detection.score());
  }
  return true;
}

bool LidarObjectDetectionComponent::ProcessPointPillars(
    const std::shared_ptr<const apollo::drivers::PointCloud>& message,
    LidarObjectDetectionFrameResult* result, std::string* error) {
  std::vector<float> points;
  const auto preprocess_start = std::chrono::steady_clock::now();
  if (!BuildPointPillarsInput(*message, config_.sensor_frame_id(),
                              config_.pointpillars().intensity_scale(), &points,
                              error)) {
    return false;
  }
  result->set_preprocess_ms(
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - preprocess_start)
          .count());
  result->set_input_point_count(static_cast<uint32_t>(points.size() / 4U));

  std::vector<PointPillarsDetection> detections(40000);
  std::size_t count = 0;
  char runtime_error[2048] = {};
  const auto inference_start = std::chrono::steady_clock::now();
  if (!pointpillars_infer_(pointpillars_runtime_, points.data(),
                           points.size() / 4U, detections.data(),
                           detections.size(), &count, runtime_error,
                           sizeof(runtime_error))) {
    if (error != nullptr) {
      *error = runtime_error;
    }
    return false;
  }
  result->set_inference_ms(
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - inference_start)
          .count());
  if (message->has_header()) {
    *result->mutable_header() = message->header();
  }
  result->mutable_header()->set_timestamp_sec(message->measurement_time());
  for (std::size_t i = 0; i < count; ++i) {
    const auto& detection = detections[i];
    auto* object = result->add_detection();
    object->set_center_x(detection.x);
    object->set_center_y(detection.y);
    object->set_center_z(detection.z);
    object->set_size_x(detection.dx);
    object->set_size_y(detection.dy);
    object->set_size_z(detection.dz);
    object->set_yaw(detection.yaw);
    object->set_class_id(detection.class_id);
    object->set_score(detection.score);
  }
  return true;
}

}  // namespace lidar_object_detection
}  // namespace apollo
