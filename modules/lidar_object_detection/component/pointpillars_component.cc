// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_object_detection/component/pointpillars_component.h"

#include <dlfcn.h>

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include "modules/lidar_object_detection/types/pointpillars_input.h"

namespace apollo::lidar_object_detection {
namespace {
template <typename Function>
Function Symbol(void* library, const char* name) {
  dlerror();
  auto* symbol = dlsym(library, name);
  const char* error = dlerror();
  if (error != nullptr) throw std::runtime_error(error);
  return reinterpret_cast<Function>(symbol);
}
}  // namespace
PointPillarsComponent::~PointPillarsComponent() {
  if (runtime_ != nullptr && destroy_ != nullptr) destroy_(runtime_);
  if (library_ != nullptr) dlclose(library_);
}
bool PointPillarsComponent::Init() {
  if (!GetProtoConfig(&config_) || config_.engine_path().empty() ||
      config_.runtime_library_path().empty() ||
      config_.sensor_frame_id().empty() || config_.output_channel().empty() ||
      config_.model_revision().empty() || !config_.has_intensity_scale() ||
      !std::isfinite(config_.intensity_scale()) ||
      config_.intensity_scale() <= 0) {
    AERROR << "PointPillars requires explicit engine/runtime/frame/channel/"
              "model revision/intensity scale configuration";
    return false;
  }
  // TensorRT retains registered plugin creators beyond individual runtimes.
  library_ = dlopen(config_.runtime_library_path().c_str(),
                    RTLD_NOW | RTLD_LOCAL | RTLD_NODELETE);
  if (library_ == nullptr) {
    AERROR << "Cannot load PointPillars runtime: " << dlerror();
    return false;
  }
  try {
    const auto version = Symbol<decltype(&pointpillars_runtime_abi_version)>(
        library_, "pointpillars_runtime_abi_version");
    const auto size = Symbol<decltype(&pointpillars_runtime_detection_size)>(
        library_, "pointpillars_runtime_detection_size");
    if (version() != 1 || size() != sizeof(PointPillarsDetection)) {
      AERROR << "PointPillars runtime ABI mismatch";
      return false;
    }
    destroy_ =
        Symbol<decltype(destroy_)>(library_, "pointpillars_runtime_destroy");
    infer_ = Symbol<decltype(infer_)>(library_, "pointpillars_runtime_infer");
    const auto create = Symbol<decltype(&pointpillars_runtime_create)>(
        library_, "pointpillars_runtime_create");
    char error[2048] = {};
    runtime_ = create(config_.engine_path().c_str(), config_.gpu_device_id(),
                      error, sizeof(error));
    if (runtime_ == nullptr) {
      AERROR << "Cannot initialize PointPillars runtime: " << error;
      return false;
    }
  } catch (const std::runtime_error& error) {
    AERROR << "PointPillars symbol lookup failed: " << error.what();
    return false;
  }
  writer_ =
      node_->CreateWriter<PointPillarsFrameResult>(config_.output_channel());
  if (writer_ == nullptr) {
    AERROR << "Cannot create PointPillars result writer";
    return false;
  }
  return true;
}
bool PointPillarsComponent::Proc(
    const std::shared_ptr<drivers::PointCloud>& cloud) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (cloud == nullptr) {
    AERROR << "Null PointPillars input cloud";
    return false;
  }
  std::vector<float> points;
  std::string input_error;
  if (!BuildPointPillarsInput(*cloud, config_.sensor_frame_id(),
                              config_.intensity_scale(), &points,
                              &input_error)) {
    AERROR << input_error;
    return false;
  }
  std::vector<PointPillarsDetection> detections(40000);
  std::size_t count = 0;
  char error[2048] = {};
  const auto start = std::chrono::steady_clock::now();
  if (!infer_(runtime_, points.data(), points.size() / 4, detections.data(),
              detections.size(), &count, error, sizeof(error))) {
    AERROR << "PointPillars inference failed: " << error;
    return false;
  }
  auto result = std::make_shared<PointPillarsFrameResult>();
  *result->mutable_header() = cloud->header();
  result->mutable_header()->set_timestamp_sec(cloud->measurement_time());
  result->set_sensor_frame_id(config_.sensor_frame_id());
  result->set_model_revision(config_.model_revision());
  result->set_inference_ms(std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - start)
                               .count());
  for (std::size_t i = 0; i < count; ++i) {
    const auto& detection = detections[i];
    auto* box = result->add_detection();
    box->set_center_x(detection.x);
    box->set_center_y(detection.y);
    box->set_center_z(detection.z);
    box->set_size_x(detection.dx);
    box->set_size_y(detection.dy);
    box->set_size_z(detection.dz);
    box->set_yaw(detection.yaw);
    box->set_class_id(detection.class_id);
    box->set_score(detection.score);
  }
  if (!writer_->Write(result)) {
    AERROR << "Cannot publish PointPillars result";
    return false;
  }
  return true;
}
}  // namespace apollo::lidar_object_detection
