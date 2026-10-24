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

#include "modules/lidar_object_detection/component/centerpoint_component.h"

#include <memory>
#include <string>
#include <utility>

#include "modules/lidar_object_detection/inference/centerpoint_runtime_backend.h"

namespace apollo {
namespace lidar_object_detection {

namespace {

bool HasRequiredConfig(const CenterPointComponentConfig& config) {
  return config.has_model() && !config.input_channel().empty() &&
         !config.output_channel().empty() && !config.sensor_name().empty() &&
         !config.sensor_parent_frame_id().empty() &&
         !config.sensor_frame_id().empty() &&
         !config.world_frame_id().empty() && !config.vehicle_frame_id().empty();
}

}  // namespace

bool CenterPointComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Failed to load CenterPoint component config";
    return false;
  }
  if (!HasRequiredConfig(config_)) {
    AERROR << "CenterPoint component config is missing required "
              "model, channel, sensor, or transform-frame values";
    return false;
  }
  if (config_.sweep_count() != 10U) {
    AERROR << "The configured CenterPoint model requires exactly 10 "
              "sweeps";
    return false;
  }

  std::string error;
  if (!sweep_fusion_.Init(config_.sweep_count(),
                          config_.max_input_point_count(), &error)) {
    AERROR << "Failed to initialize CenterPoint sweep fusion: " << error;
    return false;
  }

  transform_ = std::make_unique<perception::onboard::TransformWrapper>();
  transform_->Init(config_.sensor_parent_frame_id(), config_.sensor_frame_id(),
                   config_.world_frame_id(), config_.vehicle_frame_id());

  backend_ = CreateCenterPointRuntimeBackend();
  if (backend_ == nullptr) {
    AERROR << "Failed to create CenterPoint inference backend";
    return false;
  }
  runner_ = std::make_unique<CenterPointRunner>(backend_.get());
  if (!runner_->Init(config_.model(), &error)) {
    AERROR << "Failed to initialize CenterPoint runner: " << error;
    return false;
  }

  writer_ =
      node_->CreateWriter<CenterPointFrameResult>(config_.output_channel());
  if (writer_ == nullptr) {
    AERROR << "Failed to create CenterPoint output writer on channel "
           << config_.output_channel();
    return false;
  }
  AINFO << "Initialized CenterPoint component, input="
        << config_.input_channel() << ", output=" << config_.output_channel()
        << ", sensor=" << config_.sensor_name()
        << ", sweeps=" << config_.sweep_count();
  return true;
}

bool CenterPointComponent::Proc(
    const std::shared_ptr<apollo::drivers::PointCloud>& message) {
  if (message == nullptr) {
    AERROR << "CenterPoint component received a null PointCloud";
    return false;
  }
  if (message->has_frame_id() && !message->frame_id().empty() &&
      message->frame_id() != config_.sensor_frame_id()) {
    AERROR << "PointCloud frame_id " << message->frame_id()
           << " does not match configured sensor frame "
           << config_.sensor_frame_id();
    return false;
  }
  const double timestamp = message->measurement_time();
  Eigen::Affine3d sensor_to_world = Eigen::Affine3d::Identity();
  if (!transform_->GetSensor2worldTrans(timestamp, &sensor_to_world)) {
    AERROR << "Failed to query sensor-to-world transform at " << timestamp
           << " for sensor " << config_.sensor_name();
    return false;
  }

  apollo::drivers::PointCloud fused_cloud;
  std::string error;
  if (!sweep_fusion_.AddFrame(message, sensor_to_world, &fused_cloud, &error)) {
    AERROR << "Failed to fuse CenterPoint sweeps: " << error;
    return false;
  }

  CenterPointFrameResult result;
  if (!runner_->Run(fused_cloud, std::string(), config_.sensor_name(), &result,
                    &error)) {
    AERROR << "CenterPoint inference failed at " << timestamp << ": " << error;
    return false;
  }
  if (!writer_->Write(
          std::make_shared<CenterPointFrameResult>(std::move(result)))) {
    AERROR << "Failed to publish CenterPoint result at " << timestamp;
    return false;
  }
  return true;
}

}  // namespace lidar_object_detection
}  // namespace apollo
