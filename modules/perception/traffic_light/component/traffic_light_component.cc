/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/
#include "modules/perception/traffic_light/component/traffic_light_component.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/util.h"
#include "modules/perception/traffic_light/domain/traffic_light_component_config.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace component {

using apollo::cyber::common::GetAbsolutePath;

bool TrafficLightComponent::Init() {
  if (!InitConfig()) {
    return false;
  }
  writer_ = node_->CreateWriter<apollo::perception::TrafficLightDetection>(
      config_.traffic_light_output_channel_name);
  if (!system_.Init(config_, pipeline_config_)) {
    return false;
  }
  return InitListeners();
}

bool TrafficLightComponent::InitConfig() {
  if (!GetProtoConfig(&component_config_proto_)) {
    AERROR << "Failed to load traffic light component config proto.";
    return false;
  }
  config_ = domain::BuildTrafficLightComponentConfig(component_config_proto_);

  std::string work_root = apollo::perception::camera::GetCyberWorkRoot();
  std::string pipeline_config_file = GetAbsolutePath(
      config_.pipeline_config_root_dir, config_.pipeline_config_file);
  pipeline_config_file = GetAbsolutePath(work_root, pipeline_config_file);
  if (!cyber::common::GetProtoFromFile(pipeline_config_file, &pipeline_config_)) {
    AERROR << "Failed to load traffic light pipeline config: "
           << pipeline_config_file;
    return false;
  }
  return true;
}

bool TrafficLightComponent::InitListeners() {
  const auto& camera_names = system_.camera_names();
  const auto& camera_channels = system_.camera_channel_names();
  for (size_t i = 0; i < camera_names.size(); ++i) {
    const auto& camera_name = camera_names[i];
    const auto& camera_channel = camera_channels[i];
    node_->CreateReader(camera_channel,
                        std::bind(&TrafficLightComponent::OnReceiveImage, this,
                                  std::placeholders::_1, camera_name));
  }
  node_->CreateReader(system_.v2x_input_channel_name(),
                      std::bind(&TrafficLightComponent::OnReceiveV2XMsg, this,
                                std::placeholders::_1));
  return true;
}

void TrafficLightComponent::OnReceiveImage(
    const std::shared_ptr<apollo::drivers::Image> image,
    const std::string& camera_name) {
  std::shared_ptr<apollo::perception::TrafficLightDetection> out_msg;
  if (!system_.ProcessImage(image, camera_name, &out_msg)) {
    return;
  }
  writer_->Write(out_msg);
}

void TrafficLightComponent::OnReceiveV2XMsg(
    const std::shared_ptr<apollo::v2x::IntersectionTrafficLightData> v2x_msg) {
  system_.PushV2X(*v2x_msg);
}

}  // namespace component
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
