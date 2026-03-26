/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <memory>
#include <string>

#include "cyber/component/component.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/common_msgs/v2x_msgs/v2x_traffic_light.pb.h"
#include "modules/perception/traffic_light/application/traffic_light_system.h"
#include "modules/perception/traffic_light/proto/traffic_light_component.pb.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace component {

class TrafficLightComponent : public apollo::cyber::Component<> {
 public:
  bool Init() override;

 private:
  void OnReceiveImage(const std::shared_ptr<apollo::drivers::Image> image,
                      const std::string& camera_name);
  void OnReceiveV2XMsg(
      const std::shared_ptr<apollo::v2x::IntersectionTrafficLightData> v2x_msg);
  bool InitConfig();
  bool InitListeners();

 private:
    proto::TrafficLightComponentConfig component_config_proto_;
  domain::TrafficLightComponentConfig config_;
  pipeline::PipelineConfig pipeline_config_;
  application::TrafficLightSystem system_;
  std::shared_ptr<
      apollo::cyber::Writer<apollo::perception::TrafficLightDetection>>
      writer_;
};

CYBER_REGISTER_COMPONENT(TrafficLightComponent);

}  // namespace component
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
