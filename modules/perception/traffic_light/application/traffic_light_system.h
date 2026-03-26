/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <boost/circular_buffer.hpp>

#include "modules/common_msgs/map_msgs/map_geometry.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/common_msgs/v2x_msgs/v2x_traffic_light.pb.h"
#include "modules/perception/camera/common/data_provider.h"
#include "modules/perception/onboard/common_flags/common_flags.h"
#include "modules/perception/traffic_light/application/traffic_light_perception_pipeline.h"
#include "modules/perception/traffic_light/domain/traffic_light_component_config.h"
#include "modules/perception/traffic_light/domain/traffic_light_result.h"
#include "modules/perception/traffic_light/infra/traffic_light_scene_gateway.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace application {

class TrafficLightSystem {
 public:
  bool Init(const domain::TrafficLightComponentConfig& config,
            const pipeline::PipelineConfig& pipeline_config);
  bool ProcessImage(const std::shared_ptr<apollo::drivers::Image>& image,
                    const std::string& camera_name,
                    std::shared_ptr<apollo::perception::TrafficLightDetection>*
                        out_msg);
  void PushV2X(const apollo::v2x::IntersectionTrafficLightData& v2x_msg);

  const std::vector<std::string>& camera_names() const {
    return config_.camera_names;
  }
  const std::vector<std::string>& camera_channel_names() const {
    return config_.camera_channel_names;
  }
  const std::string& output_channel_name() const {
    return config_.traffic_light_output_channel_name;
  }
  const std::string& v2x_input_channel_name() const {
    return config_.v2x_trafficlights_input_channel_name;
  }

 private:
  bool CreateDebugDir();
  bool InitCameraFrame();
  void SyncV2XTrafficLights(camera::CameraFrame* frame);
  bool BuildOutputMessage(camera::CameraFrame* frame,
                          const std::string& camera_name,
                          std::shared_ptr<apollo::perception::TrafficLightDetection>*
                              out_msg);
  bool BuildDebugMessage(const camera::CameraFrame* frame,
               const std::string& camera_name,
                         std::shared_ptr<apollo::perception::TrafficLightDetection>*
                             out_msg,
                         const domain::TrafficLightDecision& decision);
  void Visualize(const camera::CameraFrame& frame,
                 const std::vector<base::TrafficLightPtr>& lights,
                 const domain::TrafficLightDecision& decision) const;
  void TransRect2Box(const base::RectI& rect,
                     apollo::perception::TrafficLightBox* box) const;

 private:
  mutable std::mutex mutex_;
  domain::TrafficLightComponentConfig config_;
  infra::TrafficLightSceneGateway scene_gateway_;
  TrafficLightPerceptionPipeline pipeline_;
  std::map<std::string, std::shared_ptr<camera::DataProvider>>
      data_providers_map_;
  camera::DataProvider::InitOptions data_provider_init_options_;
  std::shared_ptr<camera::CameraFrame> frame_;
  boost::circular_buffer<apollo::v2x::IntersectionTrafficLightData>
      v2x_msg_buffer_;
  double last_proc_image_ts_ = 0.0;
};

}  // namespace application
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
