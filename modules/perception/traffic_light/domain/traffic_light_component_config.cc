/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/
#include "modules/perception/traffic_light/domain/traffic_light_component_config.h"

#include <algorithm>

#include <boost/algorithm/string.hpp>

#include "modules/perception/traffic_light/proto/traffic_light_component.pb.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace domain {

TrafficLightComponentConfig BuildTrafficLightComponentConfig(
    const apollo::perception::traffic_light::proto::TrafficLightComponentConfig&
        proto) {
  TrafficLightComponentConfig config;
  config.tf2_frame_id = proto.tl_tf2_frame_id();
  config.tf2_child_frame_id = proto.tl_tf2_child_frame_id();
  config.tf2_timeout_second = proto.tf2_timeout_second();
  boost::algorithm::split(config.camera_names, proto.camera_names(),
                          boost::algorithm::is_any_of(","));
  boost::algorithm::split(config.camera_channel_names,
                          proto.camera_channel_names(),
                          boost::algorithm::is_any_of(","));
    for (auto& camera_name : config.camera_names) {
        boost::algorithm::trim(camera_name);
    }
    config.camera_names.erase(
            std::remove(config.camera_names.begin(), config.camera_names.end(), ""),
            config.camera_names.end());
    for (auto& channel_name : config.camera_channel_names) {
        boost::algorithm::trim(channel_name);
    }
    config.camera_channel_names.erase(
            std::remove(config.camera_channel_names.begin(),
                                    config.camera_channel_names.end(), ""),
            config.camera_channel_names.end());
  config.image_timestamp_offset = proto.tl_image_timestamp_offset();
  config.max_process_image_fps = proto.max_process_image_fps();
  config.proc_interval_seconds =
      1.0 / std::max(1, proto.max_process_image_fps());
  config.query_tf_interval_seconds = proto.query_tf_interval_seconds();
  config.valid_hdmap_interval = proto.valid_hdmap_interval();
  config.image_sys_ts_diff_threshold = proto.image_sys_ts_diff_threshold();
  config.sync_interval_seconds = proto.sync_interval_seconds();
  config.default_image_border_size = proto.default_image_border_size();
  config.traffic_light_output_channel_name =
      proto.traffic_light_output_channel_name();
  config.simulation_channel_name = proto.simulation_channel_name();
  config.v2x_trafficlights_input_channel_name =
      proto.v2x_trafficlights_input_channel_name();
  config.v2x_sync_interval_seconds = proto.v2x_sync_interval_seconds();
  config.max_v2x_msg_buff_size = proto.max_v2x_msg_buff_size();
  config.tl_preprocessor_name = proto.tl_preprocessor_name();
  config.pipeline_config_root_dir =
      proto.camera_traffic_light_perception_conf_dir();
  config.pipeline_config_file = proto.camera_traffic_light_perception_conf_file();
  return config;
}

}  // namespace domain
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
