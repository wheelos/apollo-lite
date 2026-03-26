/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <string>
#include <vector>

namespace apollo {
namespace perception {
namespace traffic_light {
namespace proto {
class TrafficLightComponentConfig;
}  // namespace proto
namespace domain {

struct TrafficLightComponentConfig {
  std::string tf2_frame_id;
  std::string tf2_child_frame_id;
  double tf2_timeout_second = 0.01;
  std::vector<std::string> camera_names;
  std::vector<std::string> camera_channel_names;
  double image_timestamp_offset = 0.0;
  int max_process_image_fps = 8;
  double proc_interval_seconds = 0.125;
  double query_tf_interval_seconds = 0.3;
  double valid_hdmap_interval = 1.5;
  double image_sys_ts_diff_threshold = 0.5;
  double sync_interval_seconds = 0.5;
  int default_image_border_size = 100;
  std::string traffic_light_output_channel_name;
  std::string simulation_channel_name;
  std::string v2x_trafficlights_input_channel_name;
  double v2x_sync_interval_seconds = 0.1;
  int max_v2x_msg_buff_size = 50;
  std::string tl_preprocessor_name;
  std::string pipeline_config_root_dir;
  std::string pipeline_config_file;
  bool enable_undistortion = false;
  int image_width = 1920;
  int image_height = 1080;
  int image_channel_num = 3;
  double check_image_status_interval_thresh = 1.0;
};

TrafficLightComponentConfig BuildTrafficLightComponentConfig(
  const apollo::perception::traffic_light::proto::TrafficLightComponentConfig&
    proto);

}  // namespace domain
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
