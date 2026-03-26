/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/perception/traffic_light/domain/traffic_light_component_config.h"

#include "gtest/gtest.h"
#include "modules/perception/traffic_light/proto/traffic_light_component.pb.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace domain {

TEST(TrafficLightComponentConfigTest, BuildFromProto) {
  proto::TrafficLightComponentConfig proto_config;
  proto_config.set_tl_tf2_frame_id("world");
  proto_config.set_tl_tf2_child_frame_id("novatel");
  proto_config.set_camera_names("front_6mm,front_12mm");
  proto_config.set_camera_channel_names("/cam/front_6mm,/cam/front_12mm");
  proto_config.set_max_process_image_fps(10);
  proto_config.set_camera_traffic_light_perception_conf_dir("/apollo/conf");
  proto_config.set_camera_traffic_light_perception_conf_file("tl.pb.txt");

  TrafficLightComponentConfig config =
      BuildTrafficLightComponentConfig(proto_config);

  EXPECT_EQ(config.tf2_frame_id, "world");
  EXPECT_EQ(config.tf2_child_frame_id, "novatel");
  EXPECT_EQ(config.camera_names.size(), 2U);
  EXPECT_EQ(config.camera_channel_names.size(), 2U);
  EXPECT_DOUBLE_EQ(config.proc_interval_seconds, 0.1);
  EXPECT_EQ(config.pipeline_config_root_dir, "/apollo/conf");
  EXPECT_EQ(config.pipeline_config_file, "tl.pb.txt");
}

}  // namespace domain
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
