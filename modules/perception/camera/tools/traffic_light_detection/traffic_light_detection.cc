/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/

#include <memory>

#include "gflags/gflags.h"

#include "modules/perception/traffic_light/tools/traffic_light_detection/tool_runner.h"

DEFINE_int32(height, 1080, "image height");
DEFINE_int32(width, 1920, "image width");
DEFINE_int32(gpu_id, 0, "gpu id");
DEFINE_string(dest_dir, "./data", "output dir");
DEFINE_string(root_dir,
              "/apollo/modules/perception/camera/tools/traffic_light_detection",
              "image root dir");  // NOLINT
DEFINE_string(image_ext, ".jpg", "extension of image name");
DEFINE_string(
    test_list,
    "/apollo/modules/perception/camera/tools/traffic_light_detection/images/"
    "image_test_list.txt",
    "test image list");  // NOLINT
DEFINE_string(
    tf_conf_file,
    "/apollo/modules/perception/pipeline/config/trafficlights_perception.pb.txt",
    "traffic light pipeline config file");  // NOLINT
DEFINE_string(sensor_name, "front_6mm", "sensor name");

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::perception::traffic_light::tools::TrafficLightDetectionToolOptions
      options;
  options.pipeline_config_path = FLAGS_tf_conf_file;
  options.image_root_dir = FLAGS_root_dir;
  options.test_list_path = FLAGS_test_list;
  options.image_ext = FLAGS_image_ext;
  options.output_dir = FLAGS_dest_dir;
  options.sensor_name = FLAGS_sensor_name;
  options.image_width = FLAGS_width;
  options.image_height = FLAGS_height;
  options.gpu_id = FLAGS_gpu_id;

  auto seed_provider =
      std::unique_ptr<apollo::perception::traffic_light::tools::TrafficLightSeedProvider>(
          new apollo::perception::traffic_light::tools::FullFrameSeedProvider());
  auto result_writer =
      std::unique_ptr<apollo::perception::traffic_light::tools::TrafficLightResultWriter>(
          new apollo::perception::traffic_light::tools::LocalFileResultWriter(
              options.output_dir));

  apollo::perception::traffic_light::tools::TrafficLightDetectionToolRunner
      runner;
  if (!runner.Init(options, std::move(seed_provider), std::move(result_writer))) {
    return 1;
  }
  return runner.RunBatch() ? 0 : 1;
}
