/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/

#include <memory>

#include "gflags/gflags.h"

#include "modules/perception/traffic_light/tools/traffic_light_detection/tool_runner.h"

DEFINE_string(pipeline_config_path,
              "/apollo/modules/perception/pipeline/config/trafficlights_perception.pb.txt",
              "Pipeline config path.");
DEFINE_string(image_root_dir,
              "/apollo/modules/perception/camera/tools/traffic_light_detection",
              "Root dir containing images/ and test list.");
DEFINE_string(test_list_path,
              "/apollo/modules/perception/camera/tools/traffic_light_detection/images/image_test_list.txt",
              "File containing sample ids, one per line.");
DEFINE_string(image_ext, ".jpg", "Input image extension.");
DEFINE_string(output_dir,
              "/tmp/traffic_light_detection_tool",
              "Result output directory.");
DEFINE_string(sensor_name, "front_6mm", "Data provider sensor name.");
DEFINE_int32(image_width, 1920, "Input width for tool runtime.");
DEFINE_int32(image_height, 1080, "Input height for tool runtime.");
DEFINE_int32(gpu_id, 0, "GPU id for data provider runtime.");

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::perception::traffic_light::tools::TrafficLightDetectionToolOptions
      options;
  options.pipeline_config_path = FLAGS_pipeline_config_path;
  options.image_root_dir = FLAGS_image_root_dir;
  options.test_list_path = FLAGS_test_list_path;
  options.image_ext = FLAGS_image_ext;
  options.output_dir = FLAGS_output_dir;
  options.sensor_name = FLAGS_sensor_name;
  options.image_width = FLAGS_image_width;
  options.image_height = FLAGS_image_height;
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
