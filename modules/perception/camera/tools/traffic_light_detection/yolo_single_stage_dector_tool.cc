/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <algorithm>
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "modules/perception/pipeline/proto/pipeline_config.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/base/traffic_light.h"
#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/common/data_provider.h"
#include "modules/perception/camera/lib/traffic_light/detector/yolo_single_stage_dector.h"

DEFINE_string(image_path, "", "Input JPG/PNG path. Example: /tmp/frame.jpg");
DEFINE_string(output_path, "/tmp/yolo_single_stage_dector_result.jpg",
              "Output annotated JPG path.");
DEFINE_string(config_path,
              "/apollo/modules/perception/pipeline/config/"
              "trafficlights_perception_ultralytics_efficientnet.pb.txt",
              "Pipeline config path containing TRAFFIC_LIGHT_DETECTION stage.");
DEFINE_int32(num_dummy_lights, 32,
             "Number of dummy hdmap lights used to surface detector outputs.");
DEFINE_int32(gpu_id, -1, "Override gpu_id in detection config when >= 0.");
DEFINE_string(sensor_name, "front_6mm", "Sensor name for DataProvider.");

namespace apollo {
namespace perception {
namespace camera {

namespace {

bool LoadDetectionConfig(const std::string& config_path,
                         TrafficLightDetectionConfig* detection_config) {
  if (detection_config == nullptr) {
    return false;
  }

  pipeline::PipelineConfig pipeline_config;
  if (!cyber::common::GetProtoFromFile(config_path, &pipeline_config)) {
    AERROR << "Failed to load pipeline config: " << config_path;
    return false;
  }

  for (const auto& stage_config : pipeline_config.stage_config()) {
    if (stage_config.stage_type() == pipeline::TRAFFIC_LIGHT_DETECTION &&
        stage_config.has_traffic_light_detection_config()) {
      *detection_config = stage_config.traffic_light_detection_config();
      return true;
    }
  }

  AERROR << "No TRAFFIC_LIGHT_DETECTION stage found in: " << config_path;
  return false;
}

std::string ColorToString(base::TLColor color) {
  switch (color) {
    case base::TLColor::TL_GREEN:
      return "Green";
    case base::TLColor::TL_RED:
      return "Red";
    case base::TLColor::TL_YELLOW:
      return "Yellow";
    case base::TLColor::TL_BLACK:
      return "Black";
    default:
      return "Unknown";
  }
}

cv::Scalar ColorToScalar(base::TLColor color) {
  switch (color) {
    case base::TLColor::TL_GREEN:
      return cv::Scalar(0, 255, 0);
    case base::TLColor::TL_RED:
      return cv::Scalar(0, 0, 255);
    case base::TLColor::TL_YELLOW:
      return cv::Scalar(0, 255, 255);
    case base::TLColor::TL_BLACK:
      return cv::Scalar(64, 64, 64);
    default:
      return cv::Scalar(255, 255, 255);
  }
}

void AddDummyLights(int num_lights, CameraFrame* frame) {
  frame->traffic_lights.clear();
  frame->traffic_lights.reserve(num_lights);
  for (int index = 0; index < num_lights; ++index) {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->id = "dummy_" + std::to_string(index);
    light->region.outside_image = true;
    light->region.projection_roi = base::RectI(0, 0, 0, 0);
    frame->traffic_lights.push_back(light);
  }
}

bool InitDataProvider(const cv::Mat& image, int gpu_id,
                      DataProvider* provider) {
  if (provider == nullptr) {
    return false;
  }

  DataProvider::InitOptions init_options;
  init_options.image_height = image.rows;
  init_options.image_width = image.cols;
  init_options.do_undistortion = false;
  init_options.sensor_name = FLAGS_sensor_name;
  init_options.device_id = gpu_id;

  if (!provider->Init(init_options)) {
    AERROR << "Failed to init data provider";
    return false;
  }

  if (!provider->FillImageData(image.rows, image.cols, image.data, "bgr8")) {
    AERROR << "Failed to fill image data";
    return false;
  }

  return true;
}

void DrawDetections(const CameraFrame& frame, cv::Mat* image) {
  if (image == nullptr) {
    return;
  }

  for (const auto& light : frame.traffic_lights) {
    if (light == nullptr || !light->region.is_detected) {
      continue;
    }
    const auto& roi = light->region.detection_roi;
    const cv::Scalar color = ColorToScalar(light->status.color);
    cv::rectangle(*image, cv::Rect(roi.x, roi.y, roi.width, roi.height), color,
                  2);

    const std::string label = ColorToString(light->status.color) + " " +
                              cv::format("%.2f", light->status.confidence);
    int base_line = 0;
    const cv::Size text_size =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.8, 2, &base_line);
    const int text_left = std::max(0, roi.x);
    const int text_top = std::max(text_size.height + 4, roi.y);
    cv::rectangle(*image, cv::Point(text_left, text_top - text_size.height - 8),
                  cv::Point(text_left + text_size.width + 6, text_top + 2),
                  color, cv::FILLED);
    cv::putText(*image, label, cv::Point(text_left + 3, text_top - 4),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
  }
}

}  // namespace

bool RunDetectorTool() {
  if (FLAGS_image_path.empty()) {
    AERROR << "--image_path is required";
    return false;
  }

  cv::Mat image = cv::imread(FLAGS_image_path, cv::IMREAD_COLOR);
  if (image.empty()) {
    AERROR << "Failed to read image: " << FLAGS_image_path;
    return false;
  }

  TrafficLightDetectionConfig detection_config;
  if (!LoadDetectionConfig(FLAGS_config_path, &detection_config)) {
    return false;
  }
  if (detection_config.detector_type() !=
      TL_DETECTION_YOLO_SINGLE_STAGE_DECTOR) {
    AERROR << "Detection config is not TL_DETECTION_YOLO_SINGLE_STAGE_DECTOR";
    return false;
  }
  if (FLAGS_gpu_id >= 0) {
    detection_config.set_gpu_id(FLAGS_gpu_id);
  }

  TrafficLightYoloSingleStageDector detector;
  if (!detector.Init(detection_config)) {
    AERROR << "Failed to init TrafficLightYoloSingleStageDector";
    return false;
  }

  CameraFrame frame;
  DataProvider provider;
  frame.data_provider = &provider;
  if (!InitDataProvider(image, detection_config.gpu_id(), &provider)) {
    return false;
  }

  AddDummyLights(std::max(1, FLAGS_num_dummy_lights), &frame);

  if (!detector.Detect(&frame)) {
    AERROR << "Detector inference failed";
    return false;
  }

  int detected_count = 0;
  for (const auto& light : frame.traffic_lights) {
    if (light != nullptr && light->region.is_detected) {
      ++detected_count;
    }
  }
  AINFO << "Detected lights: " << detected_count;

  cv::Mat vis = image.clone();
  DrawDetections(frame, &vis);
  if (!cv::imwrite(FLAGS_output_path, vis)) {
    AERROR << "Failed to save result image: " << FLAGS_output_path;
    return false;
  }

  AINFO << "Saved detector result to: " << FLAGS_output_path;
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  return apollo::perception::camera::RunDetectorTool() ? 0 : 1;
}
