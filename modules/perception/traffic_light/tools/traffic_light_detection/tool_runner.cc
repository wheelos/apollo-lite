/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/

#include "modules/perception/traffic_light/tools/traffic_light_detection/tool_runner.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <utility>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/pipeline/proto/pipeline_config.pb.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace tools {

namespace {

std::string ColorToString(base::TLColor color) {
  switch (color) {
    case base::TLColor::TL_RED:
      return "RED";
    case base::TLColor::TL_YELLOW:
      return "YELLOW";
    case base::TLColor::TL_GREEN:
      return "GREEN";
    case base::TLColor::TL_BLACK:
      return "BLACK";
    default:
      return "UNKNOWN";
  }
}

cv::Scalar ColorToBgr(base::TLColor color) {
  switch (color) {
    case base::TLColor::TL_RED:
      return cv::Scalar(0, 0, 255);
    case base::TLColor::TL_YELLOW:
      return cv::Scalar(0, 255, 255);
    case base::TLColor::TL_GREEN:
      return cv::Scalar(0, 255, 0);
    case base::TLColor::TL_BLACK:
      return cv::Scalar(64, 64, 64);
    default:
      return cv::Scalar(255, 255, 255);
  }
}

std::string JoinPath(const std::string& left, const std::string& right) {
  if (left.empty()) {
    return right;
  }
  if (left.back() == '/') {
    return left + right;
  }
  return left + "/" + right;
}

}  // namespace

bool FullFrameSeedProvider::Seed(const std::string& sample_id,
                                 const cv::Mat& image,
                                 camera::CameraFrame* frame) const {
  (void)sample_id;
  if (frame == nullptr) {
    return false;
  }

  frame->traffic_lights.clear();
  base::TrafficLightPtr light(new base::TrafficLight);
  light->id = "seed_0";
  light->region.projection_roi = base::RectI(0, 0, image.cols, image.rows);
  light->region.crop_roi = light->region.projection_roi;
  light->region.outside_image = false;
  frame->traffic_lights.push_back(light);
  return true;
}

LocalFileResultWriter::LocalFileResultWriter(std::string output_dir)
    : output_dir_(std::move(output_dir)) {}

bool LocalFileResultWriter::Write(const std::string& sample_id,
                                  const cv::Mat& image,
                                  const camera::CameraFrame& frame) const {
  if (!cyber::common::EnsureDirectory(output_dir_)) {
    AERROR << "Failed to ensure output directory: " << output_dir_;
    return false;
  }

  const std::string txt_path = JoinPath(output_dir_, sample_id + ".txt");
  std::ofstream ofs(txt_path, std::ios::out | std::ios::trunc);
  if (!ofs.is_open()) {
    AERROR << "Failed to open result file: " << txt_path;
    return false;
  }

  cv::Mat vis = image.clone();
  for (const auto& light : frame.traffic_lights) {
    if (light == nullptr) {
      continue;
    }

    const auto& roi = light->region.detection_roi;
    ofs << light->id << " " << ColorToString(light->status.color) << " "
        << light->status.confidence << " " << roi.x << " " << roi.y << " "
        << roi.width << " " << roi.height << "\n";

    if (!light->region.is_detected || roi.width <= 0 || roi.height <= 0) {
      continue;
    }

    const cv::Scalar color = ColorToBgr(light->status.color);
    cv::rectangle(vis, cv::Rect(roi.x, roi.y, roi.width, roi.height), color, 2);
    const std::string label =
        ColorToString(light->status.color) + " " +
        cv::format("%.2f", light->status.confidence);
    cv::putText(vis, label,
                cv::Point(std::max(0, roi.x), std::max(16, roi.y - 4)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
  }
  ofs.close();

  const std::string vis_path = JoinPath(output_dir_, sample_id + ".jpg");
  if (!cv::imwrite(vis_path, vis)) {
    AERROR << "Failed to write visualization file: " << vis_path;
    return false;
  }
  return true;
}

TrafficLightDetectionToolRunner::TrafficLightDetectionToolRunner() = default;

bool TrafficLightDetectionToolRunner::Init(
    TrafficLightDetectionToolOptions options,
    std::unique_ptr<TrafficLightSeedProvider> seed_provider,
    std::unique_ptr<TrafficLightResultWriter> result_writer) {
  options_ = std::move(options);
  seed_provider_ = std::move(seed_provider);
  result_writer_ = std::move(result_writer);
  if (seed_provider_ == nullptr || result_writer_ == nullptr) {
    AERROR << "Seed provider or result writer is null.";
    return false;
  }

  pipeline::PipelineConfig pipeline_config;
  if (!cyber::common::GetProtoFromFile(options_.pipeline_config_path,
                                       &pipeline_config)) {
    AERROR << "Failed to load pipeline config: "
           << options_.pipeline_config_path;
    return false;
  }
  return pipeline_.Init(pipeline_config);
}

bool TrafficLightDetectionToolRunner::RunBatch() {
  std::vector<std::string> samples;
  if (!LoadSampleList(&samples)) {
    return false;
  }

  if (samples.empty()) {
    AERROR << "No samples found in test list.";
    return false;
  }

  if (!EnsureDataProviderReady()) {
    return false;
  }

  for (const auto& sample_id : samples) {
    if (!RunSingle(sample_id)) {
      AERROR << "Failed on sample: " << sample_id;
      return false;
    }
  }
  return true;
}

bool TrafficLightDetectionToolRunner::LoadSampleList(
    std::vector<std::string>* samples) const {
  if (samples == nullptr) {
    return false;
  }

  std::ifstream ifs(options_.test_list_path);
  if (!ifs.is_open()) {
    AERROR << "Failed to open test list file: " << options_.test_list_path;
    return false;
  }

  std::string line;
  while (std::getline(ifs, line)) {
    if (!line.empty()) {
      samples->push_back(line);
    }
  }
  return true;
}

bool TrafficLightDetectionToolRunner::EnsureDataProviderReady() {
  if (data_provider_ready_) {
    return true;
  }

  camera::DataProvider::InitOptions init_options;
  init_options.sensor_name = options_.sensor_name;
  init_options.image_height = options_.image_height;
  init_options.image_width = options_.image_width;
  init_options.device_id = options_.gpu_id;
  init_options.do_undistortion = false;

  data_provider_ready_ = data_provider_.Init(init_options);
  if (!data_provider_ready_) {
    AERROR << "Failed to initialize data provider.";
  }
  return data_provider_ready_;
}

bool TrafficLightDetectionToolRunner::RunSingle(const std::string& sample_id) {
  std::string image_path = JoinPath(options_.image_root_dir, "images/");
  image_path = JoinPath(image_path, sample_id + options_.image_ext);

  cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
  if (image.empty()) {
    AERROR << "Failed to read image: " << image_path;
    return false;
  }

  if (image.cols != options_.image_width || image.rows != options_.image_height) {
    cv::resize(image, image, cv::Size(options_.image_width, options_.image_height));
  }

  if (!data_provider_.FillImageData(image.rows, image.cols, image.data, "bgr8")) {
    AERROR << "Failed to fill image to data provider: " << image_path;
    return false;
  }

  camera::CameraFrame frame;
  frame.data_provider = &data_provider_;
  frame.timestamp = 0.0;

  if (!seed_provider_->Seed(sample_id, image, &frame)) {
    AERROR << "Failed to seed traffic lights for sample: " << sample_id;
    return false;
  }

  pipeline::DataFrame data_frame;
  data_frame.camera_frame = &frame;
  if (!pipeline_.Process(&data_frame)) {
    AERROR << "Traffic light pipeline process failed for sample: " << sample_id;
    return false;
  }

  return result_writer_->Write(sample_id, image, frame);
}

}  // namespace tools
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
