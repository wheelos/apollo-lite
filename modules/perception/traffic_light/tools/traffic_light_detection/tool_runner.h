/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "modules/perception/camera/common/camera_frame.h"
#include "modules/perception/camera/common/data_provider.h"
#include "modules/perception/pipeline/data_frame.h"
#include "modules/perception/traffic_light/application/traffic_light_perception_pipeline.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace tools {

struct TrafficLightDetectionToolOptions {
  std::string pipeline_config_path;
  std::string image_root_dir;
  std::string test_list_path;
  std::string image_ext = ".jpg";
  std::string output_dir = "./output";
  std::string sensor_name = "front_6mm";
  int image_width = 1920;
  int image_height = 1080;
  int gpu_id = 0;
};

class TrafficLightSeedProvider {
 public:
  virtual ~TrafficLightSeedProvider() = default;
  virtual bool Seed(const std::string& sample_id, const cv::Mat& image,
                    camera::CameraFrame* frame) const = 0;
};

class FullFrameSeedProvider final : public TrafficLightSeedProvider {
 public:
  bool Seed(const std::string& sample_id, const cv::Mat& image,
            camera::CameraFrame* frame) const override;
};

class TrafficLightResultWriter {
 public:
  virtual ~TrafficLightResultWriter() = default;
  virtual bool Write(const std::string& sample_id, const cv::Mat& image,
                     const camera::CameraFrame& frame) const = 0;
};

class LocalFileResultWriter final : public TrafficLightResultWriter {
 public:
  explicit LocalFileResultWriter(std::string output_dir);
  bool Write(const std::string& sample_id, const cv::Mat& image,
             const camera::CameraFrame& frame) const override;

 private:
  std::string output_dir_;
};

class TrafficLightDetectionToolRunner {
 public:
  TrafficLightDetectionToolRunner();

  bool Init(TrafficLightDetectionToolOptions options,
            std::unique_ptr<TrafficLightSeedProvider> seed_provider,
            std::unique_ptr<TrafficLightResultWriter> result_writer);
  bool RunBatch();

 private:
  bool LoadSampleList(std::vector<std::string>* samples) const;
  bool EnsureDataProviderReady();
  bool RunSingle(const std::string& sample_id);

 private:
  TrafficLightDetectionToolOptions options_;
  std::unique_ptr<TrafficLightSeedProvider> seed_provider_;
  std::unique_ptr<TrafficLightResultWriter> result_writer_;
  application::TrafficLightPerceptionPipeline pipeline_;
  camera::DataProvider data_provider_;
  bool data_provider_ready_ = false;
};

}  // namespace tools
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
