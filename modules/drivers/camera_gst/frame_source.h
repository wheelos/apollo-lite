/******************************************************************************
 * Copyright 2026 The WheelOS Team. All Rights Reserved.
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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "gst/gst.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/videoio.hpp"

#include "modules/drivers/camera_gst/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

struct CapturedFrame {
  std::string source_name;
  cv::Mat image_rgb;
  double measurement_time = 0.0;
};

class FrameSource {
 public:
  virtual ~FrameSource() = default;
  virtual const std::string& name() const = 0;
  virtual bool Read(CapturedFrame* frame) = 0;
};

class GstFrameSource : public FrameSource {
 public:
  explicit GstFrameSource(const config::CameraSourceConfig& config);
  ~GstFrameSource() override;

  const std::string& name() const override { return config_.name(); }
  bool Read(CapturedFrame* frame) override;

 private:
  bool Open();
  void Close();
    void AdvancePipelineDescription();
    const std::vector<std::string>& PipelineDescriptions();
  std::vector<std::string> BuildPipelineDescriptions() const;
  std::string BuildDeviceGpuPipelineDescription(
    const std::string& device_path) const;
  std::string BuildDeviceCpuPipelineDescription(
    const std::string& device_path) const;
  std::string BuildArgusPipelineDescription(int sensor_id) const;

  bool opened_ = false;
  config::CameraSourceConfig config_;
  std::vector<std::string> pipeline_descriptions_;
  size_t next_pipeline_description_index_ = 0;
  size_t opened_pipeline_description_index_ = 0;
  bool has_opened_pipeline_description_index_ = false;
  GstElement* pipeline_ = nullptr;
  GstElement* appsink_ = nullptr;
};

class FailoverFrameSource : public FrameSource {
 public:
  FailoverFrameSource(std::unique_ptr<FrameSource> primary,
                     std::unique_ptr<FrameSource> secondary,
                     size_t primary_failure_threshold);

  const std::string& name() const override { return name_; }
  bool Read(CapturedFrame* frame) override;

 private:
  std::string name_;
  std::unique_ptr<FrameSource> primary_;
  std::unique_ptr<FrameSource> secondary_;
  size_t primary_failure_threshold_ = 1;
  size_t primary_failure_count_ = 0;
  bool using_secondary_ = false;
};

class OpenCvFrameSource : public FrameSource {
 public:
  explicit OpenCvFrameSource(const config::CameraSourceConfig& config);
  ~OpenCvFrameSource() override;

  const std::string& name() const override { return config_.name(); }
  bool Read(CapturedFrame* frame) override;

 private:
  bool Open();
  static bool IsDevicePath(const std::string& uri);
  static bool IsNumericIndex(const std::string& uri);

  bool opened_ = false;
  config::CameraSourceConfig config_;
  cv::VideoCapture capture_;
};

std::unique_ptr<FrameSource> CreateFrameSource(
    const config::CameraSourceConfig& config);

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
