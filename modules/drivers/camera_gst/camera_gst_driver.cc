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

#include "modules/drivers/camera_gst/camera_gst_driver.h"

#include <algorithm>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

CameraGstDriver::CameraGstDriver(std::unique_ptr<CameraGstStreamer> streamer)
    : streamer_(std::move(streamer)) {
  if (streamer_ == nullptr) {
    streamer_.reset(new CameraGstStreamer());
  }
}

CameraGstDriver::~CameraGstDriver() { StopStreaming(); }

bool CameraGstDriver::Init(const config::Config& config,
                           PublishCallback publish_callback) {
  config_ = config;
  if (config_.sources_size() == 0) {
    AERROR << "camera_gst requires at least one source.";
    return false;
  }

  stitcher_.reset(new GridFrameStitcher(config_));
  if (!stitcher_->valid()) {
    AERROR << "camera_gst stitcher configuration is invalid.";
    return false;
  }

  sources_.clear();
  sources_.reserve(static_cast<size_t>(config_.sources_size()));
  for (const auto& source_config : config_.sources()) {
    auto source = CreateFrameSource(source_config);
    if (source == nullptr) {
      AERROR << "Failed to create source for " << source_config.name();
      return false;
    }
    sources_.emplace_back(std::move(source));
  }

  stream_enabled_ = config_.stream().enable();
  if (!streamer_->Start(output_width(), output_height(), config_.fps(),
                        config_.stream(), std::move(publish_callback))) {
    AERROR << "camera_gst failed to start in-process GStreamer pipeline.";
    return false;
  }
  stream_started_ = streamer_->streaming_active();
  return true;
}

bool CameraGstDriver::CaptureStitchedFrame(cv::Mat* stitched_bgr,
                                           double* measurement_time) {
  if (stitched_bgr == nullptr || measurement_time == nullptr) {
    return false;
  }

  std::vector<CapturedFrame> frames;
  frames.reserve(sources_.size());
  *measurement_time = 0.0;
  for (const auto& source : sources_) {
    CapturedFrame frame;
    if (!source->Read(&frame)) {
      AWARN_EVERY(100) << "Failed to read frame from source " << source->name();
      return false;
    }
    *measurement_time = std::max(*measurement_time, frame.measurement_time);
    frames.emplace_back(std::move(frame));
  }
  return stitcher_->Stitch(frames, stitched_bgr);
}

bool CameraGstDriver::SubmitFrame(const cv::Mat& stitched_bgr,
                                  double measurement_time) {
  return streamer_->Submit(stitched_bgr, measurement_time);
}

void CameraGstDriver::StartStreaming() {
  if (!stream_enabled_ || stream_started_) {
    return;
  }
  if (!streamer_->StartStreaming()) {
    AERROR << "Failed to start camera_gst stream branch.";
    return;
  }
  stream_started_ = true;
}

void CameraGstDriver::StopStreaming() {
  if (!stream_started_) {
    return;
  }
  streamer_->StopStreaming();
  stream_started_ = false;
}

int CameraGstDriver::output_width() const {
  return stitcher_ == nullptr ? 0 : stitcher_->output_width();
}

int CameraGstDriver::output_height() const {
  return stitcher_ == nullptr ? 0 : stitcher_->output_height();
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
