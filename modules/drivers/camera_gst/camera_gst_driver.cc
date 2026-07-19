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

#include <utility>

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

CameraGstDriver::~CameraGstDriver() {
  if (streamer_ != nullptr) {
    streamer_->Stop();
  }
}

bool CameraGstDriver::Init(const config::Config& config,
                           SourcePublishCallback source_publish_callback,
                           PublishCallback stitched_publish_callback,
                           GpuFrameCallback gpu_frame_callback) {
  config_ = config;
  if (config_.sources_size() == 0) {
    AERROR << "camera_gst requires at least one source.";
    return false;
  }
  stream_enabled_ = config_.stream().enable();

  if (!streamer_->Start(config_, std::move(source_publish_callback),
                        std::move(stitched_publish_callback),
                        std::move(gpu_frame_callback))) {
    AERROR << "camera_gst failed to start the GPU capture graph.";
    return false;
  }

  stream_started_ = streamer_->streaming_active();
  return true;
}

void CameraGstDriver::StartStreaming() {
  if (!stream_enabled_ || stream_started_ || streamer_ == nullptr) {
    return;
  }
  if (!streamer_->StartStreaming()) {
    AERROR << "Failed to start camera_gst stream branch.";
    return;
  }
  stream_started_ = true;
}

void CameraGstDriver::StopStreaming() {
  if (!stream_started_ || streamer_ == nullptr) {
    return;
  }
  streamer_->StopStreaming();
  stream_started_ = false;
}

int CameraGstDriver::output_width() const {
  return streamer_ == nullptr ? 0 : streamer_->output_width();
}

int CameraGstDriver::output_height() const {
  return streamer_ == nullptr ? 0 : streamer_->output_height();
}

StreamStats CameraGstDriver::stats() const {
  return streamer_ == nullptr ? StreamStats() : streamer_->stats();
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
