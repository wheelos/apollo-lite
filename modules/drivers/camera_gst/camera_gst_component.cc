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

#include "modules/drivers/camera_gst/camera_gst_component.h"

#include <chrono>

namespace apollo {
namespace drivers {
namespace camera_gst {

bool CameraGstComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Parse config file failed: " << ConfigFilePath();
    return false;
  }
  if (config_.publish().channel_name().empty()) {
    AERROR << "camera_gst publish.channel_name must be configured.";
    return false;
  }

  writer_ = node_->CreateWriter<Image>(config_.publish().channel_name());
  if (config_.publish_rate() > 0.0) {
    rate_limiter_.reset(
        new apollo::cyber::common::TokenBucket(config_.publish_rate(), 10.0));
  }

  driver_.reset(new CameraGstDriver());
  auto publish_callback =
      [this](CameraGstStreamer::PublishedFrame&& frame) {
        if (rate_limiter_ != nullptr && !rate_limiter_->TryConsume()) {
          return;
        }
        writer_->Write(BuildImageMessage(frame));
      };
  if (!driver_->Init(config_, std::move(publish_callback))) {
    AERROR << "Failed to initialize camera_gst driver.";
    return false;
  }

  running_.store(true);
  async_result_ = cyber::Async(&CameraGstComponent::Run, this);
  return true;
}

void CameraGstComponent::Run() {
  while (running_.load() && !cyber::IsShutdown()) {
    cv::Mat stitched_bgr;
    double measurement_time = 0.0;
    if (!driver_->CaptureStitchedFrame(&stitched_bgr, &measurement_time)) {
      cyber::SleepFor(std::chrono::milliseconds(config_.capture_retry_ms()));
      continue;
    }
    if (!driver_->SubmitFrame(stitched_bgr, measurement_time)) {
      AWARN_EVERY(100) << "camera_gst failed to submit frame to pipeline.";
    }
  }
}

std::shared_ptr<Image> CameraGstComponent::BuildImageMessage(
    const CameraGstStreamer::PublishedFrame& frame) const {
  auto image = std::make_shared<Image>();
  image->mutable_header()->set_frame_id(config_.publish().frame_id());
  image->mutable_header()->set_timestamp_sec(frame.measurement_time);
  image->set_frame_id(config_.publish().frame_id());
  image->set_measurement_time(frame.measurement_time);
  image->set_width(static_cast<uint32_t>(frame.image_rgb.cols));
  image->set_height(static_cast<uint32_t>(frame.image_rgb.rows));
  image->set_encoding("rgb8");
  image->set_step(static_cast<uint32_t>(frame.image_rgb.step));
  image->set_data(reinterpret_cast<const char*>(frame.image_rgb.data),
                  static_cast<int>(frame.image_rgb.step *
                                   frame.image_rgb.rows));
  return image;
}

CameraGstComponent::~CameraGstComponent() {
  if (running_.exchange(false) && async_result_.valid()) {
    async_result_.wait();
  }
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
