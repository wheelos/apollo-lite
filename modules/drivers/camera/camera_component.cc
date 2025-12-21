/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/camera/camera_component.h"

namespace apollo {
namespace drivers {
namespace camera {

bool CameraComponent::Init() {
  camera_config_ = std::make_shared<config::Config>();
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               camera_config_.get())) {
    AERROR << "Failed to load camera config from: " << config_file_path_;
    return false;
  }
  AINFO << "Camera config: " << camera_config_->DebugString();

  try {
    camera_device_ = std::make_unique<CameraDevice>(camera_config_);
  } catch (const std::exception& e) {
    AERROR << "Failed to initialize CameraDevice: " << e.what();
    return false;
  }

  // Get the final actual resolution negotiated by the driver from CameraDevice
  const uint32_t actual_width = camera_config_->width();
  const uint32_t actual_height = camera_config_->height();

  uint32_t image_size_expected;
  std::string encoding_str;
  uint32_t step_bytes;

  if (camera_config_->output_type() == config::OutputType::YUYV) {
    image_size_expected = actual_width * actual_height * 2;
    encoding_str = "yuyv";
    step_bytes = 2 * actual_width;
  } else if (camera_config_->output_type() == config::OutputType::RGB) {
    image_size_expected = actual_width * actual_height * 3;
    encoding_str = "rgb8";
    step_bytes = 3 * actual_width;
  } else {
    AERROR << "Unsupported output type in config: "
           << camera_config_->output_type();
    return false;
  }

  if (image_size_expected > kMaxImageSize) {
    AERROR << "Image size is too big (" << image_size_expected
           << " bytes), must be less than " << kMaxImageSize << " bytes.";
    return false;
  }

  device_wait_ms_ = camera_config_->device_wait_ms();

  // Initialize Protobuf message buffer pool
  for (int i = 0; i < buffer_size_; ++i) {
    auto pb_image = std::make_shared<Image>();
    pb_image->mutable_header()->set_frame_id(camera_config_->frame_id());
    pb_image->set_width(actual_width);
    pb_image->set_height(actual_height);
    pb_image->set_encoding(encoding_str);
    pb_image->set_step(step_bytes);

    // Key: Pre-allocate memory by resizing the string. This enables Zero-Copy
    // during subsequent image processing by ensuring mutable_data()->data()
    // returns a valid pointer to the allocated memory.
    pb_image->mutable_data()->resize(image_size_expected);

    pb_image_buffer_.push_back(pb_image);
  }

  writer_ = node_->CreateWriter<Image>(camera_config_->channel_name());

  // Start asynchronous run loop
  running_.store(true);
  async_result_ = cyber::Async(&CameraComponent::Run, this);
  return true;
}

void CameraComponent::Run() {
  while (running_.load() && !cyber::IsShutdown()) {
    // Get a protobuf message from the buffer pool
    auto pb_image = pb_image_buffer_[index_];
    index_ = (index_ + 1) % buffer_size_;

    // The Poll method now directly processes data into pb_image
    if (!camera_device_->Poll(pb_image)) {
      // If Poll fails, the internal reconnection logic has been handled, just
      // wait here
      cyber::SleepFor(std::chrono::milliseconds(device_wait_ms_));
      continue;
    }

    // Set publish timestamp
    pb_image->mutable_header()->set_timestamp_sec(
        cyber::Time::Now().ToSecond());

    // measurement_time and image data have been filled in Poll()
    writer_->Write(pb_image);

    // Note: There is no extra SleepFor(spin_rate_). The loop rate is determined
    // by the blocking behavior of Poll().
  }
}

CameraComponent::~CameraComponent() {
  if (running_.exchange(false)) {  // Use exchange to ensure atomicity
    // Ensure the async thread has finished before destruction completes
    if (async_result_.valid()) {
      async_result_.wait();
    }
  }
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
