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

  try {
    camera_device_ = std::make_unique<CameraDevice>(camera_config_);
  } catch (const std::exception& e) {
    AERROR << "Failed to initialize CameraDevice: " << e.what();
    return false;
  }

  const uint32_t actual_width = camera_config_->width();
  const uint32_t actual_height = camera_config_->height();
  uint32_t image_size_expected = 0;
  std::string encoding_str;
  uint32_t step_bytes = 0;

  if (camera_config_->output_type() == config::OutputType::YUYV) {
    image_size_expected = actual_width * actual_height * 2;
    encoding_str = "yuyv";
    step_bytes = 2 * actual_width;
  } else if (camera_config_->output_type() == config::OutputType::RGB) {
    image_size_expected = actual_width * actual_height * 3;
    encoding_str = "rgb8";
    step_bytes = 3 * actual_width;
  } else {
    AERROR << "Unsupported output type.";
    return false;
  }

  device_wait_ms_ = camera_config_->device_wait_ms();
  double pub_rate = camera_config_->has_publish_rate()
                        ? camera_config_->publish_rate()
                        : camera_config_->frame_rate();
  // set burst size to 10(larger than 1 to allow some bursts) for jittery data
  rate_limiter_ =
      std::make_unique<apollo::cyber::common::TokenBucket>(pub_rate, 10.0);

  // Buffer Initialization
  for (int i = 0; i < kBufferSize; ++i) {
    auto pb_image = std::make_shared<Image>();
    pb_image->mutable_header()->set_frame_id(camera_config_->frame_id());
    pb_image->set_width(actual_width);
    pb_image->set_height(actual_height);
    pb_image->set_encoding(encoding_str);
    pb_image->set_step(step_bytes);
    pb_image->mutable_data()->resize(image_size_expected);
    pb_image_buffer_.push_back(pb_image);
  }

  writer_ = node_->CreateWriter<Image>(camera_config_->channel_name());
  running_.store(true);
  async_result_ = cyber::Async(&CameraComponent::Run, this);
  return true;
}

void CameraComponent::Run() {
  while (running_.load() && !cyber::IsShutdown()) {
    int selected_index = index_;
    int probe_count = 0;
    while (probe_count < kBufferSize &&
           pb_image_buffer_[selected_index].use_count() > 1) {
      selected_index = (selected_index + 1) % kBufferSize;
      ++probe_count;
    }

    if (probe_count == kBufferSize) {
      AWARN_EVERY(100)
          << "All camera buffers are busy, dropping this cycle.";
      cyber::SleepFor(std::chrono::milliseconds(1));
      continue;
    }

    auto pb_image = pb_image_buffer_[selected_index];

    // 1) Poll from camera device (blocking wait in driver layer)
    if (!camera_device_->Poll(pb_image)) {
      cyber::SleepFor(std::chrono::milliseconds(device_wait_ms_));
      continue;
    }

    // 2) Keep header timestamp aligned with the capture timestamp set by device.
    pb_image->mutable_header()->set_timestamp_sec(pb_image->measurement_time());

    // 3) Rate limiting
    if (!rate_limiter_->TryConsume()) {
      index_ = (selected_index + 1) % kBufferSize;
      continue;
    }

    writer_->Write(pb_image);
    index_ = (selected_index + 1) % kBufferSize;
  }
}

CameraComponent::~CameraComponent() {
  if (running_.exchange(false)) {
    if (async_result_.valid()) {
      async_result_.wait();
    }
  }
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
