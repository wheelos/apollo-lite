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

#include <utility>

namespace apollo {
namespace drivers {
namespace camera_gst {

CameraGstComponent::~CameraGstComponent() {
  driver_.reset();
  if (gpu_frames_.load() > 0) {
    AINFO << "camera_gst gpu frames observed=" << gpu_frames_.load();
  }
}

bool CameraGstComponent::ValidateGpuOnlyConfig() const {
  for (const auto& source_config : config_.sources()) {
    if (!source_config.has_publish()) {
      continue;
    }
    AERROR
        << "camera_gst is GPU-only in this workspace. Source publish/readback"
        << " is unsupported for " << source_config.name()
        << ". Use publish_gpu_channel or stream output instead.";
    return false;
  }

  if (config_.has_publish() && !config_.publish().channel_name().empty()) {
    AERROR << "camera_gst is GPU-only in this workspace. Stitched CPU image "
           << "publishing is unsupported. Use the stream branch instead.";
    return false;
  }

  if (!config_.publish_gpu_channel() && !config_.stream().enable()) {
    AERROR << "camera_gst requires publish_gpu_channel=true or an enabled "
           << "stream branch in GPU-only mode.";
    return false;
  }

  if (config_.publish_gpu_channel() && !config_.zero_copy_required()) {
    AWARN << "camera_gst GPU-only mode is most effective with "
          << "zero_copy_required=true.";
  }
  return true;
}

bool CameraGstComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Parse config file failed: " << ConfigFilePath();
    return false;
  }
  gpu_frames_.store(0);

  if (!ValidateGpuOnlyConfig()) {
    return false;
  }

  driver_.reset(new CameraGstDriver());
  CameraGstDriver::GpuFrameCallback gpu_frame_callback;
  if (config_.publish_gpu_channel()) {
    gpu_frame_callback = [this](GpuFrame&& frame) {
      ++gpu_frames_;
      AINFO_EVERY(300) << "camera_gst received NVMM GPU frame from "
                       << frame.source_name << " " << frame.width << "x"
                       << frame.height << " format=" << frame.format;
    };
  }

  if (!driver_->Init(config_, CameraGstDriver::SourcePublishCallback(),
                     CameraGstDriver::PublishCallback(),
                     std::move(gpu_frame_callback))) {
    AERROR << "Failed to initialize camera_gst driver.";
    driver_.reset();
    return false;
  }
  return true;
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
