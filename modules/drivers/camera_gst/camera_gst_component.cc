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

namespace apollo {
namespace drivers {
namespace camera_gst {

namespace {

config::PublishConfig NormalizePublishConfig(
    const config::PublishConfig& publish_config,
    const std::string& default_frame_id) {
  config::PublishConfig normalized = publish_config;
  if (normalized.frame_id().empty()) {
    normalized.set_frame_id(default_frame_id);
  }
  return normalized;
}

}  // namespace

bool CameraGstComponent::InitSourcePublishers() {
  source_publishers_.clear();
  for (const auto& source_config : config_.sources()) {
    if (!source_config.has_publish() ||
        source_config.publish().channel_name().empty()) {
      continue;
    }

    SourcePublisher publisher;
    publisher.publish_config = NormalizePublishConfig(source_config.publish(),
                                                     source_config.name());
    publisher.writer =
        node_->CreateWriter<Image>(publisher.publish_config.channel_name());
    source_publishers_.emplace(source_config.name(), std::move(publisher));
  }
  return true;
}

bool CameraGstComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Parse config file failed: " << ConfigFilePath();
    return false;
  }

  if (!InitSourcePublishers()) {
    AERROR << "camera_gst failed to initialize source publishers.";
    return false;
  }

  if (config_.has_publish() && !config_.publish().channel_name().empty()) {
    stitched_publish_config_ =
        NormalizePublishConfig(config_.publish(), "camera_gst");
    stitched_writer_ =
        node_->CreateWriter<Image>(stitched_publish_config_.channel_name());
  }

  if (source_publishers_.empty() && stitched_writer_ == nullptr &&
      !config_.stream().enable()) {
    AERROR << "camera_gst requires at least one source publish channel, a "
           << "stitched publish channel, or an enabled stream branch.";
    return false;
  }

  driver_.reset(new CameraGstDriver());
  CameraGstDriver::SourcePublishCallback source_publish_callback;
  if (!source_publishers_.empty()) {
    source_publish_callback =
        [this](const std::string& source_name,
               CameraGstStreamer::PublishedFrame&& frame) {
          auto iter = source_publishers_.find(source_name);
          if (iter == source_publishers_.end()) {
            return;
          }
          iter->second.writer->Write(
              BuildImageMessage(std::move(frame), iter->second.publish_config));
        };
  }

  CameraGstStreamer::PublishCallback stitched_publish_callback;
  if (stitched_writer_ != nullptr) {
    stitched_publish_callback =
        [this](CameraGstStreamer::PublishedFrame&& frame) {
          stitched_writer_->Write(
              BuildImageMessage(std::move(frame), stitched_publish_config_));
        };
  }

  if (!driver_->Init(config_, std::move(source_publish_callback),
                     std::move(stitched_publish_callback))) {
    AERROR << "Failed to initialize camera_gst driver.";
    return false;
  }
  return true;
}

std::shared_ptr<Image> CameraGstComponent::BuildImageMessage(
    CameraGstStreamer::PublishedFrame frame,
    const config::PublishConfig& publish_config) const {
  auto image = std::make_shared<Image>();
  image->mutable_header()->set_frame_id(publish_config.frame_id());
  image->mutable_header()->set_timestamp_sec(frame.measurement_time);
  image->set_frame_id(publish_config.frame_id());
  image->set_measurement_time(frame.measurement_time);
  image->set_width(frame.width);
  image->set_height(frame.height);
  image->set_encoding(frame.encoding.empty() ? "rgb8" : frame.encoding);
  image->set_step(frame.step);
  *image->mutable_data() = std::move(frame.data);
  return image;
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
