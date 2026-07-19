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

#include <algorithm>
#include <utility>

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

CameraGstComponent::~CameraGstComponent() {
  driver_.reset();
  StopSourcePublishers();
}

bool CameraGstComponent::InitSourcePublishers() {
  StopSourcePublishers();
  source_publishers_.clear();
  for (const auto& source_config : config_.sources()) {
    if (!source_config.has_publish() ||
        source_config.publish().channel_name().empty()) {
      continue;
    }

    auto result = source_publishers_.try_emplace(source_config.name());
    result.first->second.publish_config =
        NormalizePublishConfig(source_config.publish(), source_config.name());
    result.first->second.writer = node_->CreateWriter<Image>(
        result.first->second.publish_config.channel_name());
  }

  for (auto& entry : source_publishers_) {
    entry.second.worker = std::thread(&CameraGstComponent::SourcePublisherLoop,
                                      this, entry.first);
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
      !config_.stream().enable() && !config_.publish_gpu_channel()) {
    AERROR << "camera_gst requires at least one source publish channel, a "
           << "stitched publish channel, an enabled stream branch, or GPU "
           << "frame publishing.";
    return false;
  }

  driver_.reset(new CameraGstDriver());
  CameraGstDriver::SourcePublishCallback source_publish_callback;
  if (!source_publishers_.empty()) {
    source_publish_callback = [this](const std::string& source_name,
                                     PublishedFrame&& frame) {
      EnqueueSourceFrame(source_name, std::move(frame));
    };
  }

  CameraGstStreamer::PublishCallback stitched_publish_callback;
  if (stitched_writer_ != nullptr) {
    stitched_publish_callback = [this](PublishedFrame&& frame) {
      stitched_writer_->Write(
          BuildImageMessage(std::move(frame), stitched_publish_config_));
    };
  }

  CameraGstDriver::GpuFrameCallback gpu_frame_callback;
  if (config_.publish_gpu_channel()) {
    gpu_frame_callback = [](GpuFrame&& frame) {
      AINFO_EVERY(300) << "camera_gst received NVMM GPU frame from "
                       << frame.source_name << " " << frame.width << "x"
                       << frame.height << " format=" << frame.format;
    };
  }

  if (!driver_->Init(config_, std::move(source_publish_callback),
                     std::move(stitched_publish_callback),
                     std::move(gpu_frame_callback))) {
    AERROR << "Failed to initialize camera_gst driver.";
    driver_.reset();
    StopSourcePublishers();
    return false;
  }
  return true;
}

void CameraGstComponent::StopSourcePublishers() {
  for (auto& entry : source_publishers_) {
    {
      std::lock_guard<std::mutex> lock(entry.second.mutex);
      entry.second.stop_requested = true;
    }
    entry.second.condition.notify_all();
  }

  for (auto& entry : source_publishers_) {
    if (entry.second.worker.joinable()) {
      entry.second.worker.join();
    }
    AINFO << "camera_gst source publisher " << entry.first
          << " enqueued=" << entry.second.enqueued_frames
          << " dropped=" << entry.second.dropped_frames
          << " published=" << entry.second.published_frames;
  }
}

void CameraGstComponent::SourcePublisherLoop(const std::string& source_name) {
  auto iter = source_publishers_.find(source_name);
  if (iter == source_publishers_.end()) {
    return;
  }

  auto& publisher = iter->second;
  while (true) {
    PublishedFrame frame;
    {
      std::unique_lock<std::mutex> lock(publisher.mutex);
      publisher.condition.wait(lock, [&publisher]() {
        return publisher.stop_requested || !publisher.pending_frames.empty();
      });
      if (publisher.pending_frames.empty()) {
        if (publisher.stop_requested) {
          break;
        }
        continue;
      }
      frame = std::move(publisher.pending_frames.front());
      publisher.pending_frames.pop_front();
    }

    publisher.writer->Write(BuildImageMessage(frame, publisher.publish_config));
    ++publisher.published_frames;
  }
}

void CameraGstComponent::EnqueueSourceFrame(const std::string& source_name,
                                            PublishedFrame&& frame) {
  auto iter = source_publishers_.find(source_name);
  if (iter == source_publishers_.end()) {
    return;
  }

  auto& publisher = iter->second;
  const size_t queue_capacity =
      std::max<size_t>(1, publisher.publish_config.queue_capacity());
  {
    std::lock_guard<std::mutex> lock(publisher.mutex);
    if (publisher.stop_requested) {
      return;
    }
    if (publisher.pending_frames.size() >= queue_capacity) {
      publisher.pending_frames.pop_front();
      ++publisher.dropped_frames;
    }
    publisher.pending_frames.push_back(std::move(frame));
    ++publisher.enqueued_frames;
  }
  publisher.condition.notify_one();
}

std::shared_ptr<Image> CameraGstComponent::BuildImageMessage(
    const PublishedFrame& frame,
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
  image->mutable_data()->assign(frame.data.data(), frame.data.size());
  return image;
}

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
