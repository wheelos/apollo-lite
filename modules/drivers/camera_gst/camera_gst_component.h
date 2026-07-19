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

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/drivers/camera_gst/proto/config.pb.h"

#include "cyber/cyber.h"
#include "modules/drivers/camera_gst/camera_gst_driver.h"
#include "modules/drivers/camera_gst/frame_types.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

class CameraGstComponent : public apollo::cyber::Component<> {
 public:
  ~CameraGstComponent() override;
  bool Init() override;

 private:
  struct SourcePublisher {
    config::PublishConfig publish_config;
    std::shared_ptr<apollo::cyber::Writer<Image>> writer;
    std::mutex mutex;
    std::condition_variable condition;
    std::deque<PublishedFrame> pending_frames;
    std::thread worker;
    bool stop_requested = false;
    uint64_t enqueued_frames = 0;
    uint64_t dropped_frames = 0;
    uint64_t published_frames = 0;
  };

  bool InitSourcePublishers();
  void StopSourcePublishers();
  void SourcePublisherLoop(const std::string& source_name);
  void EnqueueSourceFrame(const std::string& source_name,
                          PublishedFrame&& frame);
  std::shared_ptr<Image> BuildImageMessage(
      const PublishedFrame& frame,
      const config::PublishConfig& publish_config) const;

  config::Config config_;
  config::PublishConfig stitched_publish_config_;
  std::shared_ptr<apollo::cyber::Writer<Image>> stitched_writer_;
  std::unordered_map<std::string, SourcePublisher> source_publishers_;
  std::unique_ptr<CameraGstDriver> driver_;
};

CYBER_REGISTER_COMPONENT(CameraGstComponent)

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
