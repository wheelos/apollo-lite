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

#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <vector>

#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/drivers/camera/proto/config.pb.h"

#include "cyber/cyber.h"
#include "modules/drivers/camera/backend/camera_device.h"

namespace apollo {
namespace drivers {
namespace camera {

/**
 * @class CameraComponent
 * @brief Apollo Cyber RT component for camera device management and image
 * publishing.
 *
 * This component initializes a CameraDevice, continuously polls for images,
 * processes them, and publishes them as protobuf messages to a Cyber RT
 * channel.
 */
class CameraComponent : public apollo::cyber::Component<> {
 public:
  bool Init() override;
  ~CameraComponent();

 private:
  // Main execution loop for image polling and publishing
  void Run();

  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::Image>>
      writer_;  ///< Cyber RT writer for image messages
  std::unique_ptr<CameraDevice> camera_device_;    ///< Camera device interface
  std::shared_ptr<config::Config> camera_config_;  ///< Camera configuration
  std::vector<std::shared_ptr<apollo::drivers::Image>>
      pb_image_buffer_;  ///< Circular buffer for protobuf image messages

  uint32_t device_wait_ms_;  ///< Delay in milliseconds after poll failure
  int index_ = 0;            ///< Current index in the circular buffer
  int buffer_size_ = 3;      ///< Size of the circular buffer

  static constexpr int32_t kMaxImageSize =
      20 * 1024 * 1024;  ///< Maximum allowed image size in bytes (20 MB)

  std::future<void>
      async_result_;  ///< Future object for managing the async run() thread
  std::atomic<bool> running_ = {
      false};  ///< Atomic flag to control the run() loop's state
};

CYBER_REGISTER_COMPONENT(CameraComponent)

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
