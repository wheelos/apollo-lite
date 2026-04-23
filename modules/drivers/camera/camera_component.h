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
#include <string>
#include <vector>

#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/drivers/camera/proto/config.pb.h"

#include "cyber/cyber.h"
#include "modules/common/util/rate_limiter.h"
#include "modules/drivers/camera/backend/camera_device.h"

namespace apollo {
namespace drivers {
namespace camera {

class CameraComponent : public apollo::cyber::Component<> {
 public:
  bool Init() override;
  ~CameraComponent();

 private:
  void Run();

  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::Image>> writer_;
  std::unique_ptr<CameraDevice> camera_device_;
  std::shared_ptr<config::Config> camera_config_;

  // Increase the ring size to reduce overwrite risk when downstream is slow.
  static const int kBufferSize = 20;
  std::vector<std::shared_ptr<apollo::drivers::Image>> pb_image_buffer_;

  uint32_t device_wait_ms_;
  int index_ = 0;

  std::future<void> async_result_;
  std::atomic<bool> running_ = {false};
  std::unique_ptr<apollo::cyber::common::TokenBucket> rate_limiter_;
};

CYBER_REGISTER_COMPONENT(CameraComponent)

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
