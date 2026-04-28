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

#include <atomic>
#include <future>
#include <memory>

#include "cyber/cyber.h"
#include "modules/common/util/rate_limiter.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/drivers/camera_gst/camera_gst_driver.h"
#include "modules/drivers/camera_gst/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace camera_gst {

class CameraGstComponent : public apollo::cyber::Component<> {
 public:
  bool Init() override;
  ~CameraGstComponent();

 private:
  void Run();
  std::shared_ptr<Image> BuildImageMessage(
      const CameraGstStreamer::PublishedFrame& frame) const;

  config::Config config_;
  std::shared_ptr<apollo::cyber::Writer<Image>> writer_;
  std::unique_ptr<CameraGstDriver> driver_;
  std::future<void> async_result_;
  std::atomic<bool> running_ = {false};
  std::unique_ptr<apollo::cyber::common::TokenBucket> rate_limiter_;
};

CYBER_REGISTER_COMPONENT(CameraGstComponent)

}  // namespace camera_gst
}  // namespace drivers
}  // namespace apollo
