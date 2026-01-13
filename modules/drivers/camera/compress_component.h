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

#include <memory>

#include <opencv2/opencv.hpp>
#include <mutex>

#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/drivers/camera/proto/config.pb.h"

#include "cyber/base/concurrent_object_pool.h"
#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace camera {

using apollo::cyber::Component;
using apollo::cyber::Writer;
using apollo::cyber::base::CCObjectPool;
using apollo::drivers::Image;
using apollo::drivers::CompressedImage;
using apollo::drivers::camera::config::Config;

class CompressComponent : public Component<Image> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Image>& image) override;

 private:
  Config config_;
  std::shared_ptr<CCObjectPool<CompressedImage>> image_pool_;
  std::shared_ptr<Writer<CompressedImage>> writer_;
  std::mutex mutex_;

  // 预分配的 Buffer，成员变量复用
  std::vector<uint8_t> compressed_buffer_;
  std::vector<int> compress_params_;
  // 临时 Mat，避免反复构造
  cv::Mat resize_mat_;
};

CYBER_REGISTER_COMPONENT(CompressComponent)
}  // namespace camera
}  // namespace drivers
}  // namespace apollo
