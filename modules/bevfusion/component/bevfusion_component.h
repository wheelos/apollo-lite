// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "modules/bevfusion/proto/bevfusion.pb.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"
#include "wheelos_msgs/sensor_msgs/sensor_image.pb.h"

#include "cyber/component/component.h"
#include "modules/bevfusion/inference/bevfusion_runner.h"
#include "modules/bevfusion/types/bounded_synchronizer.h"

namespace apollo {
namespace bevfusion {

class BevFusionComponent final
    : public cyber::Component<apollo::drivers::Image> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<apollo::drivers::Image>& message) override;

 private:
  void OnAdditionalCamera(
      std::size_t camera_index,
      const std::shared_ptr<apollo::drivers::Image>& message);
  void OnLidar(const std::shared_ptr<apollo::drivers::PointCloud>& message);
  bool EnqueueCamera(std::size_t camera_index,
                     const apollo::drivers::Image& message, std::string* error);
  bool EnqueueLidar(const apollo::drivers::PointCloud& message,
                    std::string* error);
  bool Drain(std::string* error);
  bool RunBundle(const SynchronizedPackets& packets, std::string* error);

  BevFusionComponentConfig config_;
  BevFusionRunner runner_;
  std::shared_ptr<cyber::Writer<BevFusionFrameResult>> writer_;
  std::array<float, kCameraCount * 16> camera_to_lidar_{};
  std::array<float, kCameraCount * 16> camera_intrinsics_{};
  std::array<float, kCameraCount * 16> lidar_to_image_{};
  std::array<float, kCameraCount * 16> image_augmentation_{};
  std::array<float, kPointFeatureCount> point_feature_scale_{};
  BoundedSynchronizer synchronizer_;
  std::vector<std::shared_ptr<cyber::Reader<apollo::drivers::Image>>>
      camera_readers_;
  std::shared_ptr<cyber::Reader<apollo::drivers::PointCloud>> lidar_reader_;
  std::mutex callback_mutex_;
};

CYBER_REGISTER_COMPONENT(BevFusionComponent)

}  // namespace bevfusion
}  // namespace apollo
