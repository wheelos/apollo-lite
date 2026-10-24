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

#include <memory>
#include <string>

#include "modules/lidar_object_detection/proto/lidar_object_detection.pb.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

#include "cyber/cyber.h"
#include "modules/lidar_object_detection/inference/centerpoint_backend.h"
#include "modules/lidar_object_detection/inference/centerpoint_runner.h"
#include "modules/lidar_object_detection/types/sweep_fusion.h"
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"

namespace apollo {
namespace lidar_object_detection {

class CenterPointComponent
    : public cyber::Component<apollo::drivers::PointCloud> {
 public:
  bool Init() override;
  bool Proc(
      const std::shared_ptr<apollo::drivers::PointCloud>& message) override;

 private:
  CenterPointComponentConfig config_;
  std::unique_ptr<CenterPointBackend> backend_;
  std::unique_ptr<CenterPointRunner> runner_;
  std::unique_ptr<perception::onboard::TransformWrapper> transform_;
  PointCloudSweepFusion sweep_fusion_;
  std::shared_ptr<cyber::Writer<CenterPointFrameResult>> writer_;
};

CYBER_REGISTER_COMPONENT(CenterPointComponent);

}  // namespace lidar_object_detection
}  // namespace apollo
