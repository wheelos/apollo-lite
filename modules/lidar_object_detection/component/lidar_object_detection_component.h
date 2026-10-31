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
#include <mutex>
#include <string>
#include <vector>

#include "modules/lidar_object_detection/proto/lidar_object_detection.pb.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

#include "cyber/cyber.h"
#include "modules/lidar_object_detection/inference/centerpoint_backend.h"
#include "modules/lidar_object_detection/inference/centerpoint_runner.h"
#include "modules/lidar_object_detection/inference/pointpillars_runtime_api.h"
#include "modules/lidar_object_detection/types/sweep_fusion.h"
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"

namespace apollo {
namespace lidar_object_detection {

class LidarObjectDetectionComponent
    : public cyber::Component<apollo::drivers::PointCloud> {
 public:
  ~LidarObjectDetectionComponent() override;
  bool Init() override;
  bool Proc(
      const std::shared_ptr<apollo::drivers::PointCloud>& message) override;

 private:
  bool InitCenterPoint(const CenterPointModelConfig& config,
                       std::string* error);
  bool InitPointPillars(const PointPillarsModelConfig& config,
                        std::string* error);
  bool ProcessCenterPoint(
      const std::shared_ptr<const apollo::drivers::PointCloud>& message,
      LidarObjectDetectionFrameResult* result, std::string* error);
  bool ProcessPointPillars(
      const std::shared_ptr<const apollo::drivers::PointCloud>& message,
      LidarObjectDetectionFrameResult* result, std::string* error);

  LidarObjectDetectionComponentConfig config_;
  std::unique_ptr<CenterPointBackend> centerpoint_backend_;
  std::unique_ptr<CenterPointRunner> centerpoint_runner_;
  std::unique_ptr<perception::onboard::TransformWrapper> transform_;
  PointCloudSweepFusion sweep_fusion_;
  void* pointpillars_library_ = nullptr;
  void* pointpillars_runtime_ = nullptr;
  decltype(&pointpillars_runtime_destroy) pointpillars_destroy_ = nullptr;
  decltype(&pointpillars_runtime_infer) pointpillars_infer_ = nullptr;
  std::shared_ptr<cyber::Writer<LidarObjectDetectionFrameResult>> writer_;
  std::mutex mutex_;
};

CYBER_REGISTER_COMPONENT(LidarObjectDetectionComponent);

}  // namespace lidar_object_detection
}  // namespace apollo
