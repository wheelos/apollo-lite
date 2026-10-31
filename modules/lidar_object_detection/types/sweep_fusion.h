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

#include <cstddef>
#include <deque>
#include <memory>
#include <string>

#include "Eigen/Geometry"

#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace lidar_object_detection {

class PointCloudSweepFusion {
 public:
  bool Init(std::size_t sweep_count, std::size_t max_point_count,
            std::string* error);

  bool AddFrame(const std::shared_ptr<const apollo::drivers::PointCloud>& cloud,
                const Eigen::Affine3d& sensor_to_world,
                apollo::drivers::PointCloud* fused_cloud, std::string* error);

  void Reset();

 private:
  struct SweepFrame {
    std::shared_ptr<const apollo::drivers::PointCloud> cloud;
    Eigen::Affine3d sensor_to_world = Eigen::Affine3d::Identity();
    double measurement_time = 0.0;
  };

  std::deque<SweepFrame> frames_;
  std::size_t sweep_count_ = 0U;
  std::size_t max_point_count_ = 0U;
  double last_measurement_time_ = 0.0;
  bool has_last_measurement_time_ = false;
  bool initialized_ = false;
};

}  // namespace lidar_object_detection
}  // namespace apollo
