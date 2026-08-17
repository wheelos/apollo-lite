// Copyright 2026 WheelOS All Rights Reserved.
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
#include <vector>

#include "Eigen/Geometry"

#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/processor/control/time_contract.h"

namespace apollo {
namespace drivers {
namespace lidar {

struct BufferedFrame {
  uint64_t frame_id = 0;
  std::shared_ptr<const ::apollo::drivers::PointCloud> point_cloud;
  TimeContract time_contract;
  std::vector<double> motion_sample_times;
  std::vector<Eigen::Affine3d> motion_poses;
  bool pose_prefetch_ok = false;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct FrameHandle {
  std::string sensor_id;
  std::shared_ptr<const ::apollo::drivers::PointCloud> point_cloud;
  std::shared_ptr<const BufferedFrame> buffered_frame;
  bool is_primary = false;
  uint64_t frame_id = 0;
  TimeContract time_contract;
  double clock_offset_residual_ms = 0.0;
  double overlap_quality_weight = 1.0;
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
