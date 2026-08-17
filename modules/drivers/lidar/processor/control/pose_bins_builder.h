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

#include <cstddef>
#include <vector>

#include "Eigen/Eigen"

#include "modules/drivers/lidar/processor/control/frame_handle.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_interface.h"

namespace apollo {
namespace drivers {
namespace lidar {

class PoseBinsBuilder {
 public:
  bool Build(const std::vector<FrameHandle>& frame_handles,
             LidarDeskewPolicy* deskew_policy,
             std::vector<SensorFrameContext>* contexts,
             std::vector<std::vector<double>>* motion_sample_times,
             std::vector<std::vector<Eigen::Affine3d>>* motion_poses,
             size_t* required_points) const;
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
