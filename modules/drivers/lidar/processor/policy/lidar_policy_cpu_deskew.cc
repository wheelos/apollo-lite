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

#include <algorithm>
#include <vector>

#include "modules/drivers/lidar/processor/policy/cpu_lidar_policy.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_common.h"

namespace apollo {
namespace drivers {
namespace lidar {

bool CpuLidarDeskewPolicy::Init(const LidarUnifiedComponentConfig& config,
                                apollo::transform::BufferInterface* tf_buffer) {
  config_ = config;
  tf_buffer_ = tf_buffer;
  return tf_buffer_ != nullptr;
}

bool CpuLidarDeskewPolicy::ComputeMotionCompensationPoses(
    const SensorFrameContext& frame_context, std::vector<double>* sample_times,
    std::vector<Eigen::Affine3d>* poses) {
  if (frame_context.point_cloud == nullptr || frame_context.sensor_id.empty() ||
      sample_times == nullptr || poses == nullptr || tf_buffer_ == nullptr) {
    return false;
  }

  const size_t bins = std::max<size_t>(
      1, static_cast<size_t>(config_.motion_compensation_bins()));
  if (!BuildMotionSampleTimes(*frame_context.point_cloud, bins, false, -1,
                              sample_times)) {
    return false;
  }
  poses->assign(bins, Eigen::Affine3d::Identity());

  for (size_t i = 0; i < bins; ++i) {
    const double sample_ts = (*sample_times)[i];

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    if (!QueryTransformAffine(
            tf_buffer_, config_.map_frame_id(), frame_context.sensor_id,
            cyber::Time(sample_ts),
            static_cast<float>(config_.sensor_pose_query_timeout_sec()),
            &pose)) {
      if (i == 0) {
        if (!QueryTransformAffine(
                tf_buffer_, config_.map_frame_id(), frame_context.sensor_id,
                cyber::Time(frame_context.point_cloud->measurement_time()),
                static_cast<float>(config_.sensor_pose_query_timeout_sec()),
                &pose)) {
          return false;
        }
      } else {
        pose = (*poses)[i - 1];
      }
    }
    (*poses)[i] = pose;
  }

  return true;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
