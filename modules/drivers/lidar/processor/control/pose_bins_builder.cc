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

#include "modules/drivers/lidar/processor/control/pose_bins_builder.h"

#include <utility>
#include <vector>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace lidar {

bool PoseBinsBuilder::Build(
    const std::vector<FrameHandle>& frame_handles,
    LidarDeskewPolicy* deskew_policy, std::vector<SensorFrameContext>* contexts,
    std::vector<std::vector<double>>* motion_sample_times,
    std::vector<std::vector<Eigen::Affine3d>>* motion_poses,
    size_t* required_points) const {
  if (contexts == nullptr || motion_sample_times == nullptr ||
      motion_poses == nullptr || required_points == nullptr) {
    return false;
  }

  contexts->clear();
  motion_sample_times->clear();
  motion_poses->clear();
  *required_points = 0;

  contexts->reserve(frame_handles.size());
  motion_sample_times->reserve(frame_handles.size());
  motion_poses->reserve(frame_handles.size());

  for (const auto& handle : frame_handles) {
    if (handle.point_cloud == nullptr) {
      if (handle.is_primary) {
        AERROR << "Main sensor frame is null: " << handle.sensor_id;
        return false;
      }
      AWARN << "Skip auxiliary sensor due to null frame: " << handle.sensor_id;
      continue;
    }

    SensorFrameContext context;
    context.sensor_id = handle.sensor_id;
    context.point_cloud = handle.point_cloud;
    context.is_primary = handle.is_primary;
    context.min_timestamp_sec =
        static_cast<double>(handle.time_contract.scan_begin_ns) / 1e9;
    context.max_timestamp_sec =
        static_cast<double>(handle.time_contract.scan_end_ns) / 1e9;
    context.fallback_timestamp_sec =
        static_cast<double>(handle.time_contract.canonical_anchor_ns -
                            handle.time_contract.static_offset_ns) /
        1e9;
    context.timestamp_offset_sec =
        static_cast<double>(handle.time_contract.static_offset_ns) / 1e9;
    context.fallback_timestamp_ns = static_cast<uint64_t>(
        handle.time_contract.canonical_anchor_ns -
        handle.time_contract.static_offset_ns);
    context.timestamp_offset_ns = handle.time_contract.static_offset_ns;
    context.all_points_have_timestamps =
        handle.time_contract.all_points_have_timestamps;

    std::vector<double> sample_times;
    std::vector<Eigen::Affine3d> poses;
    if (handle.buffered_frame != nullptr &&
        handle.buffered_frame->pose_prefetch_ok &&
        !handle.buffered_frame->motion_sample_times.empty() &&
        handle.buffered_frame->motion_sample_times.size() ==
            handle.buffered_frame->motion_poses.size()) {
      sample_times = handle.buffered_frame->motion_sample_times;
      poses = handle.buffered_frame->motion_poses;
    } else if (deskew_policy != nullptr &&
               deskew_policy->ComputeMotionCompensationPoses(
                   context, &sample_times, &poses) &&
               !sample_times.empty() && !poses.empty() &&
               sample_times.size() == poses.size()) {
      // Legacy fallback for call sites that have not migrated to prefetched
      // motion samples yet.
    } else {
      if (handle.is_primary) {
        AERROR << "Failed to compute motion compensation poses for main sensor "
               << handle.sensor_id;
        return false;
      }
      AWARN << "Skip auxiliary sensor due to invalid motion compensation data: "
            << handle.sensor_id;
      continue;
    }

    contexts->push_back(std::move(context));
    motion_sample_times->push_back(std::move(sample_times));
    motion_poses->push_back(std::move(poses));
    *required_points += static_cast<size_t>(handle.point_cloud->point_size());
  }

  return !contexts->empty();
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
