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

#include "modules/drivers/lidar/processor/policy/cpu_lidar_policy.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_common.h"

namespace apollo {
namespace drivers {
namespace lidar {

bool CpuLidarFilterPolicy::Init(const LidarUnifiedComponentConfig& config) {
  config_ = config;
  return true;
}

size_t CpuLidarFilterPolicy::ApplyFilters(PointCloudBuffer* io_buffer,
                                          size_t* ego_filtered_count,
                                          size_t* voxel_filtered_count) {
  if (ego_filtered_count != nullptr) {
    *ego_filtered_count = 0;
  }
  if (voxel_filtered_count != nullptr) {
    *voxel_filtered_count = 0;
  }

  if (GetHostPoints(io_buffer) == nullptr) {
    return 0;
  }

  const size_t before_ego = io_buffer->valid_count;
  const size_t after_ego = io_buffer->ego_filter_applied
                               ? io_buffer->valid_count
                               : ApplyEgoQueryFilter(io_buffer);
  if (ego_filtered_count != nullptr) {
    *ego_filtered_count = io_buffer->ego_filter_applied
                              ? io_buffer->prefiltered_ego_count
                              : before_ego - after_ego;
  }

  const size_t before_voxel = io_buffer->valid_count;
  const size_t after_voxel = ApplyVoxelFilter(io_buffer);
  if (voxel_filtered_count != nullptr) {
    *voxel_filtered_count = before_voxel - after_voxel;
  }

  return io_buffer->valid_count;
}

size_t CpuLidarFilterPolicy::ApplyVoxelFilter(PointCloudBuffer* io_buffer) {
  PointXYZIT* points = GetHostPoints(io_buffer);
  if (points == nullptr || io_buffer->valid_count == 0) {
    return 0;
  }
  if (!config_.enable_voxel_filter()) {
    return io_buffer->valid_count;
  }

  io_buffer->valid_count = ApplyDeterministicVoxelCentroidFilter(
      points, io_buffer->valid_count, config_.voxel_size());
  return io_buffer->valid_count;
}

size_t CpuLidarFilterPolicy::ApplyEgoQueryFilter(PointCloudBuffer* io_buffer) {
  PointXYZIT* points = GetHostPoints(io_buffer);
  if (points == nullptr || io_buffer->valid_count == 0 ||
      !config_.enable_ego_query_filter()) {
    return io_buffer == nullptr ? 0 : io_buffer->valid_count;
  }

  size_t write_idx = 0;
  for (size_t i = 0; i < io_buffer->valid_count; ++i) {
    const auto& point = points[i];
    const bool in_ego_box = point.x() < config_.ego_box_forward_x() &&
                            point.x() > config_.ego_box_backward_x() &&
                            point.y() < config_.ego_box_forward_y() &&
                            point.y() > config_.ego_box_backward_y();
    if (!in_ego_box) {
      if (write_idx != i) {
        points[write_idx] = point;
      }
      ++write_idx;
    }
  }

  io_buffer->valid_count = write_idx;
  return write_idx;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
