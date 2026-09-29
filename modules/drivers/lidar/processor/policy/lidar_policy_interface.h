// Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
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

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Geometry"

#include "modules/drivers/lidar/proto/lidar_unified_component_config.pb.h"

#include "modules/drivers/lidar/processor/common/pod_pointcloud_view.h"
#include "modules/transform/buffer_interface.h"

namespace apollo {
namespace drivers {
namespace lidar {

/// @brief Mutable host point-cloud workspace shared by CPU policies.
struct PointCloudBuffer {
  void* data_ptr = nullptr;  // Raw memory ptr to array of points
  size_t capacity = 0;       // Total allocated capacity (point count)
  size_t valid_count = 0;    // Number of effectively populated points
  size_t item_size = 0;      // sizeof(PointT) e.g., sizeof(PointXYZIT)
  size_t unfiltered_valid_count = 0;
  size_t prefiltered_ego_count = 0;
  bool ego_filter_applied = false;
};

using PointXYZIT = PointXYZITPod;

/// @brief Context carrying all necessary data for a single sensor frame.
struct SensorFrameContext {
  std::string sensor_id;
  std::shared_ptr<const PointCloudView> point_cloud;
  bool is_primary = false;
  double min_timestamp_sec = 0.0;
  double max_timestamp_sec = 0.0;
  double fallback_timestamp_sec = 0.0;
  double timestamp_offset_sec = 0.0;
  uint64_t fallback_timestamp_ns = 0;
  int64_t timestamp_offset_ns = 0;
  bool all_points_have_timestamps = false;
  std::vector<double> motion_sample_times;
  std::vector<Eigen::Affine3d> motion_poses;
};

// ============================================================================
// 1. Motion Compensation & Deskew Policy Interface (Generates TFs)
// ============================================================================
class LidarDeskewPolicy {
 public:
  virtual ~LidarDeskewPolicy() = default;

  virtual bool Init(const LidarUnifiedComponentConfig& config,
                    apollo::transform::BufferInterface* tf_buffer) = 0;

  virtual bool ComputeMotionCompensationPoses(
      const SensorFrameContext& frame_context,
      std::vector<double>* sample_times,
      std::vector<Eigen::Affine3d>* poses) = 0;
};

// ============================================================================
// 2. Spatial Fusion Policy Interface (Transforms & Merges Points)
// ============================================================================
class LidarFusionPolicy {
 public:
  virtual ~LidarFusionPolicy() = default;

  virtual bool Init(const LidarUnifiedComponentConfig& config,
                    apollo::transform::BufferInterface* tf_buffer) = 0;

  /// @brief Fuses multiple deskewed point clouds into a single base_link target
  /// cloud taking into account the primary sensor's reference time.
  virtual bool FuseToBaseLink(
      double reference_timestamp_sec, const Eigen::Affine3d& map2base_ref,
      const std::vector<SensorFrameContext>& frames,
      PointCloudBuffer* output_buffer) = 0;
};

// ============================================================================
// 3. Post-Processing Filter Policy Interface (Ego Car, Voxel)
// ============================================================================
class LidarFilterPolicy {
 public:
  virtual ~LidarFilterPolicy() = default;

  virtual bool Init(const LidarUnifiedComponentConfig& config) = 0;

  /// @brief Apply filtering passes (Ego box, Voxel) directly on the buffer
  /// memory.
  /// @return number of remaining valid points.
  virtual size_t ApplyFilters(PointCloudBuffer* io_buffer,
                              size_t* ego_filtered_count,
                              size_t* voxel_filtered_count) = 0;
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
