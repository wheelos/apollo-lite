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

#include <atomic>
#include <mutex>
#include <vector>

#include "modules/drivers/lidar/processor/policy/lidar_policy_cuda_kernels.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_interface.h"

// Forward declaration of CUDA kernels or utility structures
struct CUstream_st;
typedef struct CUstream_st* cudaStream_t;

namespace apollo {
namespace drivers {
namespace lidar {

/// @brief GPU acceleration policy for generating Motion Compensation
/// transforms. Typically poses are sparse enough to compute on CPU, but this
/// allows for pure GPU pipelines.
class GpuLidarDeskewPolicy : public LidarDeskewPolicy {
 public:
  bool Init(const LidarUnifiedComponentConfig& config,
            apollo::transform::BufferInterface* tf_buffer) override;

  bool ComputeMotionCompensationPoses(
      const SensorFrameContext& frame_context,
      std::vector<double>* sample_times,
      std::vector<Eigen::Affine3d>* poses) override;

 private:
  LidarUnifiedComponentConfig config_;
  apollo::transform::BufferInterface* tf_buffer_ = nullptr;
};

/// @brief GPU parallelized spatial fusion and deskew execution.
class GpuLidarFusionPolicy : public LidarFusionPolicy {
 public:
  GpuLidarFusionPolicy();
  ~GpuLidarFusionPolicy();

  bool Init(const LidarUnifiedComponentConfig& config,
            apollo::transform::BufferInterface* tf_buffer) override;

  bool FuseToBaseLink(
      double reference_timestamp_sec, const Eigen::Affine3d& map2base_ref,
      const std::vector<SensorFrameContext>& frames,
      const std::vector<std::vector<Eigen::Affine3d>>& frames_motion_poses,
      const std::vector<std::vector<double>>& frames_motion_times,
      PointCloudBuffer* output_buffer) override;

 private:
  LidarUnifiedComponentConfig config_;
  apollo::transform::BufferInterface* tf_buffer_ = nullptr;
  cudaStream_t* cuda_stream_ = nullptr;  // Asynchronous CUDA execution stream
  std::mutex scratch_mutex_;
  std::vector<CudaPointXYZIT> host_input_points_;
  std::vector<CudaPointXYZIT> host_fused_points_;
  std::vector<CudaPose> host_pose_buffer_;
  std::atomic<uint64_t> metrics_calls_{0};
  std::atomic<uint64_t> metrics_input_points_{0};
  std::atomic<uint64_t> metrics_output_points_{0};
  std::atomic<uint64_t> metrics_elapsed_ns_{0};
};

/// @brief Extremely fast GPU-based bounding box filtering & parallel bitonic
/// voxel downsample.
class GpuLidarFilterPolicy : public LidarFilterPolicy {
 public:
  GpuLidarFilterPolicy();
  ~GpuLidarFilterPolicy();

  bool Init(const LidarUnifiedComponentConfig& config) override;

  size_t ApplyFilters(PointCloudBuffer* io_buffer, size_t* ego_filtered_count,
                      size_t* voxel_filtered_count) override;

 private:
  LidarUnifiedComponentConfig config_;
  cudaStream_t* cuda_stream_ = nullptr;
  void* d_voxel_hash_table_ = nullptr;  // Persistent GPU Hash Table memory
  std::mutex scratch_mutex_;
  std::vector<CudaPointXYZIT> host_input_points_;
  std::vector<CudaPointXYZIT> host_ego_filtered_points_;
  std::vector<PointXYZIT> host_centroid_points_;
  std::atomic<uint64_t> metrics_calls_{0};
  std::atomic<uint64_t> metrics_input_points_{0};
  std::atomic<uint64_t> metrics_output_points_{0};
  std::atomic<uint64_t> metrics_elapsed_ns_{0};
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
