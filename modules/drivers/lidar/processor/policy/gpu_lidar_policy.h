#pragma once

#include "modules/drivers/lidar/processor/policy/lidar_policy_interface.h"

// Forward declaration of CUDA kernels or utility structures
struct cudaStream_t;

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
      double reference_timestamp_sec,
      const std::vector<SensorFrameContext>& frames,
      const std::vector<std::vector<Eigen::Affine3d>>& frames_motion_poses,
      const std::vector<std::vector<double>>& frames_motion_times,
      PointCloudBuffer* output_buffer) override;

 private:
  LidarUnifiedComponentConfig config_;
  apollo::transform::BufferInterface* tf_buffer_ = nullptr;
  cudaStream_t* cuda_stream_ = nullptr;  // Asynchronous CUDA execution stream
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
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
