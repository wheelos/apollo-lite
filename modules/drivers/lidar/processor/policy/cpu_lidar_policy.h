#pragma once

#include "modules/drivers/lidar/processor/policy/lidar_policy_interface.h"

namespace apollo {
namespace drivers {
namespace lidar {

class CpuLidarDeskewPolicy : public LidarDeskewPolicy {
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

class CpuLidarFusionPolicy : public LidarFusionPolicy {
 public:
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
};

class CpuLidarFilterPolicy : public LidarFilterPolicy {
 public:
  bool Init(const LidarUnifiedComponentConfig& config) override;
  size_t ApplyFilters(PointCloudBuffer* io_buffer, size_t* ego_filtered_count,
                      size_t* voxel_filtered_count) override;

 private:
  size_t ApplyVoxelFilter(PointCloudBuffer* io_buffer);
  size_t ApplyEgoQueryFilter(PointCloudBuffer* io_buffer);

  LidarUnifiedComponentConfig config_;
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
