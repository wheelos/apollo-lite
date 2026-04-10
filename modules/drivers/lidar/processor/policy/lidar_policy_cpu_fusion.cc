#include "cyber/cyber.h"
#include "modules/drivers/lidar/processor/policy/cpu_lidar_policy.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_common.h"

namespace apollo {
namespace drivers {
namespace lidar {

bool CpuLidarFusionPolicy::Init(const LidarUnifiedComponentConfig& config,
                                apollo::transform::BufferInterface* tf_buffer) {
  config_ = config;
  tf_buffer_ = tf_buffer;
  return tf_buffer_ != nullptr;
}

bool CpuLidarFusionPolicy::FuseToBaseLink(
    double reference_timestamp_sec, const Eigen::Affine3d& map2base_ref,
    const std::vector<SensorFrameContext>& frames,
    const std::vector<std::vector<Eigen::Affine3d>>& frames_motion_poses,
    const std::vector<std::vector<double>>& frames_motion_times,
    PointCloudBuffer* output_buffer) {
  PointXYZIT* output_points = GetHostPoints(output_buffer);
  if (output_points == nullptr || tf_buffer_ == nullptr ||
      frames.size() != frames_motion_poses.size() ||
      frames.size() != frames_motion_times.size()) {
    return false;
  }
  (void)reference_timestamp_sec;

  size_t write_idx = 0;
  for (size_t frame_idx = 0; frame_idx < frames.size(); ++frame_idx) {
    const auto& frame = frames[frame_idx];
    if (frame.point_cloud == nullptr) {
      continue;
    }

    const auto& sample_times = frames_motion_times[frame_idx];
    const auto& poses = frames_motion_poses[frame_idx];
    if (sample_times.empty() || poses.empty() ||
        sample_times.size() != poses.size()) {
      if (frame.is_primary) {
        return false;
      }
      AWARN << "Skip auxiliary frame due to invalid motion compensation data: "
            << frame.sensor_id;
      continue;
    }

    for (const auto& point : frame.point_cloud->point()) {
      if (write_idx >= output_buffer->capacity) {
        AWARN << "Output point buffer is full, truncating fused cloud at "
              << write_idx << " points";
        output_buffer->valid_count = write_idx;
        return true;
      }

      PointXYZIT transformed_point;
      if (!TransformPointToBase(point, frame.point_cloud->measurement_time(),
                                sample_times, poses, map2base_ref,
                                &transformed_point)) {
        continue;
      }
      output_points[write_idx++] = transformed_point;
    }
  }

  output_buffer->valid_count = write_idx;
  return true;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
