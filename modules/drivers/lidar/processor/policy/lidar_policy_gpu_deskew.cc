#include <algorithm>
#include <vector>

#include "cyber/cyber.h"
#include "modules/drivers/lidar/processor/policy/gpu_lidar_policy.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_common.h"
#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
#include "modules/drivers/lidar/processor/policy/lidar_policy_cuda_kernels.h"
#endif

namespace apollo {
namespace drivers {
namespace lidar {

bool GpuLidarDeskewPolicy::Init(const LidarUnifiedComponentConfig& config,
                                apollo::transform::BufferInterface* tf_buffer) {
  if (!EnsureGpuBackendAvailable("GpuLidarDeskewPolicy")) {
    return false;
  }
  config_ = config;
  tf_buffer_ = tf_buffer;
  return tf_buffer_ != nullptr;
}

bool GpuLidarDeskewPolicy::ComputeMotionCompensationPoses(
    const SensorFrameContext& frame_context, std::vector<double>* sample_times,
    std::vector<Eigen::Affine3d>* poses) {
#ifndef APOLLO_LIDAR_POLICY_GPU_ENABLED
  (void)frame_context;
  (void)sample_times;
  (void)poses;
  return false;
#else
  if (frame_context.point_cloud == nullptr || frame_context.sensor_id.empty() ||
      sample_times == nullptr || poses == nullptr || tf_buffer_ == nullptr) {
    return false;
  }

  const size_t bins = std::max<size_t>(
      1, static_cast<size_t>(config_.motion_compensation_bins()));
  if (!BuildMotionSampleTimes(*frame_context.point_cloud, bins, true,
                              static_cast<int>(config_.gpu_device_id()),
                              sample_times)) {
    AERROR << "GpuLidarDeskewPolicy failed to build motion samples";
    return false;
  }
  poses->assign(bins, Eigen::Affine3d::Identity());

  for (size_t i = 0; i < bins; ++i) {
    const double sample_ts = (*sample_times)[i];

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    if (!QueryTransformAffine(tf_buffer_, config_.world_frame_id(),
                              frame_context.sensor_id, cyber::Time(sample_ts),
                              &pose)) {
      if (i == 0) {
        if (!QueryTransformAffine(
                tf_buffer_, config_.world_frame_id(), frame_context.sensor_id,
                cyber::Time(frame_context.point_cloud->measurement_time()),
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
#endif
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
