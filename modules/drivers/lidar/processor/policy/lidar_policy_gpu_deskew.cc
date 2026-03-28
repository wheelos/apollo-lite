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

  std::vector<uint64_t> timestamps;
  timestamps.reserve(
      static_cast<size_t>(frame_context.point_cloud->point_size()));
  const uint64_t fallback_ts = static_cast<uint64_t>(
      frame_context.point_cloud->measurement_time() * kSecondToNano);
  for (const auto& point : frame_context.point_cloud->point()) {
    timestamps.push_back(point.timestamp() == 0U ? fallback_ts
                                                 : point.timestamp());
  }
  if (timestamps.empty()) {
    timestamps.push_back(fallback_ts);
  }

  uint64_t min_ts = 0U;
  uint64_t max_ts = 0U;
  if (!CudaComputeTimestampRange(timestamps.data(), timestamps.size(),
                                 static_cast<int>(config_.gpu_device_id()),
                                 &min_ts, &max_ts)) {
    AERROR << "GpuLidarDeskewPolicy failed to compute timestamp range on CUDA";
    return false;
  }

  const double local_min = static_cast<double>(min_ts) / kSecondToNano;
  const double local_max = static_cast<double>(max_ts) / kSecondToNano;
  const size_t bins = std::max<size_t>(
      1, static_cast<size_t>(config_.motion_compensation_bins()));
  sample_times->assign(bins, local_min);
  poses->assign(bins, Eigen::Affine3d::Identity());

  for (size_t i = 0; i < bins; ++i) {
    const double ratio =
        bins == 1 ? 0.0
                  : static_cast<double>(i) / static_cast<double>(bins - 1);
    const double sample_ts = local_min + ratio * (local_max - local_min);
    (*sample_times)[i] = sample_ts;

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
