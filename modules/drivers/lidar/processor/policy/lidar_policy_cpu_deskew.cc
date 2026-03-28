#include <algorithm>

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

  double local_min = 0.0;
  double local_max = 0.0;
  if (!ResolvePointTimestampBounds(*frame_context.point_cloud, &local_min,
                                   &local_max)) {
    return false;
  }

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
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
