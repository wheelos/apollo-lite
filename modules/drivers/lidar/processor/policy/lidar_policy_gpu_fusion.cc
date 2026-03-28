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

GpuLidarFusionPolicy::GpuLidarFusionPolicy() = default;

GpuLidarFusionPolicy::~GpuLidarFusionPolicy() = default;

bool GpuLidarFusionPolicy::Init(const LidarUnifiedComponentConfig& config,
                                apollo::transform::BufferInterface* tf_buffer) {
  if (!EnsureGpuBackendAvailable("GpuLidarFusionPolicy")) {
    return false;
  }
  config_ = config;
  tf_buffer_ = tf_buffer;
  cuda_stream_ = nullptr;
  return tf_buffer_ != nullptr;
}

bool GpuLidarFusionPolicy::FuseToBaseLink(
    double reference_timestamp_sec,
    const std::vector<SensorFrameContext>& frames,
    const std::vector<std::vector<Eigen::Affine3d>>& frames_motion_poses,
    const std::vector<std::vector<double>>& frames_motion_times,
    PointCloudBuffer* output_buffer) {
#ifndef APOLLO_LIDAR_POLICY_GPU_ENABLED
  (void)reference_timestamp_sec;
  (void)frames;
  (void)frames_motion_poses;
  (void)frames_motion_times;
  (void)output_buffer;
  return false;
#else
  PointXYZIT* output_points = GetHostPoints(output_buffer);
  if (output_points == nullptr || tf_buffer_ == nullptr ||
      frames.size() != frames_motion_poses.size() ||
      frames.size() != frames_motion_times.size()) {
    return false;
  }

  Eigen::Affine3d world_from_base_ref = Eigen::Affine3d::Identity();
  if (!QueryTransformAffine(
          tf_buffer_, config_.world_frame_id(), config_.base_link_frame_id(),
          cyber::Time(reference_timestamp_sec), &world_from_base_ref)) {
    return false;
  }
  const CudaMatrix4f world2base = ToCudaMatrix(world_from_base_ref.inverse());

  size_t write_idx = 0;
  const int device_id = static_cast<int>(config_.gpu_device_id());

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

    std::vector<CudaPointXYZIT> input_points;
    input_points.reserve(static_cast<size_t>(frame.point_cloud->point_size()));
    for (const auto& point : frame.point_cloud->point()) {
      input_points.push_back(
          ToCudaPoint(point, frame.point_cloud->measurement_time()));
    }

    std::vector<CudaMatrix4f> cuda_poses;
    cuda_poses.reserve(poses.size());
    for (const auto& pose : poses) {
      cuda_poses.push_back(ToCudaMatrix(pose));
    }

    if (write_idx >= output_buffer->capacity) {
      output_buffer->valid_count = write_idx;
      return true;
    }
    std::vector<CudaPointXYZIT> fused_points(output_buffer->capacity -
                                             write_idx);
    const size_t fused_count = CudaFuseFrameToBaseLink(
        input_points.data(), input_points.size(), sample_times.data(),
        sample_times.size(), cuda_poses.data(), &world2base,
        frame.point_cloud->measurement_time(), fused_points.data(),
        fused_points.size(), device_id);
    if (fused_count == 0U && !input_points.empty()) {
      AERROR << "GpuLidarFusionPolicy CUDA fusion failed for frame "
             << frame.sensor_id;
      return false;
    }

    for (size_t i = 0; i < fused_count; ++i) {
      output_points[write_idx++] = ToProtoPoint(fused_points[i]);
    }
  }

  output_buffer->valid_count = write_idx;
  return true;
#endif
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
