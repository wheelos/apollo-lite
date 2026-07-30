#include <mutex>
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
  const size_t max_points = std::max<size_t>(
      1, static_cast<size_t>(config_.max_full_pointcloud_points()));
  host_input_points_.reserve(max_points);
  host_fused_points_.reserve(max_points);
  host_pose_buffer_.reserve(std::max<size_t>(
      1, static_cast<size_t>(config_.motion_compensation_bins())));
  return tf_buffer_ != nullptr;
}

bool GpuLidarFusionPolicy::FuseToBaseLink(
    double reference_timestamp_sec, const Eigen::Affine3d& map2base_ref,
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
  const uint64_t begin_ns = cyber::Time::Now().ToNanosecond();

  PointXYZIT* output_points = GetHostPoints(output_buffer);
  if (output_points == nullptr || tf_buffer_ == nullptr ||
      frames.size() != frames_motion_poses.size() ||
      frames.size() != frames_motion_times.size()) {
    return false;
  }
  const Eigen::Affine3d base_from_map_ref = map2base_ref;

  size_t write_idx = 0;
  size_t total_input_points = 0;
  const int device_id = static_cast<int>(config_.gpu_device_id());

  std::lock_guard<std::mutex> lock(scratch_mutex_);

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

    const size_t frame_point_count =
        static_cast<size_t>(frame.point_cloud->point_size());
    total_input_points += frame_point_count;
    if (host_input_points_.size() < frame_point_count) {
      host_input_points_.resize(frame_point_count);
    }
    size_t input_count = 0;
    for (const auto& point : frame.point_cloud->point()) {
      host_input_points_[input_count++] =
          ToCudaPoint(point, frame.point_cloud->measurement_time());
    }

    if (host_pose_buffer_.size() < poses.size()) {
      host_pose_buffer_.resize(poses.size());
    }
    for (size_t pose_idx = 0; pose_idx < poses.size(); ++pose_idx) {
      host_pose_buffer_[pose_idx] =
          ToCudaPose(base_from_map_ref * poses[pose_idx]);
    }

    if (write_idx >= output_buffer->capacity) {
      output_buffer->valid_count = write_idx;
      const uint64_t elapsed_ns = cyber::Time::Now().ToNanosecond() - begin_ns;
      metrics_calls_.fetch_add(1, std::memory_order_relaxed);
      metrics_input_points_.fetch_add(total_input_points,
                                      std::memory_order_relaxed);
      metrics_output_points_.fetch_add(write_idx, std::memory_order_relaxed);
      metrics_elapsed_ns_.fetch_add(elapsed_ns, std::memory_order_relaxed);
      return true;
    }
    const size_t remaining_capacity = output_buffer->capacity - write_idx;
    if (host_fused_points_.size() < remaining_capacity) {
      host_fused_points_.resize(remaining_capacity);
    }
    const size_t fused_count = CudaFuseFrameToBaseLink(
        host_input_points_.data(), input_count, sample_times.data(),
        sample_times.size(), host_pose_buffer_.data(),
        frame.point_cloud->measurement_time(), host_fused_points_.data(),
        remaining_capacity, device_id);
    if (fused_count == 0U && input_count > 0U) {
      AERROR << "GpuLidarFusionPolicy CUDA fusion failed for frame "
             << frame.sensor_id;
      return false;
    }

    for (size_t i = 0; i < fused_count; ++i) {
      output_points[write_idx++] = ToProtoPoint(host_fused_points_[i]);
    }
  }

  output_buffer->valid_count = write_idx;

  const uint64_t elapsed_ns = cyber::Time::Now().ToNanosecond() - begin_ns;
  const uint64_t calls =
      metrics_calls_.fetch_add(1, std::memory_order_relaxed) + 1;
  metrics_input_points_.fetch_add(total_input_points,
                                  std::memory_order_relaxed);
  metrics_output_points_.fetch_add(write_idx, std::memory_order_relaxed);
  const uint64_t accumulated_ns =
      metrics_elapsed_ns_.fetch_add(elapsed_ns, std::memory_order_relaxed) +
      elapsed_ns;
  if (calls % 100 == 0) {
    const uint64_t total_in =
        metrics_input_points_.load(std::memory_order_relaxed);
    const uint64_t total_out =
        metrics_output_points_.load(std::memory_order_relaxed);
    CudaWorkspaceStats ws_stats;
    const bool has_ws_stats = CudaGetWorkspaceStats(device_id, &ws_stats);
    AINFO << "GpuLidarFusionPolicy metrics: calls=" << calls << ", avg_ms="
          << static_cast<double>(accumulated_ns) / static_cast<double>(calls) /
                 1e6
          << ", avg_in_points="
          << static_cast<double>(total_in) / static_cast<double>(calls)
          << ", avg_out_points="
          << static_cast<double>(total_out) / static_cast<double>(calls)
          << ", ws_expand(in/out/poses)="
          << (has_ws_stats ? ws_stats.points_in_expand_count : 0) << "/"
          << (has_ws_stats ? ws_stats.points_out_expand_count : 0) << "/"
          << (has_ws_stats ? ws_stats.poses_expand_count : 0)
          << ", ws_peak(in/out/poses)="
          << (has_ws_stats ? ws_stats.points_in_peak_capacity : 0) << "/"
          << (has_ws_stats ? ws_stats.points_out_peak_capacity : 0) << "/"
          << (has_ws_stats ? ws_stats.poses_peak_capacity : 0);
  }
  return true;
#endif
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
