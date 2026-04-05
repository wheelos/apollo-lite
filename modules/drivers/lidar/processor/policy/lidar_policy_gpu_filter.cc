#include <algorithm>
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

GpuLidarFilterPolicy::GpuLidarFilterPolicy() = default;

GpuLidarFilterPolicy::~GpuLidarFilterPolicy() = default;

bool GpuLidarFilterPolicy::Init(const LidarUnifiedComponentConfig& config) {
  if (!EnsureGpuBackendAvailable("GpuLidarFilterPolicy")) {
    return false;
  }
  config_ = config;
  cuda_stream_ = nullptr;
  d_voxel_hash_table_ = nullptr;

  const size_t max_points = std::max<size_t>(
      1, static_cast<size_t>(config_.max_full_pointcloud_points()));
  host_input_points_.reserve(max_points);
  host_ego_filtered_points_.reserve(max_points);
  host_centroid_points_.reserve(max_points);
  return true;
}

size_t GpuLidarFilterPolicy::ApplyFilters(PointCloudBuffer* io_buffer,
                                          size_t* ego_filtered_count,
                                          size_t* voxel_filtered_count) {
#ifndef APOLLO_LIDAR_POLICY_GPU_ENABLED
  (void)io_buffer;
  (void)ego_filtered_count;
  (void)voxel_filtered_count;
  return 0;
#else
  const uint64_t begin_ns = cyber::Time::Now().ToNanosecond();

  PointXYZIT* host_points = GetHostPoints(io_buffer);
  if (ego_filtered_count != nullptr) {
    *ego_filtered_count = 0;
  }
  if (voxel_filtered_count != nullptr) {
    *voxel_filtered_count = 0;
  }
  if (host_points == nullptr || io_buffer->valid_count == 0) {
    return 0;
  }

  const int device_id = static_cast<int>(config_.gpu_device_id());
  std::lock_guard<std::mutex> lock(scratch_mutex_);

  const size_t input_count = io_buffer->valid_count;
  if (host_input_points_.size() < input_count) {
    host_input_points_.resize(input_count);
  }
  for (size_t i = 0; i < input_count; ++i) {
    host_input_points_[i] = ToCudaPoint(
        host_points[i],
        static_cast<double>(host_points[i].timestamp()) / kSecondToNano);
  }

  if (host_ego_filtered_points_.size() < input_count) {
    host_ego_filtered_points_.resize(input_count);
  }
  CudaEgoFilterParams ego_params;
  ego_params.enable = config_.enable_ego_query_filter();
  ego_params.forward_x = config_.ego_box_forward_x();
  ego_params.backward_x = config_.ego_box_backward_x();
  ego_params.forward_y = config_.ego_box_forward_y();
  ego_params.backward_y = config_.ego_box_backward_y();

  const size_t after_ego = CudaApplyEgoFilter(
      host_input_points_.data(), input_count, ego_params,
      host_ego_filtered_points_.data(), input_count, device_id);
  if (ego_filtered_count != nullptr) {
    *ego_filtered_count = input_count - after_ego;
  }

  if (host_centroid_points_.size() < after_ego) {
    host_centroid_points_.resize(after_ego);
  }
  for (size_t i = 0; i < after_ego; ++i) {
    host_centroid_points_[i] = ToProtoPoint(host_ego_filtered_points_[i]);
  }
  const size_t after_voxel = ApplyDeterministicVoxelCentroidFilter(
      host_centroid_points_.data(), after_ego, config_.voxel_size());
  if (voxel_filtered_count != nullptr) {
    *voxel_filtered_count = after_ego - after_voxel;
  }

  const size_t write_count = std::min(after_voxel, io_buffer->capacity);
  for (size_t i = 0; i < write_count; ++i) {
    host_points[i] = host_centroid_points_[i];
  }
  io_buffer->valid_count = write_count;

  const uint64_t elapsed_ns = cyber::Time::Now().ToNanosecond() - begin_ns;
  const uint64_t calls =
      metrics_calls_.fetch_add(1, std::memory_order_relaxed) + 1;
  metrics_input_points_.fetch_add(input_count, std::memory_order_relaxed);
  metrics_output_points_.fetch_add(write_count, std::memory_order_relaxed);
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
    AINFO << "GpuLidarFilterPolicy metrics: calls=" << calls << ", avg_ms="
          << static_cast<double>(accumulated_ns) / static_cast<double>(calls) /
                 1e6
          << ", avg_in_points="
          << static_cast<double>(total_in) / static_cast<double>(calls)
          << ", avg_out_points="
          << static_cast<double>(total_out) / static_cast<double>(calls)
          << ", ws_expand(in/out/hash)="
          << (has_ws_stats ? ws_stats.points_in_expand_count : 0) << "/"
          << (has_ws_stats ? ws_stats.points_out_expand_count : 0) << "/"
          << (has_ws_stats ? ws_stats.hash_table_expand_count : 0)
          << ", ws_peak(in/out/hash)="
          << (has_ws_stats ? ws_stats.points_in_peak_capacity : 0) << "/"
          << (has_ws_stats ? ws_stats.points_out_peak_capacity : 0) << "/"
          << (has_ws_stats ? ws_stats.hash_table_peak_capacity : 0);
  }

  return write_count;
#endif
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
