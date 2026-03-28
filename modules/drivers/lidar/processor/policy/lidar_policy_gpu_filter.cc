#include <algorithm>
#include <vector>

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
  std::vector<CudaPointXYZIT> input_points(io_buffer->valid_count);
  for (size_t i = 0; i < io_buffer->valid_count; ++i) {
    input_points[i] = ToCudaPoint(
        host_points[i],
        static_cast<double>(host_points[i].timestamp()) / kSecondToNano);
  }

  std::vector<CudaPointXYZIT> ego_filtered(io_buffer->valid_count);
  CudaEgoFilterParams ego_params;
  ego_params.enable = config_.enable_ego_query_filter();
  ego_params.forward_x = config_.ego_box_forward_x();
  ego_params.backward_x = config_.ego_box_backward_x();
  ego_params.forward_y = config_.ego_box_forward_y();
  ego_params.backward_y = config_.ego_box_backward_y();

  const size_t after_ego =
      CudaApplyEgoFilter(input_points.data(), input_points.size(), ego_params,
                         ego_filtered.data(), ego_filtered.size(), device_id);
  if (ego_filtered_count != nullptr) {
    *ego_filtered_count = input_points.size() - after_ego;
  }

  std::vector<CudaPointXYZIT> voxel_filtered(after_ego);
  const size_t after_voxel = CudaApplyVoxelDownsample(
      ego_filtered.data(), after_ego, config_.voxel_size(),
      voxel_filtered.data(), voxel_filtered.size(), device_id);
  if (voxel_filtered_count != nullptr) {
    *voxel_filtered_count = after_ego - after_voxel;
  }

  const size_t write_count = std::min(after_voxel, io_buffer->capacity);
  for (size_t i = 0; i < write_count; ++i) {
    host_points[i] = ToProtoPoint(voxel_filtered[i]);
  }
  io_buffer->valid_count = write_count;
  return write_count;
#endif
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
