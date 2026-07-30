#pragma once

#include <cstddef>
#include <cstdint>

namespace apollo {
namespace drivers {
namespace lidar {

struct CudaPointXYZIT {
  float x;
  float y;
  float z;
  float intensity;
  uint64_t timestamp;
};

struct CudaPose {
  float tx;
  float ty;
  float tz;
  float qx;
  float qy;
  float qz;
  float qw;
};

struct CudaEgoFilterParams {
  bool enable = true;
  float forward_x = 0.0f;
  float backward_x = 0.0f;
  float forward_y = 0.0f;
  float backward_y = 0.0f;
};

struct CudaWorkspaceStats {
  uint64_t points_in_expand_count = 0;
  uint64_t points_out_expand_count = 0;
  uint64_t sample_times_expand_count = 0;
  uint64_t poses_expand_count = 0;
  uint64_t hash_table_expand_count = 0;
  size_t points_in_peak_capacity = 0;
  size_t points_out_peak_capacity = 0;
  size_t sample_times_peak_capacity = 0;
  size_t poses_peak_capacity = 0;
  size_t hash_table_peak_capacity = 0;
};

bool CudaComputeTimestampRange(const uint64_t* host_timestamps, size_t count,
                               int device_id, uint64_t* min_timestamp,
                               uint64_t* max_timestamp);

size_t CudaFuseFrameToBaseLink(
    const CudaPointXYZIT* host_input_points, size_t input_count,
    const double* host_sample_times, size_t sample_count,
  const CudaPose* host_base_from_sensor_poses, double measurement_time,
    CudaPointXYZIT* host_output_points, size_t output_capacity, int device_id);

size_t CudaApplyEgoFilter(const CudaPointXYZIT* host_input_points,
                          size_t input_count, const CudaEgoFilterParams& params,
                          CudaPointXYZIT* host_output_points,
                          size_t output_capacity, int device_id);

size_t CudaApplyVoxelDownsample(const CudaPointXYZIT* host_input_points,
                                size_t input_count, float voxel_size,
                                CudaPointXYZIT* host_output_points,
                                size_t output_capacity, int device_id);

bool CudaGetWorkspaceStats(int device_id, CudaWorkspaceStats* stats);

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
