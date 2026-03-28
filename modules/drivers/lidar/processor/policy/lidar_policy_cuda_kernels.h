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

struct CudaMatrix4f {
  float data[16];
};

struct CudaEgoFilterParams {
  bool enable = true;
  float forward_x = 0.0f;
  float backward_x = 0.0f;
  float forward_y = 0.0f;
  float backward_y = 0.0f;
};

bool CudaComputeTimestampRange(const uint64_t* host_timestamps, size_t count,
                               int device_id, uint64_t* min_timestamp,
                               uint64_t* max_timestamp);

size_t CudaFuseFrameToBaseLink(
    const CudaPointXYZIT* host_input_points, size_t input_count,
    const double* host_sample_times, size_t sample_count,
    const CudaMatrix4f* host_world_from_sensor,
    const CudaMatrix4f* host_world2base, double measurement_time,
    CudaPointXYZIT* host_output_points, size_t output_capacity, int device_id);

size_t CudaApplyEgoFilter(const CudaPointXYZIT* host_input_points,
                          size_t input_count, const CudaEgoFilterParams& params,
                          CudaPointXYZIT* host_output_points,
                          size_t output_capacity, int device_id);

size_t CudaApplyVoxelDownsample(const CudaPointXYZIT* host_input_points,
                                size_t input_count, float voxel_size,
                                CudaPointXYZIT* host_output_points,
                                size_t output_capacity, int device_id);

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
