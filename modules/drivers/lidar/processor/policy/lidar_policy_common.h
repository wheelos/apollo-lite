#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "Eigen/Geometry"

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

#include "cyber/time/time.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_interface.h"

namespace apollo {
namespace drivers {
namespace lidar {

constexpr double kSecondToNano = 1e9;
constexpr float kPointInfThreshold = 1e3f;

bool ResolvePointTimestampBounds(const PointCloud& cloud, double* min_sec,
                                 double* max_sec);

bool BuildMotionSampleTimes(const PointCloud& cloud, size_t bins,
                            bool use_gpu_timestamp_range, int gpu_device_id,
                            std::vector<double>* sample_times);

double ResolvePointTimestampSec(const PointXYZIT& point,
                                double fallback_measurement_time);

uint64_t ResolvePointTimestampNs(const PointXYZIT& point,
                                 double fallback_measurement_time);

bool InterpolateAffinePose(double point_time,
                           const std::vector<double>& sample_times,
                           const std::vector<Eigen::Affine3d>& poses,
                           Eigen::Affine3d* interpolated_pose);

bool QueryTransformAffine(apollo::transform::BufferInterface* tf_buffer,
                          const std::string& target_frame,
                          const std::string& source_frame,
                          const cyber::Time& query_time,
                          Eigen::Affine3d* transform);

size_t ApplyDeterministicVoxelCentroidFilter(PointXYZIT* points, size_t count,
                                             float voxel_size);

bool TransformPointToBase(const PointXYZIT& point, double measurement_time,
                          const std::vector<double>& sample_times,
                          const std::vector<Eigen::Affine3d>& world_from_sensor,
                          const Eigen::Affine3d& world2base_ref,
                          PointXYZIT* output_point);

PointXYZIT* GetHostPoints(PointCloudBuffer* buffer);

bool EnsureGpuBackendAvailable(const char* policy_name);

#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
struct CudaPointXYZIT;
struct CudaPose;

CudaPointXYZIT ToCudaPoint(const PointXYZIT& point,
                           double measurement_time_sec);
PointXYZIT ToProtoPoint(const CudaPointXYZIT& point);
CudaPose ToCudaPose(const Eigen::Affine3d& in);
#endif

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
