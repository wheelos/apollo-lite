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

double ResolvePointTimestampSec(const PointXYZIT& point,
                                double fallback_measurement_time);

uint64_t ResolvePointTimestampNs(const PointXYZIT& point,
                                 double fallback_measurement_time);

size_t ResolveNearestPoseIndex(double point_time,
                               const std::vector<double>& sample_times,
                               size_t pose_count);

bool QueryTransformAffine(apollo::transform::BufferInterface* tf_buffer,
                          const std::string& target_frame,
                          const std::string& source_frame,
                          const cyber::Time& query_time,
                          Eigen::Affine3d* transform);

bool TransformPointToBase(const PointXYZIT& point, double measurement_time,
                          const std::vector<double>& sample_times,
                          const std::vector<Eigen::Affine3d>& world_from_sensor,
                          const Eigen::Affine3d& world2base_ref,
                          PointXYZIT* output_point);

PointXYZIT* GetHostPoints(PointCloudBuffer* buffer);

bool EnsureGpuBackendAvailable(const char* policy_name);

#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
struct CudaPointXYZIT;
struct CudaMatrix4f;

CudaPointXYZIT ToCudaPoint(const PointXYZIT& point,
                           double measurement_time_sec);
PointXYZIT ToProtoPoint(const CudaPointXYZIT& point);
CudaMatrix4f ToCudaMatrix(const Eigen::Affine3d& in);
#endif

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
