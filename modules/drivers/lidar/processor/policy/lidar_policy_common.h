// Copyright 2026 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "Eigen/StdVector"

#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

#include "cyber/time/time.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_interface.h"

namespace apollo {
namespace drivers {
namespace lidar {

constexpr double kSecondToNano = 1e9;
constexpr float kPointInfThreshold = 1e3f;

struct UniformPoseInterpolation {
  double first_time_sec = 0.0;
  double last_time_sec = 0.0;
  double inverse_bin_duration_sec = 0.0;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      translations;
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>
      rotations;
};

bool ResolvePointTimestampBounds(const PointCloud& cloud, double* min_sec,
                                 double* max_sec);

bool BuildMotionSampleTimes(const PointCloud& cloud, size_t bins,
                            bool use_gpu_timestamp_range, int gpu_device_id,
                            std::vector<double>* sample_times,
                            bool* used_measurement_time_fallback = nullptr);

double ResolvePointTimestampSec(const PointXYZIT& point,
                                double fallback_measurement_time);

uint64_t ResolvePointTimestampNs(const PointXYZIT& point,
                                 double fallback_measurement_time);

bool InterpolateAffinePose(double point_time,
                           const std::vector<double>& sample_times,
                           const std::vector<Eigen::Affine3d>& poses,
                           Eigen::Affine3d* interpolated_pose);

bool BuildUniformPoseInterpolation(
    const std::vector<double>& sample_times,
    const std::vector<Eigen::Affine3d>& poses,
    UniformPoseInterpolation* interpolation);

bool QueryTransformAffine(apollo::transform::BufferInterface* tf_buffer,
                          const std::string& target_frame,
                          const std::string& source_frame,
                          const cyber::Time& query_time, float timeout_sec,
                          Eigen::Affine3d* transform);

size_t ApplyDeterministicVoxelCentroidFilter(PointXYZIT* points, size_t count,
                                             float voxel_size);

bool TransformPointToBase(const PointXYZIT& point, double measurement_time,
                          double timestamp_offset_sec,
                          const std::vector<double>& sample_times,
                          const std::vector<Eigen::Affine3d>& map_from_sensor,
                          const Eigen::Affine3d& map2base_ref,
                          PointXYZIT* output_point);

bool TransformPointWithInterpolatedPoses(
    const PointXYZIT& point, uint64_t fallback_timestamp_ns,
    int64_t timestamp_offset_ns, const std::vector<double>& sample_times,
    const std::vector<Eigen::Affine3d>& base_from_sensor_poses,
    PointXYZIT* output_point);

bool TransformPointWithUniformInterpolatedPoses(
    const PointXYZIT& point, uint64_t fallback_timestamp_ns,
    int64_t timestamp_offset_ns, const std::vector<double>& sample_times,
    const std::vector<Eigen::Affine3d>& base_from_sensor_poses,
    const UniformPoseInterpolation& interpolation, PointXYZIT* output_point);

PointXYZIT* GetHostPoints(PointCloudBuffer* buffer);

bool EnsureGpuBackendAvailable(const char* policy_name);

#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
struct CudaPointXYZIT;
struct CudaPose;

CudaPointXYZIT ToCudaPoint(const PointXYZIT& point,
                           double measurement_time_sec,
                           int64_t timestamp_offset_ns = 0);
PointXYZIT ToProtoPoint(const CudaPointXYZIT& point);
CudaPose ToCudaPose(const Eigen::Affine3d& in);
#endif

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
