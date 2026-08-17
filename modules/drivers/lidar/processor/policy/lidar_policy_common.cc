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

#include "modules/drivers/lidar/processor/policy/lidar_policy_common.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <sstream>

#include "cyber/cyber.h"
#include "modules/transform/transform_query.h"
#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
#include "modules/drivers/lidar/processor/policy/lidar_policy_cuda_kernels.h"
#endif

namespace apollo {
namespace drivers {
namespace lidar {

namespace {

constexpr double kDeskewTimestampFallbackThresholdSec = 1.0;

void FillUniformMotionSampleTimes(double timestamp_sec, size_t bins,
                                  std::vector<double>* sample_times) {
  sample_times->assign(bins, timestamp_sec);
}

std::string FormatTimestampSummary(double timestamp_sec) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(9) << timestamp_sec << "s";
  if (timestamp_sec > 0.0) {
    stream << " [" << cyber::Time(timestamp_sec).ToString() << "]";
  }
  return stream.str();
}

bool ShouldFallbackToMeasurementTime(double measurement_time_sec,
                                     double point_min_time_sec,
                                     double point_max_time_sec) {
  if (!std::isfinite(measurement_time_sec) || measurement_time_sec <= 0.0 ||
      !std::isfinite(point_min_time_sec) || !std::isfinite(point_max_time_sec)) {
    return false;
  }

  return std::fabs(point_min_time_sec - measurement_time_sec) >
                 kDeskewTimestampFallbackThresholdSec ||
         std::fabs(point_max_time_sec - measurement_time_sec) >
                 kDeskewTimestampFallbackThresholdSec;
}

}  // namespace

bool ResolvePointTimestampBounds(const PointCloud& cloud, double* min_sec,
                                 double* max_sec) {
  if (min_sec == nullptr || max_sec == nullptr) {
    return false;
  }

  double local_min = std::numeric_limits<double>::max();
  double local_max = std::numeric_limits<double>::lowest();
  for (const auto& point : cloud.point()) {
    const double ts = ResolvePointTimestampSec(point, cloud.measurement_time());
    local_min = std::min(local_min, ts);
    local_max = std::max(local_max, ts);
  }
  if (local_min > local_max) {
    local_min = cloud.measurement_time();
    local_max = cloud.measurement_time();
  }

  *min_sec = local_min;
  *max_sec = local_max;
  return true;
}

bool BuildMotionSampleTimes(const PointCloud& cloud, size_t bins,
                            bool use_gpu_timestamp_range, int gpu_device_id,
                            std::vector<double>* sample_times,
                            bool* used_measurement_time_fallback) {
  if (bins == 0U || sample_times == nullptr) {
    return false;
  }

  if (used_measurement_time_fallback != nullptr) {
    *used_measurement_time_fallback = false;
  }

  double local_min = 0.0;
  double local_max = 0.0;
#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
  if (use_gpu_timestamp_range) {
    std::vector<uint64_t> timestamps;
    timestamps.reserve(static_cast<size_t>(cloud.point_size()));
    const uint64_t fallback_ts =
        static_cast<uint64_t>(cloud.measurement_time() * kSecondToNano);
    for (const auto& point : cloud.point()) {
      timestamps.push_back(point.timestamp() == 0U ? fallback_ts
                                                   : point.timestamp());
    }
    if (timestamps.empty()) {
      timestamps.push_back(fallback_ts);
    }

    uint64_t min_ts = 0U;
    uint64_t max_ts = 0U;
    if (!CudaComputeTimestampRange(timestamps.data(), timestamps.size(),
                                   gpu_device_id, &min_ts, &max_ts)) {
      return false;
    }
    local_min = static_cast<double>(min_ts) / kSecondToNano;
    local_max = static_cast<double>(max_ts) / kSecondToNano;
  } else {
#endif
    if (!ResolvePointTimestampBounds(cloud, &local_min, &local_max)) {
      return false;
    }
#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
  }
#endif

  if (ShouldFallbackToMeasurementTime(cloud.measurement_time(), local_min,
                                      local_max)) {
    if (used_measurement_time_fallback != nullptr) {
      *used_measurement_time_fallback = true;
    }
    AINFO_EVERY(10)
        << "Invalid per-point timestamps detected, fallback to measurement_time"
        << " and disable intra-frame deskew. measurement="
        << FormatTimestampSummary(cloud.measurement_time())
        << ", point_range=[" << FormatTimestampSummary(local_min) << ", "
        << FormatTimestampSummary(local_max) << "]"
        << ", min_delta="
        << std::showpos << std::fixed << std::setprecision(6)
        << (local_min - cloud.measurement_time()) << "s"
        << ", max_delta=" << (local_max - cloud.measurement_time()) << "s"
        << std::noshowpos;
    FillUniformMotionSampleTimes(cloud.measurement_time(), bins, sample_times);
    return true;
  }

  sample_times->assign(bins, local_min);
  for (size_t i = 0; i < bins; ++i) {
    const double ratio =
        bins == 1 ? 0.0
                  : static_cast<double>(i) / static_cast<double>(bins - 1);
    (*sample_times)[i] = local_min + ratio * (local_max - local_min);
  }
  return true;
}

double ResolvePointTimestampSec(const PointXYZIT& point,
                                double fallback_measurement_time) {
  if (point.timestamp() == 0U) {
    return fallback_measurement_time;
  }
  return static_cast<double>(point.timestamp()) / kSecondToNano;
}

uint64_t ResolvePointTimestampNs(const PointXYZIT& point,
                                 double fallback_measurement_time) {
  if (point.timestamp() != 0U) {
    return point.timestamp();
  }
  return static_cast<uint64_t>(fallback_measurement_time * kSecondToNano);
}

bool InterpolateAffinePose(double point_time,
                           const std::vector<double>& sample_times,
                           const std::vector<Eigen::Affine3d>& poses,
                           Eigen::Affine3d* interpolated_pose) {
  if (interpolated_pose == nullptr) {
    return false;
  }

  const size_t pair_count = std::min(sample_times.size(), poses.size());
  if (pair_count == 0U) {
    return false;
  }
  if (pair_count == 1U) {
    *interpolated_pose = poses.front();
    return true;
  }

  const auto begin = sample_times.begin();
  const auto end = begin + static_cast<std::ptrdiff_t>(pair_count);
  const auto lower = std::lower_bound(begin, end, point_time);
  if (lower == begin) {
    *interpolated_pose = poses.front();
    return true;
  }
  if (lower == end) {
    *interpolated_pose = poses[pair_count - 1U];
    return true;
  }

  const size_t right_idx = static_cast<size_t>(std::distance(begin, lower));
  const size_t left_idx = right_idx - 1U;
  const double left_time = sample_times[left_idx];
  const double right_time = sample_times[right_idx];
  if (std::fabs(right_time - left_time) <= 1e-9) {
    *interpolated_pose = poses[right_idx];
    return true;
  }

  const double ratio =
      std::clamp((point_time - left_time) / (right_time - left_time), 0.0, 1.0);
  const Eigen::Vector3d translation =
      poses[left_idx].translation() +
      ratio * (poses[right_idx].translation() - poses[left_idx].translation());
  Eigen::Quaterniond left_rotation(poses[left_idx].linear());
  Eigen::Quaterniond right_rotation(poses[right_idx].linear());
  if (left_rotation.dot(right_rotation) < 0.0) {
    right_rotation.coeffs() *= -1.0;
  }
  const Eigen::Quaterniond rotation =
      left_rotation.slerp(ratio, right_rotation);
  *interpolated_pose = Eigen::Translation3d(translation) * rotation;
  return true;
}

bool BuildUniformPoseInterpolation(
    const std::vector<double>& sample_times,
    const std::vector<Eigen::Affine3d>& poses,
    UniformPoseInterpolation* interpolation) {
  if (interpolation == nullptr || sample_times.size() < 2U ||
      sample_times.size() != poses.size()) {
    return false;
  }

  const double first_time = sample_times.front();
  const double last_time = sample_times.back();
  const double duration = last_time - first_time;
  if (!std::isfinite(first_time) || !std::isfinite(last_time) ||
      duration <= 1e-9) {
    return false;
  }

  const double bin_duration =
      duration / static_cast<double>(sample_times.size() - 1U);
  for (size_t index = 1U; index + 1U < sample_times.size(); ++index) {
    const double expected =
        first_time + static_cast<double>(index) * bin_duration;
    if (std::fabs(sample_times[index] - expected) > 1e-6) {
      return false;
    }
  }

  interpolation->first_time_sec = first_time;
  interpolation->last_time_sec = last_time;
  interpolation->inverse_bin_duration_sec = 1.0 / bin_duration;
  if (!std::isfinite(interpolation->inverse_bin_duration_sec)) {
    return false;
  }
  interpolation->translations.clear();
  interpolation->rotations.clear();
  interpolation->translations.reserve(poses.size());
  interpolation->rotations.reserve(poses.size());
  for (const auto& pose : poses) {
    interpolation->translations.push_back(pose.translation());
    interpolation->rotations.emplace_back(pose.linear());
  }
  return true;
}

namespace {

bool InterpolateUniformPose(
    double point_time, const std::vector<double>& sample_times,
    const UniformPoseInterpolation& interpolation,
    Eigen::Vector3d* translation, Eigen::Quaterniond* rotation) {
  if (translation == nullptr || rotation == nullptr ||
      sample_times.size() < 2U ||
      sample_times.size() != interpolation.translations.size() ||
      sample_times.size() != interpolation.rotations.size()) {
    return false;
  }
  if (point_time <= interpolation.first_time_sec) {
    *translation = interpolation.translations.front();
    *rotation = interpolation.rotations.front();
    return true;
  }
  if (point_time > interpolation.last_time_sec) {
    *translation = interpolation.translations.back();
    *rotation = interpolation.rotations.back();
    return true;
  }

  const double scaled =
      (point_time - interpolation.first_time_sec) *
      interpolation.inverse_bin_duration_sec;
  const size_t right_index = std::clamp(
      static_cast<size_t>(std::ceil(scaled)), size_t{1U},
      interpolation.translations.size() - 1U);
  const size_t left_index = right_index - 1U;
  const double ratio =
      std::clamp((point_time - sample_times[left_index]) /
                     (sample_times[right_index] - sample_times[left_index]),
                 0.0, 1.0);
  *translation = interpolation.translations[left_index] +
                 ratio * (interpolation.translations[right_index] -
                          interpolation.translations[left_index]);
  Eigen::Quaterniond right_rotation = interpolation.rotations[right_index];
  if (interpolation.rotations[left_index].dot(right_rotation) < 0.0) {
    right_rotation.coeffs() *= -1.0;
  }
  *rotation =
      interpolation.rotations[left_index].slerp(ratio, right_rotation);
  return true;
}

bool TransformPointWithPose(
    const PointXYZIT& point, uint64_t fallback_timestamp_ns,
    int64_t timestamp_offset_ns, const std::vector<double>& sample_times,
    const std::vector<Eigen::Affine3d>& base_from_sensor_poses,
    const UniformPoseInterpolation* uniform_interpolation,
    PointXYZIT* output_point) {
  if (output_point == nullptr || !std::isfinite(point.x()) ||
      !std::isfinite(point.y()) || !std::isfinite(point.z()) ||
      std::fabs(point.x()) > kPointInfThreshold ||
      std::fabs(point.y()) > kPointInfThreshold ||
      std::fabs(point.z()) > kPointInfThreshold) {
    return false;
  }

  const uint64_t raw_timestamp =
      point.timestamp() == 0U ? fallback_timestamp_ns : point.timestamp();
  const double point_time = static_cast<double>(
      static_cast<int64_t>(raw_timestamp) + timestamp_offset_ns) /
      kSecondToNano;
  Eigen::Vector3d target;
  if (uniform_interpolation != nullptr) {
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
    if (!InterpolateUniformPose(point_time, sample_times,
                                *uniform_interpolation, &translation,
                                &rotation)) {
      return false;
    }
    target = rotation * Eigen::Vector3d(point.x(), point.y(), point.z()) +
             translation;
  } else {
    Eigen::Affine3d base_from_sensor = Eigen::Affine3d::Identity();
    if (!InterpolateAffinePose(point_time, sample_times,
                               base_from_sensor_poses,
                               &base_from_sensor)) {
      return false;
    }
    target =
        base_from_sensor * Eigen::Vector3d(point.x(), point.y(), point.z());
  }
  output_point->set_x(static_cast<float>(target.x()));
  output_point->set_y(static_cast<float>(target.y()));
  output_point->set_z(static_cast<float>(target.z()));
  output_point->set_intensity(point.intensity());
  output_point->set_timestamp(static_cast<uint64_t>(
      static_cast<int64_t>(raw_timestamp) + timestamp_offset_ns));
  return true;
}

}  // namespace

bool QueryTransformAffine(apollo::transform::BufferInterface* tf_buffer,
                          const std::string& target_frame,
                          const std::string& source_frame,
                          const cyber::Time& query_time, float timeout_sec,
                          Eigen::Affine3d* transform) {
  if (tf_buffer == nullptr || transform == nullptr) {
    return false;
  }

  apollo::transform::TransformQuery query(tf_buffer);
  std::string err;
  if (!query.LookupTransformToAffine(target_frame, source_frame, query_time,
                                     transform, timeout_sec, &err)) {
    AWARN << "Transform unavailable from " << source_frame << " to "
          << target_frame << ": " << err;
    return false;
  }

  return true;
}

namespace {

struct VoxelKey {
  int x = 0;
  int y = 0;
  int z = 0;
};

struct IndexedVoxelPoint {
  VoxelKey key;
  size_t index = 0;
};

}  // namespace

size_t ApplyDeterministicVoxelCentroidFilter(PointXYZIT* points, size_t count,
                                             float voxel_size) {
  if (points == nullptr || count == 0U || voxel_size <= 1e-4f) {
    return count;
  }

  std::vector<IndexedVoxelPoint> ordered_points;
  ordered_points.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    ordered_points.push_back(IndexedVoxelPoint{
        VoxelKey{static_cast<int>(std::floor(points[i].x() / voxel_size)),
                 static_cast<int>(std::floor(points[i].y() / voxel_size)),
                 static_cast<int>(std::floor(points[i].z() / voxel_size))},
        i});
  }

  std::stable_sort(
      ordered_points.begin(), ordered_points.end(),
      [](const IndexedVoxelPoint& lhs, const IndexedVoxelPoint& rhs) {
        if (lhs.key.x != rhs.key.x) {
          return lhs.key.x < rhs.key.x;
        }
        if (lhs.key.y != rhs.key.y) {
          return lhs.key.y < rhs.key.y;
        }
        if (lhs.key.z != rhs.key.z) {
          return lhs.key.z < rhs.key.z;
        }
        return lhs.index < rhs.index;
      });

  size_t write_idx = 0;
  for (size_t begin = 0; begin < ordered_points.size();) {
    const VoxelKey key = ordered_points[begin].key;
    size_t end = begin;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double sum_intensity = 0.0;
    long double sum_timestamp = 0.0;
    while (end < ordered_points.size() && ordered_points[end].key.x == key.x &&
           ordered_points[end].key.y == key.y &&
           ordered_points[end].key.z == key.z) {
      const PointXYZIT& point = points[ordered_points[end].index];
      sum_x += point.x();
      sum_y += point.y();
      sum_z += point.z();
      sum_intensity += point.intensity();
      sum_timestamp += static_cast<long double>(point.timestamp());
      ++end;
    }

    const double count_inv = 1.0 / static_cast<double>(end - begin);
    PointXYZIT centroid_point;
    centroid_point.set_x(static_cast<float>(sum_x * count_inv));
    centroid_point.set_y(static_cast<float>(sum_y * count_inv));
    centroid_point.set_z(static_cast<float>(sum_z * count_inv));
    centroid_point.set_intensity(static_cast<float>(sum_intensity * count_inv));
    centroid_point.set_timestamp(
        static_cast<uint64_t>(std::llround(sum_timestamp * count_inv)));
    points[write_idx++] = centroid_point;
    begin = end;
  }

  return write_idx;
}

bool TransformPointToBase(const PointXYZIT& point, double measurement_time,
                          double timestamp_offset_sec,
                          const std::vector<double>& sample_times,
                          const std::vector<Eigen::Affine3d>& map_from_sensor,
                          const Eigen::Affine3d& map2base_ref,
                          PointXYZIT* output_point) {
  if (output_point == nullptr || map_from_sensor.empty()) {
    return false;
  }

  if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
      !std::isfinite(point.z())) {
    return false;
  }
  if (std::fabs(point.x()) > kPointInfThreshold ||
      std::fabs(point.y()) > kPointInfThreshold ||
      std::fabs(point.z()) > kPointInfThreshold) {
    return false;
  }

  const double point_time =
      ResolvePointTimestampSec(point, measurement_time) + timestamp_offset_sec;
  Eigen::Affine3d interpolated_map_from_sensor = Eigen::Affine3d::Identity();
  if (!InterpolateAffinePose(point_time, sample_times, map_from_sensor,
                             &interpolated_map_from_sensor)) {
    return false;
  }

  const Eigen::Vector3d raw(point.x(), point.y(), point.z());
  // Output points stay in the reference base_link frame. The fixed/map frame
  // is only an anchor for composing sensor(t) -> base(ref) consistently.
  const Eigen::Vector3d target =
      (map2base_ref * interpolated_map_from_sensor) * raw;

  output_point->set_x(static_cast<float>(target.x()));
  output_point->set_y(static_cast<float>(target.y()));
  output_point->set_z(static_cast<float>(target.z()));
  output_point->set_intensity(point.intensity());
  const int64_t offset_ns =
      static_cast<int64_t>(std::llround(timestamp_offset_sec * kSecondToNano));
  const uint64_t raw_timestamp =
      ResolvePointTimestampNs(point, measurement_time);
  output_point->set_timestamp(static_cast<uint64_t>(
      static_cast<int64_t>(raw_timestamp) + offset_ns));
  return true;
}

bool TransformPointWithInterpolatedPoses(
    const PointXYZIT& point, uint64_t fallback_timestamp_ns,
    int64_t timestamp_offset_ns, const std::vector<double>& sample_times,
    const std::vector<Eigen::Affine3d>& base_from_sensor_poses,
    PointXYZIT* output_point) {
  return TransformPointWithPose(point, fallback_timestamp_ns,
                                timestamp_offset_ns, sample_times,
                                base_from_sensor_poses, nullptr, output_point);
}

bool TransformPointWithUniformInterpolatedPoses(
    const PointXYZIT& point, uint64_t fallback_timestamp_ns,
    int64_t timestamp_offset_ns, const std::vector<double>& sample_times,
    const std::vector<Eigen::Affine3d>& base_from_sensor_poses,
    const UniformPoseInterpolation& interpolation, PointXYZIT* output_point) {
  return TransformPointWithPose(point, fallback_timestamp_ns,
                                timestamp_offset_ns, sample_times,
                                base_from_sensor_poses, &interpolation,
                                output_point);
}

PointXYZIT* GetHostPoints(PointCloudBuffer* buffer) {
  if (buffer == nullptr || buffer->data_ptr == nullptr ||
      buffer->item_size != sizeof(PointXYZIT) ||
      buffer->device_type != MemoryDeviceType::kHost) {
    return nullptr;
  }
  return reinterpret_cast<PointXYZIT*>(buffer->data_ptr);
}

bool EnsureGpuBackendAvailable(const char* policy_name) {
#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
  (void)policy_name;
  return true;
#else
  AERROR
      << policy_name
      << " requested GPU execution, but this target is built without "
         "CUDA backend support. Rebuild with --define=lidar_gpu_backend=true";
  return false;
#endif
}

#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
CudaPointXYZIT ToCudaPoint(const PointXYZIT& point,
                           double measurement_time_sec,
                           int64_t timestamp_offset_ns) {
  CudaPointXYZIT out;
  out.x = point.x();
  out.y = point.y();
  out.z = point.z();
  out.intensity = point.intensity();
  out.timestamp = static_cast<uint64_t>(
      static_cast<int64_t>(
          ResolvePointTimestampNs(point, measurement_time_sec)) +
      timestamp_offset_ns);
  return out;
}

PointXYZIT ToProtoPoint(const CudaPointXYZIT& point) {
  PointXYZIT out;
  out.set_x(point.x);
  out.set_y(point.y);
  out.set_z(point.z);
  out.set_intensity(point.intensity);
  out.set_timestamp(point.timestamp);
  return out;
}

CudaPose ToCudaPose(const Eigen::Affine3d& in) {
  CudaPose out;
  out.tx = static_cast<float>(in.translation().x());
  out.ty = static_cast<float>(in.translation().y());
  out.tz = static_cast<float>(in.translation().z());
  const Eigen::Quaterniond rotation(in.linear());
  out.qx = static_cast<float>(rotation.x());
  out.qy = static_cast<float>(rotation.y());
  out.qz = static_cast<float>(rotation.z());
  out.qw = static_cast<float>(rotation.w());
  return out;
}
#endif

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
