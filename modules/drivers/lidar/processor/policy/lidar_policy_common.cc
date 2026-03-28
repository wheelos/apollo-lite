#include "modules/drivers/lidar/processor/policy/lidar_policy_common.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include "cyber/cyber.h"
#ifdef APOLLO_LIDAR_POLICY_GPU_ENABLED
#include "modules/drivers/lidar/processor/policy/lidar_policy_cuda_kernels.h"
#endif

namespace apollo {
namespace drivers {
namespace lidar {

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

size_t ResolveNearestPoseIndex(double point_time,
                               const std::vector<double>& sample_times,
                               size_t pose_count) {
  const size_t pair_count = std::min(sample_times.size(), pose_count);
  if (pair_count <= 1) {
    return 0;
  }

  const auto begin = sample_times.begin();
  const auto end = begin + static_cast<std::ptrdiff_t>(pair_count);
  const auto lower = std::lower_bound(begin, end, point_time);
  if (lower == begin) {
    return 0;
  }
  if (lower == end) {
    return pair_count - 1;
  }

  const size_t right_idx = static_cast<size_t>(std::distance(begin, lower));
  const size_t left_idx = right_idx - 1;
  const double left_dt = std::fabs(point_time - sample_times[left_idx]);
  const double right_dt = std::fabs(sample_times[right_idx] - point_time);
  return (left_dt <= right_dt) ? left_idx : right_idx;
}

bool QueryTransformAffine(apollo::transform::BufferInterface* tf_buffer,
                          const std::string& target_frame,
                          const std::string& source_frame,
                          const cyber::Time& query_time,
                          Eigen::Affine3d* transform) {
  if (tf_buffer == nullptr || transform == nullptr) {
    return false;
  }

  std::string err;
  if (!tf_buffer->canTransform(target_frame, source_frame, query_time, 0.02f,
                               &err)) {
    AWARN << "Transform unavailable from " << source_frame << " to "
          << target_frame << ": " << err;
    return false;
  }

  try {
    const auto stamped =
        tf_buffer->lookupTransform(target_frame, source_frame, query_time);
    *transform = Eigen::Translation3d(stamped.transform().translation().x(),
                                      stamped.transform().translation().y(),
                                      stamped.transform().translation().z()) *
                 Eigen::Quaterniond(stamped.transform().rotation().qw(),
                                    stamped.transform().rotation().qx(),
                                    stamped.transform().rotation().qy(),
                                    stamped.transform().rotation().qz());
    return true;
  } catch (const std::exception& e) {
    AWARN << "lookupTransform failed: " << e.what();
    return false;
  }
}

bool TransformPointToBase(const PointXYZIT& point, double measurement_time,
                          const std::vector<double>& sample_times,
                          const std::vector<Eigen::Affine3d>& world_from_sensor,
                          const Eigen::Affine3d& world2base_ref,
                          PointXYZIT* output_point) {
  if (output_point == nullptr || world_from_sensor.empty()) {
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

  const double point_time = ResolvePointTimestampSec(point, measurement_time);
  const size_t pose_idx = ResolveNearestPoseIndex(point_time, sample_times,
                                                  world_from_sensor.size());
  if (pose_idx >= world_from_sensor.size()) {
    return false;
  }

  const Eigen::Vector3d raw(point.x(), point.y(), point.z());
  const Eigen::Vector3d target =
      (world2base_ref * world_from_sensor[pose_idx]) * raw;

  output_point->set_x(static_cast<float>(target.x()));
  output_point->set_y(static_cast<float>(target.y()));
  output_point->set_z(static_cast<float>(target.z()));
  output_point->set_intensity(point.intensity());
  output_point->set_timestamp(ResolvePointTimestampNs(point, measurement_time));
  return true;
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
                           double measurement_time_sec) {
  CudaPointXYZIT out;
  out.x = point.x();
  out.y = point.y();
  out.z = point.z();
  out.intensity = point.intensity();
  out.timestamp = ResolvePointTimestampNs(point, measurement_time_sec);
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

CudaMatrix4f ToCudaMatrix(const Eigen::Affine3d& in) {
  CudaMatrix4f out;
  const Eigen::Matrix4d mat = in.matrix();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      out.data[row * 4 + col] = static_cast<float>(mat(row, col));
    }
  }
  return out;
}
#endif

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
