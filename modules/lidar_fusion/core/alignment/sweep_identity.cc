// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/lidar_fusion/core/alignment/sweep_identity.h"

#include <cmath>

namespace apollo {
namespace lidar_fusion {

bool SameSweep(const drivers::PointCloud& cloud, const common::Header& header) {
  if (!cloud.has_header()) {
    return false;
  }
  const auto& source = cloud.header();
  if (source.has_sequence_num() && header.has_sequence_num() &&
      source.sequence_num() != header.sequence_num()) {
    return false;
  }
  if (source.lidar_timestamp() != 0 && header.lidar_timestamp() != 0) {
    return source.lidar_timestamp() == header.lidar_timestamp();
  }
  // PointPillars uses measurement_time as timestamp_sec; the other producers
  // copy the source header. Neither case permits a nearest-neighbor sweep join.
  return source.has_sequence_num() && header.has_sequence_num() &&
         header.has_timestamp_sec() && std::isfinite(header.timestamp_sec()) &&
         ((source.has_timestamp_sec() &&
           std::abs(source.timestamp_sec() - header.timestamp_sec()) <= 1e-6) ||
          (cloud.has_measurement_time() &&
           std::abs(cloud.measurement_time() - header.timestamp_sec()) <=
               1e-6));
}

}  // namespace lidar_fusion
}  // namespace apollo
