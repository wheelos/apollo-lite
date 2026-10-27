// Copyright 2026 The WheelOS Authors. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include "modules/drivers/lidar/vanjeelidar/src/measurement_conversion.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace apollo {
namespace drivers {
namespace lidar {
namespace {

bool Reject(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

bool SecondsToNanoseconds(long double seconds, uint64_t* timestamp) {
  const long double nanoseconds = std::round(seconds * 1.0e9L);
  if (!std::isfinite(nanoseconds) || nanoseconds <= 0.0L ||
      nanoseconds >
          static_cast<long double>(std::numeric_limits<uint64_t>::max())) {
    return false;
  }
  *timestamp = static_cast<uint64_t>(nanoseconds);
  return true;
}

}  // namespace

bool ConvertVanjeePointCloud(const VanjeePointCloud& input,
                             bool use_offset_timestamp,
                             const std::string& frame_id, PointCloud* output,
                             std::string* error) {
  uint64_t frame_timestamp = 0;
  if (output == nullptr || frame_id.empty() || input.points.empty() ||
      input.points.size() >
          static_cast<size_t>(std::numeric_limits<int>::max()) ||
      !SecondsToNanoseconds(input.timestamp, &frame_timestamp)) {
    return Reject("Invalid Vanjee cloud, frame or frame timestamp", error);
  }

  PointCloud converted;
  converted.mutable_point()->Reserve(static_cast<int>(input.points.size()));
  uint64_t end_timestamp = 0;
  for (const auto& source : input.points) {
    // Frame-end-relative offsets can be negative. Add before converting to
    // unsigned nanoseconds; absolute SDK timestamps already include the anchor.
    const long double seconds =
        static_cast<long double>(source.timestamp) +
        (use_offset_timestamp ? static_cast<long double>(input.timestamp)
                              : 0.0L);
    uint64_t timestamp = 0;
    if (!SecondsToNanoseconds(seconds, &timestamp)) {
      return Reject("Invalid Vanjee point timestamp", error);
    }
    if (!std::isfinite(source.intensity) || source.intensity < 0.0F ||
        static_cast<double>(source.intensity) >
            std::numeric_limits<uint32_t>::max()) {
      return Reject("Invalid Vanjee point intensity", error);
    }
    auto* point = converted.add_point();
    point->set_x(source.x);
    point->set_y(source.y);
    point->set_z(source.z);
    point->set_intensity(static_cast<uint32_t>(source.intensity));
    point->set_timestamp(timestamp);
    end_timestamp = std::max(end_timestamp, timestamp);
  }
  converted.set_frame_id(frame_id);
  converted.mutable_header()->set_frame_id(frame_id);
  converted.mutable_header()->set_lidar_timestamp(end_timestamp);
  converted.set_measurement_time(static_cast<double>(end_timestamp) * 1.0e-9);
  converted.set_height(1);
  converted.set_width(static_cast<uint32_t>(converted.point_size()));
  converted.set_is_dense(input.is_dense);
  output->Swap(&converted);
  return true;
}

bool ConvertVanjeeImu(const ::vanjee::lidar::ImuPacket& input,
                      const std::string& frame_id, bool use_lidar_clock,
                      double publish_time, vanjee::VanjeeImu* output,
                      std::string* error) {
  uint64_t timestamp = 0;
  if (output == nullptr || frame_id.empty() ||
      !SecondsToNanoseconds(input.timestamp, &timestamp) ||
      !std::isfinite(publish_time) || publish_time <= 0.0) {
    return Reject("Invalid Vanjee IMU frame or timestamp", error);
  }
  for (size_t axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(input.linear_acce[axis]) ||
        !std::isfinite(input.angular_voc[axis])) {
      return Reject("Nonfinite Vanjee IMU measurement", error);
    }
  }
  vanjee::VanjeeImu converted;
  auto* header = converted.mutable_header();
  header->set_timestamp_sec(publish_time);
  header->set_module_name("vanjeelidar");
  header->set_frame_id(frame_id);
  header->set_sequence_num(input.seq);
  converted.set_measurement_time(input.timestamp);
  converted.set_use_lidar_clock(use_lidar_clock);
  auto* acceleration = converted.mutable_linear_acceleration();
  acceleration->set_x(input.linear_acce[0]);
  acceleration->set_y(input.linear_acce[1]);
  acceleration->set_z(input.linear_acce[2]);
  auto* angular_velocity = converted.mutable_angular_velocity();
  angular_velocity->set_x(input.angular_voc[0]);
  angular_velocity->set_y(input.angular_voc[1]);
  angular_velocity->set_z(input.angular_voc[2]);
  output->Swap(&converted);
  return true;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
