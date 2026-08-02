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

#include "modules/drivers/lidar/processor/control/time_contract.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace apollo {
namespace drivers {
namespace lidar {

namespace {

constexpr double kNsPerSecond = 1e9;
constexpr double kNsPerMillisecond = 1e6;

bool SecondsToNanoseconds(double seconds, int64_t* nanoseconds) {
  if (nanoseconds == nullptr || !std::isfinite(seconds) || seconds <= 0.0) {
    return false;
  }
  const long double value =
      static_cast<long double>(seconds) * kNsPerSecond;
  if (value > static_cast<long double>(std::numeric_limits<int64_t>::max())) {
    return false;
  }
  *nanoseconds = static_cast<int64_t>(std::llround(value));
  return *nanoseconds > 0;
}

bool AddOffset(int64_t value, int64_t offset, int64_t* result) {
  if (result == nullptr ||
      (offset > 0 && value > std::numeric_limits<int64_t>::max() - offset) ||
      (offset < 0 && value < std::numeric_limits<int64_t>::min() - offset)) {
    return false;
  }
  *result = value + offset;
  return true;
}

bool MillisecondsToNanoseconds(double milliseconds, int64_t* nanoseconds) {
  if (nanoseconds == nullptr || !std::isfinite(milliseconds) ||
      milliseconds <= 0.0) {
    return false;
  }
  const long double value =
      static_cast<long double>(milliseconds) * kNsPerMillisecond;
  if (value > static_cast<long double>(std::numeric_limits<int64_t>::max())) {
    return false;
  }
  *nanoseconds = static_cast<int64_t>(std::llround(value));
  return *nanoseconds > 0;
}

bool BuildMeasurementInterval(
    int64_t measurement_ns, int64_t expected_duration_ns,
    LidarUnifiedComponentConfig::TimestampAnchor anchor, int64_t* begin_ns,
    int64_t* end_ns) {
  switch (anchor) {
    case LidarUnifiedComponentConfig::SCAN_BEGIN:
      *begin_ns = measurement_ns;
      return AddOffset(measurement_ns, expected_duration_ns, end_ns);
    case LidarUnifiedComponentConfig::SCAN_MIDPOINT:
      return AddOffset(measurement_ns, -expected_duration_ns / 2, begin_ns) &&
             AddOffset(*begin_ns, expected_duration_ns, end_ns);
    case LidarUnifiedComponentConfig::SCAN_END:
    default:
      *end_ns = measurement_ns;
      return AddOffset(measurement_ns, -expected_duration_ns, begin_ns);
  }
}

int64_t PointAnchorNanoseconds(
    uint64_t point_min_ns, uint64_t point_max_ns,
    LidarUnifiedComponentConfig::TimestampAnchor anchor) {
  switch (anchor) {
    case LidarUnifiedComponentConfig::SCAN_BEGIN:
      return static_cast<int64_t>(point_min_ns);
    case LidarUnifiedComponentConfig::SCAN_MIDPOINT:
      return static_cast<int64_t>(
          point_min_ns + (point_max_ns - point_min_ns) / 2U);
    case LidarUnifiedComponentConfig::SCAN_END:
    default:
      return static_cast<int64_t>(point_max_ns);
  }
}

}  // namespace

double TimeContract::CanonicalAnchorSec() const {
  return static_cast<double>(canonical_anchor_ns) / kNsPerSecond;
}

bool NormalizePointCloudTime(
    const PointCloud& cloud,
    const LidarUnifiedComponentConfig::TimeSettings& settings,
    TimeContract* contract) {
  if (contract == nullptr) {
    return false;
  }

  int64_t max_duration_ns = 0;
  int64_t expected_duration_ns = 0;
  if (!MillisecondsToNanoseconds(settings.max_scan_duration_ms(),
                                 &max_duration_ns) ||
      !MillisecondsToNanoseconds(settings.expected_scan_duration_ms(),
                                 &expected_duration_ns) ||
      expected_duration_ns > max_duration_ns) {
    return false;
  }

  uint64_t point_min_ns = std::numeric_limits<uint64_t>::max();
  uint64_t point_max_ns = 0;
  bool all_points_have_timestamps = !cloud.point().empty();
  for (const auto& point : cloud.point()) {
    if (point.timestamp() == 0U ||
        point.timestamp() >
            static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
      all_points_have_timestamps = false;
      continue;
    }
    point_min_ns = std::min(point_min_ns, point.timestamp());
    point_max_ns = std::max(point_max_ns, point.timestamp());
  }

  int64_t begin_ns = 0;
  int64_t end_ns = 0;
  int64_t measurement_ns = 0;
  const bool has_measurement_time =
      SecondsToNanoseconds(cloud.measurement_time(), &measurement_ns);
  TimestampQuality quality = TimestampQuality::kMeasurementTimeFallback;
  bool point_timestamps_valid =
      point_max_ns != 0U &&
      point_max_ns - point_min_ns <= static_cast<uint64_t>(max_duration_ns);
  if (point_timestamps_valid && has_measurement_time) {
    const int64_t point_anchor_ns = PointAnchorNanoseconds(
        point_min_ns, point_max_ns, settings.measurement_time_anchor());
    point_timestamps_valid =
        std::fabs(static_cast<long double>(point_anchor_ns) -
                  static_cast<long double>(measurement_ns)) <=
        static_cast<long double>(max_duration_ns);
  }

  if (point_timestamps_valid) {
    begin_ns = static_cast<int64_t>(point_min_ns);
    end_ns = static_cast<int64_t>(point_max_ns);
    quality = TimestampQuality::kPointTimestamps;
  } else {
    if (!has_measurement_time ||
        !BuildMeasurementInterval(
            measurement_ns, expected_duration_ns,
            settings.measurement_time_anchor(), &begin_ns, &end_ns)) {
      return false;
    }
  }

  if (begin_ns <= 0 || end_ns < begin_ns ||
      end_ns - begin_ns > max_duration_ns) {
    return false;
  }

  TimeContract normalized;
  normalized.static_offset_ns = settings.static_time_offset_ns();
  normalized.all_points_have_timestamps = all_points_have_timestamps;
  normalized.quality = quality;
  if (!AddOffset(begin_ns, normalized.static_offset_ns,
                 &normalized.scan_begin_ns) ||
      !AddOffset(end_ns, normalized.static_offset_ns,
                 &normalized.scan_end_ns) ||
      normalized.scan_begin_ns <= 0) {
    return false;
  }
  normalized.canonical_anchor_ns = normalized.scan_end_ns;
  *contract = normalized;
  return true;
}

int64_t IntervalOverlapNs(const TimeContract& lhs, const TimeContract& rhs) {
  return std::max<int64_t>(
      0, std::min(lhs.scan_end_ns, rhs.scan_end_ns) -
             std::max(lhs.scan_begin_ns, rhs.scan_begin_ns));
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
