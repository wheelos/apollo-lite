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

#include <cstdint>

#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/proto/lidar_unified_component_config.pb.h"

namespace apollo {
namespace drivers {
namespace lidar {

enum class TimestampQuality {
  kPointTimestamps = 0,
  kMeasurementTimeFallback = 1,
};

struct TimeContract {
  int64_t scan_begin_ns = 0;
  int64_t scan_end_ns = 0;
  int64_t canonical_anchor_ns = 0;
  int64_t static_offset_ns = 0;
  TimestampQuality quality = TimestampQuality::kMeasurementTimeFallback;
  bool all_points_have_timestamps = false;

  double CanonicalAnchorSec() const;
};

bool NormalizePointCloudTime(
    const PointCloud& cloud,
    const LidarUnifiedComponentConfig::TimeSettings& settings,
    TimeContract* contract);

int64_t IntervalOverlapNs(const TimeContract& lhs, const TimeContract& rhs);

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
