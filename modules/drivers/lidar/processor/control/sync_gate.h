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
#include <functional>
#include <string>
#include <vector>

#include "modules/drivers/lidar/processor/control/frame_handle.h"

namespace apollo {
namespace drivers {
namespace lidar {

struct SyncGateMetrics {
  size_t expected_sensor_count = 0;
  size_t matched_sensor_count = 0;
  size_t missing_auxiliary_count = 0;
  size_t time_delta_exceeded_count = 0;
};

class SyncGate {
 public:
  using ResolveSensorIdByTopicFn = std::function<bool(
      const std::string& topic_name, std::string* sensor_id)>;

  using LookupNearestFrameFn =
      std::function<bool(const std::string& sensor_id,
                         const TimeContract& reference_time,
                         uint32_t max_ref_time_delta_ms,
                         FrameHandle* frame_handle, bool* time_delta_exceeded)>;

  bool SelectFrames(const FrameHandle& primary_handle,
                    const std::vector<std::string>& auxiliary_topics,
                    uint32_t max_ref_time_delta_ms, bool strict_auxiliary_sync,
                    const ResolveSensorIdByTopicFn& resolve_sensor_id,
                    const LookupNearestFrameFn& lookup_nearest_frame,
                    std::vector<FrameHandle>* frame_handles,
                    SyncGateMetrics* metrics) const;
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
