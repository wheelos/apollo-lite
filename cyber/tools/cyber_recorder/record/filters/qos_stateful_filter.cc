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

#include "cyber/tools/cyber_recorder/record/filters/qos_stateful_filter.h"

#include <algorithm>
#include <cmath>

namespace apollo {
namespace cyber {
namespace record {
namespace {

constexpr double kNanosecondsPerSecond = 1000000000.0;
constexpr uint64_t kBandwidthWindowNs = 1000000000ULL;

}  // namespace

QosFilterDecision QosStatefulFilter::Evaluate(const std::string& topic,
                                              const size_t payload_size,
                                              const uint64_t record_time_ns,
                                              const TopicPolicy& policy) {
  QosFilterDecision decision;
  decision.compression = policy.compression;

  std::lock_guard<std::mutex> lock(mutex_);

  // Simple policy mode: either rate OR size-drop.
  if (policy.max_rate_hz > 0.0) {
    const uint64_t required_interval_ns = std::max<uint64_t>(
        1, static_cast<uint64_t>(
               std::llround(kNanosecondsPerSecond / policy.max_rate_hz)));
    uint64_t& last_record_time_ns = last_record_time_ns_[topic];
    if (last_record_time_ns != 0) {
      const uint64_t elapsed_ns = record_time_ns > last_record_time_ns
                                      ? record_time_ns - last_record_time_ns
                                      : 0;
      if (elapsed_ns < required_interval_ns) {
        decision.should_record = false;
        decision.throttled_by_rate = true;
        return decision;
      }
    }
    last_record_time_ns = record_time_ns;
  } else if (policy.drop_message_size_bytes > 0 &&
             payload_size >= policy.drop_message_size_bytes) {
    decision.should_record = false;
    decision.dropped_by_size = true;
    return decision;
  }

  if (policy.max_bandwidth_bytes_per_sec > 0) {
    BandwidthWindow& window = bandwidth_windows_[topic];
    if (window.window_start_ns == 0 ||
        record_time_ns >= window.window_start_ns + kBandwidthWindowNs) {
      window.window_start_ns = record_time_ns;
      window.bytes = 0;
    }
    if (payload_size > policy.max_bandwidth_bytes_per_sec ||
        window.bytes + payload_size > policy.max_bandwidth_bytes_per_sec) {
      decision.should_record = false;
      decision.throttled_by_bandwidth = true;
      return decision;
    }
    window.bytes += payload_size;
  }

  return decision;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
