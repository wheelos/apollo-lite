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

#ifndef CYBER_TOOLS_CYBER_RECORDER_QOS_STATEFUL_FILTER_H_
#define CYBER_TOOLS_CYBER_RECORDER_QOS_STATEFUL_FILTER_H_

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>

#include "cyber/tools/cyber_recorder/record/selector/policy_resolver.h"

namespace apollo {
namespace cyber {
namespace record {

struct QosFilterDecision {
  bool should_record = true;
  bool dropped_by_size = false;
  bool throttled_by_rate = false;
  bool throttled_by_bandwidth = false;
  std::string compression = "none";
};

class QosStatefulFilter {
 public:
  QosStatefulFilter() = default;

  QosFilterDecision Evaluate(const std::string& topic, size_t payload_size,
                             uint64_t record_time_ns,
                             const TopicPolicy& policy);

 private:
  struct BandwidthWindow {
    uint64_t window_start_ns = 0;
    uint64_t bytes = 0;
  };

  std::unordered_map<std::string, uint64_t> last_record_time_ns_;
  std::unordered_map<std::string, BandwidthWindow> bandwidth_windows_;
  std::mutex mutex_;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_QOS_STATEFUL_FILTER_H_
