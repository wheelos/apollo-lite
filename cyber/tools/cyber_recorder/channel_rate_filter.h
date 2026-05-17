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

#ifndef CYBER_TOOLS_CYBER_RECORDER_CHANNEL_RATE_FILTER_H_
#define CYBER_TOOLS_CYBER_RECORDER_CHANNEL_RATE_FILTER_H_

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace apollo {
namespace cyber {
namespace record {

struct ChannelRateLimitRule {
  std::string channel_name;
  double max_rate_hz = 0.0;
};

struct ChannelRateFilterConfig {
  std::vector<ChannelRateLimitRule> rules;
};

struct ChannelRateFilterDecision {
  bool should_record = true;
  bool throttled_by_rate = false;
};

bool ParseChannelRateLimitRule(const std::string& value,
                               ChannelRateLimitRule* rule,
                               std::string* error);

bool ValidateChannelRateFilterConfig(const ChannelRateFilterConfig& config,
                                     std::string* error);

class ChannelRateFilter {
 public:
  explicit ChannelRateFilter(const ChannelRateFilterConfig& config = {});

  ChannelRateFilterDecision Evaluate(const std::string& channel_name,
                                     uint64_t record_time_ns);

 private:
  std::unordered_map<std::string, uint64_t> channel_interval_ns_;
  std::unordered_map<std::string, uint64_t> last_record_time_ns_;
  std::mutex mutex_;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_CHANNEL_RATE_FILTER_H_
