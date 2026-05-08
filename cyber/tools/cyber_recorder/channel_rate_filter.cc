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

#include "cyber/tools/cyber_recorder/channel_rate_filter.h"

#include <algorithm>
#include <cmath>

namespace apollo {
namespace cyber {
namespace record {

namespace {

constexpr double kNanosecondsPerSecond = 1000000000.0;

}  // namespace

ChannelRateFilter::ChannelRateFilter(const ChannelRateFilterConfig& config) {
  for (const auto& rule : config.rules) {
    if (rule.channel_name.empty() || rule.max_rate_hz <= 0.0) {
      continue;
    }
    channel_interval_ns_[rule.channel_name] = std::max<uint64_t>(
        1, static_cast<uint64_t>(
               std::llround(kNanosecondsPerSecond / rule.max_rate_hz)));
  }
}

ChannelRateFilterDecision ChannelRateFilter::Evaluate(
    const std::string& channel_name, const uint64_t record_time_ns) {
  const auto rule_iter = channel_interval_ns_.find(channel_name);
  if (rule_iter == channel_interval_ns_.end()) {
    return {};
  }

  std::lock_guard<std::mutex> lock(mutex_);
  uint64_t& last_record_time_ns = last_record_time_ns_[channel_name];
  if (last_record_time_ns != 0) {
    const uint64_t elapsed_ns =
        record_time_ns > last_record_time_ns ? record_time_ns - last_record_time_ns
                                             : 0;
    if (elapsed_ns < rule_iter->second) {
      ChannelRateFilterDecision decision;
      decision.should_record = false;
      decision.throttled_by_rate = true;
      return decision;
    }
  }

  last_record_time_ns = record_time_ns;
  return {};
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
