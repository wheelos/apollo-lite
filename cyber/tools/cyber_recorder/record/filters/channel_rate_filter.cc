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

#include "cyber/tools/cyber_recorder/record/filters/channel_rate_filter.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace apollo {
namespace cyber {
namespace record {

namespace {

constexpr double kNanosecondsPerSecond = 1000000000.0;

std::string TrimCopy(const std::string& value) {
  const auto begin = std::find_if_not(value.begin(), value.end(), [](char ch) {
    return std::isspace(static_cast<unsigned char>(ch)) != 0;
  });
  const auto end = std::find_if_not(value.rbegin(), value.rend(), [](char ch) {
                     return std::isspace(static_cast<unsigned char>(ch)) != 0;
                   }).base();
  if (begin >= end) {
    return "";
  }
  return std::string(begin, end);
}

bool ParsePositiveRateHz(const std::string& value, double* output,
                         std::string* error) {
  std::string normalized = TrimCopy(value);
  if (normalized.size() >= 2 &&
      normalized.substr(normalized.size() - 2) == "hz") {
    normalized = TrimCopy(normalized.substr(0, normalized.size() - 2));
  }

  try {
    size_t parsed = 0;
    const auto parsed_value = std::stod(normalized, &parsed);
    if (parsed != normalized.size() || parsed_value <= 0.0) {
      throw std::invalid_argument("invalid value");
    }
    *output = parsed_value;
    return true;
  } catch (const std::invalid_argument&) {
    if (error != nullptr) {
      *error = "Invalid rate value: " + value + ".";
    }
  } catch (const std::out_of_range&) {
    if (error != nullptr) {
      *error = "Rate value is out of range: " + value + ".";
    }
  }
  return false;
}

}  // namespace

bool ParseChannelRateLimitRule(const std::string& value,
                               ChannelRateLimitRule* rule,
                               std::string* error) {
  if (rule == nullptr) {
    if (error != nullptr) {
      *error = "Channel rate limit rule output must not be null.";
    }
    return false;
  }

  const std::string normalized = TrimCopy(value);
  const size_t at_pos = normalized.rfind('@');
  if (at_pos == std::string::npos) {
    if (error != nullptr) {
      *error = "Channel rate limit must use <channel>@<hz>, for example "
               "/apollo/lidar@1.";
    }
    return false;
  }

  const std::string channel_name = TrimCopy(normalized.substr(0, at_pos));
  if (channel_name.empty()) {
    if (error != nullptr) {
      *error = "Channel rate limit must specify a channel name.";
    }
    return false;
  }

  ChannelRateLimitRule parsed_rule;
  parsed_rule.channel_name = channel_name;
  if (!ParsePositiveRateHz(normalized.substr(at_pos + 1),
                           &parsed_rule.max_rate_hz, error)) {
    return false;
  }

  *rule = parsed_rule;
  return true;
}

bool ValidateChannelRateFilterConfig(const ChannelRateFilterConfig& config,
                                     std::string* error) {
  std::unordered_set<std::string> configured_channels;
  for (const auto& rule : config.rules) {
    if (rule.channel_name.empty()) {
      if (error != nullptr) {
        *error = "Channel rate limit rule must specify channel_name.";
      }
      return false;
    }
    if (rule.max_rate_hz <= 0.0) {
      if (error != nullptr) {
        *error = "Channel rate limit max_rate_hz must be positive for " +
                 rule.channel_name + ".";
      }
      return false;
    }
    if (!configured_channels.insert(rule.channel_name).second) {
      if (error != nullptr) {
        *error = "Duplicate channel rate limit rule for " + rule.channel_name +
                 ".";
      }
      return false;
    }
  }
  return true;
}

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
