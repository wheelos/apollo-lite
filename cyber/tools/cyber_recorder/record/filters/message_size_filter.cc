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

//  Created Date: 2026-04-27
//  Author: daohu527

#include "cyber/tools/cyber_recorder/record/filters/message_size_filter.h"

#include <algorithm>
#include <cctype>
#include <limits>
#include <stdexcept>

namespace apollo {
namespace cyber {
namespace record {

namespace {

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

std::string ToLowerCopy(const std::string& value) {
  std::string lower = value;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](char ch) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  });
  return lower;
}

bool ParseMessageSizeBytesInternal(const std::string& value,
                                   uint64_t default_multiplier,
                                   uint64_t* output, std::string* error) {
  const std::string normalized = ToLowerCopy(TrimCopy(value));
  if (normalized.empty()) {
    if (error != nullptr) {
      *error = "Message size value must not be empty.";
    }
    return false;
  }

  const size_t suffix_pos = normalized.find_first_not_of("0123456789");
  const std::string number_part = normalized.substr(0, suffix_pos);
  const std::string suffix =
      suffix_pos == std::string::npos ? "" : normalized.substr(suffix_pos);
  if (number_part.empty()) {
    if (error != nullptr) {
      *error = "Invalid message size value: " + value + ".";
    }
    return false;
  }

  uint64_t multiplier = 0;
  if (suffix.empty()) {
    multiplier = default_multiplier;
  } else if (suffix == "b") {
    multiplier = 1;
  } else if (suffix == "k" || suffix == "kb" || suffix == "kib") {
    multiplier = 1024ULL;
  } else if (suffix == "m" || suffix == "mb" || suffix == "mib") {
    multiplier = 1024ULL * 1024ULL;
  } else if (suffix == "g" || suffix == "gb" || suffix == "gib") {
    multiplier = 1024ULL * 1024ULL * 1024ULL;
  } else {
    if (error != nullptr) {
      *error = "Unsupported message size suffix: " + value +
               ". Use raw bytes or k/m/g units.";
    }
    return false;
  }

  try {
    size_t parsed = 0;
    const auto parsed_value = std::stoull(number_part, &parsed);
    if (parsed != number_part.size()) {
      throw std::invalid_argument("invalid trailing characters");
    }
    if (parsed_value > std::numeric_limits<uint64_t>::max() / multiplier) {
      throw std::out_of_range("size overflow");
    }
    *output = parsed_value * multiplier;
    return true;
  } catch (const std::invalid_argument&) {
    if (error != nullptr) {
      *error = "Invalid message size value: " + value + ".";
    }
  } catch (const std::out_of_range&) {
    if (error != nullptr) {
      *error = "Message size value is out of range: " + value + ".";
    }
  }
  return false;
}

}  // namespace

bool ParseMessageSizeBytes(const std::string& value, uint64_t* output,
                           std::string* error) {
  return ParseMessageSizeBytesInternal(value, 1, output, error);
}

bool ParseMessageSizeFilterPolicy(const std::string& policy,
                                  MessageSizeFilterConfig* config,
                                  std::string* error) {
  if (config == nullptr) {
    if (error != nullptr) {
      *error = "Message size filter policy output must not be null.";
    }
    return false;
  }

  const std::string trimmed_policy = TrimCopy(policy);
  if (trimmed_policy.empty()) {
    if (error != nullptr) {
      *error = "Message size filter policy must not be empty.";
    }
    return false;
  }

  MessageSizeFilterConfig parsed_config;
  size_t clause_begin = 0;
  while (clause_begin <= trimmed_policy.size()) {
    const size_t clause_end = trimmed_policy.find(',', clause_begin);
    const std::string clause = TrimCopy(
        trimmed_policy.substr(clause_begin, clause_end - clause_begin));
    if (clause.empty()) {
      if (error != nullptr) {
        *error = "Message size filter policy contains an empty clause.";
      }
      return false;
    }

    const size_t equal_pos = clause.find('=');
    if (equal_pos == std::string::npos) {
      if (error != nullptr) {
        *error = "Invalid filter clause: " + clause + ".";
      }
      return false;
    }

    const std::string key = ToLowerCopy(TrimCopy(clause.substr(0, equal_pos)));
    const std::string value = TrimCopy(clause.substr(equal_pos + 1));
    if (key == "drop") {
      if (parsed_config.drop_message_size_bytes != 0) {
        if (error != nullptr) {
          *error = "Duplicate drop clause in filter policy.";
        }
        return false;
      }
      if (!ParseMessageSizeBytesInternal(value, 1024ULL,
                                         &parsed_config.drop_message_size_bytes,
                                         error)) {
        return false;
      }
    } else {
      if (error != nullptr) {
        *error = "Unknown filter clause key: " + key +
                 ". Supported keys are drop.";
      }
      return false;
    }

    if (clause_end == std::string::npos) {
      break;
    }
    clause_begin = clause_end + 1;
  }

  if (!ValidateMessageSizeFilterConfig(parsed_config, error)) {
    return false;
  }
  *config = parsed_config;
  return true;
}

bool ValidateMessageSizeFilterConfig(const MessageSizeFilterConfig& config,
                                     std::string* error) {
  (void)config;
  (void)error;
  return true;
}

MessageSizeFilter::MessageSizeFilter(const MessageSizeFilterConfig& config)
    : config_(config) {}

MessageSizeFilterDecision MessageSizeFilter::Evaluate(
    const std::string& channel_name, size_t payload_size,
    uint64_t record_time_ns) {
  (void)channel_name;
  (void)record_time_ns;
  if (config_.drop_message_size_bytes > 0 &&
      payload_size >= config_.drop_message_size_bytes) {
    MessageSizeFilterDecision decision;
    decision.should_record = false;
    decision.dropped_by_size = true;
    return decision;
  }
  return {};
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
