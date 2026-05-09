/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/tools/roadlog/processor/roadlog_recording_service.h"

#include <algorithm>
#include <unordered_set>

#include "cyber/common/log.h"
#include "cyber/record/header_builder.h"

namespace apollo {
namespace data {

namespace {

using cyber::record::ChannelRateLimitRule;
using cyber::record::HeaderBuilder;
using cyber::record::ParseMessageSizeFilterPolicy;
using cyber::record::Recorder;
using cyber::record::ValidateMessageSizeFilterConfig;

}  // namespace

RoadlogRecordingService::RoadlogRecordingService(const RoadlogLayout& layout)
    : layout_(layout) {}

bool RoadlogRecordingService::Init(
    const SmartRecordTrigger& trigger_conf,
    const std::set<std::string>& required_channels) {
  RecorderOptions options;
  std::string error;
  if (!BuildRecorderOptions(trigger_conf, layout_.ring_file_prefix, &options,
                            &error)) {
    AERROR << "failed to build recorder options: " << error;
    return false;
  }
  if (!ValidateRequiredChannels(required_channels, options, &error)) {
    AERROR << "invalid recorder policy for trigger channels: " << error;
    return false;
  }
  recorder_ = std::make_shared<Recorder>(
      options.output_path, options.all_channels, options.include_channels,
      options.exclude_channels, options.header,
      options.message_size_filter_config, options.channel_rate_filter_config);
  return true;
}

bool RoadlogRecordingService::Start() {
  return recorder_ != nullptr && recorder_->Start();
}

void RoadlogRecordingService::Stop() {
  if (recorder_ != nullptr) {
    recorder_->Stop();
  }
}

bool RoadlogRecordingService::BuildRecorderOptions(
    const SmartRecordTrigger& trigger_conf, const std::string& output_path,
    RecorderOptions* options, std::string* error) {
  if (options == nullptr) {
    if (error != nullptr) {
      *error = "Recorder options output must not be null.";
    }
    return false;
  }

  options->output_path = output_path;
  auto header = HeaderBuilder::GetHeader();
  static constexpr uint64_t kSecondsToNanoseconds = 1000000000UL;
  static constexpr uint64_t kMegabytesToBytes = 1024UL * 1024UL;
  header.set_segment_interval(
      static_cast<uint64_t>(trigger_conf.segment_setting().time_segment()) *
      kSecondsToNanoseconds);
  header.set_segment_raw_size(
      static_cast<uint64_t>(trigger_conf.segment_setting().size_segment()) *
      kMegabytesToBytes);
  options->header = header;

  const auto& recorder_policy = trigger_conf.recorder_policy();
  options->include_channels.assign(recorder_policy.include_channels().begin(),
                                   recorder_policy.include_channels().end());
  options->exclude_channels.assign(recorder_policy.exclude_channels().begin(),
                                   recorder_policy.exclude_channels().end());
  options->all_channels = options->include_channels.empty();

  std::unordered_set<std::string> configured_channels;
  for (const auto& limit : recorder_policy.channel_rate_limits()) {
    if (limit.channel_name().empty()) {
      if (error != nullptr) {
        *error = "Recorder channel_rate_limits must specify channel_name.";
      }
      return false;
    }
    if (limit.max_rate_hz() <= 0.0) {
      if (error != nullptr) {
        *error =
            "Recorder channel_rate_limits max_rate_hz must be positive for " +
            limit.channel_name() + ".";
      }
      return false;
    }
    if (!configured_channels.insert(limit.channel_name()).second) {
      if (error != nullptr) {
        *error = "Duplicate recorder channel_rate_limits rule for " +
                 limit.channel_name() + ".";
      }
      return false;
    }
    ChannelRateLimitRule rule;
    rule.channel_name = limit.channel_name();
    rule.max_rate_hz = limit.max_rate_hz();
    options->channel_rate_filter_config.rules.push_back(rule);
  }

  if (!BuildLargeMessageFilterConfig(
          recorder_policy, &options->message_size_filter_config, error)) {
    return false;
  }
  if (!ValidateChannelPolicies(recorder_policy, *options, error)) {
    return false;
  }

  return true;
}

bool RoadlogRecordingService::BuildLargeMessageFilterConfig(
    const RecorderPolicy& recorder_policy,
    apollo::cyber::record::MessageSizeFilterConfig* config,
    std::string* error) {
  if (config == nullptr) {
    if (error != nullptr) {
      *error = "Message size filter config output must not be null.";
    }
    return false;
  }

  const bool has_legacy_policy =
      !recorder_policy.legacy_message_size_filter_policy().empty();
  const bool has_explicit_policy =
      recorder_policy.has_large_message_policy() &&
      recorder_policy.large_message_policy().drop_message_size_bytes() > 0;
  if (has_legacy_policy && has_explicit_policy) {
    if (error != nullptr) {
      *error =
          "Do not mix recorder_policy.legacy_message_size_filter_policy "
          "with recorder_policy.large_message_policy.";
    }
    return false;
  }

  if (has_explicit_policy) {
    const auto& large_message_policy = recorder_policy.large_message_policy();
    config->drop_message_size_bytes =
        large_message_policy.drop_message_size_bytes();
  } else if (has_legacy_policy) {
    if (!ParseMessageSizeFilterPolicy(
            recorder_policy.legacy_message_size_filter_policy(), config,
            error)) {
      return false;
    }
  }

  return ValidateMessageSizeFilterConfig(*config, error);
}

bool RoadlogRecordingService::ValidateChannelPolicies(
    const RecorderPolicy& recorder_policy, const RecorderOptions& options,
    std::string* error) {
  std::unordered_set<std::string> include_channels(
      options.include_channels.begin(), options.include_channels.end());
  std::unordered_set<std::string> exclude_channels(
      options.exclude_channels.begin(), options.exclude_channels.end());
  for (const auto& channel_name : include_channels) {
    if (exclude_channels.find(channel_name) != exclude_channels.end()) {
      if (error != nullptr) {
        *error = "Recorder policy channel " + channel_name +
                 " cannot be configured in both include_channels and "
                 "exclude_channels.";
      }
      return false;
    }
  }

  for (const auto& limit : recorder_policy.channel_rate_limits()) {
    if (exclude_channels.find(limit.channel_name()) != exclude_channels.end()) {
      AWARN << "roadlog recorder_policy channel_rate_limits entry for "
            << limit.channel_name()
            << " is ignored because the channel is excluded.";
      continue;
    }
    if (!include_channels.empty() &&
        include_channels.find(limit.channel_name()) == include_channels.end()) {
      AWARN << "roadlog recorder_policy channel_rate_limits entry for "
            << limit.channel_name()
            << " is ignored because include_channels does not subscribe to "
               "that topic.";
    }
  }
  return true;
}

bool RoadlogRecordingService::ValidateRequiredChannels(
    const std::set<std::string>& required_channels,
    const RecorderOptions& options, std::string* error) {
  if (required_channels.empty()) {
    return true;
  }
  const std::unordered_set<std::string> include_channels(
      options.include_channels.begin(), options.include_channels.end());
  const std::unordered_set<std::string> exclude_channels(
      options.exclude_channels.begin(), options.exclude_channels.end());
  std::unordered_set<std::string> limited_channels;
  for (const auto& rule : options.channel_rate_filter_config.rules) {
    limited_channels.insert(rule.channel_name);
  }
  for (const auto& channel_name : required_channels) {
    if (!include_channels.empty() &&
        include_channels.find(channel_name) == include_channels.end()) {
      if (error != nullptr) {
        *error = "trigger channel " + channel_name +
                 " must be present in include_channels.";
      }
      return false;
    }
    if (exclude_channels.find(channel_name) != exclude_channels.end()) {
      if (error != nullptr) {
        *error = "trigger channel " + channel_name +
                 " cannot be listed in exclude_channels.";
      }
      return false;
    }
    if (limited_channels.find(channel_name) != limited_channels.end()) {
      if (error != nullptr) {
        *error = "trigger channel " + channel_name +
                 " cannot be downsampled by channel_rate_limits.";
      }
      return false;
    }
  }
  return true;
}

}  // namespace data
}  // namespace apollo
