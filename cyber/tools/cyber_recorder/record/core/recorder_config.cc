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

#include "cyber/tools/cyber_recorder/record/core/recorder_config.h"

#include <utility>

#include "yaml-cpp/yaml.h"

namespace apollo {
namespace cyber {
namespace record {
namespace {

void SetError(std::string* error, const std::string& value) {
  if (error != nullptr) {
    *error = value;
  }
}

SelectorAction ParseSelectorAction(const std::string& value, bool* ok) {
  *ok = true;
  if (value == "include") {
    return SelectorAction::kInclude;
  }
  if (value == "exclude") {
    return SelectorAction::kExclude;
  }
  *ok = false;
  return SelectorAction::kInclude;
}

SelectorMatchType ParseMatchType(const std::string& value, bool* ok) {
  *ok = true;
  if (value == "channel") {
    return SelectorMatchType::kChannel;
  }
  if (value == "regex") {
    return SelectorMatchType::kRegex;
  }
  *ok = false;
  return SelectorMatchType::kChannel;
}

bool ParsePolicyNode(const YAML::Node& node, TopicPolicy* policy,
                     std::string* error) {
  TopicPolicy parsed;
  parsed.name = "default";
  parsed.compression = "none";

  if (node["drop_message_size_bytes"]) {
    parsed.drop_message_size_bytes = node["drop_message_size_bytes"].as<uint64_t>();
  }
  if (node["sample_rate"]) {
    if (node["sample_rate"].IsScalar() &&
        node["sample_rate"].as<std::string>() == "full") {
      parsed.max_rate_hz = 0.0;
    } else {
      parsed.max_rate_hz = node["sample_rate"].as<double>();
      if (parsed.max_rate_hz <= 0.0) {
        SetError(error, "sample_rate must be positive or 'full'.");
        return false;
      }
    }
  }
  if (parsed.drop_message_size_bytes > 0 && parsed.max_rate_hz > 0.0) {
    SetError(error,
             "policy must configure either drop_message_size_bytes or sample_rate.");
    return false;
  }
  if (node["max_bandwidth_bytes_per_sec"]) {
    parsed.max_bandwidth_bytes_per_sec =
        node["max_bandwidth_bytes_per_sec"].as<uint64_t>();
  }
  if (node["compression"]) {
    parsed.compression = node["compression"].as<std::string>();
  }

  *policy = parsed;
  return true;
}

TopicPolicy BuildDefaultPolicy(const MessageSizeFilterConfig& message_size_filter_config) {
  TopicPolicy policy;
  policy.name = "default";
  policy.drop_message_size_bytes =
      message_size_filter_config.drop_message_size_bytes;
  policy.max_rate_hz = 0.0;
  policy.max_bandwidth_bytes_per_sec = 0;
  policy.compression = "none";
  return policy;
}

}  // namespace

bool BuildRecorderConfigFromLegacyOptions(
    const bool all_channels, const std::vector<std::string>& white_channels,
    const std::vector<std::string>& black_channels,
    const MessageSizeFilterConfig& message_size_filter_config,
    const ChannelRateFilterConfig& channel_rate_filter_config,
    RecorderConfigBundle* config, std::string* error) {
  if (config == nullptr) {
    if (error != nullptr) {
      *error = "config output must not be null.";
    }
    return false;
  }
  if (message_size_filter_config.drop_message_size_bytes > 0 &&
      !channel_rate_filter_config.rules.empty()) {
    if (error != nullptr) {
      *error = "legacy options only support one mode: --drop or --channel-rate-limit.";
    }
    return false;
  }

  RecorderConfigBundle parsed;
  parsed.version = 1;

  parsed.subscription.config_version = parsed.version;
  parsed.subscription.default_action =
      all_channels ? SelectorAction::kInclude : SelectorAction::kExclude;
  for (const auto& channel : black_channels) {
    parsed.subscription.selectors.push_back(
        {"exclude_" + channel, SelectorMatchType::kChannel, channel,
         SelectorAction::kExclude});
  }
  for (const auto& channel : white_channels) {
    parsed.subscription.selectors.push_back(
        {"include_" + channel, SelectorMatchType::kChannel, channel,
         SelectorAction::kInclude});
  }

  parsed.policies.config_version = parsed.version;
  parsed.policies.default_policy = BuildDefaultPolicy(message_size_filter_config);
  for (const auto& rule : channel_rate_filter_config.rules) {
    TopicPolicy policy = parsed.policies.default_policy;
    policy.name = rule.channel_name;
    policy.drop_message_size_bytes = 0;
    policy.max_rate_hz = rule.max_rate_hz;
    parsed.policies.rules.push_back(
        {rule.channel_name, SelectorMatchType::kChannel, rule.channel_name, policy});
  }

  *config = std::move(parsed);
  return true;
}

bool LoadRecorderConfigFromYamlFile(const std::string& path,
                                    RecorderConfigBundle* config,
                                    std::string* error) {
  if (config == nullptr) {
    if (error != nullptr) {
      *error = "config output must not be null.";
    }
    return false;
  }
  try {
    YAML::Node root = YAML::LoadFile(path);

    RecorderConfigBundle parsed;
    parsed.version = root["version"] ? root["version"].as<uint32_t>() : 1;
    parsed.subscription.config_version = parsed.version;
    parsed.policies.config_version = parsed.version;

    if (!root["subscription"] || !root["subscription"]["default_action"]) {
      SetError(error, "subscription.default_action is required.");
      return false;
    }
    bool action_ok = false;
    parsed.subscription.default_action = ParseSelectorAction(
        root["subscription"]["default_action"].as<std::string>(), &action_ok);
    if (!action_ok) {
      SetError(error, "subscription.default_action must be include or exclude.");
      return false;
    }

    if (root["subscription"]["selectors"]) {
      for (const auto& selector_node : root["subscription"]["selectors"]) {
        SubscriptionRule selector;
        selector.name = selector_node["name"].as<std::string>();
        const auto match_node = selector_node["match"];
        if (!match_node || !match_node["type"]) {
          SetError(error, "subscription selector match.type is required.");
          return false;
        }
        bool match_type_ok = false;
        selector.match_type =
            ParseMatchType(match_node["type"].as<std::string>(), &match_type_ok);
        if (!match_type_ok) {
          SetError(error,
                   "subscription selector match.type must be channel or regex.");
          return false;
        }
        selector.match_expression =
            selector.match_type == SelectorMatchType::kChannel
                ? match_node["value"].as<std::string>()
                : match_node["pattern"].as<std::string>();
        selector.action =
            ParseSelectorAction(selector_node["action"].as<std::string>(),
                                &action_ok);
        if (!action_ok) {
          SetError(error, "subscription selector action must be include or exclude.");
          return false;
        }
        parsed.subscription.selectors.push_back(std::move(selector));
      }
    }

    if (!root["policies"] || !root["policies"]["default"] ||
        !root["policies"]["default"]["policy"]) {
      SetError(error, "policies.default.policy is required.");
      return false;
    }
    if (!ParsePolicyNode(root["policies"]["default"]["policy"],
                         &parsed.policies.default_policy, error)) {
      return false;
    }

    if (root["policies"]["rules"]) {
      for (const auto& rule_node : root["policies"]["rules"]) {
        PolicyRule rule;
        rule.name = rule_node["name"].as<std::string>();
        const auto match_node = rule_node["match"];
        if (!match_node || !match_node["type"]) {
          SetError(error, "policy rule match.type is required.");
          return false;
        }
        bool match_type_ok = false;
        rule.match_type =
            ParseMatchType(match_node["type"].as<std::string>(), &match_type_ok);
        if (!match_type_ok) {
          SetError(error, "policy rule match.type must be channel or regex.");
          return false;
        }
        rule.match_expression =
            rule.match_type == SelectorMatchType::kChannel
                ? match_node["value"].as<std::string>()
                : match_node["pattern"].as<std::string>();
        if (!ParsePolicyNode(rule_node["policy"], &rule.policy, error)) {
          return false;
        }
        rule.policy.name = rule.name;
        parsed.policies.rules.push_back(std::move(rule));
      }
    }

    *config = std::move(parsed);
    return true;
  } catch (const YAML::Exception& e) {
    SetError(error, e.what());
    return false;
  }
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
