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

#ifndef CYBER_TOOLS_CYBER_RECORDER_SUBSCRIPTION_SELECTOR_H_
#define CYBER_TOOLS_CYBER_RECORDER_SUBSCRIPTION_SELECTOR_H_

#include <string>
#include <vector>

#include "cyber/tools/cyber_recorder/record/selector/selector_match_type.h"

namespace apollo {
namespace cyber {
namespace record {

enum class SelectorAction {
  kInclude = 0,
  kExclude = 1,
};

struct TopicMetadata {
  std::string topic;
  std::string message_type;
};

struct SubscriptionRule {
  std::string name;
  SelectorMatchType match_type = SelectorMatchType::kChannel;
  std::string match_expression;
  SelectorAction action = SelectorAction::kInclude;
};

struct SubscriptionSelectorConfig {
  uint32_t config_version = 1;
  SelectorAction default_action = SelectorAction::kInclude;
  std::vector<SubscriptionRule> selectors;
};

struct SubscriptionDecision {
  bool should_subscribe = true;
  bool has_conflict = false;
  std::vector<std::string> include_matches;
  std::vector<std::string> exclude_matches;
};

class SubscriptionSelector {
 public:
  explicit SubscriptionSelector(const SubscriptionSelectorConfig& config = {});

  SubscriptionDecision Evaluate(const TopicMetadata& metadata) const;

 private:
  SubscriptionSelectorConfig config_;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_SUBSCRIPTION_SELECTOR_H_
