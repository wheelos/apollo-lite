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

#include "cyber/tools/cyber_recorder/record/selector/subscription_selector.h"

#include "cyber/tools/cyber_recorder/record/selector/topic_matcher.h"

namespace apollo {
namespace cyber {
namespace record {
namespace {

bool SelectorMatches(const SubscriptionRule& selector,
                     const TopicMetadata& metadata) {
  return TopicMatchesRule(selector.match_type, selector.match_expression,
                          metadata.topic);
}

bool ActionToSubscriptionDecision(const SelectorAction action) {
  return action == SelectorAction::kInclude;
}

}  // namespace

SubscriptionSelector::SubscriptionSelector(const SubscriptionSelectorConfig& config)
    : config_(config) {}

SubscriptionDecision SubscriptionSelector::Evaluate(
    const TopicMetadata& metadata) const {
  SubscriptionDecision decision;
  decision.should_subscribe =
      ActionToSubscriptionDecision(config_.default_action);

  bool has_match = false;
  SelectorAction selected_action = config_.default_action;
  for (const auto& selector : config_.selectors) {
    if (!SelectorMatches(selector, metadata)) {
      continue;
    }
    if (selector.action == SelectorAction::kInclude) {
      decision.include_matches.push_back(selector.name);
    } else {
      decision.exclude_matches.push_back(selector.name);
    }
    selected_action = selector.action;
    has_match = true;
  }
  if (has_match) {
    decision.should_subscribe = ActionToSubscriptionDecision(selected_action);
  }
  decision.has_conflict = !decision.include_matches.empty() &&
                          !decision.exclude_matches.empty();
  return decision;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
