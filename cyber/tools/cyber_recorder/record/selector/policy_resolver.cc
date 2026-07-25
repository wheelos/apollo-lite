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

#include "cyber/tools/cyber_recorder/record/selector/policy_resolver.h"

#include "cyber/tools/cyber_recorder/record/selector/topic_matcher.h"

namespace apollo {
namespace cyber {
namespace record {
namespace {

bool RuleMatches(const PolicyRule& rule, const std::string& topic) {
  return TopicMatchesRule(rule.match_type, rule.match_expression, topic);
}

}  // namespace

PolicyResolver::PolicyResolver(const PolicyResolverConfig& config)
    : config_(config) {}

ResolvedPolicy PolicyResolver::Resolve(const std::string& topic) const {
  ResolvedPolicy resolved;
  resolved.policy = config_.default_policy;
  resolved.selected_rule = "default";

  bool has_selected_rule = false;
  for (const auto& rule : config_.rules) {
    if (!RuleMatches(rule, topic)) {
      continue;
    }
    if (!has_selected_rule) {
      has_selected_rule = true;
      resolved.selected_rule = rule.name;
      resolved.policy = rule.policy;
      continue;
    }
    resolved.shadowed_rules.push_back(rule.name);
  }

  resolved.has_conflict = !resolved.shadowed_rules.empty();
  return resolved;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
