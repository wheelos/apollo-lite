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

#ifndef CYBER_TOOLS_CYBER_RECORDER_POLICY_RESOLVER_H_
#define CYBER_TOOLS_CYBER_RECORDER_POLICY_RESOLVER_H_

#include <cstdint>
#include <string>
#include <vector>

#include "cyber/tools/cyber_recorder/record/selector/selector_match_type.h"

namespace apollo {
namespace cyber {
namespace record {

struct TopicPolicy {
  std::string name = "default";
  uint64_t drop_message_size_bytes = 0;
  double max_rate_hz = 0.0;
  uint64_t max_bandwidth_bytes_per_sec = 0;
  std::string compression = "none";
};

struct PolicyRule {
  std::string name;
  SelectorMatchType match_type = SelectorMatchType::kChannel;
  std::string match_expression;
  TopicPolicy policy;
};

struct PolicyResolverConfig {
  uint32_t config_version = 1;
  std::vector<PolicyRule> rules;
  TopicPolicy default_policy;
};

struct ResolvedPolicy {
  TopicPolicy policy;
  bool has_conflict = false;
  std::string selected_rule;
  std::vector<std::string> shadowed_rules;
};

class PolicyResolver {
 public:
  explicit PolicyResolver(const PolicyResolverConfig& config = {});

  ResolvedPolicy Resolve(const std::string& topic) const;

 private:
  PolicyResolverConfig config_;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_POLICY_RESOLVER_H_
