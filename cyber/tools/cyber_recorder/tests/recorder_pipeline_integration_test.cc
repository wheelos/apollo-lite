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
#include "cyber/tools/cyber_recorder/record/filters/qos_stateful_filter.h"
#include "cyber/tools/cyber_recorder/record/selector/subscription_selector.h"

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace record {
namespace {

TEST(RecorderPipelineIntegrationTest, UsesRateOnlyModeWhenPolicySetsRate) {
  SubscriptionSelectorConfig selector_config;
  selector_config.default_action = SelectorAction::kInclude;
  SubscriptionSelector selector(selector_config);
  EXPECT_TRUE(selector.Evaluate({"/lidar/front", "apollo.test"}).should_subscribe);

  PolicyResolverConfig policy_config;
  policy_config.default_policy.name = "default";
  policy_config.default_policy.drop_message_size_bytes = 128;
  policy_config.rules.push_back(
      {"lidar_front", SelectorMatchType::kChannel, "/lidar/front",
       {"lidar_front", 0, 5.0, 0, "none"}});
  PolicyResolver resolver(policy_config);
  const auto resolved = resolver.Resolve("/lidar/front");
  ASSERT_EQ("lidar_front", resolved.selected_rule);
  EXPECT_EQ(0ULL, resolved.policy.drop_message_size_bytes);
  EXPECT_DOUBLE_EQ(5.0, resolved.policy.max_rate_hz);

  QosStatefulFilter filter;
  const uint64_t kStartNs = 1000000000ULL;
  const auto first = filter.Evaluate("/lidar/front", 1024, kStartNs, resolved.policy);
  EXPECT_TRUE(first.should_record);
  const auto second =
      filter.Evaluate("/lidar/front", 1024, kStartNs + 100000000ULL, resolved.policy);
  EXPECT_FALSE(second.should_record);
  EXPECT_TRUE(second.throttled_by_rate);
}

TEST(RecorderPipelineIntegrationTest, UsesDropOnlyModeWhenPolicySetsDrop) {
  PolicyResolverConfig policy_config;
  policy_config.default_policy.name = "default";
  policy_config.default_policy.drop_message_size_bytes = 100;
  policy_config.default_policy.max_rate_hz = 0.0;
  PolicyResolver resolver(policy_config);
  const auto resolved = resolver.Resolve("/localization");
  EXPECT_EQ("default", resolved.selected_rule);

  QosStatefulFilter filter;
  const auto dropped = filter.Evaluate("/localization", 128, 1000000000ULL,
                                       resolved.policy);
  EXPECT_FALSE(dropped.should_record);
  EXPECT_TRUE(dropped.dropped_by_size);
  EXPECT_FALSE(dropped.throttled_by_rate);
}

}  // namespace
}  // namespace record
}  // namespace cyber
}  // namespace apollo
