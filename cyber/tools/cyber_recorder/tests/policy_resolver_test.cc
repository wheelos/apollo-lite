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

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace record {
namespace {

TEST(PolicyResolverTest, SelectsFirstMatchedPolicyAndTracksShadowedRules) {
  PolicyResolverConfig config;
  config.default_policy.name = "default";
  config.default_policy.drop_message_size_bytes = 0;
  config.default_policy.max_rate_hz = 0.0;
  config.default_policy.max_bandwidth_bytes_per_sec = 0;
  config.default_policy.compression = "none";
  config.rules = {
      {"lidar_front", SelectorMatchType::kChannel, "/lidar/front",
       {"lidar_front", 1024, 10.0, 0, "zstd"}},
      {"all_lidar", SelectorMatchType::kRegex, "^/lidar/.*",
       {"all_lidar", 2048, 5.0, 52428800ULL, "zstd"}},
  };

  PolicyResolver resolver(config);
  const auto resolved = resolver.Resolve("/lidar/front");
  EXPECT_EQ("lidar_front", resolved.selected_rule);
  EXPECT_EQ("lidar_front", resolved.policy.name);
  EXPECT_EQ(1024ULL, resolved.policy.drop_message_size_bytes);
  EXPECT_DOUBLE_EQ(10.0, resolved.policy.max_rate_hz);
  EXPECT_TRUE(resolved.has_conflict);
  ASSERT_EQ(1U, resolved.shadowed_rules.size());
  EXPECT_EQ("all_lidar", resolved.shadowed_rules[0]);

  const auto fallback = resolver.Resolve("/localization");
  EXPECT_EQ("default", fallback.selected_rule);
  EXPECT_EQ("default", fallback.policy.name);
  EXPECT_FALSE(fallback.has_conflict);
}

}  // namespace
}  // namespace record
}  // namespace cyber
}  // namespace apollo
