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

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace record {
namespace {

TEST(ChannelRateFilterTest, ParsesChannelRateLimitRule) {
  ChannelRateLimitRule rule;
  std::string error;
  EXPECT_TRUE(ParseChannelRateLimitRule("/apollo/lidar@2", &rule, &error));
  EXPECT_EQ("/apollo/lidar", rule.channel_name);
  EXPECT_DOUBLE_EQ(2.0, rule.max_rate_hz);
}

TEST(ChannelRateFilterTest, RejectsInvalidChannelRateLimitRule) {
  ChannelRateLimitRule rule;
  std::string error;
  EXPECT_FALSE(ParseChannelRateLimitRule("/apollo/lidar", &rule, &error));
  EXPECT_FALSE(error.empty());
  EXPECT_FALSE(ParseChannelRateLimitRule("@2", &rule, &error));
  EXPECT_FALSE(error.empty());
}

TEST(ChannelRateFilterTest, ValidatesChannelRateFilterConfig) {
  ChannelRateFilterConfig config;
  config.rules.push_back({"/apollo/lidar", 1.0});
  config.rules.push_back({"/apollo/camera", 2.0});
  std::string error;
  EXPECT_TRUE(ValidateChannelRateFilterConfig(config, &error));
  EXPECT_TRUE(error.empty());

  config.rules.push_back({"/apollo/lidar", 3.0});
  EXPECT_FALSE(ValidateChannelRateFilterConfig(config, &error));
  EXPECT_FALSE(error.empty());
}

TEST(ChannelRateFilterTest, AllowsUnconfiguredChannel) {
  ChannelRateFilter filter;
  const auto decision = filter.Evaluate("/apollo/chatter", 100);
  EXPECT_TRUE(decision.should_record);
  EXPECT_FALSE(decision.throttled_by_rate);
}

TEST(ChannelRateFilterTest, ThrottlesConfiguredChannel) {
  ChannelRateFilterConfig config;
  config.rules.push_back({"/apollo/lidar", 2.0});
  ChannelRateFilter filter(config);

  EXPECT_TRUE(filter.Evaluate("/apollo/lidar", 100).should_record);

  const auto throttled = filter.Evaluate("/apollo/lidar", 200);
  EXPECT_FALSE(throttled.should_record);
  EXPECT_TRUE(throttled.throttled_by_rate);

  EXPECT_TRUE(filter.Evaluate("/apollo/lidar", 600000000).should_record);
}

TEST(ChannelRateFilterTest, KeepsChannelsIndependent) {
  ChannelRateFilterConfig config;
  config.rules.push_back({"/apollo/lidar", 1.0});
  config.rules.push_back({"/apollo/camera", 10.0});
  ChannelRateFilter filter(config);

  EXPECT_TRUE(filter.Evaluate("/apollo/lidar", 100).should_record);
  EXPECT_TRUE(filter.Evaluate("/apollo/camera", 100).should_record);
  EXPECT_FALSE(filter.Evaluate("/apollo/lidar", 500000000).should_record);
  EXPECT_TRUE(filter.Evaluate("/apollo/camera", 200000000).should_record);
}

}  // namespace
}  // namespace record
}  // namespace cyber
}  // namespace apollo
