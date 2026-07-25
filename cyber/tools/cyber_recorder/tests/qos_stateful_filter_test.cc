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

#include "cyber/tools/cyber_recorder/record/filters/qos_stateful_filter.h"

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace record {
namespace {

TEST(QosStatefulFilterTest, AppliesRateModeWhenConfigured) {
  QosStatefulFilter filter;

  TopicPolicy policy;
  policy.drop_message_size_bytes = 100;
  policy.max_rate_hz = 5.0;
  policy.max_bandwidth_bytes_per_sec = 0;
  policy.compression = "none";

  const uint64_t kStartNs = 1000000000ULL;
  const auto first = filter.Evaluate("/camera/front", 128, kStartNs, policy);
  EXPECT_TRUE(first.should_record);
  EXPECT_FALSE(first.dropped_by_size);

  const auto second =
      filter.Evaluate("/camera/front", 64, kStartNs + 50000000ULL, policy);
  EXPECT_FALSE(second.should_record);
  EXPECT_TRUE(second.throttled_by_rate);
}

TEST(QosStatefulFilterTest, AppliesDropModeWhenRateDisabled) {
  QosStatefulFilter filter;

  TopicPolicy policy;
  policy.drop_message_size_bytes = 100;
  policy.max_rate_hz = 0.0;

  const auto dropped = filter.Evaluate("/camera/front", 128, 1000000000ULL, policy);
  EXPECT_FALSE(dropped.should_record);
  EXPECT_TRUE(dropped.dropped_by_size);
}

}  // namespace
}  // namespace record
}  // namespace cyber
}  // namespace apollo
