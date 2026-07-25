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

//  Created Date: 2026-04-27
//  Author: daohu527


#include "cyber/tools/cyber_recorder/record/filters/message_size_filter.h"

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace record {

TEST(MessageSizeFilterTest, ValidatesConfigRelationships) {
  MessageSizeFilterConfig config;
  std::string error;
  config.drop_message_size_bytes = 2048;
  EXPECT_TRUE(ValidateMessageSizeFilterConfig(config, &error));
  EXPECT_TRUE(error.empty());
}

TEST(MessageSizeFilterTest, ParsesHumanReadableMessageSizes) {
  uint64_t value = 0;
  std::string error;

  EXPECT_TRUE(ParseMessageSizeBytes("256", &value, &error));
  EXPECT_EQ(value, 256ULL);

  EXPECT_TRUE(ParseMessageSizeBytes("256k", &value, &error));
  EXPECT_EQ(value, 256ULL * 1024ULL);

  EXPECT_TRUE(ParseMessageSizeBytes("1MiB", &value, &error));
  EXPECT_EQ(value, 1024ULL * 1024ULL);

  EXPECT_TRUE(ParseMessageSizeBytes("2gb", &value, &error));
  EXPECT_EQ(value, 2ULL * 1024ULL * 1024ULL * 1024ULL);

  EXPECT_FALSE(ParseMessageSizeBytes("12x", &value, &error));
  EXPECT_FALSE(error.empty());
}

TEST(MessageSizeFilterTest, ParsesCompactPolicyString) {
  MessageSizeFilterConfig config;
  std::string error;

  EXPECT_TRUE(ParseMessageSizeFilterPolicy("drop=1m", &config, &error));
  EXPECT_EQ(config.drop_message_size_bytes, 1024ULL * 1024ULL);
}

TEST(MessageSizeFilterTest, RejectsInvalidPolicyString) {
  MessageSizeFilterConfig config;
  std::string error;

  EXPECT_FALSE(ParseMessageSizeFilterPolicy("filter=1m", &config, &error));
  EXPECT_FALSE(error.empty());

  EXPECT_FALSE(ParseMessageSizeFilterPolicy("throttle=256k@2", &config, &error));
  EXPECT_FALSE(error.empty());
}

TEST(MessageSizeFilterTest, DropsOversizedMessages) {
  MessageSizeFilterConfig config;
  config.drop_message_size_bytes = 2048;
  MessageSizeFilter filter(config);
  const auto decision = filter.Evaluate("/apollo/test", 2048, 1000);
  EXPECT_FALSE(decision.should_record);
  EXPECT_TRUE(decision.dropped_by_size);
}

TEST(MessageSizeFilterTest, SmallMessagesAlwaysPass) {
  MessageSizeFilterConfig config;
  config.drop_message_size_bytes = 4096;
  MessageSizeFilter filter(config);

  EXPECT_TRUE(filter.Evaluate("/apollo/test", 1024, 1000).should_record);
  EXPECT_TRUE(filter.Evaluate("/apollo/test", 1024, 1001).should_record);
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
