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
#include "cyber/tools/cyber_recorder/record/filters/message_size_filter.h"

#include <unistd.h>

#include <cstdio>
#include <string>

#include "gtest/gtest.h"

#include "cyber/record/header_builder.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"

namespace apollo {
namespace cyber {
namespace record {
namespace {

std::string MakeOutputPath(const std::string& suffix) {
  return "/tmp/" + suffix + "_" + std::to_string(::getpid()) + ".record";
}

bool MaybeWriteMessage(const std::string& channel_name,
                       const std::string& payload, uint64_t record_time_ns,
                       MessageSizeFilter* message_size_filter,
                       ChannelRateFilter* channel_rate_filter,
                       RecordWriter* writer) {
  if (!message_size_filter->Evaluate(channel_name, payload.size(), record_time_ns)
           .should_record) {
    return false;
  }
  if (!channel_rate_filter->Evaluate(channel_name, record_time_ns)
           .should_record) {
    return false;
  }
  if (writer->IsNewChannel(channel_name) &&
      !writer->WriteChannel(channel_name, "apollo.cyber.message.RawMessage",
                            "")) {
    return false;
  }
  return writer->WriteMessage(channel_name, payload, record_time_ns);
}

TEST(RecorderFilterIntegrationTest, SizeFilterPrecedesChannelRateLimit) {
  const std::string channel_name =
      "/apollo/test/cyber_recorder/filters_" + std::to_string(::getpid());
  const std::string output_path =
      MakeOutputPath("cyber_recorder_filter_integration");
  std::remove(output_path.c_str());

  MessageSizeFilterConfig message_size_filter_config;
  message_size_filter_config.drop_message_size_bytes = 100;
  MessageSizeFilter message_size_filter(message_size_filter_config);

  ChannelRateFilterConfig channel_rate_filter_config;
  channel_rate_filter_config.rules.push_back({channel_name, 5.0});
  ChannelRateFilter channel_rate_filter(channel_rate_filter_config);

  RecordWriter writer(HeaderBuilder::GetHeaderWithSegmentParams(0, 0));
  ASSERT_TRUE(writer.Open(output_path));

  const uint64_t kStartNs = 1000000000ULL;
  EXPECT_FALSE(MaybeWriteMessage(channel_name, std::string(128, 'x'), kStartNs,
                                 &message_size_filter, &channel_rate_filter,
                                 &writer));
  EXPECT_TRUE(MaybeWriteMessage(channel_name, std::string(64, 'a'),
                                kStartNs + 50000000ULL, &message_size_filter,
                                &channel_rate_filter, &writer));
  EXPECT_FALSE(MaybeWriteMessage(channel_name, std::string(64, 'b'),
                                 kStartNs + 100000000ULL, &message_size_filter,
                                 &channel_rate_filter, &writer));
  EXPECT_TRUE(MaybeWriteMessage(channel_name, std::string(64, 'c'),
                                kStartNs + 350000000ULL, &message_size_filter,
                                &channel_rate_filter, &writer));
  writer.Close();

  RecordReader reader(output_path);
  ASSERT_TRUE(reader.IsValid());
  EXPECT_EQ(2ULL, reader.GetMessageNumber(channel_name));

  RecordMessage message;
  ASSERT_TRUE(reader.ReadMessage(&message));
  EXPECT_EQ(channel_name, message.channel_name);
  EXPECT_EQ(std::string(64, 'a'), message.content);

  ASSERT_TRUE(reader.ReadMessage(&message));
  EXPECT_EQ(std::string(64, 'c'), message.content);

  EXPECT_FALSE(reader.ReadMessage(&message));
  std::remove(output_path.c_str());
}

}  // namespace
}  // namespace record
}  // namespace cyber
}  // namespace apollo
