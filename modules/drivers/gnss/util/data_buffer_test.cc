// Copyright 2025 WheelOS All Rights Reserved.
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

//  Created Date: 2025-4-15
//  Author: daohu527

#include "modules/drivers/gnss/util/data_buffer.h"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace gnss {

TEST(DataBufferTest, AppendsSearchesAndRetrievesBytes) {
  DataBuffer buffer(8);
  buffer.Append("abc");

  EXPECT_EQ("abc", buffer.Peek());
  const auto match = buffer.Find("bc");
  ASSERT_TRUE(match.has_value());
  EXPECT_EQ(1u, *match);
  EXPECT_FALSE(buffer.Find(""));
  const auto first = buffer.Poll();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ('a', *first);
  EXPECT_EQ((std::vector<uint8_t>{'b', 'c'}), buffer.RetrieveAsVector(2));
  EXPECT_TRUE(buffer.IsEmpty());
  EXPECT_FALSE(buffer.Poll().has_value());
}

TEST(DataBufferTest, PreservesUnreadBytesWhenCompactingAndGrowing) {
  DataBuffer buffer(8);
  buffer.Append("abcdefgh");
  buffer.Drain(4);
  buffer.Append("ijk");
  EXPECT_EQ("efghijk", buffer.Peek());

  buffer.Append("lmnopqrst");
  EXPECT_EQ("efghijklmnopqrst", buffer.Peek());
}

TEST(DataBufferTest, RejectsOverreadWithoutChangingData) {
  DataBuffer buffer;
  buffer.Append("abc");

  EXPECT_THROW(buffer.RetrieveAsVector(4), std::out_of_range);
  EXPECT_EQ("abc", buffer.Peek());
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
