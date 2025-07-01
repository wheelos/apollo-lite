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

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace gnss {

TEST(DataBufferTest, ConstructorDefaultCapacity) {
  DataBuffer buffer;
  EXPECT_EQ(buffer.Capacity(), 1024);
  EXPECT_EQ(buffer.Size(), 0);
  EXPECT_TRUE(buffer.IsEmpty());
}

TEST(DataBufferTest, ConstructorCustomCapacity) {
  DataBuffer buffer(512);
  EXPECT_EQ(buffer.Capacity(), 512);
  EXPECT_EQ(buffer.Size(), 0);
  EXPECT_TRUE(buffer.IsEmpty());
}

TEST(DataBufferTest, AppendString) {
  DataBuffer buffer(10);
  EXPECT_TRUE(buffer.Append("hello"));
  EXPECT_EQ(buffer.Size(), 5);
  EXPECT_FALSE(buffer.IsEmpty());
  EXPECT_TRUE(buffer.Append(" world"));
  EXPECT_EQ(buffer.Size(), 10);
  EXPECT_FALSE(buffer.Append("!"));  // Capacity reached
  EXPECT_EQ(buffer.Size(), 10);
}

TEST(DataBufferTest, Capacity) {
  DataBuffer buffer1;
  EXPECT_EQ(buffer1.Capacity(), 1024);
  DataBuffer buffer2(256);
  EXPECT_EQ(buffer2.Capacity(), 256);
}

TEST(DataBufferTest, IsEmpty) {
  DataBuffer buffer;
  EXPECT_TRUE(buffer.IsEmpty());
  buffer.Append("data");
  EXPECT_FALSE(buffer.IsEmpty());
  buffer.Read(new uint8_t[4], 4);
  EXPECT_TRUE(buffer.IsEmpty());
}

TEST(DataBufferTest, CanAppend) {
  DataBuffer buffer(10);
  EXPECT_TRUE(buffer.CanAppend(5));
  EXPECT_TRUE(buffer.CanAppend(10));
  buffer.Append("abcdef");
  EXPECT_TRUE(buffer.CanAppend(4));
  EXPECT_FALSE(buffer.CanAppend(5));
}

TEST(DataBufferTest, FetchSuccess) {
  DataBuffer buffer;
  buffer.Append("abcdef");
  uint8_t output[3];
  EXPECT_TRUE(buffer.Read(output, 3));
  EXPECT_EQ(std::string(output, output + 3), "abc");
  EXPECT_EQ(buffer.Size(), 3);
  uint8_t remaining_output[3];
  EXPECT_TRUE(buffer.Read(remaining_output, 3));
  EXPECT_EQ(std::string(remaining_output, remaining_output + 3), "def");
  EXPECT_TRUE(buffer.IsEmpty());
}

TEST(DataBufferTest, FetchNotEnoughData) {
  DataBuffer buffer;
  buffer.Append("abc");
  uint8_t output[4];
  EXPECT_FALSE(buffer.Read(output, 4));
  EXPECT_EQ(buffer.Size(), 3);
}

TEST(DataBufferTest, FetchEmptyBuffer) {
  DataBuffer buffer;
  uint8_t output[1];
  EXPECT_FALSE(buffer.Read(output, 1));
  EXPECT_TRUE(buffer.IsEmpty());
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
