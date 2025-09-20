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

#include "modules/drivers/gnss/util/util.h"

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace gnss {

TEST(CommonTest, IsZero) {
  EXPECT_TRUE(is_zero(0));
  EXPECT_TRUE(is_zero(0.0f));
  EXPECT_TRUE(is_zero(0.0));
  EXPECT_TRUE(is_zero(static_cast<int8_t>(0)));
  EXPECT_FALSE(is_zero(1));
  EXPECT_FALSE(is_zero(-1.0f));
  EXPECT_FALSE(is_zero(0.0001f));  // Should be false for standard epsilon
  EXPECT_FALSE(is_zero(-0.0000001));
}

TEST(CommonTest, Crc32Word) {
  EXPECT_EQ(crc32_word(0), 0);
  EXPECT_EQ(crc32_word(1), 0xEDB88320);
  EXPECT_EQ(crc32_word(0xFFFFFFFF), 0xC9E95ECB);
}

TEST(CommonTest, Crc32Block) {
  std::vector<uint8_t> empty_buffer = {};
  EXPECT_EQ(crc32_block(empty_buffer.data(), empty_buffer.size()), 0);

  std::vector<uint8_t> single_byte_buffer = {0x01};
  EXPECT_EQ(crc32_block(single_byte_buffer.data(), single_byte_buffer.size()),
            0xEDB88320);

  std::vector<uint8_t> multi_byte_buffer = {0x12, 0x34, 0x56, 0x78};
  EXPECT_EQ(crc32_block(multi_byte_buffer.data(), multi_byte_buffer.size()),
            0x87D6E60C);
}

TEST(CommonTest, AzimuthDegToYawRad) {
  EXPECT_FLOAT_EQ(azimuth_deg_to_yaw_rad(0.0), M_PI / 2.0);
  EXPECT_FLOAT_EQ(azimuth_deg_to_yaw_rad(90.0), 0.0);
  EXPECT_FLOAT_EQ(azimuth_deg_to_yaw_rad(180.0), -M_PI / 2.0);
  EXPECT_FLOAT_EQ(azimuth_deg_to_yaw_rad(270.0), -M_PI);
  EXPECT_FLOAT_EQ(azimuth_deg_to_yaw_rad(360.0), M_PI / 2.0);
  EXPECT_FLOAT_EQ(azimuth_deg_to_yaw_rad(45.0), M_PI * 0.25);
}

TEST(CommonTest, RfuToFlu) {
  apollo::common::Point3D flu;
  rfu_to_flu(1.0, 2.0, 3.0, &flu);
  EXPECT_FLOAT_EQ(flu.x(), 2.0);
  EXPECT_FLOAT_EQ(flu.y(), -1.0);
  EXPECT_FLOAT_EQ(flu.z(), 3.0);

  rfu_to_flu(-1.0, 0.5, -2.0, &flu);
  EXPECT_FLOAT_EQ(flu.x(), 0.5);
  EXPECT_FLOAT_EQ(flu.y(), 1.0);
  EXPECT_FLOAT_EQ(flu.z(), -2.0);

  rfu_to_flu(0.0, 0.0, 0.0, &flu);
  EXPECT_FLOAT_EQ(flu.x(), 0.0);
  EXPECT_FLOAT_EQ(flu.y(), 0.0);
  EXPECT_FLOAT_EQ(flu.z(), 0.0);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
