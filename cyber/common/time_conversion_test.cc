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

//  Created Date: 2025-01-03
//  Author: daohu527

#include "cyber/common/time_conversion.h"

#include <gtest/gtest.h>

namespace apollo {
namespace cyber {
namespace common {

/**
 * @test Verify string to unix seconds conversion using UTC.
 */
TEST(TimeConversionTest, StringToUnixSeconds_UTC) {
  // Test time: 2026-01-03 12:00:00 UTC
  std::string time_str = "2026-01-03 12:00:00";
  uint64_t expected_unix = 1767441600;

  // Even if the system clock is set to Beijing or Los Angeles time,
  // this must return the same UTC timestamp.
  EXPECT_EQ(StringToUnixSeconds(time_str), expected_unix);
}

/**
 * @test Verify unix seconds to string conversion using UTC.
 */
TEST(TimeConversionTest, UnixSecondsToString_UTC) {
  uint64_t unix_seconds = 1767441600;
  std::string expected_str = "2026-01-03-12:00:00";

  EXPECT_EQ(UnixSecondsToString(unix_seconds), expected_str);
}

/**
 * @test Verify custom format support.
 */
TEST(TimeConversionTest, CustomFormat) {
  std::string time_str = "03/01/2026 12-00-00";
  std::string format = "%d/%m/%Y %H-%M-%S";
  uint64_t expected_unix = 1767441600;

  EXPECT_EQ(StringToUnixSeconds(time_str, format), expected_unix);
  EXPECT_EQ(UnixSecondsToString(expected_unix, "%d-%m-%Y"), "03-01-2026");
}

/**
 * @test Verify handling of invalid inputs.
 */
TEST(TimeConversionTest, InvalidInputs) {
  // Random text
  EXPECT_EQ(StringToUnixSeconds("apollo-auto"), 0);
  // Logical error in date
  EXPECT_EQ(StringToUnixSeconds("2026-02-30 12:00:00"), 0);
  // Empty string
  EXPECT_EQ(StringToUnixSeconds(""), 0);
}

/**
 * @test Verify 2038+ support (64-bit time).
 */
TEST(TimeConversionTest, Post2038Support) {
  // 2040-05-04 00:00:00 UTC
  std::string time_str = "2040-05-04 00:00:00";

  // The correct Unix timestamp for 2040-05-04 00:00:00 UTC is 2219702400
  uint64_t post_2038_unix = 2219702400;

  EXPECT_EQ(StringToUnixSeconds(time_str), post_2038_unix);

  // Note: Your interface uses '-' as separator for the default output format
  EXPECT_EQ(UnixSecondsToString(post_2038_unix), "2040-05-04-00:00:00");
}

}  // namespace common
}  // namespace cyber
}  // namespace apollo
