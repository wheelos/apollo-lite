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

#ifndef CYBER_COMMON_TIME_CONVERSION_H_
#define CYBER_COMMON_TIME_CONVERSION_H_

#include <cstdint>
#include <string>

#include "absl/time/time.h"

namespace apollo {
namespace cyber {
namespace common {

/**
 * @brief Converts a time string to Unix seconds.
 * Refactored to use absl::Time for thread-safety and UTC consistency.
 */
inline uint64_t StringToUnixSeconds(
    const std::string& time_str,
    const std::string& format_str = "%Y-%m-%d %H:%M:%S") {
  absl::Time t;
  std::string err;

  // Industrial Practice: Use UTCTimeZone to ensure identical results across
  // different vehicles and cloud environments.
  if (absl::ParseTime(format_str, time_str, absl::UTCTimeZone(), &t, &err)) {
    return static_cast<uint64_t>(absl::ToUnixSeconds(t));
  }

  return 0;
}

/**
 * @brief Converts Unix seconds to a formatted string.
 * Refactored to eliminate manual buffer management and C-style global locks.
 */
inline std::string UnixSecondsToString(
    uint64_t unix_seconds,
    const std::string& format_str = "%Y-%m-%d-%H:%M:%S") {
  const absl::Time t =
      absl::FromUnixSeconds(static_cast<int64_t>(unix_seconds));

  // UTCTimeZone avoids the "8-hour offset" bug when analyzing data across
  // regions.
  return absl::FormatTime(format_str, t, absl::UTCTimeZone());
}

}  // namespace common
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_COMMON_TIME_CONVERSION_H_
