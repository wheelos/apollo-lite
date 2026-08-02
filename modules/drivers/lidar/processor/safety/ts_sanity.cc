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

#include "modules/drivers/lidar/processor/safety/ts_sanity.h"

#include <cmath>

namespace apollo {
namespace drivers {
namespace lidar {

void TsSanity::SetConfig(uint32_t min_interval_ms, uint32_t max_interval_ms,
                         uint32_t max_jump_ms) {
  min_interval_ms_ = min_interval_ms;
  max_interval_ms_ = max_interval_ms;
  max_jump_ms_ = max_jump_ms;
}

TsSanityResult TsSanity::Check(double timestamp_seconds) {
  TsSanityResult result;

  if (last_timestamp_s_ < 0.0) {
    last_timestamp_s_ = timestamp_seconds;
    result.status = TsSanityStatus::kFirstFrame;
    result.interval_ms = 0.0;
    result.consecutive_errors = 0;
    return result;
  }

  const double delta_s = timestamp_seconds - last_timestamp_s_;
  const double abs_delta_ms = std::fabs(delta_s) * 1000.0;
  result.interval_ms = abs_delta_ms;

  TsSanityStatus status = TsSanityStatus::kOk;

  if (abs_delta_ms > static_cast<double>(max_jump_ms_)) {
    status = TsSanityStatus::kJump;
  } else if (abs_delta_ms > static_cast<double>(max_interval_ms_)) {
    status = TsSanityStatus::kIntervalTooLong;
  } else if (abs_delta_ms < static_cast<double>(min_interval_ms_)) {
    status = TsSanityStatus::kIntervalTooShort;
  }

  if (status == TsSanityStatus::kOk) {
    consecutive_errors_ = 0;
  } else {
    ++consecutive_errors_;
  }

  last_timestamp_s_ = timestamp_seconds;
  result.status = status;
  result.consecutive_errors = consecutive_errors_;
  return result;
}

void TsSanity::Reset() {
  last_timestamp_s_ = -1.0;
  consecutive_errors_ = 0;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
