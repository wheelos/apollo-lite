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

#pragma once

#include <cstdint>

namespace apollo {
namespace drivers {
namespace lidar {

enum class TsSanityStatus {
  kOk = 0,
  kFirstFrame = 1,        // no previous timestamp to compare
  kIntervalTooShort = 2,  // frames arriving faster than min_interval_ms
  kIntervalTooLong = 3,   // gap exceeds max_interval_ms (late / dropped)
  kJump = 4,              // |delta| > max_jump_ms (clock reset / backward jump)
};

struct TsSanityResult {
  TsSanityStatus status = TsSanityStatus::kFirstFrame;
  double interval_ms = 0.0;    // |current - last| in milliseconds
  int consecutive_errors = 0;  // rolling count, reset on kOk
};

// Stateful per-sensor timestamp sanity checker.
// Thread-safe: external locking expected (one LidarUnifiedComponent per Proc).
class TsSanity {
 public:
  // Configure thresholds (milliseconds).
  void SetConfig(uint32_t min_interval_ms, uint32_t max_interval_ms,
                 uint32_t max_jump_ms);

  // Check a new frame timestamp (seconds). Updates internal state.
  TsSanityResult Check(double timestamp_seconds);

  // Reset to initial state (e.g. after degrade recovery).
  void Reset();

 private:
  double last_timestamp_s_ = -1.0;
  int consecutive_errors_ = 0;

  uint32_t min_interval_ms_ = 40;
  uint32_t max_interval_ms_ = 200;
  uint32_t max_jump_ms_ = 500;
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
