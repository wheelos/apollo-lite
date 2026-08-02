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

// Processing degrade levels (ordered by severity).
enum class DegradeMode {
  kNormal = 0,       // All sensors online, full fusion pipeline.
  kSingleLidar = 1,  // Auxiliary sensors dropped; primary-only fusion.
};

// Reason codes for degrade transitions.
enum class DegradeReason {
  kNone = 0,
  kTsAnomaly = 1,    // Timestamp sanity error threshold exceeded.
  kManualReset = 2,  // Explicit reset via Reset().
};

struct DegradeEvent {
  DegradeMode previous_mode;
  DegradeMode new_mode;
  DegradeReason reason;
  bool mode_changed;
};

// State-machine that escalates / recovers degrade level based on error signals.
// Not thread-safe by itself; callers must serialise from the component thread.
class DegradePolicy {
 public:
  void SetConfig(bool degrade_on_ts_anomaly,
                 uint32_t ts_max_consecutive_errors);

  // Called when TsSanity reports a non-OK result.
  DegradeEvent OnTsAnomaly(int consecutive_errors);

  // Called after a fully successful processing cycle.
  // Allows recovery: repeated successes lower the degrade level.
  DegradeEvent OnFrameOk();

  DegradeMode CurrentMode() const;

  // Hard-reset to kNormal and clear recovery counters.
  DegradeEvent Reset();

 private:
  DegradeMode mode_ = DegradeMode::kNormal;

  bool degrade_on_ts_anomaly_ = true;
  uint32_t ts_max_consecutive_errors_ = 3;

  // Consecutive successful frames needed to step mode down one level.
  static constexpr uint32_t kRecoveryOkFrames = 10;
  uint32_t ok_frames_since_last_error_ = 0;

  DegradeEvent MakeEvent(DegradeMode prev, DegradeMode next, DegradeReason r);
};

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
