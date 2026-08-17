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

#include "modules/drivers/lidar/processor/safety/degrade_policy.h"

namespace apollo {
namespace drivers {
namespace lidar {

void DegradePolicy::SetConfig(bool degrade_on_ts_anomaly,
                              uint32_t ts_max_consecutive_errors) {
  degrade_on_ts_anomaly_ = degrade_on_ts_anomaly;
  ts_max_consecutive_errors_ = ts_max_consecutive_errors;
}

DegradeEvent DegradePolicy::OnTsAnomaly(int consecutive_errors) {
  const DegradeMode prev = mode_;
  ok_frames_since_last_error_ = 0;

  if (!degrade_on_ts_anomaly_) {
    return MakeEvent(prev, mode_, DegradeReason::kNone);
  }

  if (consecutive_errors >= static_cast<int>(ts_max_consecutive_errors_)) {
    mode_ = DegradeMode::kSingleLidar;
  }

  return MakeEvent(prev, mode_, DegradeReason::kTsAnomaly);
}

DegradeEvent DegradePolicy::OnFrameOk() {
  const DegradeMode prev = mode_;

  if (mode_ == DegradeMode::kNormal) {
    ok_frames_since_last_error_ = 0;
    return MakeEvent(prev, mode_, DegradeReason::kNone);
  }

  ++ok_frames_since_last_error_;
  if (ok_frames_since_last_error_ >= kRecoveryOkFrames) {
    ok_frames_since_last_error_ = 0;
    if (mode_ == DegradeMode::kSingleLidar) {
      mode_ = DegradeMode::kNormal;
    }
  }

  return MakeEvent(prev, mode_, DegradeReason::kNone);
}

DegradeMode DegradePolicy::CurrentMode() const { return mode_; }

DegradeEvent DegradePolicy::Reset() {
  const DegradeMode prev = mode_;
  mode_ = DegradeMode::kNormal;
  ok_frames_since_last_error_ = 0;
  return MakeEvent(prev, mode_, DegradeReason::kManualReset);
}

DegradeEvent DegradePolicy::MakeEvent(DegradeMode prev, DegradeMode next,
                                      DegradeReason r) {
  return DegradeEvent{prev, next, r, prev != next};
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
