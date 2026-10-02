// Copyright 2026 WheelOS. All Rights Reserved.
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

//  Created Date: 2026-09-06
//  Author: daohu527

#pragma once

#include <cstdint>

#include "modules/localization_health/proto/localization_health.pb.h"

namespace apollo {
namespace localization {

/**
 * @class ReasonAggregator
 * @brief Aggregates diagnostic reasons, manages active/latched bitmasks,
 * and determines the highest-priority primary reason.
 */
class ReasonAggregator {
 public:
  ReasonAggregator() = default;
  ~ReasonAggregator() = default;

  void Reset();

  // Update active reasons from independent checks and estimator faults,
  // and update session latched reasons.
  void Update(uint64_t independent_reasons, uint64_t algorithm_faults);

  uint64_t active_reasons() const { return active_reasons_; }
  uint64_t latched_reasons() const { return latched_reasons_; }
  HealthReason primary_reason() const { return primary_reason_; }

  static int ReasonToBitIndex(HealthReason reason);
  static HealthReason BitIndexToReason(int bit_index);
  static uint64_t ReasonToBit(HealthReason reason);
  static bool HasReason(uint64_t reason_mask, HealthReason reason);
  static void AddReason(uint64_t* reason_mask, HealthReason reason);

  static bool IsHardFault(HealthReason reason);
  static HealthReason SelectPrimaryReason(uint64_t active_reasons);

 private:
  uint64_t active_reasons_ = 0;
  uint64_t latched_reasons_ = 0;
  HealthReason primary_reason_ = REASON_NONE;
};

}  // namespace localization
}  // namespace apollo
