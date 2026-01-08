// Copyright 2025 WheelOS. All Rights Reserved.
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

//  Created Date: 2026-01-03
//  Author: daohu527

#pragma once

#include <cstdint>

namespace apollo {
namespace control {

/**
 * @class CounterDebouncer
 * @brief A counter-based signal debouncer.
 * It confirms a fault only when the internal counter reaches a specific
 * threshold.
 */
class CounterDebouncer {
 public:
  /**
   * @brief Constructor
   * @param threshold The number of consecutive fault detections required to
   * confirm a fault.
   */
  explicit CounterDebouncer(uint32_t threshold)
      : threshold_(threshold), count_(0) {}

  /**
   * @brief Update the debouncer state with a new sample.
   * @param is_fault Current status of the monitored signal.
   * @return True if the fault is confirmed (counter >= threshold).
   */
  bool Update(bool is_fault) {
    if (is_fault) {
      if (count_ < threshold_) {
        count_++;
      }
    } else {
      // Immediate reset strategy: resets the counter to zero as soon as a
      // normal signal is received. This ensures high confidence for fault
      // recovery.
      count_ = 0;
    }
    return IsActive();
  }

  /**
   * @brief Reset the internal counter to zero.
   */
  void Reset() { count_ = 0; }

  /**
   * @brief Check if the fault is currently active/confirmed.
   * @return True if the counter has reached the threshold.
   */
  bool IsActive() const { return count_ >= threshold_ && threshold_ > 0; }

  /**
   * @brief Get the current counter value (useful for telemetry).
   */
  uint32_t count() const { return count_; }

 private:
  uint32_t threshold_;
  uint32_t count_;
};

}  // namespace control
}  // namespace apollo
