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

#include <string>

#include "modules/localization_health/proto/localization_health.pb.h"

namespace apollo {
namespace localization {

/**
 * @class HealthStateMachine
 * @brief Manages availability state transitions adhering to the fail-fast,
 * recover-slow safety principle with hysteresis and event generation.
 */
class HealthStateMachine {
 public:
  HealthStateMachine();
  ~HealthStateMachine() = default;

  void Reset();
  void ResetSession(const std::string& session_id, double now_sec);

  // Determine desired state according to safety criteria and active reasons
  AvailabilityState DetermineDesiredState(bool has_assessment,
                                          bool estimator_running,
                                          bool estimator_converged,
                                          bool has_pose, bool c_min_met,
                                          bool c_nominal_met,
                                          uint64_t active_reasons);

  // Step state machine with hysteresis / debouncing
  void Update(AvailabilityState desired_state, RecoveryPhase recovery_phase,
              double now_sec, const std::string& session_id,
              HealthReason primary_reason, uint64_t active_reasons,
              const LocalizationHealthConfig& config);

  AvailabilityState current_state() const { return current_state_; }
  RecoveryPhase current_recovery_phase() const {
    return current_recovery_phase_;
  }
  uint64_t transition_id() const { return transition_id_; }

  bool PopPendingEvent(LocalizationHealthEvent* event);

  static int StateSeverity(AvailabilityState state);

 private:
  AvailabilityState current_state_ = AVAILABILITY_UNKNOWN;
  RecoveryPhase current_recovery_phase_ = RECOVERY_IDLE;
  uint64_t transition_id_ = 0;

  // Hysteresis & Debouncing
  double state_worsen_start_time_sec_ = 0.0;
  AvailabilityState debounce_target_state_ = AVAILABILITY_UNKNOWN;
  double recovery_start_time_sec_ = 0.0;
  AvailabilityState recovery_target_state_ = AVAILABILITY_UNKNOWN;

  // Event generation
  LocalizationHealthEvent pending_event_;
  bool has_pending_event_ = false;
};

}  // namespace localization
}  // namespace apollo
