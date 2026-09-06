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

#include "modules/localization_health/health_state_machine.h"

#include <string>

#include "modules/localization_health/reason_aggregator.h"

namespace apollo {
namespace localization {

HealthStateMachine::HealthStateMachine() = default;

int HealthStateMachine::StateSeverity(AvailabilityState state) {
  switch (state) {
    case AVAILABILITY_UNKNOWN:
      return 4;
    case AVAILABILITY_INVALID:
      return 3;
    case AVAILABILITY_INITIALIZING:
      return 2;
    case AVAILABILITY_DEGRADED:
      return 1;
    case AVAILABILITY_NOMINAL:
      return 0;
    default:
      return 5;
  }
}

void HealthStateMachine::Reset() {
  current_state_ = AVAILABILITY_UNKNOWN;
  current_recovery_phase_ = RECOVERY_IDLE;
  state_worsen_start_time_sec_ = 0.0;
  debounce_target_state_ = AVAILABILITY_UNKNOWN;
  recovery_start_time_sec_ = 0.0;
  recovery_target_state_ = AVAILABILITY_UNKNOWN;
  has_pending_event_ = false;
}

void HealthStateMachine::ResetSession(const std::string& session_id,
                                      double now_sec) {
  AvailabilityState old_state = current_state_;
  current_state_ = AVAILABILITY_INITIALIZING;
  current_recovery_phase_ = RECOVERY_IDLE;

  state_worsen_start_time_sec_ = 0.0;
  debounce_target_state_ = AVAILABILITY_UNKNOWN;
  recovery_start_time_sec_ = 0.0;
  recovery_target_state_ = AVAILABILITY_UNKNOWN;

  transition_id_++;
  pending_event_.mutable_header()->set_timestamp_sec(now_sec);
  pending_event_.set_stamp(now_sec);
  pending_event_.set_session_id(session_id);
  pending_event_.set_transition_id(transition_id_);
  pending_event_.set_previous_state(old_state);
  pending_event_.set_current_state(current_state_);
  pending_event_.set_recovery_phase(current_recovery_phase_);
  pending_event_.set_primary_reason(REASON_NONE);
  pending_event_.set_active_reasons(0);
  has_pending_event_ = true;
}

AvailabilityState HealthStateMachine::DetermineDesiredState(
    bool has_assessment, bool estimator_running, bool estimator_converged,
    bool has_pose, bool c_min_met, bool c_nominal_met,
    uint64_t active_reasons) {
  if (!has_assessment || !estimator_running || !has_pose) {
    return AVAILABILITY_INITIALIZING;
  }

  if (!estimator_converged) {
    return AVAILABILITY_INITIALIZING;
  }

  // Hard faults force INVALID
  for (int i = 0; i < 33; ++i) {
    HealthReason reason = ReasonAggregator::BitIndexToReason(i);
    if (ReasonAggregator::IsHardFault(reason) &&
        ReasonAggregator::HasReason(active_reasons, reason)) {
      return AVAILABILITY_INVALID;
    }
  }

  // Loss of minimum safe local capability
  if (!c_min_met) {
    return AVAILABILITY_INVALID;
  }

  // Full nominal capabilities met
  if (c_nominal_met) {
    return AVAILABILITY_NOMINAL;
  }

  // Safe for degraded operation
  return AVAILABILITY_DEGRADED;
}

void HealthStateMachine::Update(AvailabilityState desired_state,
                                RecoveryPhase recovery_phase, double now_sec,
                                const std::string& session_id,
                                HealthReason primary_reason,
                                uint64_t active_reasons,
                                const LocalizationHealthConfig& config) {
  int cur_sev = StateSeverity(current_state_);
  int des_sev = StateSeverity(desired_state);

  AvailabilityState old_state = current_state_;

  if (des_sev > cur_sev) {
    // Fail-fast transition to worse state
    recovery_start_time_sec_ = 0.0;
    recovery_target_state_ = AVAILABILITY_UNKNOWN;

    bool immediate = false;
    if (desired_state == AVAILABILITY_INVALID) {
      for (int i = 0; i < 33; ++i) {
        HealthReason r = ReasonAggregator::BitIndexToReason(i);
        if (ReasonAggregator::IsHardFault(r) &&
            ReasonAggregator::HasReason(active_reasons, r)) {
          immediate = true;
          break;
        }
      }
    }

    if (immediate) {
      current_state_ = desired_state;
      state_worsen_start_time_sec_ = 0.0;
      debounce_target_state_ = AVAILABILITY_UNKNOWN;
    } else {
      double debounce_duration = (desired_state == AVAILABILITY_INVALID)
                                     ? config.failure_debounce_duration()
                                     : config.degraded_debounce_duration();
      if (debounce_target_state_ != desired_state) {
        debounce_target_state_ = desired_state;
        state_worsen_start_time_sec_ = now_sec;
      } else if (now_sec - state_worsen_start_time_sec_ >= debounce_duration) {
        current_state_ = desired_state;
        state_worsen_start_time_sec_ = 0.0;
        debounce_target_state_ = AVAILABILITY_UNKNOWN;
      }
    }
  } else if (des_sev < cur_sev) {
    // Recover-slow transition to better state with hysteresis
    state_worsen_start_time_sec_ = 0.0;
    debounce_target_state_ = AVAILABILITY_UNKNOWN;

    if (recovery_target_state_ != desired_state) {
      recovery_target_state_ = desired_state;
      recovery_start_time_sec_ = now_sec;
    } else if (now_sec - recovery_start_time_sec_ >=
               config.recovery_duration_threshold()) {
      current_state_ = desired_state;
      recovery_start_time_sec_ = 0.0;
      recovery_target_state_ = AVAILABILITY_UNKNOWN;
    }
  } else {
    // Stable state
    state_worsen_start_time_sec_ = 0.0;
    debounce_target_state_ = AVAILABILITY_UNKNOWN;
    recovery_start_time_sec_ = 0.0;
    recovery_target_state_ = AVAILABILITY_UNKNOWN;
  }

  RecoveryPhase old_phase = current_recovery_phase_;
  current_recovery_phase_ = recovery_phase;

  // Trigger event on state or recovery phase change
  if (old_state != current_state_ || old_phase != current_recovery_phase_) {
    transition_id_++;
    pending_event_.mutable_header()->set_timestamp_sec(now_sec);
    pending_event_.set_stamp(now_sec);
    pending_event_.set_session_id(session_id);
    pending_event_.set_transition_id(transition_id_);
    pending_event_.set_previous_state(old_state);
    pending_event_.set_current_state(current_state_);
    pending_event_.set_recovery_phase(current_recovery_phase_);
    pending_event_.set_primary_reason(primary_reason);
    pending_event_.set_active_reasons(active_reasons);
    has_pending_event_ = true;
  }
}

bool HealthStateMachine::PopPendingEvent(LocalizationHealthEvent* event) {
  if (!has_pending_event_ || !event) {
    return false;
  }
  *event = pending_event_;
  has_pending_event_ = false;
  return true;
}

}  // namespace localization
}  // namespace apollo
