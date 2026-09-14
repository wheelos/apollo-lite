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

#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>

#include "modules/localization_health/proto/localization_health.pb.h"
#include "wheelos_msgs/localization_msgs/localization.pb.h"

#include "modules/localization_health/capability_deriver.h"
#include "modules/localization_health/health_state_machine.h"
#include "modules/localization_health/independent_checker.h"
#include "modules/localization_health/reason_aggregator.h"

namespace apollo {
namespace localization {

/**
 * @class LocalizationHealth
 * @brief Independent health assessment, capability derivation, and semantic
 * state machine for vehicle localization.
 *
 * Architecture:
 * 1. IndependentChecker: Low-complexity time, numerical, and kinematic checks.
 * 2. CapabilityDeriver: Derives capability flags, C_min, and C_nominal.
 * 3. ReasonAggregator: Merges independent and algorithmic faults, selects
 *                      primary reason, tracks latched reasons.
 * 4. HealthStateMachine: Enforces fail-fast, recover-slow state transitions
 *                        and emits transition events.
 */
class LocalizationHealth {
 public:
  LocalizationHealth();
  explicit LocalizationHealth(const LocalizationHealthConfig& config);
  ~LocalizationHealth() = default;

  void Init(const LocalizationHealthConfig& config);
  void ResetSession(const std::string& session_id);

  // Ingest raw assessment from estimator core
  void UpdateAssessment(const LocalizationAssessment& assessment,
                        double receive_time_sec = 0.0);

  // Ingest high-frequency pose estimate
  void UpdatePose(const LocalizationEstimate& pose,
                  double receive_time_sec = 0.0);

  // Evaluate independent checks, derive capabilities, advance state machine,
  // and generate periodic status snapshot.
  LocalizationHealthStatus Evaluate(double now_sec = 0.0);

  // Check and consume state transition event (if state/phase changed)
  bool PopTransitionEvent(LocalizationHealthEvent* event);

  // Read-only state accessors
  AvailabilityState current_state() const;
  RecoveryPhase current_recovery_phase() const;
  uint64_t current_capabilities() const;
  uint64_t active_reasons() const;
  uint64_t latched_reasons() const;
  HealthReason primary_reason() const;
  uint64_t transition_id() const;
  const LocalizationHealthConfig& config() const;

  // Static utility methods for capability bits
  static bool HasCapability(uint64_t capabilities, Capability cap) {
    return (capabilities & static_cast<uint64_t>(cap)) != 0;
  }
  static void SetCapability(uint64_t* capabilities, Capability cap) {
    *capabilities |= static_cast<uint64_t>(cap);
  }
  static void ClearCapability(uint64_t* capabilities, Capability cap) {
    *capabilities &= ~static_cast<uint64_t>(cap);
  }

  // Reason bitmask mapping helpers (forwarded to ReasonAggregator)
  static int ReasonToBitIndex(HealthReason reason) {
    return ReasonAggregator::ReasonToBitIndex(reason);
  }
  static HealthReason BitIndexToReason(int bit_index) {
    return ReasonAggregator::BitIndexToReason(bit_index);
  }
  static uint64_t ReasonToBit(HealthReason reason) {
    return ReasonAggregator::ReasonToBit(reason);
  }
  static bool HasReason(uint64_t reason_mask, HealthReason reason) {
    return ReasonAggregator::HasReason(reason_mask, reason);
  }
  static void AddReason(uint64_t* reason_mask, HealthReason reason) {
    ReasonAggregator::AddReason(reason_mask, reason);
  }

 private:
  void PopulateStatus(double now_sec, LocalizationHealthStatus* status);

 private:
  LocalizationHealthConfig config_;
  mutable std::mutex mutex_;

  // Modular subcomponents
  IndependentChecker checker_;
  CapabilityDeriver capability_deriver_;
  ReasonAggregator reason_aggregator_;
  HealthStateMachine state_machine_;

  // Session & sequence
  std::string current_session_id_;
  uint64_t status_sequence_ = 0;
  uint64_t current_capabilities_ = 0;

  // Cached inputs
  bool has_assessment_ = false;
  LocalizationAssessment latest_assessment_;
  double latest_assessment_receive_time_sec_ = 0.0;

  bool has_pose_ = false;
  LocalizationEstimate latest_pose_;
  double latest_pose_receive_time_sec_ = 0.0;

  // History for continuity & kinematic checks
  bool has_prev_pose_ = false;
  LocalizationEstimate prev_pose_;
  double prev_pose_time_sec_ = 0.0;
};

}  // namespace localization
}  // namespace apollo
