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

//  Created Date: 2025-12-07
//  Author: daohu527

#include "modules/planning/scenarios/deciders/escape_decider.h"

#include <cmath>

#include "cyber/time/clock.h"

namespace apollo {
namespace planning {
namespace scenario {

// TODO(zero): need complete!
constexpr double kStuckSpeedThreshold = 0.1;   // m/s
constexpr double kStuckTimeThreshold = 10.0;   // seconds
constexpr double kValidStopRange = 5.0;        // meters
constexpr double kDestinationThreshold = 2.0;  // meters

// Constants for Decision Score
constexpr uint32_t kScoreEscapeSticky = 100;  // Highest priority
constexpr uint32_t kScoreEscapeTrigger = 95;  // Very high priority

ScenarioDecisionResult EscapeDecider::MakeDecision(
    const DeciderContext& context) {
  // 1. Sticky Strategy
  // If the current scenario is already ESCAPE, we must maintain this state
  // until the scenario itself decides to finish (e.g., maneuver complete).
  // This prevents the planner from switching back to LaneFollow while
  // the car is backing up (non-zero speed).
  const auto current_scenario = context.current_scenario;
  if (current_scenario->Type() == ScenarioType::ESCAPE) {
    if (current_scenario->GetStatus() !=
        Scenario::ScenarioStatus::STATUS_DONE) {
      return ScenarioDecisionResult(ScenarioType::ESCAPE,
                                    ScenarioGrade::MANEUVER, kScoreEscapeSticky,
                                    "Escape maneuver in progress (Sticky)");
    }
  }

  // 2. Stuck Detection
  // Only check if we are not already escaping.
  if (IsStuck(context)) {
    return ScenarioDecisionResult(ScenarioType::ESCAPE, ScenarioGrade::MANEUVER,
                                  kScoreEscapeTrigger,
                                  "Vehicle stuck detected (> 10s)");
  }

  // 3. Fallback
  return ScenarioDecisionResult();
}

bool EscapeDecider::IsStuck(const DeciderContext& context) {
  const auto& frame = context.frame;

  // A. Check Vehicle Speed
  const auto& vehicle_state = frame->vehicle_state();
  double speed = std::abs(vehicle_state.linear_velocity());

  // B. Get Persistent Status from Injector
  auto* blocking_status = injector_->history()->mutable_blocking_status();

  // C. Analyze Stop Reason
  bool is_stopped = (speed < kStuckSpeedThreshold);
  bool is_waiting_rule = IsWaitingForTrafficRule(context);
  bool is_at_destination = IsCloseToDestination(frame);

  // D. Timer Logic
  // We consider it "Stuck" ONLY if:
  // 1. Physically stopped
  // 2. NOT waiting for a signal/sign
  // 3. NOT at the destination
  if (is_stopped && !is_waiting_rule && !is_at_destination) {
    double now = cyber::Clock::NowInSeconds();

    // Start timer if not started
    if (!blocking_status->has_start_stuck_time()) {
      blocking_status->set_start_stuck_time(now);
    }

    // Check duration
    double stuck_duration = now - blocking_status->start_stuck_time();
    if (stuck_duration > kStuckTimeThreshold) {
      return true;
    }
  } else {
    // Reset timer immediately if the vehicle moves or a valid stop reason
    // appears
    blocking_status->clear_start_stuck_time();
  }

  return false;
}

bool EscapeDecider::IsWaitingForTrafficRule(const DeciderContext& context) {
  const auto& overlaps = context.first_encountered_overlaps;
  const auto& frame = context.frame;

  if (frame->reference_line_info().empty()) {
    return false;
  }

  // Calculate distance from ADC front to the overlap
  const double adc_front_s =
      frame->reference_line_info().front().AdcSlBoundary().end_s();

  auto check_overlap_close = [&](ReferenceLineInfo::OverlapType type) {
    if (overlaps->count(type)) {
      const auto& overlap = overlaps->at(type);
      double distance = overlap.start_s - adc_front_s;
      // Check if the overlap is effectively in front of us
      // Using -1.0 to handle slight overshoot
      if (distance > -1.0 && distance < kValidStopRange) {
        return true;
      }
    }
    return false;
  };

  if (check_overlap_close(ReferenceLineInfo::SIGNAL)) return true;
  if (check_overlap_close(ReferenceLineInfo::STOP_SIGN)) return true;
  if (check_overlap_close(ReferenceLineInfo::YIELD_SIGN)) return true;
  if (check_overlap_close(ReferenceLineInfo::PNC_JUNCTION)) return true;

  return false;
}

bool EscapeDecider::IsCloseToDestination(const Frame* frame) {
  if (frame->reference_line_info().empty()) {
    return false;
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  double dist_to_end = reference_line_info.SDistanceToDestination();

  if (dist_to_end < kDestinationThreshold) {
    return true;
  }
  return false;
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
