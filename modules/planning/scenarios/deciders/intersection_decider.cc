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

#include "modules/planning/scenarios/deciders/intersection_decider.h"

#include <cmath>
#include <vector>

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace scenario {

ScenarioDecisionResult IntersectionDecider::MakeDecision(
    const DeciderContext& context) {
  // 1. Check Traffic Light (Priority 1)
  // If Red/Yellow, we MUST handle it.
  auto decision = CheckTrafficLight(context);
  if (decision.IsValid()) {
    return decision;
  }

  // 2. Check Stop Sign (Priority 2)
  decision = CheckStopSign(context);
  if (decision.IsValid()) {
    return decision;
  }

  // 3. Check Yield Sign (Priority 3)
  decision = CheckYieldSign(context);
  if (decision.IsValid()) {
    return decision;
  }

  // 4. Check Bare Intersection (Priority 4)
  // Triggered only if no lights/signs exist AND we don't have Right of Way.
  decision = CheckBareIntersection(context);
  if (decision.IsValid()) {
    return decision;
  }

  return ScenarioDecisionResult();
}

double IntersectionDecider::CalculateDistanceScore(double distance,
                                                   double max_distance,
                                                   double base_score) {
  if (distance < 0 || distance > max_distance) return 0.0;
  // Linear Interpolation: Closer = Higher Score
  // Score Range: [base, base + 0.1]
  double ratio = 1.0 - (distance / max_distance);
  return base_score + (ratio * 0.1);
}

ScenarioDecisionResult IntersectionDecider::CheckTrafficLight(
    const DeciderContext& context) {
  // 0. Load Config
  const auto& config_protected = config_.traffic_light_protected_config();
  double start_range = config_protected.start_traffic_light_scenario_distance();
  uint32_t entry_score = config_protected.scenario_entry_score();

  const auto current_scenario = context.current_scenario;
  auto current_type = current_scenario->Type();

  // 1. Sticky Strategy
  if (current_type == ScenarioType::TRAFFIC_LIGHT_PROTECTED ||
      current_type == ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN ||
      current_type == ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN) {
    if (current_scenario->GetStatus() !=
        Scenario::ScenarioStatus::STATUS_DONE) {
      return ScenarioDecisionResult(current_type, ScenarioGrade::MANEUVER,
                                    entry_score,
                                    "Traffic Light In Progress (Sticky)");
    }
  }

  // 2. Find Signal Overlap
  const auto& overlaps = context.first_encountered_overlaps;
  auto it = overlaps->find(ReferenceLineInfo::SIGNAL);
  if (it == overlaps->end()) {
    return ScenarioDecisionResult();
  }

  const auto& first_signal_overlap = it->second;
  const auto& frame = context.frame;
  if (frame->reference_line_info().empty()) {
    return ScenarioDecisionResult();
  }
  const auto& reference_line_info = frame->reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  // 3. Grouping Logic
  std::vector<hdmap::PathOverlap> signal_group;
  const auto& all_signals =
      reference_line_info.reference_line().map_path().signal_overlaps();

  static constexpr double kGroupingDist = 2.0;
  for (const auto& signal : all_signals) {
    if (std::abs(signal.start_s - first_signal_overlap.start_s) <=
        kGroupingDist) {
      signal_group.push_back(signal);
    }
  }

  // 4. Check Status (Red/Yellow check)
  bool is_stop_required = false;
  bool is_in_range = false;

  for (const auto& signal_overlap : signal_group) {
    double dist = signal_overlap.start_s - adc_front_edge_s;
    if (dist <= 0.0 || dist > start_range) {
      continue;
    }

    is_in_range = true;
    const auto& color = frame->GetSignal(signal_overlap.object_id).color();

    // Any light other than green is considered to require action (red, yellow,
    // black, unknown).
    if (color != perception::TrafficLight::GREEN) {
      is_stop_required = true;
      break;
    }
  }

  if (!is_in_range || !is_stop_required) {
    return ScenarioDecisionResult();
  }

  // 5. Determine Specific Scenario Type
  ScenarioType type = ScenarioType::TRAFFIC_LIGHT_PROTECTED;
  const auto& turn_type =
      reference_line_info.GetPathTurnType(first_signal_overlap.start_s);

  if (turn_type == hdmap::Lane::LEFT_TURN) {
    type = ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN;
  } else if (turn_type == hdmap::Lane::RIGHT_TURN) {
    type = ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN;
  }

  // Recalculate dist for scoring
  double final_dist = first_signal_overlap.start_s - adc_front_edge_s;

  return ScenarioDecisionResult(
      type, ScenarioGrade::MANEUVER,
      CalculateDistanceScore(final_dist, start_range, entry_score),
      "Traffic Light Red/Yellow Detected");
}

ScenarioDecisionResult IntersectionDecider::CheckBareIntersection(
    const DeciderContext& context) {
  // 0. Load Config
  const auto& config = config_.bare_intersection_unprotected_config();
  double start_range = config.start_bare_intersection_scenario_distance();
  uint32_t entry_score = config.scenario_entry_score();

  // 1. Sticky Strategy
  const auto current_scenario = context.current_scenario;
  if (current_scenario->Type() == ScenarioType::BARE_INTERSECTION_UNPROTECTED) {
    if (current_scenario->GetStatus() !=
        Scenario::ScenarioStatus::STATUS_DONE) {
      return ScenarioDecisionResult(ScenarioType::BARE_INTERSECTION_UNPROTECTED,
                                    ScenarioGrade::MANEUVER, entry_score,
                                    "Bare Junction In Progress (Sticky)");
    }
  }

  // 2. Find Junction Overlap
  const auto& overlaps = context.first_encountered_overlaps;
  auto it = overlaps->find(ReferenceLineInfo::PNC_JUNCTION);
  if (it == overlaps->end()) {
    return ScenarioDecisionResult();
  }

  const auto& junction_overlap = it->second;
  const auto& frame = context.frame;
  if (frame->reference_line_info().empty()) {
    return ScenarioDecisionResult();
  }
  const auto& reference_line_info = frame->reference_line_info().front();

  // 3. Right-of-Way Check
  if (reference_line_info.GetIntersectionRightofWayStatus(junction_overlap)) {
    return ScenarioDecisionResult();
  }

  // 4. Exclusion / Arbitration Logic
  static constexpr double kJunctionDelta = 10.0;

  for (const auto& overlap_pair : *overlaps) {
    auto type = overlap_pair.first;
    if (type == ReferenceLineInfo::SIGNAL ||
        type == ReferenceLineInfo::STOP_SIGN ||
        type == ReferenceLineInfo::YIELD_SIGN) {
      double sign_s = overlap_pair.second.start_s;
      double junction_s = junction_overlap.start_s;

      // If a traffic sign and an intersection are very close together, it means
      // the intersection is controlled by a sign and is not a Bare Junction.
      if (std::fabs(sign_s - junction_s) < kJunctionDelta) {
        return ScenarioDecisionResult();
      }
    }
  }

  // 5. Distance Check
  double dist =
      junction_overlap.start_s - reference_line_info.AdcSlBoundary().end_s();

  if (dist > 0.0 && dist <= start_range) {
    return ScenarioDecisionResult(
        ScenarioType::BARE_INTERSECTION_UNPROTECTED, ScenarioGrade::MANEUVER,
        CalculateDistanceScore(dist, start_range, entry_score),
        "Bare Intersection Detected");
  }

  return ScenarioDecisionResult();
}

ScenarioDecisionResult IntersectionDecider::CheckStopSign(
    const DeciderContext& context) {
  // 0. Load Config
  const auto& config = config_.stop_sign_unprotected_config();
  double start_range = config.start_stop_sign_scenario_distance();
  uint32_t scenario_entry_score = config.scenario_entry_score();

  // 1. Sticky Strategy (High Priority)
  const auto current_scenario = context.current_scenario;
  if (current_scenario->Type() == ScenarioType::STOP_SIGN_UNPROTECTED ||
      current_scenario->Type() == ScenarioType::STOP_SIGN_PROTECTED) {
    if (current_scenario->GetStatus() !=
        Scenario::ScenarioStatus::STATUS_DONE) {
      return ScenarioDecisionResult(
          current_scenario->Type(), ScenarioGrade::MANEUVER,
          scenario_entry_score, "Stop Sign In Progress (Sticky)");
    }
  }

  // 2. Overlap Check
  const auto& overlaps = context.first_encountered_overlaps;
  auto it = overlaps->find(ReferenceLineInfo::STOP_SIGN);
  if (it == overlaps->end()) {
    return ScenarioDecisionResult();
  }

  const auto& overlap = it->second;
  const auto& frame = context.frame;

  // Safety check
  if (frame->reference_line_info().empty()) {
    return ScenarioDecisionResult();
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  double dist = overlap.start_s - reference_line_info.AdcSlBoundary().end_s();

  // 3. Distance Check & Trigger
  if (dist > 0.0 && dist <= start_range) {
    // Default to Unprotected Stop Sign as per original logic
    return ScenarioDecisionResult(
        ScenarioType::STOP_SIGN_UNPROTECTED, ScenarioGrade::MANEUVER,
        CalculateDistanceScore(dist, start_range, scenario_entry_score),
        "Stop Sign Detected");
  }

  return ScenarioDecisionResult();
}

ScenarioDecisionResult IntersectionDecider::CheckYieldSign(
    const DeciderContext& context) {
  // 0. Load Config
  const auto& config = config_.yield_sign_config();
  double start_range = config.start_yield_sign_scenario_distance();
  uint32_t scenario_entry_score = config.scenario_entry_score();

  // 1. Sticky Strategy
  const auto current_scenario = context.current_scenario;
  if (current_scenario->Type() == ScenarioType::YIELD_SIGN) {
    if (current_scenario->GetStatus() !=
        Scenario::ScenarioStatus::STATUS_DONE) {
      return ScenarioDecisionResult(
          ScenarioType::YIELD_SIGN, ScenarioGrade::MANEUVER,
          scenario_entry_score, "Yield Sign In Progress (Sticky)");
    }
  }

  // 2. Overlap Check
  const auto& overlaps = context.first_encountered_overlaps;
  auto it = overlaps->find(ReferenceLineInfo::YIELD_SIGN);
  if (it == overlaps->end()) {
    return ScenarioDecisionResult();
  }

  const auto& overlap = it->second;
  const auto& frame = context.frame;
  if (frame->reference_line_info().empty()) {
    return ScenarioDecisionResult();
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  double dist = overlap.start_s - reference_line_info.AdcSlBoundary().end_s();

  // 3. Distance Check
  if (dist > 0.0 && dist <= start_range) {
    return ScenarioDecisionResult(
        ScenarioType::YIELD_SIGN, ScenarioGrade::MANEUVER,
        CalculateDistanceScore(dist, start_range, scenario_entry_score),
        "Yield Sign Detected");
  }

  return ScenarioDecisionResult();
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
