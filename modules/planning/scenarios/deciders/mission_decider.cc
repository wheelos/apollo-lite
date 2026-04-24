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

#include "modules/planning/scenarios/deciders/mission_decider.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
namespace scenario {

// Decision Scores
constexpr uint32_t kScoreMissionIdle = 99;   // Just below Emergency
constexpr uint32_t kScoreNarrowStreet = 80;  // High priority mission mode
constexpr uint32_t kScoreUTurn = 75;         // Priority maneuver

// Thresholds
constexpr double kNarrowStreetWidthThreshold = 0.5;  // meters
constexpr double kUTurnActivationDistance = 20.0;    // meters
constexpr double kStuckSpeedThreshold = 0.1;         // m/s
constexpr double kDestinationThreshold = 2.0;        // meters

ScenarioDecisionResult MissionDecider::MakeDecision(
    const DeciderContext& context) {
  // 1. Mission Idle (L1)
  // Highest priority for this decider. If the mission is done, we must stop.
  auto decision = CheckMissionIdle(context);
  if (decision.IsValid()) return decision;

  // 2. Narrow Street (L1)
  // Specific environmental constraint that changes driving behavior globally.
  decision = CheckNarrowStreet(context);
  if (decision.IsValid()) return decision;

  // 3. U-Turn (L2)
  // Complex topological maneuver.
  decision = CheckUTurn(context);
  if (decision.IsValid()) return decision;

  return ScenarioDecisionResult();
}

ScenarioDecisionResult MissionDecider::CheckMissionIdle(
    const DeciderContext& context) {
  const auto& frame = context.frame;

  if (context.planning_command != nullptr) {
    const auto& command = *context.planning_command;
    if (command.has_action() && command.action() != COMMAND_CANCEL &&
        command.has_requested_scene() &&
        command.requested_scene() != SCENE_UNKNOWN &&
        command.requested_scene() != SCENE_LANE_CRUISE) {
      return ScenarioDecisionResult();
    }
  }

  // Safe check for routing availability
  if (!frame->local_view().routing ||
      frame->local_view().routing->routing_request().waypoint().empty()) {
    // If we have no routing, or routing is empty, we are Idle.
    return ScenarioDecisionResult(ScenarioType::MISSION_IDLE,
                                  ScenarioGrade::MISSION, kScoreMissionIdle,
                                  "Routing Empty or Finished");
  }

  // Additionally, check if we have reached the destination point exactly
  // A. Check Vehicle Speed
  const auto& vehicle_state = frame->vehicle_state();
  double speed = std::abs(vehicle_state.linear_velocity());

  bool is_stopped = (speed < kStuckSpeedThreshold);
  bool is_at_destination = IsCloseToDestination(frame);
  if (is_stopped && is_at_destination) {
    return ScenarioDecisionResult(ScenarioType::MISSION_IDLE,
                                  ScenarioGrade::MISSION, kScoreMissionIdle,
                                  "Destination");
  }

  return ScenarioDecisionResult();
}

bool MissionDecider::IsCloseToDestination(const Frame* frame) {
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

ScenarioDecisionResult MissionDecider::CheckNarrowStreet(
    const DeciderContext& context) {
  const auto& frame = context.frame;
  if (frame->reference_line_info().empty()) return ScenarioDecisionResult();

  const auto& reference_line_info = frame->reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();

  // Get ADC current position
  double adc_s = reference_line_info.AdcSlBoundary().end_s();

  // Check lane width at current position
  // In a robust implementation, we should check a range ahead (e.g., +10m)
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  reference_line.GetLaneWidth(adc_s, &lane_left_width, &lane_right_width);

  const double vehicle_width = common::VehicleConfigHelper::Instance()
                                   ->GetConfig()
                                   .vehicle_param()
                                   .width();
  double buffer = lane_left_width + lane_right_width - vehicle_width;
  if (buffer > 0.0 && buffer < kNarrowStreetWidthThreshold) {
    return ScenarioDecisionResult(ScenarioType::NARROW_STREET_MANEUVER,
                                  ScenarioGrade::MISSION, kScoreNarrowStreet,
                                  "Narrow Lane Detected (<2.8m)");
  }
  return ScenarioDecisionResult();
}

ScenarioDecisionResult MissionDecider::CheckUTurn(
    const DeciderContext& context) {
  const auto& overlaps = context.first_encountered_overlaps;
  const auto& frame = context.frame;

  if (frame->reference_line_info().empty()) return ScenarioDecisionResult();

  // 1. Find the nearest Junction
  auto it = overlaps->find(ReferenceLineInfo::PNC_JUNCTION);
  if (it == overlaps->end()) return ScenarioDecisionResult();

  const auto& overlap = it->second;
  const auto& reference_line_info = frame->reference_line_info().front();

  // 2. Check if Routing requires a U-Turn at this junction
  // The ReferenceLine contains topology information derived from routing.
  auto turn_type = reference_line_info.GetPathTurnType(overlap.start_s);

  if (turn_type == hdmap::Lane::U_TURN) {
    double dist_to_junction =
        overlap.start_s - reference_line_info.AdcSlBoundary().end_s();

    // 3. Activation Range Check
    // Only activate U-Turn scenario when approaching the junction.
    if (dist_to_junction > 0.0 && dist_to_junction < kUTurnActivationDistance) {
      return ScenarioDecisionResult(ScenarioType::NARROW_STREET_U_TURN,
                                    ScenarioGrade::MANEUVER, kScoreUTurn,
                                    "U-Turn Routing Request");
    }
  }
  return ScenarioDecisionResult();
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
