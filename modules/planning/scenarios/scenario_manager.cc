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

#include "modules/planning/scenarios/scenario_manager.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "modules/common/util/util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/scenarios/cruise/lane_follow/lane_follow_scenario.h"
#include "modules/planning/scenarios/cruise/lane_keeping/lane_keeping_scenario.h"
#include "modules/planning/scenarios/deciders/cruise_decider.h"
#include "modules/planning/scenarios/deciders/emergency_decider.h"
#include "modules/planning/scenarios/deciders/escape_decider.h"
#include "modules/planning/scenarios/deciders/intersection_decider.h"
#include "modules/planning/scenarios/deciders/mission_decider.h"
#include "modules/planning/scenarios/deciders/park_decider.h"
#include "modules/planning/scenarios/deciders/scenario_decider.h"
#include "modules/planning/scenarios/emergency/emergency_pull_over/emergency_pull_over_scenario.h"
#include "modules/planning/scenarios/emergency/emergency_stop/emergency_stop_scenario.h"
#include "modules/planning/scenarios/intersection/bare_intersection/unprotected/bare_intersection_unprotected_scenario.h"
#include "modules/planning/scenarios/intersection/stop_sign/unprotected/stop_sign_unprotected_scenario.h"
#include "modules/planning/scenarios/intersection/traffic_light/protected/traffic_light_protected_scenario.h"
#include "modules/planning/scenarios/intersection/traffic_light/unprotected_left_turn/traffic_light_unprotected_left_turn_scenario.h"
#include "modules/planning/scenarios/intersection/traffic_light/unprotected_right_turn/traffic_light_unprotected_right_turn_scenario.h"
#include "modules/planning/scenarios/intersection/yield_sign/yield_sign_scenario.h"
#include "modules/planning/scenarios/learning_model/learning_model_sample_scenario.h"
#include "modules/planning/scenarios/maneuver/deadend_turnaround/deadend_turnaround_scenario.h"
#include "modules/planning/scenarios/maneuver/escape/escape_scenario.h"
#include "modules/planning/scenarios/maneuver/narrow_street/narrow_street_scenario.h"
#include "modules/planning/scenarios/maneuver/narrow_street_u_turn/narrow_street_u_turn_scenario.h"
#include "modules/planning/scenarios/park/mission_idle/mission_idle_scenario.h"
#include "modules/planning/scenarios/park/park_and_go/park_and_go_scenario.h"
#include "modules/planning/scenarios/park/pull_over/pull_over_scenario.h"
#include "modules/planning/scenarios/park/valet_parking/valet_parking_scenario.h"

namespace {

using apollo::planning::ScenarioType;

bool IsTrafficLightScenario(const ScenarioType& type) {
  return type == ScenarioType::TRAFFIC_LIGHT_PROTECTED ||
         type == ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN ||
         type == ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN;
}

bool IsStopSignScenario(const ScenarioType& type) {
  return type == ScenarioType::STOP_SIGN_UNPROTECTED ||
         type == ScenarioType::STOP_SIGN_PROTECTED;
}

bool IsYieldSignScenario(const ScenarioType& type) {
  return type == ScenarioType::YIELD_SIGN;
}

bool IsBareIntersectionScenario(const ScenarioType& type) {
  return type == ScenarioType::BARE_INTERSECTION_UNPROTECTED;
}

}  // namespace

namespace apollo {
namespace planning {
namespace scenario {

using apollo::hdmap::PathOverlap;

ScenarioManager::ScenarioManager(
    const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector) {}

bool ScenarioManager::Init(const PlanningConfig& planning_config) {
  planning_config_.CopyFrom(planning_config);

  // 1. Register Configs & Deciders
  RegisterScenarios();
  RegisterDeciders();

  // 2. Init Transition Guard
  transition_guard_.Init(planning_config_);

  // 3. Init Default Scenario
  default_scenario_type_ = ScenarioType::LANE_FOLLOW;
  current_scenario_ = CreateScenario(default_scenario_type_);

  if (!current_scenario_) {
    AERROR << "Failed to init default scenario: "
           << ScenarioType_Name(default_scenario_type_);
    return false;
  }
  current_scenario_->Init();

  return true;
}

void ScenarioManager::RegisterScenarios() {
  auto load_conf = [&](ScenarioType type, const std::string& file) {
    ACHECK(Scenario::LoadConfig(file, &config_map_[type]));
  };

  if (planning_config_.learning_mode() == PlanningConfig::HYBRID ||
      planning_config_.learning_mode() == PlanningConfig::HYBRID_TEST) {
    load_conf(ScenarioType::LANE_FOLLOW,
              FLAGS_scenario_lane_follow_hybrid_config_file);
  } else {
    load_conf(ScenarioType::LANE_FOLLOW,
              FLAGS_scenario_lane_follow_config_file);
  }

  // --- Basic and intersection categories ---
  load_conf(ScenarioType::BARE_INTERSECTION_UNPROTECTED,
            FLAGS_scenario_bare_intersection_unprotected_config_file);
  load_conf(ScenarioType::STOP_SIGN_UNPROTECTED,
            FLAGS_scenario_stop_sign_unprotected_config_file);
  load_conf(ScenarioType::TRAFFIC_LIGHT_PROTECTED,
            FLAGS_scenario_traffic_light_protected_config_file);
  load_conf(ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN,
            FLAGS_scenario_traffic_light_unprotected_left_turn_config_file);
  load_conf(ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN,
            FLAGS_scenario_traffic_light_unprotected_right_turn_config_file);
  load_conf(ScenarioType::YIELD_SIGN, FLAGS_scenario_yield_sign_config_file);

  // --- Parking and Emergency ---
  load_conf(ScenarioType::PULL_OVER, FLAGS_scenario_pull_over_config_file);
  load_conf(ScenarioType::VALET_PARKING,
            FLAGS_scenario_valet_parking_config_file);
  load_conf(ScenarioType::EMERGENCY_PULL_OVER,
            FLAGS_scenario_emergency_pull_over_config_file);
  load_conf(ScenarioType::EMERGENCY_STOP,
            FLAGS_scenario_emergency_stop_config_file);

  // --- Task Intent and Escape Category ---
  load_conf(ScenarioType::MISSION_IDLE,
            FLAGS_scenario_mission_idle_config_file);
  load_conf(ScenarioType::ESCAPE, FLAGS_scenario_escape_config_file);
  load_conf(ScenarioType::NARROW_STREET_MANEUVER,
            FLAGS_scenario_narrow_street_maneuver_config_file);
  load_conf(ScenarioType::NARROW_STREET_U_TURN,
            FLAGS_scenario_narrow_street_u_turn_config_file);
  load_conf(ScenarioType::DEADEND_TURNAROUND,
            FLAGS_scenario_deadend_turnaround_config_file);

  // --- Line following and starting classes ---
  load_conf(ScenarioType::PARK_AND_GO, FLAGS_scenario_park_and_go_config_file);

  // --- learning ---
  load_conf(ScenarioType::LEARNING_MODEL_SAMPLE,
            FLAGS_scenario_learning_model_sample_config_file);
}

void ScenarioManager::RegisterDeciders() {
  deciders_.clear();

  // Helper to safely get config (fallback to LaneFollow if specific not found,
  // though RegisterScenarios ensures existence)
  auto get_conf = [&](ScenarioType type) -> const ScenarioConfig& {
    if (config_map_.count(type)) return config_map_[type];
    return config_map_[ScenarioType::LANE_FOLLOW];
  };

  // 1. Safety & Emergency
  deciders_.emplace_back(std::make_unique<EmergencyDecider>(
      get_conf(ScenarioType::EMERGENCY_STOP), injector_));

  // 2. Mission & Parking
  deciders_.emplace_back(std::make_unique<ParkDecider>(
      get_conf(ScenarioType::VALET_PARKING), injector_));
  deciders_.emplace_back(std::make_unique<MissionDecider>(
      get_conf(ScenarioType::LANE_FOLLOW), injector_));

  // 3. Intersection & Maneuver
  deciders_.emplace_back(std::make_unique<IntersectionDecider>(
      get_conf(ScenarioType::BARE_INTERSECTION_UNPROTECTED), injector_));
  // EscapeDecider logic usually fits within Parking or LaneFollow context
  deciders_.emplace_back(std::make_unique<EscapeDecider>(
      get_conf(ScenarioType::PARK_AND_GO), injector_));

  // 4. Cruise
  deciders_.emplace_back(std::make_unique<CruiseDecider>(
      get_conf(ScenarioType::LANE_FOLLOW), injector_));
}

void ScenarioManager::Update(const common::TrajectoryPoint& ego_point,
                             const Frame& frame) {
  ACHECK(!frame.reference_line_info().empty());

  // 1. Observe (Build lookup map) - Pass const ref
  Observe(frame);

  // 2. Dispatch (Decision Making) - Pass const ref
  ScenarioDispatch(frame);

  // 3. Update Context (Environment Info) - Pass const ref
  UpdatePlanningContext(frame, current_scenario_->Type());

  // 4. Process (Execution)
  // Scenario::Process requires Frame* because it modifies the frame.
  // This is the ONLY place where we cast away const.
  current_scenario_->Process(const_cast<Frame*>(&frame));
}

void ScenarioManager::Observe(const Frame& frame) {
  first_encountered_overlap_map_.clear();
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();

  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first == ReferenceLineInfo::PNC_JUNCTION ||
        overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::STOP_SIGN ||
        overlap.first == ReferenceLineInfo::YIELD_SIGN) {
      first_encountered_overlap_map_[overlap.first] = overlap.second;
    }
  }
}

void ScenarioManager::ScenarioDispatch(const Frame& frame) {
  CHECK_NOTNULL(current_scenario_.get());

  // E2E / Learning Mode Handling
  if (planning_config_.learning_mode() == PlanningConfig::E2E ||
      planning_config_.learning_mode() == PlanningConfig::E2E_TEST) {
    // Assuming ScenarioDispatchLearning is also updated to take const Frame&
    ScenarioType learning_type = ScenarioDispatchLearning(frame);
    if (learning_type != current_scenario_->Type()) {
      SwitchToScenario(learning_type, frame);
    }
    return;
  }

  const auto current_type = current_scenario_->Type();
  const auto current_status = current_scenario_->GetStatus();

  // 1. Init Best Decision
  ScenarioDecisionResult best_decision;
  best_decision.type = current_type;
  best_decision.grade = current_scenario_->Grade();
  best_decision.score = 0.0;
  best_decision.reason = "Keep Current";

  // 2. Construct Context
  DeciderContext context;
  context.frame = &frame;  // Take address of const reference
  context.current_scenario = current_scenario_.get();
  context.first_encountered_overlaps = &first_encountered_overlap_map_;

  // 3. Bidding
  for (const auto& decider : deciders_) {
    auto decision = decider->MakeDecision(context);

    if (!decision.IsValid()) continue;

    AINFO << decision.DebugString();

    bool allowed = transition_guard_.IsTransitionAllowed(
        current_type, current_status, decision.type, decision.grade);

    if (allowed) {
      if (decision.IsBetterThan(best_decision)) {
        best_decision = decision;
      }
    } else {
      ADEBUG << "Transition blocked: " << ScenarioType_Name(current_type)
             << " -> " << ScenarioType_Name(decision.type);
    }
  }

  // 5. Finalize
  if (best_decision.type != current_type) {
    SwitchToScenario(best_decision.type, frame);
  } else {
    // Edge case: Current DONE but no winner -> Fallback to LaneFollow
    if (current_status == Scenario::ScenarioStatus::STATUS_DONE &&
        current_type != ScenarioType::LANE_FOLLOW) {
      if (transition_guard_.IsTransitionAllowed(current_type, current_status,
                                                ScenarioType::LANE_FOLLOW,
                                                ScenarioGrade::CRUISE)) {
        SwitchToScenario(ScenarioType::LANE_FOLLOW, frame);
      }
    }
  }
}

void ScenarioManager::SwitchToScenario(ScenarioType new_scenario_type,
                                       const Frame& frame) {
  AINFO << "Switching Scenario: "
        << ScenarioType_Name(current_scenario_->Type()) << " -> "
        << ScenarioType_Name(new_scenario_type);

  // Note: OnExit/OnEnter likely require Frame* if they modify state.
  // Using const_cast here to adapt to Scenario interface.
  current_scenario_->OnExit(const_cast<Frame*>(&frame));

  auto new_scenario = CreateScenario(new_scenario_type);
  if (!new_scenario) {
    AERROR << "Create scenario failed: " << ScenarioType_Name(new_scenario_type)
           << ". Fallback to default.";
    new_scenario = CreateScenario(default_scenario_type_);
  }

  current_scenario_ = std::move(new_scenario);
  current_scenario_->Init();
  current_scenario_->OnEnter(const_cast<Frame*>(&frame));
}

std::unique_ptr<Scenario> ScenarioManager::CreateScenario(
    ScenarioType scenario_type) {
  if (config_map_.find(scenario_type) == config_map_.end()) {
    AERROR << "Scenario config not found: " << ScenarioType_Name(scenario_type);
    return nullptr;
  }
  const auto& config = config_map_[scenario_type];

  switch (scenario_type) {
    case ScenarioType::LANE_FOLLOW:
      return std::make_unique<lane_follow::LaneFollowScenario>(config, nullptr,
                                                               injector_);
    // --- Intersections ---
    case ScenarioType::BARE_INTERSECTION_UNPROTECTED:
      return std::make_unique<
          bare_intersection::BareIntersectionUnprotectedScenario>(
          config, nullptr, injector_);
    case ScenarioType::STOP_SIGN_UNPROTECTED:
      return std::make_unique<stop_sign::StopSignUnprotectedScenario>(
          config, nullptr, injector_);
    case ScenarioType::TRAFFIC_LIGHT_PROTECTED:
      return std::make_unique<traffic_light::TrafficLightProtectedScenario>(
          config, nullptr, injector_);
    case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
      return std::make_unique<
          traffic_light::TrafficLightUnprotectedLeftTurnScenario>(
          config, nullptr, injector_);
    case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
      return std::make_unique<
          traffic_light::TrafficLightUnprotectedRightTurnScenario>(
          config, nullptr, injector_);
    case ScenarioType::YIELD_SIGN:
      return std::make_unique<yield_sign::YieldSignScenario>(config, nullptr,
                                                             injector_);

    // --- Parking & Go ---
    case ScenarioType::PULL_OVER:
      return std::make_unique<pull_over::PullOverScenario>(config, nullptr,
                                                           injector_);
    case ScenarioType::VALET_PARKING:
      return std::make_unique<valet_parking::ValetParkingScenario>(
          config, nullptr, injector_);
    case ScenarioType::PARK_AND_GO:
      return std::make_unique<park_and_go::ParkAndGoScenario>(config, nullptr,
                                                              injector_);

    // --- Emergency ---
    case ScenarioType::EMERGENCY_PULL_OVER:
      return std::make_unique<emergency_pull_over::EmergencyPullOverScenario>(
          config, nullptr, injector_);
    case ScenarioType::EMERGENCY_STOP:
      return std::make_unique<emergency_stop::EmergencyStopScenario>(
          config, nullptr, injector_);

    // --- Narrow Street & Maneuver ---
    case ScenarioType::NARROW_STREET_U_TURN:
      return std::make_unique<narrow_street_u_turn::NarrowStreetUTurnScenario>(
          config, nullptr, injector_);
    case ScenarioType::NARROW_STREET_MANEUVER:
      return std::make_unique<narrow_street::NarrowStreetScenario>(
          config, nullptr, injector_);
    case ScenarioType::DEADEND_TURNAROUND:
      return std::make_unique<deadend_turnaround::DeadendTurnaroundScenario>(
          config, nullptr, injector_);
    case ScenarioType::ESCAPE:
      return std::make_unique<escape::EscapeScenario>(config, nullptr,
                                                      injector_);

    // --- Path Follow & Mission ---
    case ScenarioType::MISSION_IDLE:
      return std::make_unique<mission_idle::MissionIdleScenario>(
          config, nullptr, injector_);

    // --- Learning Model ---
    case ScenarioType::LEARNING_MODEL_SAMPLE:
      return std::make_unique<learning_model::LearningModelSampleScenario>(
          config, nullptr, injector_);

    default:
      AERROR << "Unknown scenario type: " << ScenarioType_Name(scenario_type);
      return nullptr;
  }
}

void ScenarioManager::UpdatePlanningContext(const Frame& frame,
                                            const ScenarioType& type) {
  ScenarioType current_running_type = current_scenario_->Type();

  UpdateContextBareIntersection(type, current_running_type);
  UpdateContextEmergencyStop(type);
  UpdateContextPullOver(frame, type);
  UpdateContextStopSign(type, current_running_type);
  UpdateContextTrafficLight(frame, type, current_running_type);
  UpdateContextYieldSign(type, current_running_type);
}

void ScenarioManager::UpdateContextBareIntersection(
    const ScenarioType& type, const ScenarioType& current_running_type) {
  auto* status = injector_->planning_context()
                     ->mutable_planning_status()
                     ->mutable_bare_intersection();

  if (!IsBareIntersectionScenario(type)) {
    status->Clear();
    return;
  }

  if (type == current_running_type) {
    return;
  }

  if (first_encountered_overlap_map_.count(ReferenceLineInfo::PNC_JUNCTION)) {
    status->set_current_pnc_junction_overlap_id(
        first_encountered_overlap_map_[ReferenceLineInfo::PNC_JUNCTION]
            .object_id);
  }
}

void ScenarioManager::UpdateContextStopSign(
    const ScenarioType& type, const ScenarioType& current_running_type) {
  auto* status = injector_->planning_context()
                     ->mutable_planning_status()
                     ->mutable_stop_sign();

  if (!IsStopSignScenario(type)) {
    status->Clear();
    return;
  }

  if (type == current_running_type) {
    return;
  }

  if (first_encountered_overlap_map_.count(ReferenceLineInfo::STOP_SIGN)) {
    status->set_current_stop_sign_overlap_id(
        first_encountered_overlap_map_[ReferenceLineInfo::STOP_SIGN].object_id);
  }
}

void ScenarioManager::UpdateContextYieldSign(
    const ScenarioType& type, const ScenarioType& current_running_type) {
  auto* status = injector_->planning_context()
                     ->mutable_planning_status()
                     ->mutable_yield_sign();

  if (!IsYieldSignScenario(type)) {
    status->Clear();
    return;
  }

  if (type == current_running_type) {
    return;
  }

  if (first_encountered_overlap_map_.count(ReferenceLineInfo::YIELD_SIGN)) {
    status->add_current_yield_sign_overlap_id(
        first_encountered_overlap_map_[ReferenceLineInfo::YIELD_SIGN]
            .object_id);
  }
}

void ScenarioManager::UpdateContextTrafficLight(
    const Frame& frame, const ScenarioType& type,
    const ScenarioType& current_running_type) {
  auto* status = injector_->planning_context()
                     ->mutable_planning_status()
                     ->mutable_traffic_light();

  if (!IsTrafficLightScenario(type)) {
    status->Clear();
    return;
  }

  if (type == current_running_type) {
    return;
  }

  if (first_encountered_overlap_map_.find(ReferenceLineInfo::SIGNAL) ==
      first_encountered_overlap_map_.end()) {
    status->Clear();
    return;
  }

  std::string current_id =
      first_encountered_overlap_map_[ReferenceLineInfo::SIGNAL].object_id;

  const auto& signals = frame.reference_line_info()
                            .front()
                            .reference_line()
                            .map_path()
                            .signal_overlaps();

  auto it = std::find_if(
      signals.begin(), signals.end(),
      [&](const PathOverlap& o) { return o.object_id == current_id; });

  if (it == signals.end()) {
    status->Clear();
    return;
  }

  status->clear_current_traffic_light_overlap_id();
  static constexpr double kGroupingDist = 2.0;
  for (const auto& overlap : signals) {
    if (std::fabs(overlap.start_s - it->start_s) <= kGroupingDist) {
      status->add_current_traffic_light_overlap_id(overlap.object_id);
    }
  }
}

void ScenarioManager::UpdateContextPullOver(const Frame& frame,
                                            const ScenarioType& type) {
  auto* pull_over = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_pull_over();

  if (type == ScenarioType::PULL_OVER) {
    pull_over->set_pull_over_type(PullOverStatus::PULL_OVER);
    pull_over->set_plan_pull_over_path(true);
    return;
  } else if (type == ScenarioType::EMERGENCY_PULL_OVER) {
    pull_over->set_pull_over_type(PullOverStatus::EMERGENCY_PULL_OVER);
    return;
  }

  pull_over->set_plan_pull_over_path(false);

  if (pull_over->has_position()) {
    const auto& routing = frame.local_view().routing;
    if (routing->routing_request().waypoint_size() >= 2) {
      const auto& reference_line_info = frame.reference_line_info().front();
      const auto& reference_line = reference_line_info.reference_line();

      common::SLPoint dest_sl;
      const auto& routing_end =
          *(routing->routing_request().waypoint().rbegin());
      reference_line.XYToSL(routing_end.pose(), &dest_sl);

      common::SLPoint pull_over_sl;
      reference_line.XYToSL(pull_over->position(), &pull_over_sl);

      static constexpr double kDestMaxDelta = 30.0;
      if (std::fabs(dest_sl.s() - pull_over_sl.s()) > kDestMaxDelta) {
        injector_->planning_context()
            ->mutable_planning_status()
            ->clear_pull_over();
      }
    }
  }
}

void ScenarioManager::UpdateContextEmergencyStop(const ScenarioType& type) {
  if (type != ScenarioType::EMERGENCY_STOP) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_emergency_stop()
        ->Clear();
  }
}

ScenarioType ScenarioManager::ScenarioDispatchLearning(const Frame& frame) {
  ScenarioType scenario_type = ScenarioType::LEARNING_MODEL_SAMPLE;
  return scenario_type;
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
