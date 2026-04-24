/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/planning_coordinator.h"

#include "cyber/time/clock.h"
#include "modules/planning/environment/capability_extractor.h"
#include "modules/planning/navi_planning.h"
#include "modules/planning/on_lane_planning.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

namespace {

constexpr std::size_t kSafetyHoldPointCount = 8;

}  // namespace

PlanningCoordinator::PlanningCoordinator(
    const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector) {}

common::Status PlanningCoordinator::Init(const PlanningConfig& config,
                                         bool use_navigation_mode) {
  config_ = config;
  use_navigation_mode_ = use_navigation_mode;

  if (use_navigation_mode_) {
    corridor_planner_ = std::make_unique<NaviPlanning>(injector_);
    return corridor_planner_->Init(config_);
  }

  lane_graph_planner_ = std::make_unique<OnLanePlanning>(injector_);
  return lane_graph_planner_->Init(config_);
}

void PlanningCoordinator::RunOnce(const LocalView& local_view,
                                  ADCTrajectory* const adc_trajectory) {
  state_ = BuildState(local_view);
  LocalView planner_local_view = local_view;
  planner_local_view.planning_state =
      std::make_shared<PlanningCoordinatorState>(state_);
  PlanningBase* planner = GetPlannerForMode(state_.resolved_mode);
  if (planner == nullptr) {
    if (state_.resolved_mode == MODE_SAFETY_HOLD) {
      GenerateSafetyHoldTrajectory(planner_local_view, adc_trajectory);
      return;
    }
    auto* not_ready = adc_trajectory->mutable_decision()
                          ->mutable_main_decision()
                          ->mutable_not_ready();
    not_ready->set_reason(state_.reason.empty() ? "planning mode unavailable"
                                                : state_.reason);
    return;
  }
  planner->RunOnce(planner_local_view, adc_trajectory);
}

PlanningCoordinatorState PlanningCoordinator::BuildState(
    const LocalView& local_view) const {
  PlanningCoordinatorState state;
  const auto resolution = ModeResolution::Resolve(
      local_view.planning_command.get(), local_view.capability_set.get(),
      BuildModeShellAvailability(), ResolveLegacyMode());
  state.requested_mode = resolution.requested_mode;
  state.resolved_mode = resolution.resolved_mode;
  state.reason = resolution.reason;
  state.blockers = resolution.blockers;

  if (local_view.planning_command != nullptr) {
    const auto& command = *local_view.planning_command;
    if (command.has_mission_id()) {
      state.mission_id = command.mission_id();
    }
    if (command.has_command_id()) {
      state.command_id = command.command_id();
    }
    if (command.has_requested_scene()) {
      state.active_scene = command.requested_scene();
    }
  }

  if (state.resolved_mode != MODE_UNKNOWN &&
      state.requested_mode != state.resolved_mode && state.reason.empty()) {
    state.reason = "requested mode degraded to an executable shell";
  }

  return state;
}

ModeShellAvailability PlanningCoordinator::BuildModeShellAvailability() const {
  ModeShellAvailability availability;
  availability.lane_graph_available = lane_graph_planner_ != nullptr;
  availability.corridor_available = corridor_planner_ != nullptr;
  availability.safety_hold_available = true;
  return availability;
}

void PlanningCoordinator::GenerateSafetyHoldTrajectory(
    const LocalView& local_view, ADCTrajectory* adc_trajectory) const {
  CHECK_NOTNULL(adc_trajectory);
  adc_trajectory->clear_trajectory_point();
  adc_trajectory->mutable_header()->set_timestamp_sec(
      cyber::Clock::NowInSeconds());

  if (local_view.localization_estimate == nullptr ||
      !local_view.localization_estimate->has_pose()) {
    auto* not_ready = adc_trajectory->mutable_decision()
                          ->mutable_main_decision()
                          ->mutable_not_ready();
    not_ready->set_reason("safety-hold localization unavailable");
    return;
  }

  const auto& pose = local_view.localization_estimate->pose();
  const double unit_t = FLAGS_fallback_time_unit;
  for (std::size_t index = 0; index < kSafetyHoldPointCount; ++index) {
    auto* point = adc_trajectory->add_trajectory_point();
    point->set_relative_time(static_cast<double>(index) * unit_t);
    point->set_v(0.0);
    point->set_a(0.0);
    auto* path_point = point->mutable_path_point();
    path_point->set_x(pose.position().x());
    path_point->set_y(pose.position().y());
    path_point->set_z(pose.position().z());
    path_point->set_theta(pose.heading());
    path_point->set_s(0.0);
  }

  if (local_view.chassis != nullptr) {
    adc_trajectory->set_gear(local_view.chassis->gear_location());
  }
  if (local_view.planning_command != nullptr &&
      local_view.planning_command->has_requested_scene() &&
      local_view.planning_command->requested_scene() == SCENE_EMERGENCY_STOP) {
    auto* estop = adc_trajectory->mutable_estop();
    estop->set_is_estop(true);
    estop->set_reason("emergency stop command");
  }
}

PlanningMode PlanningCoordinator::ResolveLegacyMode() const {
  return use_navigation_mode_ ? MODE_CORRIDOR : MODE_LANE_GRAPH;
}

PlanningBase* PlanningCoordinator::GetPlannerForMode(PlanningMode mode) const {
  if (lane_graph_planner_ != nullptr && lane_graph_planner_->Mode() == mode) {
    return lane_graph_planner_.get();
  }
  if (corridor_planner_ != nullptr && corridor_planner_->Mode() == mode) {
    return corridor_planner_.get();
  }
  return nullptr;
}

}  // namespace planning
}  // namespace apollo
