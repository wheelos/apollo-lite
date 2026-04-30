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

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/published_trajectory_gear.h"
#include "modules/planning/environment/capability_extractor.h"
#include "modules/planning/navi_planning.h"
#include "modules/planning/on_lane_planning.h"
#include "modules/planning/open_space_planning.h"
#include "modules/planning/planning_shell_bridge.h"

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
  shell_registry_.Clear();

  std::vector<std::string> init_errors;
  auto register_shell = [this, &init_errors](
                            PlanningMode mode, PlanningShellType shell,
                            PlanningOperatingDomain domain,
                            const std::string& shell_name,
                            std::unique_ptr<PlanningBase> planner) {
    if (shell_registry_.Register(mode, shell, domain, shell_name,
                                 std::move(planner))) {
      return true;
    }
    init_errors.emplace_back("failed to register shell: " + shell_name);
    AWARN << init_errors.back();
    return false;
  };

  if (apollo::hdmap::HDMapUtil::BaseMapPtr() != nullptr) {
    auto lane_graph_planner = std::make_unique<OnLanePlanning>(injector_);
    auto lane_graph_status = lane_graph_planner->Init(config_);
    if (lane_graph_status.ok()) {
      register_shell(MODE_LANE_GRAPH, PLANNING_SHELL_ON_LANE,
                     DOMAIN_HDMAP_ROUTED, "OnLanePlanningShell",
                     std::move(lane_graph_planner));
    } else {
      init_errors.emplace_back("lane_graph init failed: " +
                               lane_graph_status.error_message());
      AWARN << init_errors.back();
    }
  } else {
    AWARN
        << "HDMap unavailable during planning init; lane-graph shell disabled";
  }

  auto open_space_planner = std::make_unique<OpenSpacePlanning>(injector_);
  auto open_space_status = open_space_planner->Init(config_);
  if (open_space_status.ok()) {
    register_shell(MODE_OPEN_SPACE, PLANNING_SHELL_OPEN_SPACE,
                   DOMAIN_OPEN_SPACE, "OpenSpacePlanningShell",
                   std::move(open_space_planner));
  } else {
    init_errors.emplace_back("open_space init failed: " +
                             open_space_status.error_message());
    AWARN << init_errors.back();
  }

  auto corridor_planner = std::make_unique<NaviPlanning>(injector_);
  auto corridor_status = corridor_planner->Init(config_);
  if (corridor_status.ok()) {
    auto* corridor_planner_raw = corridor_planner.get();
    auto structured_mapless_planner = std::make_unique<PlanningShellBridge>(
        injector_, MODE_FREE_SPACE, "StructuredMaplessPlanningShell",
        corridor_planner_raw);
    if (register_shell(MODE_CORRIDOR, PLANNING_SHELL_CORRIDOR,
                       DOMAIN_HDMAP_ROUTED, "CorridorPlanningShell",
                       std::move(corridor_planner))) {
      register_shell(MODE_FREE_SPACE, PLANNING_SHELL_STRUCTURED_MAPLESS,
                     DOMAIN_STRUCTURED_MAPLESS,
                     "StructuredMaplessPlanningShell",
                     std::move(structured_mapless_planner));
    }
  } else {
    init_errors.emplace_back("corridor init failed: " +
                             corridor_status.error_message());
    AWARN << init_errors.back();
  }

  if (!shell_registry_.HasShell(MODE_LANE_GRAPH) &&
      !shell_registry_.HasShell(MODE_CORRIDOR) &&
      !shell_registry_.HasShell(MODE_OPEN_SPACE)) {
    return common::Status(common::ErrorCode::PLANNING_ERROR,
                          init_errors.empty()
                              ? "failed to initialize any planning shell"
                              : init_errors.front());
  }

  return common::Status::OK();
}

PlanningCoordinatorState PlanningCoordinator::PreviewState(
    const LocalView& local_view) const {
  return BuildState(local_view);
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
  const auto availability = BuildModeShellAvailability();
  state.previous_shell = state_.active_shell;
  state.previous_mode = state_.resolved_mode;

  std::string command_id;
  PlanningSceneType active_scene = SCENE_LANE_CRUISE;
  if (local_view.planning_command != nullptr) {
    const auto& command = *local_view.planning_command;
    if (command.has_command_id()) {
      command_id = command.command_id();
    }
    if (command.has_requested_scene()) {
      active_scene = command.requested_scene();
    }
  }

  const auto resolution = ModeResolution::Resolve(
      local_view.planning_command.get(), local_view.capability_set.get(),
      availability, ResolveLegacyMode());
  state.requested_mode = resolution.requested_mode;
  const auto transition = ShellTransitionPolicy::Apply(
      resolution, state_, command_id, active_scene,
      local_view.capability_set.get(), availability, &shell_transition_state_);
  state.desired_mode = transition.desired_mode;
  state.desired_shell = transition.desired_shell;
  state.resolved_mode = transition.active_mode;
  state.active_shell = transition.active_shell;
  state.active_domain = ResolveOperatingDomainForMode(state.resolved_mode);
  state.transition_pending = transition.transition_pending;
  state.continuity_hold = transition.continuity_hold;
  state.reason = transition.reason;
  state.blockers = resolution.blockers;

  if (local_view.planning_command != nullptr) {
    const auto& command = *local_view.planning_command;
    if (command.has_mission_id()) {
      state.mission_id = command.mission_id();
    }
    state.command_id = command_id;
    state.active_scene = active_scene;
  }

  if (state.resolved_mode != MODE_UNKNOWN &&
      state.requested_mode != state.resolved_mode && state.reason.empty()) {
    state.reason = "requested mode degraded to an executable shell";
  }

  return state;
}

ModeShellAvailability PlanningCoordinator::BuildModeShellAvailability() const {
  ModeShellAvailability availability = shell_registry_.BuildAvailability();
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

  adc_trajectory->set_gear(ResolvePublishedGear(PublishedGearInput{
      MODE_SAFETY_HOLD, local_view.chassis.get(), canbus::Chassis::GEAR_NONE,
      canbus::Chassis::GEAR_NONE, false, true}));
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
  return shell_registry_.GetPlannerForMode(mode);
}

}  // namespace planning
}  // namespace apollo
