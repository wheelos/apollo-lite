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

#include "modules/planning/open_space_planning.h"

#include <chrono>
#include <utility>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/published_trajectory_gear.h"
#include "modules/planning/common/trajectory_stitcher.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/park/park_and_go/park_and_go_scenario.h"
#include "modules/planning/scenarios/park/pull_over/pull_over_scenario.h"
#include "modules/planning/scenarios/park/valet_parking/valet_parking_scenario.h"
#include "modules/planning/scenarios/scenario.h"

namespace apollo {
namespace planning {

namespace {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::cyber::Clock;
using apollo::planning::scenario::Scenario;

const routing::ParkingInfo* GetRequestedParkingInfo(
    const LocalView& local_view) {
  if (local_view.planning_command != nullptr &&
      local_view.planning_command->has_goal() &&
      local_view.planning_command->goal().has_parking_goal()) {
    return &local_view.planning_command->goal().parking_goal();
  }
  if (local_view.routing != nullptr &&
      local_view.routing->routing_request().has_parking_info()) {
    return &local_view.routing->routing_request().parking_info();
  }
  return nullptr;
}

bool ExtractStageConfig(const ScenarioConfig& scenario_config,
                        StageType stage_type,
                        ScenarioConfig::StageConfig* stage_config) {
  CHECK_NOTNULL(stage_config);
  for (const auto& config : scenario_config.stage_config()) {
    if (config.stage_type() == stage_type) {
      stage_config->CopyFrom(config);
      return true;
    }
  }
  return false;
}

}  // namespace

std::string OpenSpacePlanning::Name() const { return "open_space_planning"; }

void OpenSpacePlanning::OpenSpaceRuntimeState::Reset() {
  active_recipe_kind = RecipeKind::kUnknown;
  recipe_progress = RecipeProgress::kActive;
  active_command_id.clear();
  active_stage_type = StageType::NO_STAGE;
  open_space_stage.reset();
  last_gear = canbus::Chassis::GEAR_DRIVE;
}

bool OpenSpacePlanning::SupportsOpenSpaceCommand(const LocalView& local_view) {
  if (local_view.planning_command != nullptr &&
      local_view.planning_command->has_requested_scene()) {
    switch (local_view.planning_command->requested_scene()) {
      case SCENE_PARK_IN:
        return true;
      case SCENE_PULL_OUT:
        return true;
      case SCENE_PULL_OVER:
        return local_view.planning_command->has_preferred_mode() &&
               local_view.planning_command->preferred_mode() == MODE_OPEN_SPACE;
      default:
        break;
    }
  }
  return GetRequestedParkingInfo(local_view) != nullptr;
}

Status OpenSpacePlanning::Init(const PlanningConfig& config) {
  config_ = config;
  PlanningBase::Init(config_);
  ResetRecipeState();

  injector_->history()->Clear();
  injector_->planning_context()->mutable_planning_status()->Clear();

  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  ACHECK(hdmap_) << "Failed to load map";

  return LoadRecipes();
}

Status OpenSpacePlanning::LoadRecipes() {
  park_in_recipe_ = Recipe{};
  pull_over_recipe_ = Recipe{};
  pull_out_recipe_ = Recipe{};

  ScenarioConfig valet_parking_config;
  if (!Scenario::LoadConfig(FLAGS_scenario_valet_parking_config_file,
                            &valet_parking_config)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "failed to load valet parking scenario config");
  }
  ScenarioConfig::StageConfig stage_config;
  if (!ExtractStageConfig(valet_parking_config, VALET_PARKING_PARKING,
                          &stage_config)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "failed to find valet parking open-space stage config");
  }
  park_in_recipe_.name = "park_in";
  park_in_recipe_.scenario_type = ScenarioType::VALET_PARKING;
  park_in_recipe_.entry_stage_type = VALET_PARKING_PARKING;
  park_in_recipe_.scenario_config = valet_parking_config;
  park_in_recipe_.stage_factory =
      std::make_unique<scenario::valet_parking::ValetParkingScenario>(
          park_in_recipe_.scenario_config, nullptr, injector_);
  park_in_recipe_.stage_factory->Init();

  ScenarioConfig pull_over_config;
  if (!Scenario::LoadConfig(FLAGS_scenario_pull_over_config_file,
                            &pull_over_config)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "failed to load pull-over scenario config");
  }
  if (!ExtractStageConfig(pull_over_config, PULL_OVER_RETRY_PARKING,
                          &stage_config)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "failed to find pull-over open-space stage config");
  }
  pull_over_recipe_.name = "pull_over";
  pull_over_recipe_.scenario_type = ScenarioType::PULL_OVER;
  pull_over_recipe_.entry_stage_type = PULL_OVER_RETRY_PARKING;
  pull_over_recipe_.scenario_config = pull_over_config;
  pull_over_recipe_.stage_factory =
      std::make_unique<scenario::pull_over::PullOverScenario>(
          pull_over_recipe_.scenario_config, nullptr, injector_);
  pull_over_recipe_.stage_factory->Init();

  ScenarioConfig park_and_go_config;
  if (!Scenario::LoadConfig(FLAGS_scenario_park_and_go_config_file,
                            &park_and_go_config)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "failed to load park-and-go scenario config");
  }
  if (!ExtractStageConfig(park_and_go_config, PARK_AND_GO_CHECK,
                          &stage_config) ||
      !ExtractStageConfig(park_and_go_config, PARK_AND_GO_ADJUST,
                          &stage_config) ||
      !ExtractStageConfig(park_and_go_config, PARK_AND_GO_PRE_CRUISE,
                          &stage_config)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "failed to find park-and-go open-space stage config");
  }
  pull_out_recipe_.name = "pull_out";
  pull_out_recipe_.scenario_type = ScenarioType::PARK_AND_GO;
  pull_out_recipe_.entry_stage_type = PARK_AND_GO_CHECK;
  pull_out_recipe_.terminal_handoff_stage = PARK_AND_GO_CRUISE;
  pull_out_recipe_.scenario_config = park_and_go_config;
  pull_out_recipe_.stage_factory =
      std::make_unique<scenario::park_and_go::ParkAndGoScenario>(
          pull_out_recipe_.scenario_config, nullptr, injector_);
  pull_out_recipe_.stage_factory->Init();

  return Status::OK();
}

void OpenSpacePlanning::ResetRecipeState() {
  runtime_state_.Reset();
  last_publishable_trajectory_.reset();
  injector_->frame_history()->Clear();
  injector_->history()->Clear();
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_open_space()
      ->Clear();
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_scenario()
      ->Clear();
}

canbus::Chassis::GearPosition OpenSpacePlanning::ResolveOpenSpacePublishedGear(
    const canbus::Chassis::GearPosition segment_gear) const {
  return ResolvePublishedGear(
      PublishedGearInput{MODE_OPEN_SPACE, local_view_.chassis.get(),
                         segment_gear, runtime_state_.last_gear, true, true});
}

void OpenSpacePlanning::ApplyOpenSpaceTrajectoryMetadata(
    ADCTrajectory* ptr_trajectory_pb, const bool handoff_continuation) const {
  CHECK_NOTNULL(ptr_trajectory_pb);
  auto* engage_advice = ptr_trajectory_pb->mutable_engage_advice();
  if (injector_->vehicle_state()->vehicle_state().driving_mode() !=
      canbus::Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
    engage_advice->set_advice(apollo::common::EngageAdvice::READY_TO_ENGAGE);
    engage_advice->set_reason(
        handoff_continuation
            ? "Ready to engage when continuing an open-space handoff"
            : "Ready to engage when starting with OPEN_SPACE_PLANNER");
  } else {
    engage_advice->set_advice(apollo::common::EngageAdvice::KEEP_ENGAGED);
    engage_advice->set_reason(handoff_continuation
                                  ? "Keep engage while completing open-space "
                                    "handoff"
                                  : "Keep engage while in open-space maneuver");
  }

  if (runtime_state_.active_recipe_kind == RecipeKind::kParkIn ||
      runtime_state_.active_recipe_kind == RecipeKind::kPullOver) {
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_parking()
        ->set_status(MainParking::IN_PARKING);
  }
}

void OpenSpacePlanning::ApplyTerminalCompletionDecision(
    ADCTrajectory* ptr_trajectory_pb) const {
  CHECK_NOTNULL(ptr_trajectory_pb);
  if (runtime_state_.recipe_progress != RecipeProgress::kCompleted) {
    return;
  }
  if (runtime_state_.active_recipe_kind != RecipeKind::kParkIn &&
      runtime_state_.active_recipe_kind != RecipeKind::kPullOver) {
    return;
  }
  if (ptr_trajectory_pb->trajectory_point().empty()) {
    return;
  }

  const auto& final_point =
      ptr_trajectory_pb->trajectory_point().rbegin()->path_point();
  auto* mission_complete = ptr_trajectory_pb->mutable_decision()
                               ->mutable_main_decision()
                               ->mutable_mission_complete();
  mission_complete->mutable_stop_point()->set_x(final_point.x());
  mission_complete->mutable_stop_point()->set_y(final_point.y());
  mission_complete->mutable_stop_point()->set_z(final_point.z());
  mission_complete->set_stop_heading(final_point.theta());
}

OpenSpacePlanning::RecipeKind OpenSpacePlanning::ResolveRecipeKind(
    const LocalView& local_view) const {
  if (!SupportsOpenSpaceCommand(local_view)) {
    return RecipeKind::kUnknown;
  }
  if (local_view.planning_command != nullptr &&
      local_view.planning_command->has_requested_scene() &&
      local_view.planning_command->requested_scene() == SCENE_PULL_OVER &&
      local_view.planning_command->has_preferred_mode() &&
      local_view.planning_command->preferred_mode() == MODE_OPEN_SPACE) {
    return RecipeKind::kPullOver;
  }
  if (local_view.planning_command != nullptr &&
      local_view.planning_command->has_requested_scene() &&
      local_view.planning_command->requested_scene() == SCENE_PULL_OUT) {
    return RecipeKind::kPullOut;
  }
  if (GetRequestedParkingInfo(local_view) != nullptr ||
      (local_view.planning_command != nullptr &&
       local_view.planning_command->has_requested_scene() &&
       local_view.planning_command->requested_scene() == SCENE_PARK_IN)) {
    return RecipeKind::kParkIn;
  }
  return RecipeKind::kUnknown;
}

const OpenSpacePlanning::Recipe* OpenSpacePlanning::FindRecipe(
    RecipeKind recipe_kind) const {
  switch (recipe_kind) {
    case RecipeKind::kParkIn:
      return park_in_recipe_.entry_stage_type == StageType::NO_STAGE
                 ? nullptr
                 : &park_in_recipe_;
    case RecipeKind::kPullOver:
      return pull_over_recipe_.entry_stage_type == StageType::NO_STAGE
                 ? nullptr
                 : &pull_over_recipe_;
    case RecipeKind::kPullOut:
      return pull_out_recipe_.entry_stage_type == StageType::NO_STAGE
                 ? nullptr
                 : &pull_out_recipe_;
    case RecipeKind::kUnknown:
    default:
      return nullptr;
  }
}

Status OpenSpacePlanning::CreateStage(const Recipe& recipe,
                                      StageType stage_type) {
  if (recipe.stage_factory == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "open-space recipe missing a stage factory");
  }
  ScenarioConfig::StageConfig stage_config;
  if (!ExtractStageConfig(recipe.scenario_config, stage_type, &stage_config)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "open-space recipe missing stage config");
  }
  auto next_stage = recipe.stage_factory->CreateStage(stage_config, injector_);
  if (next_stage == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "failed to create open-space stage");
  }
  runtime_state_.active_stage_type = stage_type;
  runtime_state_.open_space_stage = std::move(next_stage);
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_scenario()
      ->set_scenario_type(recipe.scenario_type);
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_scenario()
      ->set_stage_type(stage_type);
  return Status::OK();
}

Status OpenSpacePlanning::EnsureStage(RecipeKind recipe_kind) {
  const auto* recipe = FindRecipe(recipe_kind);
  if (recipe == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "open-space recipe unavailable for the requested scene");
  }

  std::string command_id;
  if (local_view_.planning_command != nullptr &&
      local_view_.planning_command->has_command_id()) {
    command_id = local_view_.planning_command->command_id();
  }

  if (runtime_state_.active_recipe_kind == recipe_kind &&
      runtime_state_.active_command_id == command_id &&
      (runtime_state_.open_space_stage != nullptr ||
       runtime_state_.recipe_progress != RecipeProgress::kActive)) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_scenario()
        ->set_scenario_type(recipe->scenario_type);
    if (runtime_state_.active_stage_type != StageType::NO_STAGE) {
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_scenario()
          ->set_stage_type(runtime_state_.active_stage_type);
    }
    return Status::OK();
  }

  ResetRecipeState();
  runtime_state_.active_recipe_kind = recipe_kind;
  runtime_state_.active_command_id = command_id;
  return CreateStage(*recipe, recipe->entry_stage_type);
}

VehicleState OpenSpacePlanning::AlignTimeStamp(
    const VehicleState& vehicle_state, double curr_timestamp) const {
  VehicleState aligned_vehicle_state = vehicle_state;
  aligned_vehicle_state.set_timestamp(curr_timestamp);
  return aligned_vehicle_state;
}

void OpenSpacePlanning::GenerateStopTrajectory(
    ADCTrajectory* ptr_trajectory_pb) const {
  ptr_trajectory_pb->clear_trajectory_point();

  const auto& vehicle_state = injector_->vehicle_state()->vehicle_state();
  const double max_t = FLAGS_fallback_total_time;
  const double unit_t = FLAGS_fallback_time_unit;

  TrajectoryPoint tp;
  auto* path_point = tp.mutable_path_point();
  path_point->set_x(vehicle_state.x());
  path_point->set_y(vehicle_state.y());
  path_point->set_theta(vehicle_state.heading());
  path_point->set_s(0.0);
  tp.set_v(0.0);
  tp.set_a(0.0);
  for (double t = 0.0; t < max_t; t += unit_t) {
    tp.set_relative_time(t);
    auto* next_point = ptr_trajectory_pb->add_trajectory_point();
    next_point->CopyFrom(tp);
  }
}

Status OpenSpacePlanning::InitFrame(const uint32_t sequence_num,
                                    const TrajectoryPoint& planning_start_point,
                                    const VehicleState& vehicle_state) {
  frame_ = std::make_unique<Frame>(sequence_num, local_view_,
                                   planning_start_point, vehicle_state);
  if (frame_ == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "failed to initialize frame");
  }

  const auto* parking_info = GetRequestedParkingInfo(local_view_);
  if (parking_info != nullptr && parking_info->has_parking_space_id()) {
    *frame_->mutable_open_space_info()->mutable_target_parking_spot_id() =
        parking_info->parking_space_id();
  }

  auto status = frame_->InitForOpenSpace(injector_->vehicle_state(),
                                         injector_->ego_info());
  if (!status.ok()) {
    return status;
  }
  return Status::OK();
}

void OpenSpacePlanning::RunOnce(const LocalView& local_view,
                                ADCTrajectory* const ptr_trajectory_pb) {
  local_view_ = local_view;
  const double start_timestamp = Clock::NowInSeconds();
  const double start_system_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();

  if (local_view_.localization_estimate == nullptr ||
      local_view_.chassis == nullptr) {
    const std::string msg =
        "open-space planning missing localization or chassis";
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);
    ptr_trajectory_pb->set_gear(ResolveOpenSpacePublishedGear());
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    GenerateStopTrajectory(ptr_trajectory_pb);
    return;
  }

  Status status = injector_->vehicle_state()->Update(
      *local_view_.localization_estimate, *local_view_.chassis);
  VehicleState vehicle_state = injector_->vehicle_state()->vehicle_state();
  const double vehicle_state_timestamp = vehicle_state.timestamp();

  if (!status.ok() || !util::IsVehicleStateValid(vehicle_state)) {
    const std::string msg =
        "Update VehicleStateProvider failed or the vehicle state is out dated.";
    AERROR << msg;
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    ptr_trajectory_pb->set_gear(ResolveOpenSpacePublishedGear());
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    GenerateStopTrajectory(ptr_trajectory_pb);
    return;
  }

  if (start_timestamp - vehicle_state_timestamp <
      FLAGS_message_latency_threshold) {
    vehicle_state = AlignTimeStamp(vehicle_state, start_timestamp);
  }

  const double planning_cycle_time =
      1.0 / static_cast<double>(FLAGS_planning_loop_rate);
  std::string replan_reason;
  std::vector<TrajectoryPoint> stitching_trajectory =
      TrajectoryStitcher::ComputeStitchingTrajectory(
          vehicle_state, start_timestamp, planning_cycle_time,
          FLAGS_trajectory_stitching_preserved_length, true,
          last_publishable_trajectory_.get(), &replan_reason);

  injector_->ego_info()->Update(stitching_trajectory.back(), vehicle_state);
  const uint32_t frame_num = static_cast<uint32_t>(seq_num_++);
  status = InitFrame(frame_num, stitching_trajectory.back(), vehicle_state);

  if (FLAGS_enable_record_debug && status.ok()) {
    frame_->RecordInputDebug(ptr_trajectory_pb->mutable_debug());
  }
  ptr_trajectory_pb->mutable_latency_stats()->set_init_frame_time_ms(
      Clock::NowInSeconds() - start_timestamp);

  if (!status.ok()) {
    AERROR << status.ToString();
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(status.ToString());
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    ptr_trajectory_pb->set_gear(ResolveOpenSpacePublishedGear());
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    GenerateStopTrajectory(ptr_trajectory_pb);
    if (frame_ != nullptr) {
      frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
      injector_->frame_history()->Add(frame_->SequenceNum(), std::move(frame_));
    }
    return;
  }

  status = Plan(start_timestamp, stitching_trajectory, ptr_trajectory_pb);

  const auto end_system_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  const auto time_diff_ms =
      (end_system_timestamp - start_system_timestamp) * 1000.0;
  ptr_trajectory_pb->mutable_latency_stats()->set_total_time_ms(time_diff_ms);

  if (!status.ok()) {
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    AERROR << "Open-space planning failed: " << status.ToString();
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(status.error_message());
    GenerateStopTrajectory(ptr_trajectory_pb);
    ptr_trajectory_pb->set_gear(ResolveOpenSpacePublishedGear());
  }

  ptr_trajectory_pb->set_is_replan(stitching_trajectory.size() == 1);
  if (ptr_trajectory_pb->is_replan()) {
    ptr_trajectory_pb->set_replan_reason(replan_reason);
  }

  FillPlanningPb(start_timestamp, ptr_trajectory_pb);

  if (status.ok()) {
    last_publishable_trajectory_ =
        std::make_unique<PublishableTrajectory>(*ptr_trajectory_pb);
    runtime_state_.last_gear = ptr_trajectory_pb->gear();
  }

  frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
  injector_->frame_history()->Add(frame_->SequenceNum(), std::move(frame_));
}

void OpenSpacePlanning::FillOpenSpaceTrajectory(
    ADCTrajectory* ptr_trajectory_pb) const {
  CHECK_NOTNULL(frame_);
  const auto& publishable_trajectory =
      frame_->open_space_info().publishable_trajectory_data().first;
  const auto& publishable_trajectory_gear =
      frame_->open_space_info().publishable_trajectory_data().second;
  publishable_trajectory.PopulateTrajectoryProtobuf(ptr_trajectory_pb);
  ptr_trajectory_pb->set_gear(
      ResolveOpenSpacePublishedGear(publishable_trajectory_gear));
  ApplyOpenSpaceTrajectoryMetadata(ptr_trajectory_pb,
                                   /*handoff_continuation=*/false);

  if (FLAGS_enable_record_debug) {
    frame_->mutable_open_space_info()->RecordDebug(
        ptr_trajectory_pb->mutable_debug());
  }
}

void OpenSpacePlanning::FillLastTrajectory(
    ADCTrajectory* ptr_trajectory_pb) const {
  CHECK_NOTNULL(ptr_trajectory_pb);
  ptr_trajectory_pb->clear_trajectory_point();
  if (last_publishable_trajectory_ != nullptr) {
    last_publishable_trajectory_->PopulateTrajectoryProtobuf(ptr_trajectory_pb);
  } else {
    GenerateStopTrajectory(ptr_trajectory_pb);
  }
  ptr_trajectory_pb->set_gear(ResolveOpenSpacePublishedGear());
  ApplyOpenSpaceTrajectoryMetadata(ptr_trajectory_pb,
                                   /*handoff_continuation=*/true);
  ApplyTerminalCompletionDecision(ptr_trajectory_pb);
}

Status OpenSpacePlanning::Plan(
    const double current_time_stamp,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {
  CHECK_NOTNULL(frame_);
  (void)current_time_stamp;
  auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
  if (FLAGS_enable_record_debug) {
    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
        stitching_trajectory.back());
    frame_->mutable_open_space_info()->set_debug(ptr_debug);
    frame_->mutable_open_space_info()->sync_debug_instance();
  }

  const auto recipe_kind = ResolveRecipeKind(local_view_);
  if (recipe_kind == RecipeKind::kUnknown) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "open-space command lacks a supported recipe");
  }

  auto stage_status = EnsureStage(recipe_kind);
  if (!stage_status.ok()) {
    return stage_status;
  }

  const auto* recipe = FindRecipe(recipe_kind);
  CHECK_NOTNULL(recipe);
  if (runtime_state_.recipe_progress != RecipeProgress::kActive) {
    FillLastTrajectory(ptr_trajectory_pb);
    return Status::OK();
  }
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_scenario()
      ->set_stage_type(runtime_state_.active_stage_type);
  if (runtime_state_.open_space_stage == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "open-space runtime lost its active stage");
  }

  const auto result = runtime_state_.open_space_stage->Process(
      stitching_trajectory.back(), frame_.get());
  if (result == scenario::Stage::ERROR) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "open-space stage execution failed");
  }
  if (!frame_->open_space_info().is_on_open_space_trajectory()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "open-space stage did not publish an open-space trajectory");
  }

  if (FLAGS_enable_record_debug) {
    frame_->mutable_open_space_info()->sync_debug_instance();
  }
  FillOpenSpaceTrajectory(ptr_trajectory_pb);

  if (result == scenario::Stage::FINISHED) {
    const auto next_stage = runtime_state_.open_space_stage->NextStage();
    if (next_stage == StageType::NO_STAGE) {
      runtime_state_.recipe_progress = RecipeProgress::kCompleted;
      runtime_state_.active_stage_type = StageType::NO_STAGE;
      runtime_state_.open_space_stage.reset();
    } else if (recipe->terminal_handoff_stage != StageType::NO_STAGE &&
               next_stage == recipe->terminal_handoff_stage) {
      runtime_state_.recipe_progress = RecipeProgress::kHandoffReady;
      runtime_state_.active_stage_type = next_stage;
      runtime_state_.open_space_stage.reset();
    } else if (next_stage != runtime_state_.open_space_stage->stage_type()) {
      auto next_status = CreateStage(*recipe, next_stage);
      if (!next_status.ok()) {
        return next_status;
      }
    }
  }
  ApplyTerminalCompletionDecision(ptr_trajectory_pb);
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
