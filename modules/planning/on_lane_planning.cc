/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/on_lane_planning.h"

#include <algorithm>
#include <list>
#include <utility>

#include "absl/strings/str_cat.h"

#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/planning/proto/planning_semantic_map_config.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/published_trajectory_gear.h"
#include "modules/planning/common/trajectory_stitcher.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/learning_based/img_feature_renderer/birdview_img_feature_renderer.h"
#include "modules/planning/reference_line/reference_line_provider.h"
#include "modules/planning/tasks/task_factory.h"
#include "modules/planning/traffic_rules/traffic_decider.h"

namespace apollo {
namespace planning {
using apollo::canbus::Chassis;
using apollo::common::EngageAdvice;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::common::VehicleStateProvider;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;
using apollo::hdmap::HDMapUtil;

namespace {

const apollo::routing::ParkingInfo* GetRequestedParkingInfo(
    const apollo::planning::LocalView& local_view) {
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

}  // namespace

OnLanePlanning::~OnLanePlanning() {
  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }
  planner_->Stop();
  injector_->frame_history()->Clear();
  injector_->history()->Clear();
  injector_->planning_context()->mutable_planning_status()->Clear();
  last_routing_.Clear();
  injector_->ego_info()->Clear();
}

std::string OnLanePlanning::Name() const { return "on_lane_planning"; }

Status OnLanePlanning::Init(const PlanningConfig& config) {
  config_ = config;
  if (!CheckPlanningConfig(config_)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "planning config error: " + config_.DebugString());
  }

  PlanningBase::Init(config_);

  ACHECK(apollo::cyber::common::GetProtoFromFile(
      FLAGS_traffic_rule_config_filename, &traffic_rule_configs_))
      << "Failed to load traffic rule config file "
      << FLAGS_traffic_rule_config_filename;

  // clear planning history
  injector_->history()->Clear();

  // clear planning status
  injector_->planning_context()->mutable_planning_status()->Clear();

  // load map
  hdmap_ = HDMapUtil::BaseMapPtr();
  ACHECK(hdmap_) << "Failed to load map";

  // instantiate reference line provider
  reference_line_provider_ = std::make_unique<ReferenceLineProvider>(
      injector_->vehicle_state(), hdmap_);
  reference_line_provider_->Start();

  auto planner_status =
      PlannerSelector::CreateStandardPlanner(config_, injector_, &planner_);
  if (!planner_status.ok() || !planner_) {
    return planner_status.ok()
               ? Status(ErrorCode::PLANNING_ERROR,
                        "failed to create planner for standard shell")
               : planner_status;
  }

  if (config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    PlanningSemanticMapConfig renderer_config;
    ACHECK(apollo::cyber::common::GetProtoFromFile(
        FLAGS_planning_birdview_img_feature_renderer_config_file,
        &renderer_config))
        << "Failed to load renderer config"
        << FLAGS_planning_birdview_img_feature_renderer_config_file;

    BirdviewImgFeatureRenderer::Instance()->Init(renderer_config);
  }

  start_time_ = Clock::NowInSeconds();
  return planner_->Init(config_);
}

Status OnLanePlanning::InitFrame(const uint32_t sequence_num,
                                 const TrajectoryPoint& planning_start_point,
                                 const VehicleState& vehicle_state) {
  frame_.reset(new Frame(sequence_num, local_view_, planning_start_point,
                         vehicle_state, reference_line_provider_.get()));

  if (frame_ == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "Fail to init frame: nullptr.");
  }

  const auto* parking_info = GetRequestedParkingInfo(local_view_);
  if (parking_info != nullptr && parking_info->has_parking_space_id()) {
    *(frame_->mutable_open_space_info()->mutable_target_parking_spot_id()) =
        parking_info->parking_space_id();
  }

  std::list<ReferenceLine> reference_lines;
  std::list<hdmap::RouteSegments> segments;
  if (!reference_line_provider_->GetReferenceLines(&reference_lines,
                                                   &segments)) {
    const std::string msg = "Failed to create reference line";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  DCHECK_EQ(reference_lines.size(), segments.size());

  auto forward_limit =
      hdmap::PncMap::LookForwardDistance(vehicle_state.linear_velocity());

  for (auto& ref_line : reference_lines) {
    if (!ref_line.Segment(Vec2d(vehicle_state.x(), vehicle_state.y()),
                          FLAGS_look_backward_distance, forward_limit)) {
      const std::string msg = "Fail to shrink reference line.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  for (auto& seg : segments) {
    if (!seg.Shrink(Vec2d(vehicle_state.x(), vehicle_state.y()),
                    FLAGS_look_backward_distance, forward_limit)) {
      const std::string msg = "Fail to shrink routing segments.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  auto status = frame_->Init(
      injector_->vehicle_state(), reference_lines, segments,
      reference_line_provider_->FutureRouteWaypoints(), injector_->ego_info());
  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }
  return Status::OK();
}

// TODO(all): fix this! this will cause unexpected behavior from controller
void OnLanePlanning::GenerateStopTrajectory(ADCTrajectory* ptr_trajectory_pb) {
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
    auto next_point = ptr_trajectory_pb->add_trajectory_point();
    next_point->CopyFrom(tp);
  }
}

void OnLanePlanning::InitializeCycleState(
    PlanningCycleState* cycle_state) const {
  CHECK_NOTNULL(cycle_state);
  cycle_state->start_timestamp = Clock::NowInSeconds();
  cycle_state->start_system_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
}

void OnLanePlanning::PublishNotReadyStopTrajectory(
    const double start_timestamp, const Status& status,
    const std::string& reason, ADCTrajectory* ptr_trajectory_pb) {
  CHECK_NOTNULL(ptr_trajectory_pb);
  ptr_trajectory_pb->mutable_decision()
      ->mutable_main_decision()
      ->mutable_not_ready()
      ->set_reason(reason);
  if (!status.ok()) {
    auto mutable_status = status;
    mutable_status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
  }
  ptr_trajectory_pb->set_gear(ResolvePublishedGear(
      PublishedGearInput{Mode(), local_view_.chassis.get()}));
  FillPlanningPb(start_timestamp, ptr_trajectory_pb);
  GenerateStopTrajectory(ptr_trajectory_pb);
}

bool OnLanePlanning::UpdateVehicleStateForCycle(
    PlanningCycleState* cycle_state, ADCTrajectory* ptr_trajectory_pb) {
  CHECK_NOTNULL(cycle_state);
  if (local_view_.localization_estimate == nullptr ||
      local_view_.chassis == nullptr) {
    const std::string msg = "on-lane planning missing localization or chassis";
    const Status status(ErrorCode::PLANNING_ERROR, msg);
    AERROR << msg;
    PublishNotReadyStopTrajectory(cycle_state->start_timestamp, status, msg,
                                  ptr_trajectory_pb);
    return false;
  }

  Status status = injector_->vehicle_state()->Update(
      *local_view_.localization_estimate, *local_view_.chassis);
  cycle_state->vehicle_state = injector_->vehicle_state()->vehicle_state();
  const double vehicle_state_timestamp = cycle_state->vehicle_state.timestamp();
  DCHECK_GE(cycle_state->start_timestamp, vehicle_state_timestamp)
      << "start_timestamp is behind vehicle_state_timestamp by "
      << cycle_state->start_timestamp - vehicle_state_timestamp << " secs";

  if (!status.ok() || !util::IsVehicleStateValid(cycle_state->vehicle_state)) {
    const std::string msg =
        "Update VehicleStateProvider failed or the vehicle state is out dated.";
    AERROR << msg;
    PublishNotReadyStopTrajectory(cycle_state->start_timestamp, status, msg,
                                  ptr_trajectory_pb);
    return false;
  }

  if (cycle_state->start_timestamp - vehicle_state_timestamp <
      FLAGS_message_latency_threshold) {
    cycle_state->vehicle_state = AlignTimeStamp(cycle_state->vehicle_state,
                                                cycle_state->start_timestamp);
  }
  return true;
}

bool OnLanePlanning::RefreshReferenceLineForCycle(
    const PlanningCycleState& cycle_state, ADCTrajectory* ptr_trajectory_pb) {
  if (local_view_.routing == nullptr) {
    const std::string msg = "on-lane planning requires routing input";
    const Status status(ErrorCode::PLANNING_ERROR, msg);
    AERROR << msg;
    PublishNotReadyStopTrajectory(cycle_state.start_timestamp, status, msg,
                                  ptr_trajectory_pb);
    return false;
  }

  reference_line_provider_->UpdateVehicleState(cycle_state.vehicle_state);
  if (util::IsDifferentRouting(last_routing_, *local_view_.routing)) {
    last_routing_ = *local_view_.routing;
    injector_->history()->Clear();
    injector_->planning_context()->mutable_planning_status()->Clear();
    reference_line_provider_->UpdateRoutingResponse(*local_view_.routing);
    const auto planner_status = planner_->Init(config_);
    if (!planner_status.ok()) {
      const std::string msg =
          "Failed to reinitialize on-lane planner after routing update.";
      AERROR << msg << " " << planner_status.ToString();
      PublishNotReadyStopTrajectory(cycle_state.start_timestamp, planner_status,
                                    msg, ptr_trajectory_pb);
      return false;
    }
    AINFO << "On-lane planning reset after routing update";
  }

  if (!reference_line_provider_->UpdatedReferenceLine()) {
    const std::string msg = "Failed to update reference line after rerouting.";
    const Status status(ErrorCode::PLANNING_ERROR, msg);
    AERROR << msg;
    PublishNotReadyStopTrajectory(cycle_state.start_timestamp, status, msg,
                                  ptr_trajectory_pb);
    return false;
  }
  return true;
}

void OnLanePlanning::ComputeStitchingTrajectory(
    PlanningCycleState* cycle_state) {
  CHECK_NOTNULL(cycle_state);
  const double planning_cycle_time =
      1.0 / static_cast<double>(FLAGS_planning_loop_rate);
  cycle_state->stitching_trajectory =
      TrajectoryStitcher::ComputeStitchingTrajectory(
          cycle_state->vehicle_state, cycle_state->start_timestamp,
          planning_cycle_time, FLAGS_trajectory_stitching_preserved_length,
          true, last_publishable_trajectory_.get(),
          &cycle_state->replan_reason);
  injector_->ego_info()->Update(cycle_state->stitching_trajectory.back(),
                                cycle_state->vehicle_state);
}

bool OnLanePlanning::PrepareFrameForCycle(PlanningCycleState* cycle_state,
                                          ADCTrajectory* ptr_trajectory_pb) {
  CHECK_NOTNULL(cycle_state);
  cycle_state->frame_num = static_cast<uint32_t>(seq_num_++);
  auto status = InitFrame(cycle_state->frame_num,
                          cycle_state->stitching_trajectory.back(),
                          cycle_state->vehicle_state);
  if (status.ok()) {
    injector_->ego_info()->CalculateFrontObstacleClearDistance(
        frame_->obstacles());
  }

  if (FLAGS_enable_record_debug && frame_ != nullptr) {
    frame_->RecordInputDebug(ptr_trajectory_pb->mutable_debug());
  }
  ptr_trajectory_pb->mutable_latency_stats()->set_init_frame_time_ms(
      Clock::NowInSeconds() - cycle_state->start_timestamp);

  if (status.ok()) {
    return true;
  }

  AERROR << status.ToString();
  if (FLAGS_publish_estop) {
    ADCTrajectory estop_trajectory;
    EStop* estop = estop_trajectory.mutable_estop();
    estop->set_is_estop(true);
    estop->set_reason(status.error_message());
    status.Save(estop_trajectory.mutable_header()->mutable_status());
    ptr_trajectory_pb->CopyFrom(estop_trajectory);
  } else {
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(status.ToString());
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    GenerateStopTrajectory(ptr_trajectory_pb);
  }
  ptr_trajectory_pb->set_gear(ResolvePublishedGear(
      PublishedGearInput{Mode(), local_view_.chassis.get()}));
  FillPlanningPb(cycle_state->start_timestamp, ptr_trajectory_pb);
  FinalizeFrameHistory(ptr_trajectory_pb);
  return false;
}

void OnLanePlanning::ApplyTrafficRulesToFrame() {
  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    TrafficDecider traffic_decider;
    traffic_decider.Init(traffic_rule_configs_);
    const auto traffic_status =
        traffic_decider.Execute(frame_.get(), &ref_line_info, injector_);
    if (!traffic_status.ok() || !ref_line_info.IsDrivable()) {
      ref_line_info.SetDrivable(false);
      AWARN << "Reference line " << ref_line_info.Lanes().Id()
            << " traffic decider failed";
    }
  }
}

void OnLanePlanning::FinalizeFrameHistory(ADCTrajectory* ptr_trajectory_pb) {
  if (frame_ == nullptr) {
    return;
  }
  frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
  injector_->frame_history()->Add(frame_->SequenceNum(), std::move(frame_));
}

void OnLanePlanning::LogPlanningCycle(const PlanningCycleState& cycle_state,
                                      const Status& plan_status,
                                      const ADCTrajectory& trajectory_pb) {
  const bool open_space_trajectory =
      frame_ != nullptr &&
      frame_->open_space_info().is_on_open_space_trajectory();
  const std::string summary = absl::StrCat(
      "On-lane cycle frame=", cycle_state.frame_num,
      " status=", plan_status.ok() ? "OK" : plan_status.error_message(),
      " replan=", trajectory_pb.is_replan() ? "true" : "false",
      " open_space=", open_space_trajectory ? "true" : "false",
      " points=", trajectory_pb.trajectory_point_size(),
      " total_time_ms=", trajectory_pb.latency_stats().total_time_ms());
  const bool should_log_info = !plan_status.ok() || trajectory_pb.is_replan() ||
                               open_space_trajectory ||
                               cycle_state.frame_num % 200 == 0;
  if (should_log_info) {
    AINFO << summary;
    return;
  }
  ADEBUG << summary;
}

void OnLanePlanning::FinalizeTrajectoryForCycle(
    const PlanningCycleState& cycle_state, const Status& plan_status,
    ADCTrajectory* ptr_trajectory_pb) {
  CHECK_NOTNULL(ptr_trajectory_pb);

  const auto end_system_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  const auto time_diff_ms =
      (end_system_timestamp - cycle_state.start_system_timestamp) * 1000;
  ptr_trajectory_pb->mutable_latency_stats()->set_total_time_ms(time_diff_ms);

  if (!plan_status.ok()) {
    auto mutable_status = plan_status;
    mutable_status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    AERROR << "Planning failed:" << plan_status.ToString();
    if (FLAGS_publish_estop) {
      AERROR << "Planning failed and set estop";
      EStop* estop = ptr_trajectory_pb->mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(plan_status.error_message());
    }
  }

  ptr_trajectory_pb->set_is_replan(cycle_state.stitching_trajectory.size() ==
                                   1);
  if (ptr_trajectory_pb->is_replan()) {
    ptr_trajectory_pb->set_replan_reason(cycle_state.replan_reason);
  }

  if (frame_->open_space_info().is_on_open_space_trajectory()) {
    FillPlanningPb(cycle_state.start_timestamp, ptr_trajectory_pb);
  } else {
    auto* ref_line_task =
        ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
    ref_line_task->set_time_ms(reference_line_provider_->LastTimeDelay() *
                               1000.0);
    ref_line_task->set_name("ReferenceLineProvider");
    ptr_trajectory_pb->set_gear(ResolvePublishedGear(
        PublishedGearInput{Mode(), local_view_.chassis.get()}));
    FillPlanningPb(cycle_state.start_timestamp, ptr_trajectory_pb);

    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
    if (FLAGS_enable_planning_smoother) {
      planning_smoother_.Smooth(injector_->frame_history(), frame_.get(),
                                ptr_trajectory_pb);
    }
  }

  LogPlanningCycle(cycle_state, plan_status, *ptr_trajectory_pb);
  FinalizeFrameHistory(ptr_trajectory_pb);
}

void OnLanePlanning::InitializePlannerDebug(
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* ptr_trajectory_pb) {
  if (!FLAGS_enable_record_debug) {
    return;
  }
  auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
  ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
      stitching_trajectory.back());
  frame_->mutable_open_space_info()->set_debug(ptr_debug);
  frame_->mutable_open_space_info()->sync_debug_instance();
}

void OnLanePlanning::PopulateOpenSpacePlanResult(
    ADCTrajectory* ptr_trajectory_pb) {
  frame_->mutable_open_space_info()->sync_debug_instance();
  const auto& publishable_trajectory =
      frame_->open_space_info().publishable_trajectory_data().first;
  const auto& publishable_trajectory_gear =
      frame_->open_space_info().publishable_trajectory_data().second;
  publishable_trajectory.PopulateTrajectoryProtobuf(ptr_trajectory_pb);
  ptr_trajectory_pb->set_gear(ResolvePublishedGear(PublishedGearInput{
      MODE_OPEN_SPACE, local_view_.chassis.get(), publishable_trajectory_gear,
      publishable_trajectory_gear, true, true}));

  auto* engage_advice = ptr_trajectory_pb->mutable_engage_advice();
  if (injector_->vehicle_state()->vehicle_state().driving_mode() !=
      Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
    engage_advice->set_advice(EngageAdvice::READY_TO_ENGAGE);
    engage_advice->set_reason(
        "Ready to engage when staring with OPEN_SPACE_PLANNER");
  } else {
    engage_advice->set_advice(EngageAdvice::KEEP_ENGAGED);
    engage_advice->set_reason("Keep engage while in parking");
  }
  ptr_trajectory_pb->mutable_decision()
      ->mutable_main_decision()
      ->mutable_parking()
      ->set_status(MainParking::IN_PARKING);

  if (FLAGS_enable_record_debug) {
    auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
    frame_->mutable_open_space_info()->RecordDebug(ptr_debug);
    debug_exporter_.ExportOpenSpaceChart(frame_.get(),
                                         ptr_trajectory_pb->debug(),
                                         *ptr_trajectory_pb, ptr_debug);
  }
}

void OnLanePlanning::BuildFallbackPathForNextCycle(
    const ReferenceLineInfo& best_ref_info,
    const std::vector<TrajectoryPoint>& stitching_trajectory) {
  DiscretizedPath current_frame_planned_path;
  for (const auto& trajectory_point : stitching_trajectory) {
    current_frame_planned_path.push_back(trajectory_point.path_point());
  }
  const auto& best_ref_path = best_ref_info.path_data().discretized_path();
  std::copy(best_ref_path.begin() + 1, best_ref_path.end(),
            std::back_inserter(current_frame_planned_path));
  frame_->set_current_frame_planned_path(current_frame_planned_path);
}

void OnLanePlanning::ExportOnLanePlanDebug(
    const ReferenceLineInfo& best_ref_info,
    planning_internal::Debug* ptr_debug) {
  ptr_debug->MergeFrom(best_ref_info.debug());
  if (FLAGS_export_chart) {
    debug_exporter_.ExportOnLaneChart(best_ref_info.debug(), ptr_debug);
  } else {
    debug_exporter_.ExportReferenceLineDebug(frame_.get(), ptr_debug);
    const auto* failed_ref_info = frame_->FindFailedReferenceLineInfo();
    if (failed_ref_info != nullptr) {
      debug_exporter_.ExportFailedLaneChangeSTChart(failed_ref_info->debug(),
                                                    ptr_debug);
    }
  }
  debug_exporter_.ExportPlanningReferenceLinePath(best_ref_info, ptr_debug);
}

Status OnLanePlanning::PopulateOnLanePlanResult(
    const double current_time_stamp,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* ptr_trajectory_pb) {
  const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
  const auto* target_ref_info = frame_->FindTargetReferenceLineInfo();
  if (best_ref_info == nullptr) {
    const std::string msg = "planner failed to make a driving plan";
    AERROR << msg;
    if (last_publishable_trajectory_) {
      last_publishable_trajectory_->Clear();
    }
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  BuildFallbackPathForNextCycle(*best_ref_info, stitching_trajectory);

  auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
  ExportOnLanePlanDebug(*best_ref_info, ptr_debug);
  ptr_trajectory_pb->mutable_latency_stats()->MergeFrom(
      best_ref_info->latency_stats());
  ptr_trajectory_pb->set_right_of_way_status(
      best_ref_info->GetRightOfWayStatus());

  for (const auto& id : best_ref_info->TargetLaneId()) {
    ptr_trajectory_pb->add_lane_id()->CopyFrom(id);
  }
  if (target_ref_info != nullptr) {
    for (const auto& id : target_ref_info->TargetLaneId()) {
      ptr_trajectory_pb->add_target_lane_id()->CopyFrom(id);
    }
  }

  ptr_trajectory_pb->set_trajectory_type(best_ref_info->trajectory_type());
  if (FLAGS_enable_rss_info) {
    *ptr_trajectory_pb->mutable_rss_info() = best_ref_info->rss_info();
  }

  best_ref_info->ExportDecision(ptr_trajectory_pb->mutable_decision(),
                                injector_->planning_context());

  last_publishable_trajectory_ = std::make_unique<PublishableTrajectory>(
      current_time_stamp, best_ref_info->trajectory());
  last_publishable_trajectory_->PrependTrajectoryPoints(
      std::vector<TrajectoryPoint>(stitching_trajectory.begin(),
                                   stitching_trajectory.end() - 1));
  last_publishable_trajectory_->PopulateTrajectoryProtobuf(ptr_trajectory_pb);

  best_ref_info->ExportEngageAdvice(ptr_trajectory_pb->mutable_engage_advice(),
                                    injector_->planning_context());
  return Status::OK();
}

void OnLanePlanning::RunOnce(const LocalView& local_view,
                             ADCTrajectory* const ptr_trajectory_pb) {
  local_view_ = local_view;
  PlanningCycleState cycle_state;
  InitializeCycleState(&cycle_state);

  // Step 1: refresh ego state for this cycle.
  if (!UpdateVehicleStateForCycle(&cycle_state, ptr_trajectory_pb)) {
    return;
  }

  // Step 2: refresh routing/reference-line state before building the frame.
  if (!RefreshReferenceLineForCycle(cycle_state, ptr_trajectory_pb)) {
    return;
  }

  // Step 3: compute stitching trajectory and initialize the planning frame.
  ComputeStitchingTrajectory(&cycle_state);
  if (!PrepareFrameForCycle(&cycle_state, ptr_trajectory_pb)) {
    return;
  }

  // Step 4: apply traffic rules and execute the planner.
  ApplyTrafficRulesToFrame();
  const auto plan_status =
      Plan(cycle_state.start_timestamp, cycle_state.stitching_trajectory,
           ptr_trajectory_pb);

  // Step 5: finalize trajectory metadata, smoothing, and frame history.
  FinalizeTrajectoryForCycle(cycle_state, plan_status, ptr_trajectory_pb);
}

Status OnLanePlanning::Plan(
    const double current_time_stamp,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {
  InitializePlannerDebug(stitching_trajectory, ptr_trajectory_pb);

  auto status = planner_->Plan(stitching_trajectory.back(), frame_.get(),
                               ptr_trajectory_pb);

  ptr_trajectory_pb->mutable_debug()
      ->mutable_planning_data()
      ->set_front_clear_distance(injector_->ego_info()->front_clear_distance());

  if (frame_->open_space_info().is_on_open_space_trajectory()) {
    PopulateOpenSpacePlanResult(ptr_trajectory_pb);
    return status;
  }

  const auto publish_status = PopulateOnLanePlanResult(
      current_time_stamp, stitching_trajectory, ptr_trajectory_pb);
  if (!publish_status.ok()) {
    return publish_status;
  }

  return status;
}

bool OnLanePlanning::CheckPlanningConfig(const PlanningConfig& config) {
  if (!config.has_standard_planning_config()) {
    return false;
  }
  const auto planner_type = PlannerSelector::ResolveStandardPlannerType(config);
  if (planner_type == PlannerType::PUBLIC_ROAD &&
      !config.standard_planning_config().has_planner_public_road_config()) {
    return false;
  }
  return planner_type == PlannerType::PUBLIC_ROAD ||
         planner_type == PlannerType::LATTICE ||
         planner_type == PlannerType::RTK;
}

VehicleState OnLanePlanning::AlignTimeStamp(const VehicleState& vehicle_state,
                                            const double curr_timestamp) const {
  // TODO(Jinyun): use the same method in trajectory stitching
  //               for forward prediction
  auto future_xy = injector_->vehicle_state()->EstimateFuturePosition(
      curr_timestamp - vehicle_state.timestamp());

  VehicleState aligned_vehicle_state = vehicle_state;
  aligned_vehicle_state.set_x(future_xy.x());
  aligned_vehicle_state.set_y(future_xy.y());
  aligned_vehicle_state.set_timestamp(curr_timestamp);
  return aligned_vehicle_state;
}

}  // namespace planning
}  // namespace apollo
