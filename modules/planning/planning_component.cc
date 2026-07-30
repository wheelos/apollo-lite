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
#include "modules/planning/planning_component.h"

#include <sstream>

#include "cyber/common/file.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {

using apollo::cyber::ComponentBase;
using apollo::hdmap::HDMapUtil;
using apollo::perception::TrafficLightDetection;
using apollo::planning::PlanningCommand;
using apollo::relative_map::MapMsg;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using apollo::storytelling::Stories;

namespace {

bool ModeNeedsHdMap(PlanningMode mode) { return mode == MODE_LANE_GRAPH; }

bool ModeNeedsRelativeMap(PlanningMode mode) {
  return mode == MODE_CORRIDOR || mode == MODE_FREE_SPACE;
}

PlanningShellType ResolveShellForMode(PlanningMode mode) {
  switch (mode) {
    case MODE_LANE_GRAPH:
      return PLANNING_SHELL_ON_LANE;
    case MODE_CORRIDOR:
      return PLANNING_SHELL_CORRIDOR;
    case MODE_FREE_SPACE:
      return PLANNING_SHELL_STRUCTURED_MAPLESS;
    case MODE_OPEN_SPACE:
      return PLANNING_SHELL_OPEN_SPACE;
    case MODE_SAFETY_HOLD:
      return PLANNING_SHELL_SAFETY_HOLD;
    case MODE_UNKNOWN:
    default:
      return PLANNING_SHELL_UNKNOWN;
  }
}

bool BuildRoutingRequest(
    const PlanningCommand& command,
    const localization::LocalizationEstimate& localization,
    RoutingRequest* request) {
  CHECK_NOTNULL(request);
  if (!command.has_goal() || !localization.has_pose() ||
      !localization.pose().has_position()) {
    return false;
  }

  auto* start = request->add_waypoint();
  start->mutable_pose()->CopyFrom(localization.pose().position());
  if (localization.pose().has_heading()) {
    start->set_heading(localization.pose().heading());
  }

  if (command.has_route_hint()) {
    for (const auto& waypoint : command.route_hint().waypoint()) {
      request->add_waypoint()->CopyFrom(waypoint);
    }
  }

  auto* destination = request->add_waypoint();
  switch (command.goal().target_case()) {
    case GoalSpec::kGoalPose:
      destination->mutable_pose()->CopyFrom(command.goal().goal_pose());
      break;
    case GoalSpec::kParkingGoal:
      request->mutable_parking_info()->CopyFrom(
          command.goal().parking_goal());
      if (!command.goal().parking_goal().has_parking_point()) {
        return false;
      }
      destination->mutable_pose()->CopyFrom(
          command.goal().parking_goal().parking_point());
      break;
    case GoalSpec::kTargetPolygon:
    case GoalSpec::kSemanticTargetId:
    case GoalSpec::TARGET_NOT_SET:
    default:
      return false;
  }
  if (command.goal().has_goal_heading()) {
    destination->set_heading(command.goal().goal_heading());
  }
  return true;
}

PlanningSemanticInput BuildSemanticInput(
    const LocalView& local_view,
    const PlanningCoordinator* planning_coordinator,
    const ADCTrajectory* trajectory,
    const ValidationResult& validation_result) {
  PlanningSemanticInput input;
  if (planning_coordinator != nullptr) {
    input.planning_state = &planning_coordinator->state();
  }
  input.chassis = local_view.chassis.get();
  input.localization = local_view.localization_estimate.get();
  input.trajectory = trajectory;
  input.validation_should_hold = validation_result.should_hold;
  input.validation_reason = validation_result.reason;
  return input;
}

ControlExecutionChannel ResolveExecutionChannel(
    const ADCTrajectory& trajectory) {
  if (trajectory.has_control_intent() &&
      trajectory.control_intent().has_execution_channel()) {
    return trajectory.control_intent().execution_channel();
  }
  if (trajectory.trajectory_point_size() > 0) {
    return EXECUTION_CHANNEL_TRAJECTORY;
  }
  return EXECUTION_CHANNEL_UNKNOWN;
}

}  // namespace

bool PlanningComponent::Init() {
  injector_ = std::make_shared<DependencyInjector>();
  motion_plan_builder_.SetProducerEpoch(
      "planning-" +
      std::to_string(cyber::Time::Now().ToNanosecond()));
  planning_coordinator_ = std::make_unique<PlanningCoordinator>(injector_);

  ACHECK(ComponentBase::GetProtoConfig(&config_))
      << "failed to load planning config file "
      << ComponentBase::ConfigFilePath();

  if (FLAGS_planning_offline_learning ||
      config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    if (!message_process_.Init(config_, injector_)) {
      AERROR << "failed to init MessageProcess";
      return false;
    }
  }

  auto init_status =
      planning_coordinator_->Init(config_, FLAGS_use_navigation_mode);
  if (!init_status.ok()) {
    AERROR << "failed to init PlanningCoordinator: " << init_status.ToString();
    return false;
  }

  if (!FLAGS_use_navigation_mode) {
    routing_service_ = std::make_unique<routing::RoutingService>();
    auto routing_status = routing_service_->Init();
    if (routing_status.ok()) {
      routing_status = routing_service_->Start();
    }
    if (!routing_status.ok()) {
      AERROR << "failed to initialize RoutingService: "
             << routing_status.ToString();
      return false;
    }
  }

  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      config_.topic_config().traffic_light_detection_topic(),
      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
        std::lock_guard<std::mutex> lock(mutex_);
        traffic_light_.CopyFrom(*traffic_light);
      });

  pad_msg_reader_ = node_->CreateReader<PadMessage>(
      config_.topic_config().planning_pad_topic(),
      [this](const std::shared_ptr<PadMessage>& pad_msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        pad_msg_.CopyFrom(*pad_msg);
      });

  planning_command_reader_ = node_->CreateReader<PlanningCommand>(
      config_.topic_config().planning_command_topic(),
      [this](const std::shared_ptr<PlanningCommand>& planning_command) {
        std::lock_guard<std::mutex> lock(mutex_);
        planning_command_.CopyFrom(*planning_command);
      });

  mission_directive_reader_ = node_->CreateReader<MissionDirective>(
      config_.topic_config().mission_directive_topic(),
      [this](const std::shared_ptr<MissionDirective>& directive) {
        std::lock_guard<std::mutex> lock(mutex_);
        mission_directive_.CopyFrom(*directive);
      });
  control_runtime_status_reader_ =
      node_->CreateReader<control::ControlRuntimeStatus>(
          config_.topic_config().control_runtime_status_topic(),
          [this](const std::shared_ptr<control::ControlRuntimeStatus>& status) {
            std::lock_guard<std::mutex> lock(mutex_);
            control_runtime_status_.CopyFrom(*status);
          });

  story_telling_reader_ = node_->CreateReader<Stories>(
      config_.topic_config().story_telling_topic(),
      [this](const std::shared_ptr<Stories>& stories) {
        std::lock_guard<std::mutex> lock(mutex_);
        stories_.CopyFrom(*stories);
      });

  relative_map_reader_ = node_->CreateReader<MapMsg>(
      config_.topic_config().relative_map_topic(),
      [this](const std::shared_ptr<MapMsg>& map_message) {
        std::lock_guard<std::mutex> lock(mutex_);
        relative_map_.CopyFrom(*map_message);
      });
  planning_writer_ = node_->CreateWriter<ADCTrajectory>(
      config_.topic_config().planning_trajectory_topic());
  planning_runtime_status_writer_ = node_->CreateWriter<PlanningRuntimeStatus>(
      config_.topic_config().planning_runtime_status_topic());
  motion_directive_writer_ = node_->CreateWriter<MotionDirective>(
      config_.topic_config().motion_directive_topic());

  planning_learning_data_writer_ = node_->CreateWriter<PlanningLearningData>(
      config_.topic_config().planning_learning_data_topic());

  return true;
}

void PlanningComponent::ApplyControlMotionStatus() {
  control::ControlRuntimeStatus status;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status.CopyFrom(control_runtime_status_);
  }
  if (!status.has_motion_execution() || !status.has_motion_scope()) {
    return;
  }
  const std::string fingerprint = status.SerializeAsString();
  if (fingerprint == applied_control_motion_status_fingerprint_) {
    return;
  }
  applied_control_motion_status_fingerprint_ = fingerprint;
  motion_plan_builder_.ObserveControlStatus(status.motion_execution(),
                                            status.motion_scope());
  if (planning_coordinator_ != nullptr &&
      planning_coordinator_->state().mission_session_state ==
          MISSION_SESSION_ACCEPTED &&
      status.motion_scope() == MOTION_SCOPE_MISSION_DESCENDANT &&
      status.executor_owner_active() &&
      (status.motion_execution().state() ==
           MOTION_EXECUTION_EXECUTING_TRAJECTORY ||
       status.motion_execution().state() ==
           MOTION_EXECUTION_EXECUTING_PRIMITIVE)) {
    const auto result = planning_coordinator_->MarkMissionExecuting();
    if (!result.accepted) {
      AERROR << "Failed to mark Mission executing: " << result.reason;
    }
  }
  if (planning_coordinator_ != nullptr &&
      planning_coordinator_->state().mission_session_state ==
          MISSION_SESSION_CANCELLING &&
      status.motion_scope() == MOTION_SCOPE_PLANNING_IDLE_HOLD &&
      status.executor_owner_active() &&
      (status.motion_execution().state() ==
           MOTION_EXECUTION_EXECUTING_PRIMITIVE ||
       status.motion_execution().state() == MOTION_EXECUTION_HOLDING)) {
    const auto result =
        planning_coordinator_->ConfirmMissionCancellation(true);
    if (!result.accepted) {
      AERROR << "Failed to confirm Mission cancellation: " << result.reason;
    }
  }
  if (planning_coordinator_ != nullptr &&
      planning_coordinator_->state().mission_session_state ==
          MISSION_SESSION_COMPLETING &&
      status.motion_scope() == MOTION_SCOPE_PLANNING_IDLE_HOLD &&
      status.executor_owner_active() &&
      (status.motion_execution().state() ==
           MOTION_EXECUTION_EXECUTING_PRIMITIVE ||
       status.motion_execution().state() == MOTION_EXECUTION_HOLDING)) {
    const auto result = planning_coordinator_->CompleteMission();
    if (!result.accepted) {
      AERROR << "Failed to complete Mission: " << result.reason;
    }
  }
}

void PlanningComponent::PublishMotionPlan(
    const PlanningCoordinatorState& coordinator_state,
    const PlanningSemanticSummary& semantic_summary,
    const canbus::Chassis& chassis,
    const localization::LocalizationEstimate& localization,
    const ADCTrajectory& trajectory) {
  if (motion_directive_writer_ == nullptr) {
    return;
  }
  auto motion_state = coordinator_state;
  if (semantic_summary.command_completed &&
      semantic_summary.full_stop_reached &&
      coordinator_state.mission_session_state ==
          MISSION_SESSION_EXECUTING) {
    const auto transition =
        planning_coordinator_->BeginMissionCompleting();
    if (transition.accepted) {
      motion_state.mission_session_state = MISSION_SESSION_COMPLETING;
    }
  }
  auto result = motion_plan_builder_.Build(
      motion_state, semantic_summary, chassis, localization, trajectory,
      cyber::Clock::NowInSeconds());
  if (!result.has_directive) {
    AWARN_EVERY(100) << "MotionPlanBuilder did not emit: " << result.reason;
    return;
  }
  common::util::FillHeader(node_->Name(), &result.directive);
  motion_directive_writer_->Write(result.directive);
}

void PlanningComponent::RefreshLocalView(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  local_view_.prediction_obstacles = prediction_obstacles;
  local_view_.chassis = chassis;
  local_view_.localization_estimate = localization_estimate;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!local_view_.routing ||
      hdmap::PncMap::IsNewRouting(*local_view_.routing, routing_)) {
    local_view_.routing = std::make_shared<routing::RoutingResponse>(routing_);
  }
  local_view_.traffic_light =
      std::make_shared<TrafficLightDetection>(traffic_light_);
  local_view_.relative_map = std::make_shared<MapMsg>(relative_map_);
  local_view_.pad_msg = std::make_shared<PadMessage>(pad_msg_);
  local_view_.planning_command =
      std::make_shared<PlanningCommand>(planning_command_);
  local_view_.stories = std::make_shared<Stories>(stories_);
}

void PlanningComponent::RefreshEnvironmentState() {
  local_view_.environment_model = std::make_shared<EnvironmentModel>(
      environment_model_builder_.Build(local_view_));
  local_view_.capability_set = std::make_shared<CapabilitySet>(
      capability_extractor_.Extract(*local_view_.environment_model));
}

void PlanningComponent::ApplyPendingMissionDirective(
    const localization::LocalizationEstimate& localization) {
  MissionDirective directive;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    directive.CopyFrom(mission_directive_);
  }
  if (!directive.has_identity() || planning_coordinator_ == nullptr) {
    return;
  }
  const std::string fingerprint = directive.SerializeAsString();
  if (fingerprint == applied_mission_directive_fingerprint_) {
    return;
  }
  const auto result = planning_coordinator_->ApplyMissionDirective(
      directive, localization, cyber::Clock::NowInSeconds());
  if (!result.accepted) {
    AERROR << "Mission Directive V2 rejected: " << result.reason;
  } else {
    applied_mission_directive_fingerprint_ = fingerprint;
    AINFO << "Mission Directive V2 admitted: " << result.reason;
  }
}

void PlanningComponent::ProcessLearningInputs() {
  message_process_.OnChassis(*local_view_.chassis);
  message_process_.OnPrediction(*local_view_.prediction_obstacles);
  message_process_.OnRoutingResponse(*local_view_.routing);
  message_process_.OnStoryTelling(*local_view_.stories);
  message_process_.OnTrafficLightDetection(*local_view_.traffic_light);
  message_process_.OnLocalization(*local_view_.localization_estimate);
}

bool PlanningComponent::PublishLearningDataFrame() {
  PlanningLearningData planning_learning_data;
  LearningDataFrame* learning_data_frame =
      injector_->learning_based_data()->GetLatestLearningDataFrame();
  if (learning_data_frame == nullptr) {
    AERROR << "failed to generate planning learning data frame";
    return false;
  }
  planning_learning_data.mutable_learning_data_frame()->CopyFrom(
      *learning_data_frame);
  common::util::FillHeader(node_->Name(), &planning_learning_data);
  planning_learning_data_writer_->Write(planning_learning_data);
  return true;
}

void PlanningComponent::FinalizeTrajectoryTiming(
    double original_start_time_sec, ADCTrajectory* trajectory) const {
  CHECK_NOTNULL(trajectory);
  common::util::FillHeader(node_->Name(), trajectory);
  const double dt =
      original_start_time_sec - trajectory->header().timestamp_sec();
  for (auto& point : *trajectory->mutable_trajectory_point()) {
    point.set_relative_time(point.relative_time() + dt);
  }
}

RuntimeState PlanningComponent::InferCoordinatorRuntimeState() const {
  if (planning_coordinator_ == nullptr) {
    return RUNTIME_UNKNOWN;
  }
  switch (planning_coordinator_->state().mission_session_state) {
    case MISSION_SESSION_COMPLETED:
      return RUNTIME_COMPLETED;
    case MISSION_SESSION_CANCELLED:
      return RUNTIME_CANCELLED;
    case MISSION_SESSION_FAILED:
      return RUNTIME_FAILED;
    case MISSION_SESSION_CANCELLING:
    case MISSION_SESSION_COMPLETING:
      return RUNTIME_HOLDING;
    default:
      break;
  }
  if (planning_coordinator_->state().requested_mode !=
      planning_coordinator_->state().resolved_mode) {
    return planning_coordinator_->state().resolved_mode == MODE_UNKNOWN
               ? RUNTIME_HOLDING
               : RUNTIME_DEGRADED;
  }
  return RUNTIME_RUNNING;
}

HybridManeuverSummary PlanningComponent::EvaluateHybridManeuver(
    const PlanningCoordinatorState& coordinator_state,
    RuntimeState runtime_state) const {
  return hybrid_maneuver_supervisor_.Evaluate(
      coordinator_state, &injector_->planning_context()->planning_status(),
      runtime_state);
}

PlanningExecutionContext PlanningComponent::ResolvePublishedExecutionContext(
    const PlanningCoordinatorState& coordinator_state,
    const ADCTrajectory& trajectory) const {
  PlanningExecutionContext execution;
  if (!coordinator_state.mission_id.empty()) {
    execution.set_mission_id(coordinator_state.mission_id);
  }
  if (!coordinator_state.command_id.empty()) {
    execution.set_command_id(coordinator_state.command_id);
  }
  execution.set_active_scene(coordinator_state.active_scene);
  execution.set_requested_mode(coordinator_state.requested_mode);
  execution.set_active_mode(coordinator_state.resolved_mode);
  execution.set_active_shell(coordinator_state.active_shell);
  execution.set_active_domain(coordinator_state.active_domain);
  execution.set_execution_channel(ResolveExecutionChannel(trajectory));
  if (!coordinator_state.reason.empty()) {
    execution.set_reason(coordinator_state.reason);
  }
  for (const auto& blocker : coordinator_state.blockers) {
    execution.add_blockers(blocker);
  }

  if (!trajectory.has_execution()) {
    return execution;
  }

  const auto& published_execution = trajectory.execution();
  if (published_execution.has_mission_id()) {
    execution.set_mission_id(published_execution.mission_id());
  }
  if (published_execution.has_command_id()) {
    execution.set_command_id(published_execution.command_id());
  }
  if (published_execution.has_reason()) {
    execution.set_reason(published_execution.reason());
  }
  if (published_execution.blockers_size() > 0) {
    execution.clear_blockers();
    for (const auto& blocker : published_execution.blockers()) {
      execution.add_blockers(blocker);
    }
  }
  if (published_execution.has_execution_channel()) {
    execution.set_execution_channel(published_execution.execution_channel());
  }
  return execution;
}

bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  ACHECK(prediction_obstacles != nullptr);

  PlanningCycleState cycle_state;

  // Step 1: service latched planning side effects from previous cycles.
  CheckRerouting();

  // Step 2: build the cycle snapshot from fast inputs and latched inputs.
  RefreshLocalView(prediction_obstacles, chassis, localization_estimate);
  ApplyPendingMissionDirective(*localization_estimate);
  UpdateRoutingForMission(*localization_estimate);
  ApplyControlMotionStatus();
  RefreshEnvironmentState();
  if (planning_coordinator_ != nullptr) {
    cycle_state.preview_state =
        planning_coordinator_->PreviewState(local_view_);
  }

  // Step 3: reject or hold early if the selected shell cannot legally execute.
  if (!CheckInput(cycle_state.preview_state, &cycle_state.validation_result)) {
    return false;
  }

  // Step 4: feed the learning path before any planner execution side effects.
  if (config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    ProcessLearningInputs();
  }

  // Step 5: publish learning-only data when the component is in RL test mode.
  if (config_.learning_mode() == PlanningConfig::RL_TEST) {
    return PublishLearningDataFrame();
  }

  // Step 6: execute the selected shell once and normalize the trajectory
  // header.
  ADCTrajectory adc_trajectory_pb;
  planning_coordinator_->RunOnce(local_view_, &adc_trajectory_pb);
  auto start_time = adc_trajectory_pb.header().timestamp_sec();
  FinalizeTrajectoryTiming(start_time, &adc_trajectory_pb);

  // Step 7: validate the raw planning output before applying semantics.
  cycle_state.validation_result =
      validation_supervisor_.Validate(ValidationInput{
          &local_view_, &planning_coordinator_->state(), &adc_trajectory_pb});
  if (cycle_state.validation_result.should_hold) {
    terminal_servo_session_state_ = TerminalServoSessionState();
    auto* not_ready = adc_trajectory_pb.mutable_decision()
                          ->mutable_main_decision()
                          ->mutable_not_ready();
    if (!not_ready->has_reason() &&
        !cycle_state.validation_result.reason.empty()) {
      not_ready->set_reason(cycle_state.validation_result.reason);
    }
    cycle_state.semantic_summary = InferPlanningSemantics(
        BuildSemanticInput(local_view_, planning_coordinator_.get(),
                           &adc_trajectory_pb, cycle_state.validation_result),
        cycle_state.validation_result.command_admissible ? RUNTIME_HOLDING
                                                         : RUNTIME_REJECTED);
    cycle_state.hybrid_summary =
        EvaluateHybridManeuver(planning_coordinator_->state(),
                               cycle_state.semantic_summary.runtime_state);
    ApplyPlanningSemanticsToTrajectory(cycle_state.semantic_summary,
                                       &adc_trajectory_pb);
    PopulateTrajectoryExecutionContext(
        planning_coordinator_->state(), cycle_state.semantic_summary,
        cycle_state.hybrid_summary, &adc_trajectory_pb);
    PublishMotionPlan(planning_coordinator_->state(),
                      cycle_state.semantic_summary, *chassis,
                      *localization_estimate, adc_trajectory_pb);
    planning_writer_->Write(adc_trajectory_pb);
    PublishRuntimeStatus(
        cycle_state.semantic_summary, cycle_state.hybrid_summary,
        cycle_state.validation_result, planning_coordinator_->state(),
        adc_trajectory_pb.execution(), cycle_state.validation_result.reason);
    LogPlanningCycle(planning_coordinator_->state(),
                     cycle_state.semantic_summary, cycle_state.hybrid_summary,
                     cycle_state.validation_result.reason);
    return false;
  }

  // Step 8: infer the runtime contract, guard terminal servo behavior, and
  // publish the execution/runtime summaries.
  cycle_state.runtime_state = InferCoordinatorRuntimeState();
  cycle_state.semantic_summary = InferPlanningSemantics(
      BuildSemanticInput(local_view_, planning_coordinator_.get(),
                         &adc_trajectory_pb, cycle_state.validation_result),
      cycle_state.runtime_state);
  auto guarded_semantic_summary = cycle_state.semantic_summary;
  ApplyPlanningSemanticsToTrajectory(guarded_semantic_summary,
                                     &adc_trajectory_pb);
  cycle_state.publish_reason = ApplyTerminalServoGuardrails(
      planning_coordinator_ != nullptr
          ? planning_coordinator_->state().command_id
          : "",
      cyber::Clock::NowInSeconds(), &terminal_servo_session_state_,
      &guarded_semantic_summary, &adc_trajectory_pb);
  cycle_state.hybrid_summary = EvaluateHybridManeuver(
      planning_coordinator_->state(), guarded_semantic_summary.runtime_state);
  PopulateTrajectoryExecutionContext(
      planning_coordinator_->state(), guarded_semantic_summary,
      cycle_state.hybrid_summary, &adc_trajectory_pb);
  PublishMotionPlan(planning_coordinator_->state(),
                    guarded_semantic_summary, *chassis,
                    *localization_estimate, adc_trajectory_pb);
  planning_writer_->Write(adc_trajectory_pb);
  PublishRuntimeStatus(
      guarded_semantic_summary, cycle_state.hybrid_summary,
      cycle_state.validation_result, planning_coordinator_->state(),
      adc_trajectory_pb.execution(), cycle_state.publish_reason);
  LogPlanningCycle(planning_coordinator_->state(), guarded_semantic_summary,
                   cycle_state.hybrid_summary, cycle_state.publish_reason);

  // Step 9: persist the published trajectory for history/debug consumers.
  auto* history = injector_->history();
  history->Add(adc_trajectory_pb);

  return true;
}

void PlanningComponent::CheckRerouting() {
  auto* rerouting = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_rerouting();
  if (!rerouting->need_rerouting()) {
    return;
  }
  rerouting->set_need_rerouting(false);
  if (routing_service_ == nullptr) {
    AERROR << "rerouting requested without RoutingService";
    return;
  }
  auto request = rerouting->routing_request();
  common::util::FillHeader(node_->Name(), &request);
  RoutingResponse response;
  if (!routing_service_->ComputeRoute(request, &response)) {
    AERROR << "RoutingService failed to recompute route";
    return;
  }
  common::util::FillHeader(node_->Name(), &response);
  std::lock_guard<std::mutex> lock(mutex_);
  routing_.CopyFrom(response);
}

void PlanningComponent::UpdateRoutingForCommand(
    const localization::LocalizationEstimate& localization) {
  PlanningCommand command;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    command.CopyFrom(planning_command_);
  }

  if (!command.has_command_id() || command.command_id().empty()) {
    return;
  }
  const std::string fingerprint = command.SerializeAsString();
  if (fingerprint == routed_command_fingerprint_) {
    return;
  }
  if (command.has_action() && command.action() == COMMAND_CANCEL) {
    std::lock_guard<std::mutex> lock(mutex_);
    routing_.Clear();
    routed_command_fingerprint_ = fingerprint;
    return;
  }

  RoutingRequest request;
  if (!BuildRoutingRequest(command, localization, &request)) {
    std::lock_guard<std::mutex> lock(mutex_);
    routing_.Clear();
    routed_command_fingerprint_ = fingerprint;
    AINFO << "planning command has no routed A-to-B goal";
    return;
  }
  if (routing_service_ == nullptr) {
    AERROR << "routed planning command received without RoutingService";
    std::lock_guard<std::mutex> lock(mutex_);
    routing_.Clear();
    routed_command_fingerprint_ = fingerprint;
    return;
  }
  common::util::FillHeader(node_->Name(), &request);
  RoutingResponse response;
  if (!routing_service_->ComputeRoute(request, &response)) {
    AERROR << "RoutingService failed for planning command "
           << command.command_id();
    std::lock_guard<std::mutex> lock(mutex_);
    routing_.Clear();
    routed_command_fingerprint_ = fingerprint;
    return;
  }
  common::util::FillHeader(node_->Name(), &response);
  std::lock_guard<std::mutex> lock(mutex_);
  routing_.CopyFrom(response);
  routed_command_fingerprint_ = fingerprint;
}

void PlanningComponent::UpdateRoutingForMission(
    const localization::LocalizationEstimate& localization) {
  if (planning_coordinator_ == nullptr ||
      !planning_coordinator_->mission_session_manager().HasActiveSession()) {
    return;
  }
  const auto& guidance =
      planning_coordinator_->mission_session_manager().guidance();
  if (guidance.state == MISSION_SESSION_CANCELLING) {
    std::lock_guard<std::mutex> lock(mutex_);
    routing_.Clear();
    return;
  }
  const std::string fingerprint =
      guidance.identity.SerializeAsString() + guidance.plan.SerializeAsString();
  if (fingerprint == routed_command_fingerprint_) {
    return;
  }

  PlanningCommand command;
  command.set_mission_id(guidance.identity.aggregate_id());
  command.set_command_id(guidance.identity.command_id());
  command.set_action(COMMAND_ACTIVATE);
  if (guidance.plan.has_goal()) {
    command.mutable_goal()->CopyFrom(guidance.plan.goal());
  }
  if (guidance.plan.has_route_hint()) {
    command.mutable_route_hint()->CopyFrom(guidance.plan.route_hint());
  }
  RoutingRequest request;
  if (!BuildRoutingRequest(command, localization, &request)) {
    return;
  }
  common::util::FillHeader(node_->Name(), &request);
  RoutingResponse response;
  MissionRouteContext route;
  route.set_request_id(guidance.identity.command_id() + "-" +
                       std::to_string(guidance.identity.revision()));
  if (routing_service_ == nullptr ||
      !routing_service_->ComputeRoute(request, &response)) {
    route.set_state(MISSION_ROUTE_FAILED);
    route.set_reason("RoutingService failed for MissionDirective");
    planning_coordinator_->UpdateMissionRoute(guidance.identity, route);
    return;
  }
  common::util::FillHeader(node_->Name(), &response);
  route.set_state(MISSION_ROUTE_READY);
  route.set_map_version(response.has_map_version() ? response.map_version()
                                                   : "unknown");
  route.set_route_id(route.request_id());
  const auto update =
      planning_coordinator_->UpdateMissionRoute(guidance.identity, route);
  if (!update.accepted) {
    AERROR << "Mission route correlation failed: " << update.reason;
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    routing_.CopyFrom(response);
  }
  routed_command_fingerprint_ = fingerprint;
}

bool PlanningComponent::CheckInput(
    const PlanningCoordinatorState& preview_state,
    ValidationResult* validation_result) {
  ADCTrajectory trajectory_pb;
  auto* not_ready = trajectory_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  if (local_view_.localization_estimate == nullptr) {
    not_ready->set_reason("localization not ready");
  } else if (local_view_.chassis == nullptr) {
    not_ready->set_reason("chassis not ready");
  } else if (preview_state.resolved_mode == MODE_UNKNOWN) {
    not_ready->set_reason(preview_state.reason.empty()
                              ? "planning mode unavailable"
                              : preview_state.reason);
  } else if (ModeNeedsHdMap(preview_state.resolved_mode) &&
             HDMapUtil::BaseMapPtr() == nullptr) {
    not_ready->set_reason("hdmap not ready for routed planning");
  } else if (ModeNeedsRelativeMap(preview_state.resolved_mode) &&
             (local_view_.relative_map == nullptr ||
              !local_view_.relative_map->has_header())) {
    not_ready->set_reason("relative map not ready for mapless planning");
  } else {
    // nothing
  }

  if (not_ready->has_reason()) {
    terminal_servo_session_state_ = TerminalServoSessionState();
    AWARN_EVERY(100) << not_ready->reason() << "; skip the planning cycle.";
    common::util::FillHeader(node_->Name(), &trajectory_pb);
    if (validation_result != nullptr) {
      validation_result->command_admissible = true;
      validation_result->trajectory_valid = false;
      validation_result->should_publish = false;
      validation_result->should_hold = true;
      validation_result->fallback_active = true;
      validation_result->reason = not_ready->reason();
    }
    const auto semantic_summary = InferPlanningSemantics(
        BuildSemanticInput(local_view_, nullptr, &trajectory_pb,
                           validation_result != nullptr ? *validation_result
                                                        : ValidationResult()),
        RUNTIME_HOLDING);
    const auto hybrid_summary =
        EvaluateHybridManeuver(preview_state, semantic_summary.runtime_state);
    ApplyPlanningSemanticsToTrajectory(semantic_summary, &trajectory_pb);
    PopulateTrajectoryExecutionContext(preview_state, semantic_summary,
                                       hybrid_summary, &trajectory_pb);
    planning_writer_->Write(trajectory_pb);
    PublishRuntimeStatus(
        semantic_summary, hybrid_summary,
        validation_result != nullptr ? *validation_result : ValidationResult(),
        preview_state, trajectory_pb.execution(), not_ready->reason());
    LogPlanningCycle(preview_state, semantic_summary, hybrid_summary,
                     not_ready->reason());
    return false;
  }
  return true;
}

void PlanningComponent::PopulateTrajectoryExecutionContext(
    const PlanningCoordinatorState& coordinator_state,
    const PlanningSemanticSummary& semantic_summary,
    const HybridManeuverSummary& hybrid_summary,
    ADCTrajectory* trajectory) const {
  (void)semantic_summary;
  if (trajectory == nullptr) {
    return;
  }
  trajectory->mutable_execution()->CopyFrom(
      ResolvePublishedExecutionContext(coordinator_state, *trajectory));
  auto* execution = trajectory->mutable_execution();
  if (!execution->has_execution_channel()) {
    execution->set_execution_channel(ResolveExecutionChannel(*trajectory));
  }
  hybrid_maneuver_supervisor_.Apply(hybrid_summary, execution);
}

void PlanningComponent::PublishRuntimeStatus(
    const PlanningSemanticSummary& semantic_summary,
    const HybridManeuverSummary& hybrid_summary,
    const ValidationResult& validation_result,
    const PlanningCoordinatorState& coordinator_state,
    const PlanningExecutionContext& execution, const std::string& reason) {
  if (planning_runtime_status_writer_ == nullptr) {
    return;
  }

  PlanningRuntimeStatus runtime_status;
  common::util::FillHeader(node_->Name(), &runtime_status);
  runtime_status.set_state(semantic_summary.runtime_state);
  runtime_status.set_active_scene(execution.has_active_scene()
                                      ? execution.active_scene()
                                      : coordinator_state.active_scene);
  runtime_status.set_active_mode(execution.has_active_mode()
                                     ? execution.active_mode()
                                     : coordinator_state.resolved_mode);
  runtime_status.set_requested_mode(execution.has_requested_mode()
                                        ? execution.requested_mode()
                                        : coordinator_state.requested_mode);
  runtime_status.set_active_shell(execution.has_active_shell()
                                      ? execution.active_shell()
                                      : coordinator_state.active_shell);
  runtime_status.set_active_domain(execution.has_active_domain()
                                       ? execution.active_domain()
                                       : coordinator_state.active_domain);
  if (execution.has_execution_channel()) {
    runtime_status.set_execution_channel(execution.execution_channel());
  }
  if (coordinator_state.transition_pending) {
    auto* transition = runtime_status.mutable_transition();
    transition->set_from_mode(coordinator_state.resolved_mode);
    transition->set_to_mode(coordinator_state.desired_mode);
    transition->set_from_shell(coordinator_state.active_shell);
    transition->set_to_shell(coordinator_state.desired_shell);
    transition->set_approved(false);
    transition->set_continuity_hold(coordinator_state.continuity_hold);
    if (!reason.empty()) {
      transition->set_trigger(reason);
    } else if (!coordinator_state.reason.empty()) {
      transition->set_trigger(coordinator_state.reason);
    } else {
      transition->set_trigger("planner shell transition pending");
    }
  } else if (coordinator_state.previous_mode != MODE_UNKNOWN &&
             (coordinator_state.previous_mode !=
                  coordinator_state.resolved_mode ||
              coordinator_state.previous_shell !=
                  coordinator_state.active_shell)) {
    auto* transition = runtime_status.mutable_transition();
    transition->set_from_mode(coordinator_state.previous_mode);
    transition->set_to_mode(coordinator_state.resolved_mode);
    transition->set_from_shell(coordinator_state.previous_shell);
    transition->set_to_shell(coordinator_state.active_shell);
    transition->set_approved(coordinator_state.resolved_mode != MODE_UNKNOWN);
    transition->set_continuity_hold(false);
    if (reason.empty() && !coordinator_state.reason.empty()) {
      transition->set_trigger(coordinator_state.reason);
    } else if (!reason.empty()) {
      transition->set_trigger(reason);
    } else {
      transition->set_trigger("planner shell switched");
    }
  } else if (coordinator_state.requested_mode !=
             coordinator_state.resolved_mode) {
    auto* transition = runtime_status.mutable_transition();
    transition->set_from_mode(coordinator_state.requested_mode);
    transition->set_to_mode(coordinator_state.resolved_mode);
    transition->set_from_shell(
        ResolveShellForMode(coordinator_state.requested_mode));
    transition->set_to_shell(coordinator_state.active_shell);
    if (reason.empty() && !coordinator_state.reason.empty()) {
      transition->set_trigger(coordinator_state.reason);
    }
    transition->set_approved(semantic_summary.runtime_state !=
                                 RUNTIME_REJECTED &&
                             coordinator_state.resolved_mode != MODE_UNKNOWN);
    transition->set_continuity_hold(false);
  }
  if (execution.has_mission_id()) {
    runtime_status.set_mission_id(execution.mission_id());
  } else if (!coordinator_state.mission_id.empty()) {
    runtime_status.set_mission_id(coordinator_state.mission_id);
  }
  if (execution.has_command_id()) {
    runtime_status.set_command_id(execution.command_id());
  } else if (!coordinator_state.command_id.empty()) {
    runtime_status.set_command_id(coordinator_state.command_id);
  }
  if (coordinator_state.mission_identity.has_revision()) {
    runtime_status.mutable_mission_identity()->CopyFrom(
        coordinator_state.mission_identity);
    runtime_status.set_mission_session_state(
        coordinator_state.mission_session_state);
    runtime_status.set_mission_phase(coordinator_state.mission_phase);
    if (coordinator_state.accepted_start.has_snapshot_time_sec()) {
      runtime_status.mutable_accepted_start()->CopyFrom(
          coordinator_state.accepted_start);
    }
    const auto& accepted_directive =
        planning_coordinator_->mission_session_manager()
            .last_accepted_directive_identity();
    if (accepted_directive.has_revision()) {
      runtime_status.mutable_accepted_directive_identity()->CopyFrom(
          accepted_directive);
    }
    if (coordinator_state.mission_route.has_state()) {
      runtime_status.mutable_mission_route()->CopyFrom(
          coordinator_state.mission_route);
    }
  }
  if (execution.blockers_size() > 0) {
    for (const auto& blocker : execution.blockers()) {
      runtime_status.add_blockers(blocker);
    }
  } else {
    for (const auto& blocker : coordinator_state.blockers) {
      runtime_status.add_blockers(blocker);
    }
  }
  if (reason.empty() && execution.has_reason()) {
    runtime_status.set_reason(execution.reason());
  } else if (reason.empty() && !coordinator_state.reason.empty()) {
    runtime_status.set_reason(coordinator_state.reason);
  }

  if (!reason.empty()) {
    bool has_same_blocker = false;
    for (const auto& blocker : runtime_status.blockers()) {
      if (blocker == reason) {
        has_same_blocker = true;
        break;
      }
    }
    if (!has_same_blocker) {
      runtime_status.add_blockers(reason);
    }
    runtime_status.set_reason(reason);
    if (runtime_status.has_transition() &&
        !runtime_status.transition().has_trigger()) {
      runtime_status.mutable_transition()->set_trigger(reason);
    }
  }

  ApplyPlanningSemanticsToRuntimeStatus(semantic_summary, &runtime_status);
  hybrid_maneuver_supervisor_.Apply(hybrid_summary, &runtime_status);

  if (local_view_.capability_set != nullptr) {
    auto* capability = runtime_status.mutable_capability();
    capability->set_has_lane_graph(local_view_.capability_set->has_lane_graph);
    capability->set_has_route_semantics(
        local_view_.capability_set->has_route_semantics);
    capability->set_has_local_corridor(
        local_view_.capability_set->has_local_corridor);
    capability->set_has_drivable_area(
        local_view_.capability_set->has_drivable_area);
    capability->set_has_parking_roi(
        local_view_.capability_set->has_parking_roi);
    capability->set_has_goal_pose(local_view_.capability_set->has_goal_pose);
    capability->set_has_stop_target(
        local_view_.capability_set->has_stop_target);
    capability->set_has_regulatory_context(
        local_view_.capability_set->has_regulatory_context);
    capability->set_can_run_on_lane_shell(
        local_view_.capability_set->can_run_on_lane_shell);
    capability->set_can_run_corridor_shell(
        local_view_.capability_set->can_run_corridor_shell);
    capability->set_can_run_safety_hold_shell(
        local_view_.capability_set->can_run_safety_hold_shell);
    capability->set_has_structured_mapless_context(
        local_view_.capability_set->has_structured_mapless_context);
    capability->set_can_run_structured_mapless_shell(
        local_view_.capability_set->can_run_structured_mapless_shell);
    capability->set_can_run_open_space_shell(
        local_view_.capability_set->can_run_open_space_shell);
    capability->set_has_known_open_space_environment(
        local_view_.capability_set->has_known_open_space_environment);
    capability->set_supports_open_space_exploration(
        local_view_.capability_set->supports_open_space_exploration);
    capability->set_topology_confidence(
        local_view_.capability_set->topology_confidence);
    capability->set_drivable_area_confidence(
        local_view_.capability_set->drivable_area_confidence);
    capability->set_target_geometry_confidence(
        local_view_.capability_set->target_geometry_confidence);
  }

  auto* validation = runtime_status.mutable_validation();
  validation->set_trajectory_valid(validation_result.trajectory_valid);
  validation->set_command_admissible(validation_result.command_admissible);
  validation->set_fallback_active(validation_result.fallback_active);
  if (!validation_result.reason.empty()) {
    validation->set_validation_reason(validation_result.reason);
  } else if (!reason.empty()) {
    validation->set_validation_reason(reason);
  }

  planning_runtime_status_writer_->Write(runtime_status);
}

void PlanningComponent::LogPlanningCycle(
    const PlanningCoordinatorState& coordinator_state,
    const PlanningSemanticSummary& semantic_summary,
    const HybridManeuverSummary& hybrid_summary, const std::string& reason) {
  const bool should_log_info =
      coordinator_state.command_id != last_logged_command_id_ ||
      coordinator_state.resolved_mode != last_logged_mode_ ||
      coordinator_state.active_shell != last_logged_shell_ ||
      coordinator_state.transition_pending ||
      semantic_summary.runtime_state != RUNTIME_RUNNING ||
      hybrid_summary.handoff_state != HANDOFF_STATE_NONE;

  std::ostringstream stream;
  stream << "planning cycle: cmd=" << coordinator_state.command_id
         << " scene=" << PlanningSceneType_Name(coordinator_state.active_scene)
         << " mode=" << PlanningMode_Name(coordinator_state.resolved_mode)
         << " shell=" << PlanningShellType_Name(coordinator_state.active_shell)
         << " runtime=" << RuntimeState_Name(semantic_summary.runtime_state);
  if (hybrid_summary.active_maneuver != HYBRID_MANEUVER_NONE) {
    stream << " hybrid="
           << HybridManeuverType_Name(hybrid_summary.active_maneuver) << "/"
           << ManeuverSegmentType_Name(hybrid_summary.active_segment) << "/"
           << HandoffState_Name(hybrid_summary.handoff_state);
  }
  if (!reason.empty()) {
    stream << " reason=" << reason;
  }

  if (should_log_info) {
    AINFO << stream.str();
    last_logged_command_id_ = coordinator_state.command_id;
    last_logged_mode_ = coordinator_state.resolved_mode;
    last_logged_shell_ = coordinator_state.active_shell;
    return;
  }
  ADEBUG << stream.str();
}

}  // namespace planning
}  // namespace apollo
