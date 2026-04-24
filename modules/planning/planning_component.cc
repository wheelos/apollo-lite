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

PlanningSemanticInput BuildSemanticInput(
    const LocalView& local_view, const PlanningCoordinator* planning_coordinator,
    const ADCTrajectory* trajectory, const ValidationResult& validation_result) {
  PlanningSemanticInput input;
  if (planning_coordinator != nullptr) {
    input.planning_state = &planning_coordinator->state();
  }
  input.chassis = local_view.chassis.get();
  input.localization = local_view.localization_estimate.get();
  input.trajectory = trajectory;
  input.validation_should_hold = validation_result.should_hold;
  return input;
}

}  // namespace

bool PlanningComponent::Init() {
  injector_ = std::make_shared<DependencyInjector>();
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
    AERROR << "failed to init PlanningCoordinator: "
           << init_status.ToString();
    return false;
  }

  routing_reader_ = node_->CreateReader<RoutingResponse>(
      config_.topic_config().routing_response_topic(),
      [this](const std::shared_ptr<RoutingResponse>& routing) {
        AINFO << "Received routing data: run routing callback."
              << routing->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        routing_.CopyFrom(*routing);
      });

  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      config_.topic_config().traffic_light_detection_topic(),
      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
        ADEBUG << "Received traffic light data: run traffic light callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        traffic_light_.CopyFrom(*traffic_light);
      });

  pad_msg_reader_ = node_->CreateReader<PadMessage>(
      config_.topic_config().planning_pad_topic(),
      [this](const std::shared_ptr<PadMessage>& pad_msg) {
        ADEBUG << "Received pad data: run pad callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        pad_msg_.CopyFrom(*pad_msg);
      });

  planning_command_reader_ = node_->CreateReader<PlanningCommand>(
      config_.topic_config().planning_command_topic(),
      [this](const std::shared_ptr<PlanningCommand>& planning_command) {
        ADEBUG << "Received planning command data: run planning command callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        planning_command_.CopyFrom(*planning_command);
      });

  story_telling_reader_ = node_->CreateReader<Stories>(
      config_.topic_config().story_telling_topic(),
      [this](const std::shared_ptr<Stories>& stories) {
        ADEBUG << "Received story_telling data: run story_telling callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        stories_.CopyFrom(*stories);
      });

  relative_map_reader_ = node_->CreateReader<MapMsg>(
      config_.topic_config().relative_map_topic(),
      [this](const std::shared_ptr<MapMsg>& map_message) {
        ADEBUG << "Received relative map data: run relative map callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        relative_map_.CopyFrom(*map_message);
      });
  planning_writer_ = node_->CreateWriter<ADCTrajectory>(
      config_.topic_config().planning_trajectory_topic());
  planning_runtime_status_writer_ = node_->CreateWriter<PlanningRuntimeStatus>(
      config_.topic_config().planning_runtime_status_topic());

  rerouting_writer_ = node_->CreateWriter<RoutingRequest>(
      config_.topic_config().routing_request_topic());

  planning_learning_data_writer_ = node_->CreateWriter<PlanningLearningData>(
      config_.topic_config().planning_learning_data_topic());

  return true;
}

bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  ACHECK(prediction_obstacles != nullptr);

  // check and process possible rerouting request
  CheckRerouting();

  // process fused input data
  local_view_.prediction_obstacles = prediction_obstacles;
  local_view_.chassis = chassis;
  local_view_.localization_estimate = localization_estimate;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.routing ||
        hdmap::PncMap::IsNewRouting(*local_view_.routing, routing_)) {
      local_view_.routing =
          std::make_shared<routing::RoutingResponse>(routing_);
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.traffic_light =
        std::make_shared<TrafficLightDetection>(traffic_light_);
    local_view_.relative_map = std::make_shared<MapMsg>(relative_map_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.pad_msg = std::make_shared<PadMessage>(pad_msg_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.planning_command =
        std::make_shared<PlanningCommand>(planning_command_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.stories = std::make_shared<Stories>(stories_);
  }
  local_view_.environment_model = std::make_shared<EnvironmentModel>(
      environment_model_builder_.Build(local_view_));
  local_view_.capability_set = std::make_shared<CapabilitySet>(
      capability_extractor_.Extract(*local_view_.environment_model));

  ValidationResult validation_result;
  if (!CheckInput(&validation_result)) {
    return false;
  }

  if (config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    // data process for online training
    message_process_.OnChassis(*local_view_.chassis);
    message_process_.OnPrediction(*local_view_.prediction_obstacles);
    message_process_.OnRoutingResponse(*local_view_.routing);
    message_process_.OnStoryTelling(*local_view_.stories);
    message_process_.OnTrafficLightDetection(*local_view_.traffic_light);
    message_process_.OnLocalization(*local_view_.localization_estimate);
  }

  // publish learning data frame for RL test
  if (config_.learning_mode() == PlanningConfig::RL_TEST) {
    PlanningLearningData planning_learning_data;
    LearningDataFrame* learning_data_frame =
        injector_->learning_based_data()->GetLatestLearningDataFrame();
    if (learning_data_frame) {
      planning_learning_data.mutable_learning_data_frame()
                            ->CopyFrom(*learning_data_frame);
      common::util::FillHeader(node_->Name(), &planning_learning_data);
      planning_learning_data_writer_->Write(planning_learning_data);
    } else {
      AERROR << "fail to generate learning data frame";
      return false;
    }
    return true;
  }

  ADCTrajectory adc_trajectory_pb;
  planning_coordinator_->RunOnce(local_view_, &adc_trajectory_pb);
  auto start_time = adc_trajectory_pb.header().timestamp_sec();
  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);

  // modify trajectory relative time due to the timestamp change in header
  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
  }
  validation_result = validation_supervisor_.Validate(
      ValidationInput{&local_view_, &planning_coordinator_->state(),
                      &adc_trajectory_pb});
  if (validation_result.should_hold) {
    terminal_servo_session_state_ = TerminalServoSessionState();
    auto* not_ready = adc_trajectory_pb.mutable_decision()
                          ->mutable_main_decision()
                          ->mutable_not_ready();
    if (!not_ready->has_reason() && !validation_result.reason.empty()) {
      not_ready->set_reason(validation_result.reason);
    }
    const auto semantic_summary = InferPlanningSemantics(
        BuildSemanticInput(local_view_, planning_coordinator_.get(),
                           &adc_trajectory_pb, validation_result),
        validation_result.command_admissible ? RUNTIME_HOLDING
                                             : RUNTIME_REJECTED);
    ApplyPlanningSemanticsToTrajectory(semantic_summary, &adc_trajectory_pb);
    planning_writer_->Write(adc_trajectory_pb);
    PublishRuntimeStatus(semantic_summary, validation_result,
                         validation_result.reason);
    return false;
  }
  RuntimeState runtime_state = RUNTIME_RUNNING;
  if (planning_coordinator_ != nullptr &&
      planning_coordinator_->state().requested_mode !=
          planning_coordinator_->state().resolved_mode) {
    runtime_state = planning_coordinator_->state().resolved_mode == MODE_UNKNOWN
                        ? RUNTIME_HOLDING
                        : RUNTIME_DEGRADED;
  }
  const auto semantic_summary = InferPlanningSemantics(
      BuildSemanticInput(local_view_, planning_coordinator_.get(),
                         &adc_trajectory_pb, validation_result),
      runtime_state);
  auto guarded_semantic_summary = semantic_summary;
  ApplyPlanningSemanticsToTrajectory(guarded_semantic_summary,
                                     &adc_trajectory_pb);
  const std::string servo_guard_reason = ApplyTerminalServoGuardrails(
      planning_coordinator_ != nullptr ? planning_coordinator_->state().command_id
                                       : "",
      cyber::Clock::NowInSeconds(), &terminal_servo_session_state_,
      &guarded_semantic_summary, &adc_trajectory_pb);
  planning_writer_->Write(adc_trajectory_pb);
  PublishRuntimeStatus(guarded_semantic_summary, validation_result,
                       servo_guard_reason);

  // record in history
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
  common::util::FillHeader(node_->Name(), rerouting->mutable_routing_request());
  rerouting->set_need_rerouting(false);
  rerouting_writer_->Write(rerouting->routing_request());
}

bool PlanningComponent::CheckInput(ValidationResult* validation_result) {
  ADCTrajectory trajectory_pb;
  auto* not_ready = trajectory_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();

  if (local_view_.localization_estimate == nullptr) {
    not_ready->set_reason("localization not ready");
  } else if (local_view_.chassis == nullptr) {
    not_ready->set_reason("chassis not ready");
  } else if (HDMapUtil::BaseMapPtr() == nullptr) {
    not_ready->set_reason("map not ready");
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
        BuildSemanticInput(
            local_view_, nullptr, &trajectory_pb,
            validation_result != nullptr ? *validation_result
                                         : ValidationResult()),
        RUNTIME_HOLDING);
    ApplyPlanningSemanticsToTrajectory(semantic_summary, &trajectory_pb);
    planning_writer_->Write(trajectory_pb);
    PublishRuntimeStatus(semantic_summary,
                         validation_result != nullptr ? *validation_result
                                                     : ValidationResult(),
                         not_ready->reason());
    return false;
  }
  return true;
}

void PlanningComponent::PublishRuntimeStatus(
    const PlanningSemanticSummary& semantic_summary,
    const ValidationResult& validation_result,
                                             const std::string& reason) {
  if (planning_runtime_status_writer_ == nullptr) {
    return;
  }

  PlanningRuntimeStatus runtime_status;
  common::util::FillHeader(node_->Name(), &runtime_status);
  runtime_status.set_state(semantic_summary.runtime_state);
  if (planning_coordinator_ != nullptr) {
    const auto& coordinator_state = planning_coordinator_->state();
    runtime_status.set_active_scene(coordinator_state.active_scene);
    runtime_status.set_active_mode(coordinator_state.resolved_mode);
    runtime_status.set_requested_mode(coordinator_state.requested_mode);
    if (coordinator_state.requested_mode != coordinator_state.resolved_mode) {
      auto* transition = runtime_status.mutable_transition();
      transition->set_from_mode(coordinator_state.requested_mode);
      transition->set_to_mode(coordinator_state.resolved_mode);
      if (reason.empty() && !coordinator_state.reason.empty()) {
        transition->set_trigger(coordinator_state.reason);
      }
      transition->set_approved(semantic_summary.runtime_state != RUNTIME_REJECTED &&
                               coordinator_state.resolved_mode != MODE_UNKNOWN);
    }
    if (!coordinator_state.mission_id.empty()) {
      runtime_status.set_mission_id(coordinator_state.mission_id);
    }
    if (!coordinator_state.command_id.empty()) {
      runtime_status.set_command_id(coordinator_state.command_id);
    }
    for (const auto& blocker : coordinator_state.blockers) {
      runtime_status.add_blockers(blocker);
    }
    if (reason.empty() && !coordinator_state.reason.empty()) {
      runtime_status.set_reason(coordinator_state.reason);
    }
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

}  // namespace planning
}  // namespace apollo
