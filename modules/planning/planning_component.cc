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

#include <chrono>
#include <cmath>
#include <utility>

#include "cyber/common/file.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"
#include "modules/execution_state_sync/execution_state_gflags.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/motion_envelope.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/planning_runtime_status_builder.h"

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

constexpr double kMaxMotionEnvelopeFrameAgeSec = 0.2;

using PlanningClock = std::chrono::steady_clock;

void RecordPhaseTiming(const std::string& name, PlanningClock::time_point start,
                       std::vector<PlanningPhaseTiming>* timings) {
  CHECK_NOTNULL(timings);
  timings->push_back({name, std::chrono::duration<double, std::milli>(
                                PlanningClock::now() - start)
                                .count()});
}

bool ModeNeedsHdMap(PlanningMode mode) { return mode == MODE_LANE_GRAPH; }

bool ModeNeedsRelativeMap(PlanningMode mode) {
  return mode == MODE_CORRIDOR || mode == MODE_FREE_SPACE;
}

const MotionExecutionCommand* DirectiveCommand(
    const MotionDirective& directive) {
  if (directive.has_execute()) {
    return &directive.execute().command();
  }
  if (directive.has_replace()) {
    return &directive.replace().command();
  }
  return nullptr;
}

bool HasNormalMotionConstraints(const MotionExecutionCommand& command) {
  return command.has_constraints() &&
         command.constraints().has_max_speed_mps() &&
         std::isfinite(command.constraints().max_speed_mps()) &&
         command.constraints().max_speed_mps() >= 0.0 &&
         command.constraints().has_max_acceleration_mps2() &&
         std::isfinite(command.constraints().max_acceleration_mps2()) &&
         command.constraints().max_acceleration_mps2() > 0.0 &&
         command.constraints().has_max_deceleration_mps2() &&
         std::isfinite(command.constraints().max_deceleration_mps2()) &&
         command.constraints().max_deceleration_mps2() > 0.0 &&
         command.constraints().has_max_jerk_mps3() &&
         std::isfinite(command.constraints().max_jerk_mps3()) &&
         command.constraints().max_jerk_mps3() > 0.0;
}

bool IsFencedCleanupDirective(const MotionDirective& directive) {
  if (directive.has_cancel()) {
    return directive.scope() == MOTION_SCOPE_MISSION_DESCENDANT &&
           directive.has_parent_mission_identity() &&
           directive.parent_mission_identity().has_revision() &&
           directive.cancel().has_target_identity() &&
           directive.cancel().target_identity().has_revision() &&
           directive.cancel().fence_parent_mission();
  }
  const auto* command = DirectiveCommand(directive);
  if (command == nullptr || !command->has_identity() ||
      !command->identity().has_revision() || !command->has_completion() ||
      !command->has_spatial_envelope() ||
      command->spatial_envelope().boundary_size() < 3 ||
      !HasNormalMotionConstraints(*command)) {
    return false;
  }
  if (command->identity().command_id() == "controlled-stop") {
    if (!command->has_control_intent()) {
      return false;
    }
    const auto& intent = command->control_intent();
    return directive.scope() == MOTION_SCOPE_MISSION_DESCENDANT &&
           directive.has_parent_mission_identity() &&
           directive.parent_mission_identity().has_revision() &&
           command->has_trajectory() &&
           command->trajectory().point_size() >= 2 &&
           intent.tracking_mode() == TRACKING_MODE_TRAJECTORY &&
           intent.execution_channel() == EXECUTION_CHANNEL_TRAJECTORY &&
           intent.primitive_type() == CONTROL_PRIMITIVE_NONE &&
           intent.longitudinal_intent() == LON_INTENT_MRM_STOP &&
           intent.require_full_stop();
  }
  return directive.scope() == MOTION_SCOPE_PLANNING_IDLE_HOLD &&
         command->identity().command_id() == "idle-hold" &&
         command->has_primitive() &&
         command->primitive().type() == MOTION_PRIMITIVE_STANDSTILL_HOLD &&
         command->constraints().max_speed_mps() == 0.0;
}

bool BuildRoutingRequest(const PlanningCommand& command,
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
      request->mutable_parking_info()->CopyFrom(command.goal().parking_goal());
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
  const std::string producer_epoch =
      "planning-" + std::to_string(cyber::Time::Now().ToNanosecond());
  motion_plan_builder_.SetProducerEpoch(producer_epoch);
  execution_state_transport_ =
      std::make_unique<PlanningExecutionStateTransport>();
  const auto sync_result = execution_state_transport_->Init(
      FLAGS_execution_state_db_path, producer_epoch);
  if (!sync_result.ok()) {
    AERROR << "Planning requires execution-state database: "
           << sync_result.message;
    return false;
  }
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
  if (control_status_kind_ !=
      execution_state_sync::ControlStatusKind::kMotionResult) {
    return;
  }
  control::ControlRuntimeStatus status;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status.CopyFrom(control_runtime_status_);
  }
  const double now = cyber::Clock::NowInSeconds();
  if (!status.has_motion_execution() || !status.has_motion_scope() ||
      !status.has_header() || !status.header().has_timestamp_sec() ||
      !std::isfinite(status.header().timestamp_sec()) ||
      status.header().timestamp_sec() > now ||
      now - status.header().timestamp_sec() > 1.0 ||
      !motion_plan_builder_.IsCorrelatedStatus(status.motion_execution())) {
    return;
  }
  if (status.motion_scope() == MOTION_SCOPE_MISSION_DESCENDANT &&
      (!status.motion_execution().has_parent_mission_identity() ||
       status.motion_execution()
               .parent_mission_identity()
               .SerializeAsString() !=
           planning_coordinator_->mission_session_manager()
               .guidance()
               .identity.SerializeAsString())) {
    return;
  }
  const std::string fingerprint = status.SerializeAsString();
  if (fingerprint == applied_control_motion_status_fingerprint_) {
    return;
  }
  applied_control_motion_status_fingerprint_ = fingerprint;
  motion_plan_builder_.ObserveControlStatus(status.motion_execution(),
                                            status.motion_scope());
  const auto& guidance =
      planning_coordinator_->mission_session_manager().guidance();
  const auto& motion = status.motion_execution();
  const bool mission_descendant =
      status.motion_scope() == MOTION_SCOPE_MISSION_DESCENDANT &&
      guidance.identity.has_revision() &&
      motion.has_parent_mission_identity() &&
      motion.parent_mission_identity().SerializeAsString() ==
          guidance.identity.SerializeAsString();
  const bool lifecycle_hold =
      status.motion_scope() == MOTION_SCOPE_PLANNING_IDLE_HOLD &&
      guidance.identity.has_revision() &&
      (guidance.state == MISSION_SESSION_CANCELLING ||
       guidance.state == MISSION_SESSION_COMPLETING) &&
      motion.has_authorized_mission_identity() &&
      motion.authorized_mission_identity().SerializeAsString() ==
          guidance.identity.SerializeAsString();
  if ((mission_descendant || lifecycle_hold) &&
      (motion.state() == MOTION_EXECUTION_REJECTED ||
       motion.state() == MOTION_EXECUTION_FAILED ||
       motion.state() == MOTION_EXECUTION_TIMED_OUT)) {
    const auto result = planning_coordinator_->FailMission(
        motion.has_reason() && !motion.reason().empty()
            ? motion.reason()
            : "Control reported motion failure");
    if (!result.accepted) {
      AERROR << "Failed to propagate Control motion failure to Mission: "
             << result.reason;
    }
    return;
  }
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
  const auto* terminal_evidence =
      status.motion_execution().has_terminal_evidence()
          ? &status.motion_execution().terminal_evidence()
          : nullptr;
  const bool terminal_evidence_fresh =
      terminal_evidence != nullptr &&
      terminal_evidence->has_observed_at_sec() &&
      std::isfinite(terminal_evidence->observed_at_sec()) &&
      terminal_evidence->observed_at_sec() <=
          status.header().timestamp_sec() &&
      status.header().timestamp_sec() -
              terminal_evidence->observed_at_sec() <=
          1.0;
  if (planning_coordinator_ != nullptr &&
      planning_coordinator_->state().mission_session_state ==
          MISSION_SESSION_CANCELLING &&
      status.motion_scope() == MOTION_SCOPE_PLANNING_IDLE_HOLD &&
      status.executor_owner_active() &&
      status.motion_execution().state() == MOTION_EXECUTION_HOLDING) {
    if (!terminal_evidence_fresh) {
      AERROR << "Cancellation hold status lacks fresh terminal evidence";
      return;
    }
    const auto result = planning_coordinator_->ConfirmMissionCancellation(
        *terminal_evidence);
    if (!result.accepted) {
      AERROR << "Failed to confirm Mission cancellation: " << result.reason;
    }
  }
  if (planning_coordinator_ != nullptr &&
      planning_coordinator_->state().mission_session_state ==
          MISSION_SESSION_COMPLETING &&
      status.motion_scope() == MOTION_SCOPE_PLANNING_IDLE_HOLD &&
      status.executor_owner_active() &&
      status.motion_execution().state() == MOTION_EXECUTION_HOLDING) {
    if (!terminal_evidence_fresh) {
      AERROR << "Completion hold status lacks fresh terminal evidence";
      return;
    }
    const auto result = planning_coordinator_->CompleteMission(
        *terminal_evidence);
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
  auto motion_state = coordinator_state;
  if (semantic_summary.command_completed &&
      semantic_summary.full_stop_reached &&
      coordinator_state.mission_session_state == MISSION_SESSION_EXECUTING) {
    const auto transition = planning_coordinator_->BeginMissionCompleting();
    if (transition.accepted) {
      motion_state.mission_session_state = MISSION_SESSION_COMPLETING;
    }
  }
  MotionSpatialEnvelope envelope;
  std::string envelope_reason;
  const auto* frame = injector_->frame_history()->Latest();
  const double localization_time = localization.has_measurement_time()
                                       ? localization.measurement_time()
                                       : localization.header().timestamp_sec();
  if (frame == nullptr) {
    envelope_reason = "no current planning frame for motion authorization";
  } else if (!std::isfinite(frame->vehicle_state().timestamp()) ||
             !std::isfinite(localization_time) ||
             std::abs(frame->vehicle_state().timestamp() - localization_time) >
                 kMaxMotionEnvelopeFrameAgeSec) {
    envelope_reason =
        "planning frame is stale for current localization measurement";
  } else if (!BuildMotionEnvelope(*frame, &envelope, &envelope_reason)) {
    AERROR << "Motion envelope unavailable: " << envelope_reason;
  }
  if (!envelope_reason.empty()) {
    AERROR_EVERY(10) << "Motion directive has no spatial authorization: "
                     << envelope_reason;
  }
  motion_plan_builder_.SetSpatialEnvelope(envelope);
  auto result = motion_plan_builder_.Build(motion_state, semantic_summary,
                                           chassis, localization, trajectory,
                                           cyber::Clock::NowInSeconds());
  if (!result.has_directive) {
    AWARN_EVERY(100) << "MotionPlanBuilder did not emit: " << result.reason;
    return;
  }
  const bool cleanup =
      coordinator_state.mission_session_state == MISSION_SESSION_CANCELLING ||
      coordinator_state.mission_session_state == MISSION_SESSION_COMPLETING;
  if (cleanup && !IsFencedCleanupDirective(result.directive)) {
    AERROR << "Refusing invalid fenced motion cleanup exemption";
    execution_state_fault_ = true;
    return;
  }
  if (!SubmitExecutionState(execution_state_sync::Channel::kMotion,
                            result.directive.SerializeAsString(), cleanup)) {
    execution_state_fault_ = true;
  }
}

bool PlanningComponent::PollExecutionState(
    const localization::LocalizationEstimate& localization) {
  if (execution_state_transport_ == nullptr) {
    return false;
  }
  std::vector<execution_state_sync::Event> events;
  const auto result = execution_state_transport_->Poll(&events);
  DrainExecutionStateSubmissions();
  if (!result.ok()) {
    if (result.code != execution_state_sync::Code::kBusy) {
      AERROR << "Planning execution-state poll failed: " << result.message;
      execution_state_fault_ = true;
    }
    return false;
  }
  if (execution_state_fault_) {
    return false;
  }
  if (pending_mission_admissions_ != 0) {
    return execution_state_transport_->Healthy();
  }
  uint64_t latest_mission_sequence = 0;
  for (const auto& event : events) {
    if (event.operation.channel == execution_state_sync::Channel::kMission) {
      latest_mission_sequence = event.sequence;
    }
  }
  const auto view = execution_state_transport_->Latest();
  if (view && view->result.ok() && view->snapshot) {
    const auto& latest_mission =
        view->snapshot->latest[static_cast<size_t>(
            execution_state_sync::Channel::kMission)];
    if (latest_mission && latest_mission->sequence > latest_mission_sequence) {
      latest_mission_sequence = latest_mission->sequence;
    }
  }
  for (const auto& event : events) {
    if (event.sequence <= deferred_mission_ack_sequence_) {
      continue;
    }
    if (event.operation.channel ==
            execution_state_sync::Channel::kMission &&
        event.owner != execution_state_sync::Role::kMission) {
      AERROR << "Mission lifecycle directive was not committed by Mission";
      execution_state_fault_ = true;
      return false;
    }
    switch (event.operation.channel) {
      case execution_state_sync::Channel::kMission: {
        MissionDirective directive;
        if (!directive.ParseFromString(event.operation.payload)) {
          AERROR << "Invalid MissionDirective execution-state payload";
          execution_state_fault_ = true;
          return false;
        }
        mission_directive_.CopyFrom(directive);
        mission_event_sequence_ = event.sequence;
        motion_event_sequence_ = 0;
        ApplyPendingMissionDirective(
            localization, event.sequence < latest_mission_sequence);
        break;
      }
      case execution_state_sync::Channel::kControlStatus: {
        control::ControlRuntimeStatus status;
        if (!status.ParseFromString(event.operation.payload)) {
          AERROR << "Invalid ControlRuntimeStatus execution-state payload";
          execution_state_fault_ = true;
          return false;
        }
        control_runtime_status_.CopyFrom(status);
        control_status_kind_ = event.operation.control_status_kind;
        ApplyControlMotionStatus();
        break;
      }
      case execution_state_sync::Channel::kMotion:
        motion_event_sequence_ = event.sequence;
        break;
      case execution_state_sync::Channel::kPlanningStatus:
        break;
      case execution_state_sync::Channel::kSafetyRequest:
      case execution_state_sync::Channel::kSafetyStatus:
        break;
    }
    if (execution_state_fault_) {
      return false;
    }
    deferred_mission_ack_sequence_ = event.sequence;
    if (pending_mission_admissions_ != 0) {
      break;
    }
  }
  if (pending_mission_admissions_ == 0 &&
      !events.empty() &&
      deferred_mission_ack_sequence_ == events.back().sequence) {
    const auto ack =
        execution_state_transport_->Acknowledge(deferred_mission_ack_sequence_);
    if (!ack.ok()) {
      AERROR << "Planning Mission admission ack failed: " << ack.message;
      execution_state_fault_ = true;
      return false;
    }
    deferred_mission_ack_sequence_ = 0;
  }
  return execution_state_transport_->Healthy() && !execution_state_fault_ &&
         latest_mission_sequence <= mission_event_sequence_;
}

void PlanningComponent::DrainExecutionStateSubmissions() {
  for (const auto& submission :
       execution_state_transport_->DrainSubmissions()) {
    const auto status_kind = submission.operation.planning_status_kind;
    const bool admission =
        status_kind != execution_state_sync::PlanningStatusKind::kRuntime;
    if (!submission.result.ok()) {
      AERROR << "Planning execution-state commit failed: "
             << submission.result.message;
      if (admission && pending_mission_admissions_ > 0) {
        --pending_mission_admissions_;
        planning_coordinator_->DiscardPreparedMissionDirective();
      }
      execution_state_fault_ = true;
      continue;
    }
    if (submission.operation.channel ==
        execution_state_sync::Channel::kMotion) {
      motion_event_sequence_ = submission.commit.sequence;
      MotionDirective directive;
      if (!directive.ParseFromString(submission.operation.payload)) {
        AERROR << "Committed motion payload cannot be parsed";
        execution_state_fault_ = true;
      } else if (motion_directive_writer_ != nullptr &&
                 !motion_directive_writer_->Write(directive)) {
        AERROR << "Failed to publish committed MotionDirective mirror";
      }
    } else if (submission.operation.channel ==
               execution_state_sync::Channel::kPlanningStatus) {
      PlanningRuntimeStatus status;
      if (!status.ParseFromString(submission.operation.payload)) {
        AERROR << "Committed planning status payload cannot be parsed";
        execution_state_fault_ = true;
      } else {
        if (admission &&
            status_kind !=
                execution_state_sync::PlanningStatusKind::kAdmissionRejected) {
          if (!planning_coordinator_->CommitPreparedMissionDirective(
                  status.admission_directive_identity())) {
            AERROR
                << "Committed Mission admission has no matching staged session";
            execution_state_fault_ = true;
          }
        }
        if (admission && !execution_state_fault_) {
          applied_mission_directive_fingerprint_ =
              mission_directive_.SerializeAsString();
        }
        if (!execution_state_fault_ &&
            planning_runtime_status_writer_ != nullptr &&
            !planning_runtime_status_writer_->Write(status)) {
          AERROR << "Failed to publish committed PlanningRuntimeStatus mirror";
        }
      }
      if (admission && pending_mission_admissions_ > 0) {
        --pending_mission_admissions_;
      }
    }
  }
}

bool PlanningComponent::SubmitExecutionState(
    execution_state_sync::Channel channel, const std::string& payload,
    bool cleanup,
    execution_state_sync::PlanningStatusKind planning_status_kind) {
  const bool owner_runtime_observation =
      channel == execution_state_sync::Channel::kPlanningStatus &&
      planning_status_kind ==
          execution_state_sync::PlanningStatusKind::kRuntime;
  if (execution_state_transport_ == nullptr) {
    AERROR << "Cannot submit without an execution-state client";
    return false;
  }
  if (mission_event_sequence_ == 0 && !owner_runtime_observation) {
    AERROR << "Cannot submit lifecycle operation without a live Mission event";
    return false;
  }
  std::vector<execution_state_sync::Guard> guards;
  if (mission_event_sequence_ != 0) {
    guards.push_back(
        {execution_state_sync::Channel::kMission, mission_event_sequence_});
  }
  if (channel == execution_state_sync::Channel::kPlanningStatus &&
      planning_status_kind ==
          execution_state_sync::PlanningStatusKind::kRuntime &&
      motion_event_sequence_ != 0) {
    guards.push_back(
        {execution_state_sync::Channel::kMotion, motion_event_sequence_});
  }
  const auto result = execution_state_transport_->Submit(
      channel, payload, std::move(guards), cleanup, planning_status_kind);
  if (!result.ok()) {
    AERROR << "Planning execution-state admission failed: " << result.message;
    return false;
  }
  return true;
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
  local_view_.planning_command = std::make_shared<PlanningCommand>(
      planning_coordinator_->mission_session_manager().BuildPlanningCommand());
  local_view_.stories = std::make_shared<Stories>(stories_);
}

void PlanningComponent::RefreshEnvironmentState() {
  local_view_.environment_model = std::make_shared<EnvironmentModel>(
      environment_model_builder_.Build(local_view_));
  local_view_.capability_set = std::make_shared<CapabilitySet>(
      capability_extractor_.Extract(*local_view_.environment_model));
}

void PlanningComponent::ApplyPendingMissionDirective(
    const localization::LocalizationEstimate& localization, bool superseded) {
  MissionDirective directive;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    directive.CopyFrom(mission_directive_);
  }
  if (!directive.has_identity() || planning_coordinator_ == nullptr) {
    AERROR << "Cannot admit Mission directive without identity/coordinator";
    execution_state_fault_ = true;
    return;
  }
  const std::string fingerprint = directive.SerializeAsString();
  if (fingerprint == applied_mission_directive_fingerprint_) {
    return;
  }
  const auto result =
      superseded
          ? MissionAdmissionResult{
                false, MissionAdmissionCode::kReplay,
                "Mission directive superseded by a newer durable intent"}
          : planning_coordinator_->PrepareMissionDirective(
                directive, localization, cyber::Clock::NowInSeconds());
  PlanningRuntimeStatus admission_status;
  common::util::FillHeader(node_->Name(), &admission_status);
  admission_status.set_command_id(directive.identity().command_id());
  admission_status.set_mission_id(directive.identity().aggregate_id());
  admission_status.mutable_admission_directive_identity()->CopyFrom(
      directive.identity());
  admission_status.set_state(result.accepted ? RUNTIME_ACCEPTED
                                             : RUNTIME_REJECTED);
  auto status_kind = execution_state_sync::PlanningStatusKind::
      kAdmissionRejected;
  if (result.code == MissionAdmissionCode::kDuplicate) {
    admission_status.set_mission_admission_outcome(
        MISSION_ADMISSION_DUPLICATE);
    status_kind = execution_state_sync::PlanningStatusKind::
        kAdmissionDuplicate;
  } else if (result.accepted) {
    admission_status.set_mission_admission_outcome(MISSION_ADMISSION_ACCEPTED);
    status_kind = execution_state_sync::PlanningStatusKind::kAdmissionAccepted;
  } else {
    admission_status.set_mission_admission_outcome(MISSION_ADMISSION_REJECTED);
  }
  admission_status.set_reason(result.reason);
  const auto* prepared =
      planning_coordinator_->prepared_mission_session_manager();
  if (result.accepted && prepared == nullptr) {
    AERROR << "Accepted Mission admission has no staged session";
    execution_state_fault_ = true;
    return;
  }
  const auto& session =
      result.accepted ? *prepared
                      : planning_coordinator_->mission_session_manager();
  const auto& guidance = session.guidance();
  if (guidance.identity.has_revision()) {
    admission_status.set_mission_id(guidance.identity.aggregate_id());
    admission_status.mutable_mission_identity()->CopyFrom(guidance.identity);
    admission_status.set_mission_session_state(guidance.state);
    admission_status.mutable_accepted_start()->CopyFrom(
        guidance.accepted_start);
    admission_status.set_mission_phase(guidance.phase);
    admission_status.mutable_mission_route()->CopyFrom(guidance.route);
    admission_status.mutable_accepted_directive_identity()->CopyFrom(
        session.last_accepted_directive_identity());
  }
  if (!SubmitExecutionState(
          execution_state_sync::Channel::kPlanningStatus,
          admission_status.SerializeAsString(), false, status_kind)) {
    execution_state_fault_ = true;
    planning_coordinator_->DiscardPreparedMissionDirective();
    AERROR << "Failed to persist Mission admission result: " << result.reason;
    return;
  }
  ++pending_mission_admissions_;
  if (!result.accepted) {
    AERROR << "Mission Directive V2 rejected: " << result.reason;
  } else {
    AINFO << "Mission Directive V2 prepared, awaiting commit: " << result.reason;
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

bool PlanningComponent::ServiceExecutionState(
    const localization::LocalizationEstimate& localization) {
  if (!PollExecutionState(localization)) {
    AERROR_EVERY(10) << "Planning execution-state synchronization unavailable";
    return false;
  }
  if (mission_event_sequence_ == 0) {
    PlanningRuntimeStatus status;
    common::util::FillHeader(node_->Name(), &status);
    status.set_state(RUNTIME_IDLE);
    status.set_reason("awaiting a live Mission authorization");
    auto fingerprint_status = status;
    fingerprint_status.clear_header();
    const std::string fingerprint = fingerprint_status.SerializeAsString();
    const double now = cyber::Clock::NowInSeconds();
    if (fingerprint != last_planning_status_fingerprint_ ||
        now - last_planning_status_submit_sec_ >= 0.2) {
      if (SubmitExecutionState(execution_state_sync::Channel::kPlanningStatus,
                               status.SerializeAsString(), false)) {
        last_planning_status_fingerprint_ = fingerprint;
        last_planning_status_submit_sec_ = now;
      }
    }
    AINFO_EVERY(100) << "Planning awaits a live Mission authorization";
    return false;
  }
  return pending_mission_admissions_ == 0 &&
         deferred_mission_ack_sequence_ == 0;
}

PlanningComponent::PlanningCyclePreparation
PlanningComponent::PreparePlanningCycle(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate,
    PlanningCycleResult* result) {
  CHECK_NOTNULL(result);
  auto phase_start = PlanningClock::now();
  UpdateRoutingForMission(*localization_estimate);
  RefreshLocalView(prediction_obstacles, chassis, localization_estimate);
  RefreshEnvironmentState();
  RecordPhaseTiming("InputAndEnvironment", phase_start, &result->phase_timings);
  if (planning_coordinator_ != nullptr) {
    phase_start = PlanningClock::now();
    result->coordinator_state = planning_coordinator_->PreviewState(local_view_);
    RecordPhaseTiming("CoordinatorPreview", phase_start,
                      &result->phase_timings);
  }

  const auto reason = CheckInput(result->coordinator_state);
  if (!reason.empty()) {
    PrepareInputHoldResult(reason, result);
    return PlanningCyclePreparation::kInputHold;
  }
  return PlanningCyclePreparation::kReady;
}

bool PlanningComponent::ProcessLearningCycle(PlanningCycleResult* result) {
  CHECK_NOTNULL(result);
  if (config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    ProcessLearningInputs();
  }
  if (config_.learning_mode() != PlanningConfig::RL_TEST) {
    return false;
  }
  result->outcome = PlanningCycleOutcome::kLearningOnly;
  return true;
}

void PlanningComponent::RunPlanningCycle(PlanningCycleResult* result) {
  CHECK_NOTNULL(result);
  ADCTrajectory adc_trajectory_pb;
  auto phase_start = PlanningClock::now();
  planning_coordinator_->RunOnce(local_view_, &adc_trajectory_pb);
  RecordPhaseTiming("PlannerExecution", phase_start,
                    &result->phase_timings);
  auto start_time = adc_trajectory_pb.header().timestamp_sec();
  FinalizeTrajectoryTiming(start_time, &adc_trajectory_pb);

  result->coordinator_state = planning_coordinator_->state();
  phase_start = PlanningClock::now();
  result->validation = validation_supervisor_.Validate(ValidationInput{
      &local_view_, &planning_coordinator_->state(), &adc_trajectory_pb});
  RecordPhaseTiming("TrajectoryValidation", phase_start, &result->phase_timings);
  result->trajectory.Swap(&adc_trajectory_pb);
  if (result->validation.should_hold) {
    result->outcome = PlanningCycleOutcome::kValidationHold;
    terminal_servo_session_state_ = TerminalServoSessionState();
    auto* not_ready = result->trajectory.mutable_decision()
                          ->mutable_main_decision()
                          ->mutable_not_ready();
    if (!not_ready->has_reason() && !result->validation.reason.empty()) {
      not_ready->set_reason(result->validation.reason);
    }
    result->semantics = InferPlanningSemantics(
        BuildSemanticInput(local_view_, planning_coordinator_.get(),
                           &result->trajectory, result->validation),
        result->validation.command_admissible ? RUNTIME_HOLDING
                                              : RUNTIME_REJECTED);
    result->hybrid_maneuver = EvaluateHybridManeuver(
        result->coordinator_state, result->semantics.runtime_state);
    result->reason = result->validation.reason;
    phase_start = PlanningClock::now();
    ApplyPlanningSemanticsToTrajectory(result->semantics, &result->trajectory);
    RecordPhaseTiming("HoldResultPreparation", phase_start,
                      &result->phase_timings);
    return;
  }

  result->outcome = PlanningCycleOutcome::kPlanned;
  const auto runtime_state = InferCoordinatorRuntimeState();
  phase_start = PlanningClock::now();
  result->semantics = InferPlanningSemantics(
      BuildSemanticInput(local_view_, planning_coordinator_.get(),
                         &result->trajectory, result->validation),
      runtime_state);
  ApplyPlanningSemanticsToTrajectory(result->semantics, &result->trajectory);
  result->reason = ApplyTerminalServoGuardrails(
      planning_coordinator_ != nullptr
          ? planning_coordinator_->state().command_id
          : "",
      cyber::Clock::NowInSeconds(), &terminal_servo_session_state_,
      &result->semantics, &result->trajectory);
  result->hybrid_maneuver = EvaluateHybridManeuver(
      result->coordinator_state, result->semantics.runtime_state);
  RecordPhaseTiming("SemanticAnalysisAndGuardrails", phase_start,
                    &result->phase_timings);
}

bool PlanningComponent::CompletePlanningCycle(
    PlanningCycleResult* result, const canbus::Chassis* chassis,
    const localization::LocalizationEstimate* localization) {
  CHECK_NOTNULL(result);
  FinalizePlanningResult(result, chassis, localization);
  if (result->ShouldRecordHistory()) {
    injector_->history()->Add(result->trajectory);
  }
  return result->outcome == PlanningCycleOutcome::kPlanned;
}

bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  ACHECK(prediction_obstacles != nullptr);

  if (!ServiceExecutionState(*localization_estimate)) {
    return false;
  }
  CheckRerouting();

  PlanningCycleResult result;
  const auto preparation = PreparePlanningCycle(
      prediction_obstacles, chassis, localization_estimate, &result);
  if (preparation == PlanningCyclePreparation::kInputHold) {
    return CompletePlanningCycle(&result, chassis.get(),
                                 localization_estimate.get());
  }
  if (ProcessLearningCycle(&result)) {
    return PublishLearningDataFrame();
  }

  RunPlanningCycle(&result);
  return CompletePlanningCycle(&result, chassis.get(),
                               localization_estimate.get());
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

  const auto command =
      planning_coordinator_->mission_session_manager().BuildPlanningCommand();
  auto accepted_localization = localization;
  accepted_localization.mutable_pose()->mutable_position()->CopyFrom(
      guidance.accepted_start.position());
  accepted_localization.mutable_pose()->set_heading(
      guidance.accepted_start.heading());
  RoutingRequest request;
  if (!BuildRoutingRequest(command, accepted_localization, &request)) {
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

std::string PlanningComponent::CheckInput(
    const PlanningCoordinatorState& preview_state) const {
  if (local_view_.localization_estimate == nullptr) {
    return "localization not ready";
  }
  if (local_view_.chassis == nullptr) {
    return "chassis not ready";
  }
  if (preview_state.resolved_mode == MODE_UNKNOWN) {
    return preview_state.reason.empty() ? "planning mode unavailable"
                                        : preview_state.reason;
  }
  if (ModeNeedsHdMap(preview_state.resolved_mode) &&
      HDMapUtil::BaseMapPtr() == nullptr) {
    return "hdmap not ready for routed planning";
  }
  if (ModeNeedsRelativeMap(preview_state.resolved_mode) &&
      (local_view_.relative_map == nullptr ||
       !local_view_.relative_map->has_header())) {
    return "relative map not ready for mapless planning";
  }
  return "";
}

void PlanningComponent::PrepareInputHoldResult(
    const std::string& reason, PlanningCycleResult* result) {
  CHECK_NOTNULL(result);
  terminal_servo_session_state_ = TerminalServoSessionState();
  AWARN_EVERY(100) << reason << "; skip the planning cycle.";

  result->outcome = PlanningCycleOutcome::kInputHold;
  result->validation.command_admissible = true;
  result->validation.trajectory_valid = false;
  result->validation.should_publish = false;
  result->validation.should_hold = true;
  result->validation.fallback_active = true;
  result->validation.reason = reason;
  result->reason = reason;
  result->trajectory.mutable_decision()
      ->mutable_main_decision()
      ->mutable_not_ready()
      ->set_reason(reason);
  common::util::FillHeader(node_->Name(), &result->trajectory);

  result->semantics = InferPlanningSemantics(
      BuildSemanticInput(local_view_, nullptr, &result->trajectory,
                         result->validation),
      RUNTIME_HOLDING);
  result->hybrid_maneuver = EvaluateHybridManeuver(
      result->coordinator_state, result->semantics.runtime_state);
  ApplyPlanningSemanticsToTrajectory(result->semantics, &result->trajectory);
}

void PlanningComponent::FinalizePlanningResult(
    PlanningCycleResult* result, const canbus::Chassis* chassis,
    const localization::LocalizationEstimate* localization) {
  CHECK_NOTNULL(result);
  if (!result->ShouldPublishTrajectory()) {
    return;
  }
  const auto output_preparation_start = PlanningClock::now();
  PopulateTrajectoryExecutionContext(
      result->coordinator_state, result->hybrid_maneuver, &result->trajectory);
  if (result->ShouldPublishMotion()) {
    CHECK_NOTNULL(chassis);
    CHECK_NOTNULL(localization);
    PublishMotionPlan(result->coordinator_state, result->semantics, *chassis,
                      *localization, result->trajectory);
  }
  RecordPhaseTiming("ExecutionContextAndMotionSubmission",
                    output_preparation_start,
                    &result->phase_timings);
  for (const auto& timing : result->phase_timings) {
    auto* task = result->trajectory.mutable_latency_stats()->add_task_stats();
    task->set_name(timing.name);
    task->set_time_ms(timing.time_ms);
  }
  planning_writer_->Write(result->trajectory);
  MissionCommandIdentity accepted_directive_identity;
  if (planning_coordinator_ != nullptr) {
    accepted_directive_identity =
        planning_coordinator_->mission_session_manager()
            .last_accepted_directive_identity();
  }
  PublishRuntimeStatus(result->semantics, result->hybrid_maneuver,
                       result->validation, result->coordinator_state,
                       result->trajectory.execution(),
                       accepted_directive_identity,
                       local_view_.capability_set.get(), result->reason);
  diagnostics_.LogCycle(result->coordinator_state, result->semantics,
                        result->hybrid_maneuver, result->reason);
}

void PlanningComponent::PopulateTrajectoryExecutionContext(
    const PlanningCoordinatorState& coordinator_state,
    const HybridManeuverSummary& hybrid_summary,
    ADCTrajectory* trajectory) const {
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
    const PlanningExecutionContext& execution,
    const MissionCommandIdentity& accepted_directive_identity,
    const CapabilitySet* capability_set, const std::string& reason) {
  auto runtime_status = PlanningRuntimeStatusBuilder().Build(
      node_->Name(), semantic_summary, hybrid_summary, validation_result,
      coordinator_state, execution, accepted_directive_identity,
      capability_set, reason);
  auto fingerprint_status = runtime_status;
  fingerprint_status.clear_header();
  const std::string fingerprint = fingerprint_status.SerializeAsString();
  const double now = cyber::Clock::NowInSeconds();
  const bool semantic_change = fingerprint != last_planning_status_fingerprint_;
  if (semantic_change || now - last_planning_status_submit_sec_ >= 0.2) {
    if (SubmitExecutionState(execution_state_sync::Channel::kPlanningStatus,
                             runtime_status.SerializeAsString(), false)) {
      last_planning_status_fingerprint_ = fingerprint;
      last_planning_status_submit_sec_ = now;
    } else if (semantic_change) {
      execution_state_fault_ = true;
    } else {
      AWARN_EVERY(10) << "Planning status heartbeat deferred by backpressure";
    }
  }
}

}  // namespace planning
}  // namespace apollo
