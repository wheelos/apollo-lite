/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/control/control_component.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "absl/strings/str_cat.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/latency_recorder/latency_recorder.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/common/terminal_control_helper.h"
#include "modules/execution_state_sync/execution_state_gflags.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::cyber::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

namespace {

bool IsHoldingIntent(const apollo::planning::ControlIntent &intent) {
  return intent.tracking_mode() ==
             apollo::planning::TRACKING_MODE_STANDSTILL_HOLD ||
         intent.longitudinal_intent() ==
             apollo::planning::LON_INTENT_HOLD_STOP ||
         intent.longitudinal_intent() == apollo::planning::LON_INTENT_MRM_STOP;
}

struct TrajectoryRuntimeContext {
  std::string mission_id;
  std::string command_id;
  apollo::planning::PlanningSceneType active_scene =
      apollo::planning::SCENE_UNKNOWN;
  apollo::planning::PlanningMode active_mode = apollo::planning::MODE_UNKNOWN;
  apollo::planning::PlanningShellType active_shell =
      apollo::planning::PLANNING_SHELL_UNKNOWN;
  apollo::planning::ControlExecutionChannel execution_channel =
      apollo::planning::EXECUTION_CHANNEL_UNKNOWN;
  bool has_control_intent = false;
  apollo::planning::ControlIntent control_intent;
};

TrajectoryRuntimeContext ExtractTrajectoryRuntimeContext(
    const ADCTrajectory &trajectory) {
  TrajectoryRuntimeContext context;
  if (trajectory.has_control_intent()) {
    context.has_control_intent = true;
    context.control_intent = trajectory.control_intent();
  }
  if (trajectory.has_execution()) {
    const auto &execution = trajectory.execution();
    if (execution.has_mission_id()) {
      context.mission_id = execution.mission_id();
    }
    if (execution.has_command_id()) {
      context.command_id = execution.command_id();
    }
    if (execution.has_active_scene()) {
      context.active_scene = execution.active_scene();
    }
    if (execution.has_active_mode()) {
      context.active_mode = execution.active_mode();
    }
    if (execution.has_active_shell()) {
      context.active_shell = execution.active_shell();
    }
    if (execution.has_execution_channel()) {
      context.execution_channel = execution.execution_channel();
    }
  }
  if (context.has_control_intent &&
      context.control_intent.has_execution_channel()) {
    context.execution_channel = context.control_intent.execution_channel();
  }
  return context;
}

ControlSafetyState TranslateSafetyState(SafetyState state) {
  switch (state) {
    case SafetyState::kNormal:
      return CONTROL_SAFETY_NORMAL;
    case SafetyState::kWarning:
      return CONTROL_SAFETY_WARNING;
    case SafetyState::kSoftStop:
      return CONTROL_SAFETY_SOFT_STOP;
    case SafetyState::kHardEstop:
      return CONTROL_SAFETY_HARD_ESTOP;
    case SafetyState::kFatal:
      return CONTROL_SAFETY_FATAL;
    default:
      return CONTROL_SAFETY_UNKNOWN;
  }
}

planning::MotionEvidenceSafetyState TranslateEvidenceSafetyState(
    SafetyState state) {
  switch (state) {
    case SafetyState::kNormal:
      return planning::MOTION_EVIDENCE_SAFETY_NORMAL;
    case SafetyState::kWarning:
      return planning::MOTION_EVIDENCE_SAFETY_WARNING;
    case SafetyState::kSoftStop:
      return planning::MOTION_EVIDENCE_SAFETY_SOFT_STOP;
    case SafetyState::kHardEstop:
      return planning::MOTION_EVIDENCE_SAFETY_HARD_ESTOP;
    case SafetyState::kFatal:
      return planning::MOTION_EVIDENCE_SAFETY_FATAL;
    default:
      return planning::MOTION_EVIDENCE_SAFETY_UNKNOWN;
  }
}

planning::MotionTerminalEvidence BuildMotionTerminalEvidence(
    const planning::MotionExecutionCommand& command,
    planning::MotionTerminalEvidenceKind kind,
    planning::MotionExecutorOwnership ownership,
    planning::MotionEvidenceSafetyState safety_state,
    const common::VehicleState& state, const std::string& frame_id,
    double position_error_m, double heading_error_rad,
    double settled_duration_sec) {
  planning::MotionTerminalEvidence evidence;
  evidence.set_contract_version(1);
  evidence.set_kind(kind);
  evidence.mutable_motion_identity()->CopyFrom(command.identity());
  evidence.mutable_parent_mission_identity()->CopyFrom(
      command.authorized_mission_identity());
  evidence.set_authority_generation(command.authority_generation());
  evidence.set_observed_at_sec(state.timestamp());
  evidence.set_reference_frame_id(frame_id);
  evidence.set_position_error_m(position_error_m);
  evidence.set_heading_error_rad(heading_error_rad);
  evidence.set_absolute_speed_mps(std::abs(state.linear_velocity()));
  evidence.set_settled_duration_sec(settled_duration_sec);
  evidence.set_executor_ownership(ownership);
  evidence.set_executor_type(
      command.has_trajectory() ? planning::MOTION_EXECUTION_TYPE_TRAJECTORY
                               : planning::MOTION_EXECUTION_TYPE_PRIMITIVE);
  evidence.set_safety_state(safety_state);
  return evidence;
}

void MeasureCommandTerminalError(
    const planning::MotionExecutionCommand& command,
    const common::VehicleState& state, double* position_error_m,
    double* heading_error_rad) {
  double target_x = command.start_condition().expected_position().x();
  double target_y = command.start_condition().expected_position().y();
  double target_heading = command.start_condition().expected_heading();
  if (command.has_trajectory() && command.trajectory().point_size() > 0) {
    const auto& endpoint =
        command.trajectory().point(command.trajectory().point_size() - 1)
            .path_point();
    target_x = endpoint.x();
    target_y = endpoint.y();
    target_heading = endpoint.theta();
  } else if (command.has_primitive()) {
    const auto& primitive = command.primitive();
    if (primitive.has_pose_servo()) {
      target_x = primitive.pose_servo().target_position().x();
      target_y = primitive.pose_servo().target_position().y();
      target_heading = primitive.pose_servo().target_heading();
    } else if (primitive.has_corridor_servo()) {
      target_x = primitive.corridor_servo().target_position().x();
      target_y = primitive.corridor_servo().target_position().y();
      target_heading = primitive.corridor_servo().target_heading();
    } else if (primitive.has_rotate_in_place()) {
      target_heading = primitive.rotate_in_place().target_heading();
      if (primitive.rotate_in_place().has_pivot_position()) {
        target_x = primitive.rotate_in_place().pivot_position().x();
        target_y = primitive.rotate_in_place().pivot_position().y();
      }
    }
  }
  *position_error_m = std::hypot(state.x() - target_x, state.y() - target_y);
  *heading_error_rad = std::abs(std::atan2(
      std::sin(state.heading() - target_heading),
      std::cos(state.heading() - target_heading)));
}

}  // namespace

ControlComponent::ControlComponent()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

bool ControlComponent::Init() {
  injector_ = std::make_shared<DependencyInjector>();
  init_time_ = Clock::Now();

  AINFO << "Control init, starting ...";

  ACHECK(
      cyber::common::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_))
      << "Unable to load control conf file: " + FLAGS_control_conf_file;

  // 1. Initialize Controller Agent
  const auto& platform =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  const auto controllers_initialized = controller_profiles_.Init(
      injector_, control_conf_, platform.max_abs_speed_when_stopped(),
      platform.max_steer_angle() > 0.0
          ? 100.0 * platform.max_steer_angle_rate() / platform.max_steer_angle()
          : 0.0,
      platform.max_acceleration(), -platform.max_deceleration());
  if (!controllers_initialized.ok()) {
    monitor_logger_buffer_.ERROR(controllers_initialized.error_message());
    return false;
  }
  if (!InitMotionExecutors()) {
    return false;
  }
  execution_state_client_ = std::make_unique<execution_state_sync::Client>();
  control_epoch_ = "control-" + std::to_string(init_time_.ToNanosecond());
  safety_latch_reconciled_ = false;
  safety_status_ticket_ = 0;
  pending_safety_status_.clear();
  const auto sync_result = execution_state_client_->Init(
      FLAGS_execution_state_db_path, execution_state_sync::Role::kControl,
      control_epoch_, 8, "v1",
      {"motion-execution-v1", "control-runtime-status-v1",
       "controlled-safety-stop-v1"});
  if (!sync_result.ok()) {
    AERROR << "Control requires execution-state database: " << sync_result.message;
    return false;
  }

  // 2. Initialize Safety Manager
  safety_manager_ = std::make_unique<SafetyManager>();
  if (!safety_manager_->Init(control_conf_)) {
    AERROR << "Safety Manager Init failed!";
    return false;
  }

  // 3. Initialize Readers
  InitReaders();

  // 4. Initialize Writers
  control_cmd_writer_ =
      node_->CreateWriter<ControlCommand>(FLAGS_control_command_topic);
  control_runtime_status_writer_ = node_->CreateWriter<ControlRuntimeStatus>(
      FLAGS_control_runtime_status_topic);

  // 5. Wait for system stabilization (e.g., CAN bus readiness)
  // TODO(zero): To prevent entering estop state during startup.
  AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  pad_msg_.set_action(control_conf_.action());
  return true;
}

bool ControlComponent::InitMotionExecutors() {
  const auto& vehicle =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  const auto positive = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };
  if (!positive(control_conf_.soft_estop_brake()) ||
      control_conf_.soft_estop_brake() > 100.0) {
    AERROR << "A finite positive safe-stop brake percentage is required";
    return false;
  }
  for (double extent : {vehicle.front_edge_to_center(),
                        vehicle.back_edge_to_center(),
                        vehicle.left_edge_to_center(),
                        vehicle.right_edge_to_center()}) {
    if (!positive(extent)) {
      AERROR << "Motion monitoring requires finite vehicle reference-point extents";
      return false;
    }
  }
  const double radius = std::hypot(
      std::max(vehicle.front_edge_to_center(), vehicle.back_edge_to_center()),
      std::max(vehicle.left_edge_to_center(), vehicle.right_edge_to_center()));
  const double max_age = std::min(
      control_conf_.localization_period() * control_conf_.max_localization_miss_num(),
      control_conf_.chassis_period() * control_conf_.max_chassis_miss_num());
  if (!positive(max_age) || !positive(control_conf_.control_period())) {
    AERROR << "Motion monitoring requires positive configured sensor periods";
    return false;
  }
  motion_state_max_age_sec_ = max_age;
  // Clearance checking subdivisions derive from physical footprint geometry;
  // this is not a constructed corridor or a primitive execution limit.
  motion_execution_monitor_ = std::make_unique<MotionExecutionMonitor>(
      radius, radius / 10.0, max_age);
  MotionExecutionCapabilities capabilities;
  capabilities.supported = {planning::MOTION_CAPABILITY_TRAJECTORY_TRACKING,
                            planning::MOTION_CAPABILITY_STANDSTILL_HOLD};
  if (control_conf_.has_primitive_execution()) {
    const auto& conf = control_conf_.primitive_execution();
    for (double limit : {conf.max_speed_mps(), conf.max_acceleration_mps2(),
                         conf.max_deceleration_mps2(), conf.max_jerk_mps3(),
                         conf.max_curvature_per_m(),
                         conf.max_curvature_derivative_per_m2(),
                         conf.max_lateral_acceleration_mps2(),
                         conf.sample_period_sec(), conf.spatial_step_m(),
                         conf.max_state_age_sec(),
                         conf.max_tracking_heading_error_rad()}) {
      if (!positive(limit)) {
        AERROR << "Every configured primitive execution limit must be finite and positive";
        return false;
      }
    }
    bool lateral = false;
    bool longitudinal = false;
    for (auto type : control_conf_.active_controllers()) {
      lateral = lateral || type == ControlConf::LAT_CONTROLLER;
      longitudinal = longitudinal || type == ControlConf::LON_CONTROLLER;
      if (type != ControlConf::LAT_CONTROLLER && type != ControlConf::LON_CONTROLLER) {
        AERROR << "Primitive spatial execution requires the LAT/LON controller pair";
        return false;
      }
    }
    if (!lateral || !longitudinal) {
      AERROR << "Primitive spatial execution requires lateral and longitudinal actuation";
      return false;
    }
    if (!conf.has_max_speed_mps() || !conf.has_max_acceleration_mps2() ||
        !conf.has_max_deceleration_mps2() || !conf.has_max_jerk_mps3() ||
        !conf.has_max_curvature_per_m() ||
        !conf.has_max_curvature_derivative_per_m2() ||
        !conf.has_max_lateral_acceleration_mps2() ||
        !conf.has_footprint_margin_m() || !conf.has_sample_period_sec() ||
        !conf.has_spatial_step_m() || !conf.has_max_state_age_sec() ||
        !conf.has_max_tracking_heading_error_rad() ||
        !std::isfinite(conf.footprint_margin_m()) ||
        conf.footprint_margin_m() < 0.0 ||
        !positive(vehicle.wheel_base()) ||
        !positive(vehicle.steer_ratio()) ||
        !positive(vehicle.max_steer_angle()) ||
        !positive(vehicle.max_steer_angle_rate()) ||
        !positive(vehicle.min_turn_radius()) ||
        !positive(vehicle.max_acceleration()) ||
        !std::isfinite(vehicle.max_deceleration()) ||
        vehicle.max_deceleration() >= 0.0) {
      AERROR << "Incomplete primitive configuration or physical vehicle limits";
      return false;
    }
    const double wheel_angle =
        vehicle.max_steer_angle() / vehicle.steer_ratio();
    if (wheel_angle >= 1.5707963267948966) {
      AERROR << "Invalid physical steering limit";
      return false;
    }
    MotionPrimitiveModel model;
    model.max_speed_mps = conf.max_speed_mps();
    model.max_acceleration_mps2 =
        std::min(conf.max_acceleration_mps2(), vehicle.max_acceleration());
    model.max_deceleration_mps2 =
        std::min(conf.max_deceleration_mps2(), -vehicle.max_deceleration());
    model.max_jerk_mps3 = conf.max_jerk_mps3();
    model.max_curvature_per_m = std::min(
        conf.max_curvature_per_m(),
        std::min(1.0 / vehicle.min_turn_radius(),
                 std::tan(wheel_angle) / vehicle.wheel_base()));
    model.max_curvature_derivative_per_m2 =
        std::min(conf.max_curvature_derivative_per_m2(),
                 vehicle.max_steer_angle_rate() / vehicle.steer_ratio() /
                     vehicle.wheel_base() / conf.max_speed_mps());
    model.max_lateral_acceleration_mps2 = conf.max_lateral_acceleration_mps2();
    model.footprint_radius_m = radius + conf.footprint_margin_m();
    model.sample_period_sec = conf.sample_period_sec();
    model.spatial_step_m = conf.spatial_step_m();
    model.max_state_age_sec = conf.max_state_age_sec();
    model.max_tracking_heading_error_rad = conf.max_tracking_heading_error_rad();
    motion_primitive_executor_ = std::make_unique<MotionPrimitiveExecutor>(model);
    if (!motion_primitive_executor_->IsAvailable()) {
      AERROR << "Invalid configured primitive execution model";
      return false;
    }
    if (controller_profiles_.SupportsSpatialPrimitives()) {
      capabilities.supported.insert(planning::MOTION_CAPABILITY_POSE_SERVO);
      capabilities.supported.insert(planning::MOTION_CAPABILITY_CORRIDOR_SERVO);
    }
  }
  motion_execution_manager_ = std::make_unique<MotionExecutionManager>(
      MotionExecutionValidator(std::move(capabilities)));
  return true;
}

void ControlComponent::InitReaders() {
  cyber::ReaderConfig chassis_cfg;
  chassis_cfg.channel_name = FLAGS_chassis_topic;
  chassis_cfg.pending_queue_size = FLAGS_chassis_pending_queue_size;
  chassis_reader_ = node_->CreateReader<Chassis>(chassis_cfg, nullptr);

  cyber::ReaderConfig loc_cfg;
  loc_cfg.channel_name = FLAGS_localization_topic;
  loc_cfg.pending_queue_size = FLAGS_localization_pending_queue_size;
  localization_reader_ =
      node_->CreateReader<LocalizationEstimate>(loc_cfg, nullptr);

  cyber::ReaderConfig pad_cfg;
  pad_cfg.channel_name = FLAGS_pad_topic;
  pad_cfg.pending_queue_size = FLAGS_pad_msg_pending_queue_size;
  pad_msg_reader_ = node_->CreateReader<PadMessage>(pad_cfg, nullptr);
}

void ControlComponent::OnPad(const std::shared_ptr<PadMessage> &pad) {
  std::lock_guard<std::mutex> lock(mutex_);
  pad_msg_.CopyFrom(*pad);

  // Industrial Practice: Process Reset signal immediately via SafetyManager
  // instead of waiting for the next control cycle.
  if (safety_manager_) {
    safety_manager_->TryReset(pad_msg_);
  }
}

void ControlComponent::OnChassis(const std::shared_ptr<Chassis> &chassis) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_chassis_.CopyFrom(*chassis);
}

void ControlComponent::OnLocalization(
    const std::shared_ptr<LocalizationEstimate> &localization) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_localization_.CopyFrom(*localization);
}

MotionExecutionVehicleState ControlComponent::BuildMotionVehicleState() const {
  MotionExecutionVehicleState state;
  const auto& canonical = injector_->vehicle_state();
  if (vehicle_state_ready_ && canonical.has_x() && canonical.has_y() &&
      canonical.has_heading() && canonical.has_linear_velocity() &&
      canonical.has_timestamp()) {
    state.position.set_x(canonical.x());
    state.position.set_y(canonical.y());
    state.position.set_z(canonical.z());
    state.heading = canonical.heading();
    state.gear = canonical.gear();
    state.speed_mps = canonical.linear_velocity();
    state.timestamp_sec = canonical.timestamp();
  }
  state.reference_frame_id = local_view_.localization().header().frame_id();
  return state;
}

void ControlComponent::ReleaseMotionExecutor() {
  executor_arbiter_.Release();
  const auto reset = controller_profiles_.Reset();
  if (!reset.ok()) {
    AERROR << "Controller release failed: " << reset.error_message();
    execution_state_fault_ = true;
  }
  if (motion_primitive_executor_) {
    motion_primitive_executor_->Reset();
  }
  if (motion_execution_monitor_) {
    motion_execution_monitor_->Reset();
  }
  latest_trajectory_.Clear();
  local_view_.mutable_trajectory()->Clear();
  previous_cmd_.Clear();
  active_cleanup_mission_sequence_ = 0;
}

void ControlComponent::FailMotionExecution(double now_sec,
                                           const std::string& reason) {
  if (motion_execution_manager_->active_command()) {
    latest_motion_execution_status_ =
        motion_execution_manager_->Fail(now_sec, reason);
    reported_motion_scope_ = active_motion_scope_;
  }
  ReleaseMotionExecutor();
}

bool ControlComponent::CheckMotionAuthorization(
    const execution_state_sync::Event& event,
    const planning::MotionDirective& directive) const {
  using execution_state_sync::Channel;
  const auto view = execution_state_client_->Latest();
  if (!view || !view->result.ok() || !view->snapshot ||
      event.owner != execution_state_sync::Role::kPlanning) {
    return false;
  }
  const auto& mission = view->snapshot->latest[static_cast<size_t>(Channel::kMission)];
  if (!mission) {
    return false;
  }
  bool guarded = false;
  for (const auto& guard : event.operation.guards) {
    guarded = guarded || (guard.channel == Channel::kMission &&
                           guard.sequence == mission->sequence);
  }
  if (!guarded) {
    return false;
  }
  const bool cancel = directive.has_cancel();
  const auto* command = directive.has_execute() ? &directive.execute().command()
      : directive.has_replace() ? &directive.replace().command() : nullptr;
  const bool hold = command && command->has_primitive() &&
      command->primitive().type() == planning::MOTION_PRIMITIVE_STANDSTILL_HOLD;
  const bool controlled_stop = command && command->has_trajectory() &&
      command->identity().command_id() == "controlled-stop" &&
      directive.scope() == planning::MOTION_SCOPE_MISSION_DESCENDANT &&
      command->has_control_intent() &&
      command->control_intent().tracking_mode() == planning::TRACKING_MODE_TRAJECTORY &&
      command->control_intent().execution_channel() == planning::EXECUTION_CHANNEL_TRAJECTORY &&
      command->control_intent().primitive_type() == planning::CONTROL_PRIMITIVE_NONE &&
      command->control_intent().longitudinal_intent() == planning::LON_INTENT_MRM_STOP &&
      command->control_intent().require_full_stop();
  // This only authorizes admission. The manager still validates the complete
  // trajectory, dynamic bounds, envelope, identity and live start condition.
  if (mission->operation.mission_fenced &&
      (!event.operation.allow_when_fenced || (!cancel && !hold && !controlled_stop))) {
    return false;
  }
  planning::PlanningRuntimeStatus accepted_authority;
  if (!view->snapshot->accepted_authority ||
      !accepted_authority.ParseFromString(
          view->snapshot->accepted_authority->operation.payload) ||
      !accepted_authority.has_mission_identity()) {
    return false;
  }
  const auto& accepted_identity = accepted_authority.mission_identity();
  if (command &&
      (!command->has_authority_generation() ||
       command->authority_generation() != accepted_identity.revision() ||
       !command->has_authorized_mission_identity() ||
       command->authorized_mission_identity().SerializeAsString() !=
           accepted_identity.SerializeAsString())) {
    return false;
  }
  if (directive.scope() == planning::MOTION_SCOPE_PLANNING_IDLE_HOLD) {
    return !directive.has_parent_mission_identity() && !cancel && hold &&
           command != nullptr;
  }
  if (!directive.has_parent_mission_identity()) {
    return false;
  }
  if (directive.parent_mission_identity().SerializeAsString() ==
      accepted_identity.SerializeAsString()) {
    return true;
  }
  // Cancellation cleanup may name the revoked parent, never a new moving goal.
  return mission->operation.mission_fenced && (cancel || hold || controlled_stop) &&
      directive.parent_mission_identity().SerializeAsString() ==
          motion_execution_manager_->status().parent_mission_identity().SerializeAsString();
}

bool ControlComponent::PollExecutionState(double now_sec) {
  using execution_state_sync::Channel;
  using execution_state_sync::Code;
  if (!execution_state_client_ || execution_state_fault_) {
    return false;
  }
  std::vector<execution_state_sync::Event> events;
  const auto poll = execution_state_client_->Poll(&events);
  if (!poll.ok() && poll.code != Code::kBusy) {
    AERROR << "Control execution-state poll failed: " << poll.message;
    execution_state_fault_ = true;
    return false;
  }
  execution_state_sync::Submission submission;
  while (execution_state_client_->TakeSubmission(&submission).ok()) {
    if (!submission.result.ok()) {
      AERROR << "Control execution-state commit failed: " << submission.result.message;
      execution_state_fault_ = true;
      return false;
    }
    if (submission.operation.channel == Channel::kControlStatus) {
      status_ticket_ = 0;
      ControlRuntimeStatus committed;
      if (!committed.ParseFromString(submission.operation.payload)) {
        execution_state_fault_ = true;
        return false;
      }
      if (!control_runtime_status_writer_->Write(committed)) {
        AERROR << "Failed to publish committed control-status topic mirror";
      }
    } else if (submission.operation.channel == Channel::kSafetyStatus) {
      safety_status_ticket_ = 0;
    }
  }
  const auto view = execution_state_client_->Latest();
  if (!execution_state_client_->Ready() || !view || !view->snapshot ||
      !view->result.ok() ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    view->observed_at).count() >
          motion_state_max_age_sec_) {
    return false;
  }
  if (!safety_latch_reconciled_) {
    safety_latch_reconciled_ = true;
    const auto& previous_safety =
        view->snapshot->latest[static_cast<size_t>(Channel::kSafetyStatus)];
    if (previous_safety) {
      SafetyStopObservation restored;
      if (!restored.ParseFromString(previous_safety->operation.payload)) {
        execution_state_fault_ = true;
        AERROR << "Persisted Control safety observation is invalid";
        return false;
      }
      if (restored.safety_latched()) {
        if (!restored.has_operation_identity() ||
            restored.operation_identity().requester_epoch().empty() ||
            restored.operation_identity().request_id().empty() ||
            restored.effective_policy() != SAFETY_STOP_CONTROLLED) {
          execution_state_fault_ = true;
          AERROR << "Persisted safety latch lacks a supported identity or policy";
          return false;
        }
        const auto identity = safety_manager_->LatchExternalStop(
            restored.operation_identity(), restored.effective_policy(),
            control_epoch_);
        if (!identity.has_generation()) {
          execution_state_fault_ = true;
          AERROR << "Control failed to restore persisted safety latch";
          return false;
        }
        restored.mutable_safety_identity()->CopyFrom(identity);
        restored.set_observed_at_sec(now_sec);
        restored.set_operation_accepted(true);
        restored.set_enforced(true);
        restored.set_safety_latched(true);
        restored.clear_durably_committed();
        restored.clear_commit_sequence();
        restored.clear_stop_confirmed();
        restored.clear_absolute_linear_speed_mps();
        restored.clear_absolute_yaw_rate_radps();
        restored.clear_settled_duration_sec();
        const bool stationary =
            vehicle_state_ready_ && local_view_.chassis().has_speed_mps() &&
            std::isfinite(local_view_.chassis().speed_mps()) &&
            std::abs(local_view_.chassis().speed_mps()) <= 0.05;
        restored.set_stop_confirmed(stationary);
        if (stationary) {
          restored.set_absolute_linear_speed_mps(
              std::abs(local_view_.chassis().speed_mps()));
        }
        restored.set_reason("Control restored the durable safety latch");
        if (pending_safety_status_.size() >=
            execution_state_sync::Client::kEventBatchSize * 2) {
          execution_state_fault_ = true;
          AERROR << "Safety restoration observation backlog exceeded capacity";
          return false;
        }
        pending_safety_status_.push_back(std::move(restored));
      }
    }
  }
  const auto& mission = view->snapshot->latest[static_cast<size_t>(Channel::kMission)];
  if (mission) {
    mission_event_sequence_ = mission->sequence;
    const auto* active = motion_execution_manager_->active_command();
    if (active && active_motion_scope_ == planning::MOTION_SCOPE_MISSION_DESCENDANT) {
      planning::PlanningRuntimeStatus accepted_authority;
      const bool has_accepted_authority =
          view->snapshot->accepted_authority &&
          accepted_authority.ParseFromString(
              view->snapshot->accepted_authority->operation.payload) &&
          accepted_authority.has_mission_identity();
      if (!has_accepted_authority) {
        execution_state_fault_ = true;
        FailMotionExecution(now_sec,
                            "accepted Mission authority projection is unavailable");
        return false;
      }
      const auto& parent = motion_execution_manager_->status().parent_mission_identity();
      if (active_cleanup_mission_sequence_ != mission->sequence &&
          (mission->operation.mission_fenced ||
           accepted_authority.mission_identity().SerializeAsString() !=
               parent.SerializeAsString() ||
           active->authority_generation() !=
               accepted_authority.mission_identity().revision())) {
        planning::MotionDirective fence;
        fence.set_scope(planning::MOTION_SCOPE_MISSION_DESCENDANT);
        fence.mutable_parent_mission_identity()->CopyFrom(parent);
        fence.mutable_cancel()->mutable_target_identity()->CopyFrom(active->identity());
        fence.mutable_cancel()->set_fence_parent_mission(true);
        fence.mutable_cancel()->set_reason("committed parent authorization revoked");
        ProcessMotionDirective(fence, now_sec);
      }
    }
  }
  for (const auto& event : events) {
    if (event.operation.channel == Channel::kSafetyRequest) {
      if (!ProcessSafetyOperation(event, now_sec)) {
        execution_state_fault_ = true;
        return false;
      }
      continue;
    }
    if (event.operation.channel != Channel::kMotion) {
      continue;
    }
    motion_event_sequence_ = event.sequence;
    planning::MotionDirective directive;
    if (!directive.ParseFromString(event.operation.payload)) {
      execution_state_fault_ = true;
      FailMotionExecution(now_sec, "invalid committed motion directive");
      return false;
    }
    if (!CheckMotionAuthorization(event, directive)) {
      // Report the rejected incoming owner without destroying valid active work.
      latest_motion_execution_status_.Clear();
      latest_motion_execution_status_.mutable_header()->set_timestamp_sec(now_sec);
      latest_motion_execution_status_.set_state(planning::MOTION_EXECUTION_REJECTED);
      latest_motion_execution_status_.set_reject_reason(planning::MOTION_REJECT_PARENT_FENCED);
      latest_motion_execution_status_.set_reason("committed parent authorization is stale or fenced");
      const auto* rejected = directive.has_execute() ? &directive.execute().command()
          : directive.has_replace() ? &directive.replace().command() : nullptr;
      if (rejected) {
        latest_motion_execution_status_.mutable_identity()->CopyFrom(rejected->identity());
        latest_motion_execution_status_.set_reference_frame_id(rejected->reference_frame_id());
        if (rejected->has_authorized_mission_identity()) {
          latest_motion_execution_status_.mutable_authorized_mission_identity()
              ->CopyFrom(rejected->authorized_mission_identity());
        }
        if (rejected->has_authority_generation()) {
          latest_motion_execution_status_.set_authority_generation(
              rejected->authority_generation());
        }
      }
      if (directive.has_parent_mission_identity()) {
        latest_motion_execution_status_.mutable_parent_mission_identity()->CopyFrom(
            directive.parent_mission_identity());
      }
      directive_rejected_this_cycle_ = true;
      reported_motion_scope_ = directive.scope();
      continue;
    }
    ProcessMotionDirective(directive, now_sec, event.operation.allow_when_fenced);
  }
  if (!events.empty()) {
    const auto ack = execution_state_client_->Acknowledge(events.back().sequence);
    if (!ack.ok()) {
      execution_state_fault_ = true;
      FailMotionExecution(now_sec, "execution event acknowledgement failed");
      return false;
    }
  }
  FlushSafetyStatus();
  return !execution_state_fault_ && execution_state_client_->Healthy();
}

bool ControlComponent::ProcessSafetyOperation(
    const execution_state_sync::Event& event, double now_sec) {
  if (event.owner != execution_state_sync::Role::kMission) {
    AERROR << "Safety request was not committed by Mission";
    return false;
  }
  SafetyControlRequest request;
  if (!request.ParseFromString(event.operation.payload)) {
    AERROR << "Invalid committed SafetyControlRequest payload";
    return false;
  }
  SafetyStopObservation observation;
  observation.set_observed_at_sec(now_sec);
  observation.set_clock_domain("cyber");
  observation.set_operation_accepted(false);
  bool stationary = vehicle_state_ready_ &&
                    local_view_.chassis().has_speed_mps() &&
                    std::isfinite(local_view_.chassis().speed_mps()) &&
                    std::abs(local_view_.chassis().speed_mps()) <= 0.05;
  if (request.has_stop()) {
    const auto& stop = request.stop();
    if (!stop.has_identity() || stop.identity().requester_epoch().empty() ||
        stop.identity().request_id().empty() ||
        stop.policy() != SAFETY_STOP_CONTROLLED || stop.reason().empty()) {
      observation.set_reason("invalid controlled safety stop request");
    } else {
      const auto identity = safety_manager_->LatchExternalStop(
          stop.identity(), stop.policy(), control_epoch_);
      if (identity.has_generation()) {
        observation.mutable_operation_identity()->CopyFrom(stop.identity());
        observation.mutable_safety_identity()->CopyFrom(identity);
        observation.set_operation_accepted(true);
        observation.set_enforced(true);
        observation.set_safety_latched(true);
        observation.set_effective_policy(stop.policy());
        observation.set_stop_confirmed(stationary);
        if (stationary) {
          observation.set_absolute_linear_speed_mps(
              std::abs(local_view_.chassis().speed_mps()));
        }
        observation.set_reason("controlled stop latched in Control");
      } else {
        observation.set_reason("Control could not establish safety-stop identity");
      }
    }
  } else if (request.has_reset()) {
    const auto& reset_request = request.reset();
    if (!reset_request.has_identity() ||
        reset_request.identity().requester_epoch().empty() ||
        reset_request.identity().request_id().empty() ||
        !reset_request.has_expected_safety_identity()) {
      observation.set_reason("safety reset request identity is incomplete");
    } else {
      SafetyExecutionIdentity active_safety_identity;
      SafetyStopPolicy active_policy = SAFETY_STOP_POLICY_UNKNOWN;
      const bool had_active_stop = safety_manager_->GetExternalStopState(
          &active_safety_identity, &active_policy);
      const bool reset_accepted = safety_manager_->ResetExternalStop(
          reset_request.expected_safety_identity(), stationary);
      observation.mutable_operation_identity()->CopyFrom(
          reset_request.identity());
      if (had_active_stop) {
        observation.mutable_safety_identity()->CopyFrom(active_safety_identity);
        observation.set_effective_policy(active_policy);
      } else {
        observation.mutable_safety_identity()->CopyFrom(
            reset_request.expected_safety_identity());
      }
      observation.set_operation_accepted(reset_accepted);
      observation.set_enforced(reset_accepted);
      observation.set_safety_latched(safety_manager_->HasExternalStop());
      observation.set_stop_confirmed(stationary);
      if (stationary) {
        observation.set_absolute_linear_speed_mps(
            std::abs(local_view_.chassis().speed_mps()));
      }
      observation.set_reason(
          reset_accepted ? "safety stop reset accepted"
                         : "safety reset rejected: expected identity mismatch or vehicle not stationary");
    }
  } else {
    AERROR << "Committed safety request has no operation";
    return false;
  }
  observation.set_safety_latched(safety_manager_->HasExternalStop());
  if (pending_safety_status_.size() >=
      execution_state_sync::Client::kEventBatchSize * 2) {
    AERROR << "Safety observation backlog exceeded capacity";
    return false;
  }
  pending_safety_status_.push_back(std::move(observation));
  return true;
}

void ControlComponent::FlushSafetyStatus() {
  if (safety_status_ticket_ != 0 || pending_safety_status_.empty() ||
      !execution_state_client_ || !execution_state_client_->Ready() ||
      execution_state_fault_) {
    return;
  }
  const auto result = execution_state_client_->Submit(
      execution_state_sync::Channel::kSafetyStatus,
      pending_safety_status_.front().SerializeAsString(), {}, false, false,
      &safety_status_ticket_);
  if (result.ok()) {
    pending_safety_status_.pop_front();
  } else if (result.code != execution_state_sync::Code::kBusy &&
             result.code != execution_state_sync::Code::kQueueFull) {
    execution_state_fault_ = true;
    AERROR << "Safety observation persistence failed: " << result.message;
  }
}

void ControlComponent::FlushExecutionStatus() {
  using execution_state_sync::Channel;
  if (status_ticket_ != 0 || pending_runtime_status_.empty() ||
      !execution_state_client_ || !execution_state_client_->Ready() ||
      execution_state_fault_) {
    return;
  }
  const auto& pending = pending_runtime_status_.front();
  std::vector<execution_state_sync::Guard> guards;
  if (pending.mission_sequence != 0 && pending.motion_sequence != 0) {
    guards = {{Channel::kMission, pending.mission_sequence},
              {Channel::kMotion, pending.motion_sequence}};
  }
  const auto result = execution_state_client_->Submit(
      Channel::kControlStatus, pending.status.SerializeAsString(),
      std::move(guards), false, false, &status_ticket_,
      execution_state_sync::PlanningStatusKind::kRuntime,
      pending.mission_sequence != 0 && pending.motion_sequence != 0
          ? execution_state_sync::ControlStatusKind::kMotionResult
          : execution_state_sync::ControlStatusKind::kOwnerRuntime);
  if (result.ok()) {
    pending_runtime_status_.pop_front();
  } else if (result.code != execution_state_sync::Code::kBusy &&
             result.code != execution_state_sync::Code::kQueueFull) {
    execution_state_fault_ = true;
    FailMotionExecution(Clock::NowInSeconds(),
                        "control status persistence admission failed: " + result.message);
  }
}

void ControlComponent::ProcessMotionDirective(
    const planning::MotionDirective& directive, double now_sec,
    bool authorized_stop_cleanup) {
  motion_execution_manager_->Tick(now_sec);
  if (!motion_execution_manager_->active_command()) {
    ReleaseMotionExecutor();
  }
  auto execution_status = motion_execution_manager_->Apply(
      directive, now_sec, authorized_stop_cleanup);
  if (execution_status.state() == planning::MOTION_EXECUTION_REJECTED) {
    directive_rejected_this_cycle_ = true;
    reported_motion_scope_ = directive.scope();
    latest_motion_execution_status_ = execution_status;
    AERROR << "MotionDirective rejected: " << execution_status.reason();
    if (!motion_execution_manager_->active_command()) {
      ReleaseMotionExecutor();
    }
    return;
  }
  active_motion_scope_ = directive.scope();
  reported_motion_scope_ = active_motion_scope_;
  directive_rejected_this_cycle_ = false;
  if (execution_status.state() == planning::MOTION_EXECUTION_CANCELLING) {
    ReleaseMotionExecutor();
    const auto* command = motion_execution_manager_->active_command();
    if (command == nullptr || !vehicle_state_ready_) {
      FailMotionExecution(
          now_sec, "cannot report executor revocation without fresh vehicle state");
      return;
    }
    double position_error_m = 0.0;
    double heading_error_rad = 0.0;
    const auto& state = injector_->vehicle_state();
    MeasureCommandTerminalError(*command, state, &position_error_m,
                                &heading_error_rad);
    const auto evidence = BuildMotionTerminalEvidence(
        *command, planning::MOTION_TERMINAL_EVIDENCE_EXECUTOR_REVOCATION,
        planning::MOTION_EXECUTOR_OWNERSHIP_REVOKED,
        TranslateEvidenceSafetyState(safety_manager_->GetState()), state,
        local_view_.localization().header().frame_id(), position_error_m,
        heading_error_rad, 0.0);
    execution_status =
        motion_execution_manager_->ConfirmExecutorRevoked(evidence, now_sec);
    if (execution_status.state() != planning::MOTION_EXECUTION_CANCELLED) {
      FailMotionExecution(now_sec, execution_status.reason());
      return;
    }
  }

  if (execution_status.state() == planning::MOTION_EXECUTION_VALIDATED) {
    const auto *command = motion_execution_manager_->active_command();
    if (command != nullptr && vehicle_state_ready_) {
      const auto profile = strategy_orchestrator_.Resolve(*command);
      const auto selected = profile.supported
          ? controller_profiles_.Bind(
                profile.parameter_profile, *command,
                injector_->vehicle_state().gear(),
                injector_->vehicle_state().linear_velocity())
          : Status(ErrorCode::CONTROL_COMPUTE_ERROR, profile.profile_reason);
      if (!selected.ok()) {
        FailMotionExecution(now_sec, selected.error_message());
        return;
      }
    }
    if (command == nullptr || !vehicle_state_ready_ ||
        local_view_.chassis().driving_mode() != Chassis::COMPLETE_AUTO_DRIVE ||
        !executor_arbiter_.Acquire(*command)) {
      FailMotionExecution(now_sec, "state or executor ownership unavailable at handoff");
      return;
    }
    execution_status = motion_execution_manager_->Arm(now_sec);
    if (execution_status.state() != planning::MOTION_EXECUTION_ARMED) {
      FailMotionExecution(now_sec, "motion arm failed at ownership handoff");
      return;
    }
    execution_status =
        motion_execution_manager_->Start(BuildMotionVehicleState(), now_sec);
    if (execution_status.state() != planning::MOTION_EXECUTION_EXECUTING_TRAJECTORY &&
        execution_status.state() != planning::MOTION_EXECUTION_EXECUTING_PRIMITIVE) {
      FailMotionExecution(now_sec, "live start condition rejected at ownership handoff");
      return;
    }
    if (authorized_stop_cleanup) {
      active_cleanup_mission_sequence_ = mission_event_sequence_;
    }
  }
  latest_motion_execution_status_ = execution_status;
  if (!motion_execution_manager_->active_command()) {
    ReleaseMotionExecutor();
  }
}

void ControlComponent::AdvanceMotionExecution(double now_sec) {
  const auto current = motion_execution_manager_->Tick(now_sec);
  if (!directive_rejected_this_cycle_) {
    latest_motion_execution_status_ = current;
    reported_motion_scope_ = active_motion_scope_;
  }
  const auto* command = motion_execution_manager_->active_command();
  if (!command) {
    ReleaseMotionExecutor();
    return;
  }
  if (!vehicle_state_ready_ || !executor_arbiter_.has_owner()) {
    FailMotionExecution(now_sec, "no fresh canonical state or executor owner");
    return;
  }
  const auto& state = injector_->vehicle_state();
  const auto expected_gear = command->has_trajectory()
      ? command->trajectory().gear() : command->start_condition().expected_gear();
  if (!state.has_gear() || state.gear() != expected_gear) {
    FailMotionExecution(now_sec, "live gear no longer matches authorized direction");
    return;
  }
  const auto& frame = local_view_.localization().header().frame_id();
  const bool moving_primitive = command->has_primitive() &&
      command->primitive().type() != planning::MOTION_PRIMITIVE_STANDSTILL_HOLD;
  MotionPrimitiveResult evidence;
  if (moving_primitive) {
    if (!motion_primitive_executor_) {
      FailMotionExecution(now_sec, "primitive backend is not configured");
      return;
    }
    evidence = motion_primitive_executor_->Update(*command, state, frame, now_sec);
  } else {
    evidence = motion_execution_monitor_->Update(*command, state, frame, now_sec);
    if (evidence.accepted) {
      std::string reason;
      if (!motion_command_adapter_.ToLegacyControllerInput(
              *command, &evidence.reference, &reason)) {
        FailMotionExecution(now_sec, reason);
        return;
      }
      if (command->has_primitive()) {
        // A hold is continuously authorized, not a stale timed trajectory.
        evidence.reference.mutable_header()->set_timestamp_sec(now_sec);
      }
    }
  }
  if (!evidence.accepted) {
    FailMotionExecution(now_sec, evidence.reason);
    return;
  }
  if (evidence.completed) {
    const auto terminal_evidence = BuildMotionTerminalEvidence(
        *command, planning::MOTION_TERMINAL_EVIDENCE_COMPLETION,
        planning::MOTION_EXECUTOR_OWNERSHIP_ACTIVE,
        TranslateEvidenceSafetyState(safety_manager_->GetState()), state,
        evidence.reference_frame_id, evidence.position_error_m,
        evidence.heading_error_rad, evidence.settled_duration_sec);
    latest_motion_execution_status_ =
        motion_execution_manager_->Succeed(terminal_evidence, now_sec);
    if (latest_motion_execution_status_.state() !=
        planning::MOTION_EXECUTION_SUCCEEDED) {
      FailMotionExecution(now_sec, latest_motion_execution_status_.reason());
      return;
    }
    reported_motion_scope_ = active_motion_scope_;
    ReleaseMotionExecutor();
    return;
  }
  if (evidence.hold_confirmed && !moving_primitive) {
    const auto terminal_evidence = BuildMotionTerminalEvidence(
        *command, planning::MOTION_TERMINAL_EVIDENCE_STANDSTILL_HOLD,
        planning::MOTION_EXECUTOR_OWNERSHIP_ACTIVE,
        TranslateEvidenceSafetyState(safety_manager_->GetState()), state,
        evidence.reference_frame_id, evidence.position_error_m,
        evidence.heading_error_rad, evidence.settled_duration_sec);
    const auto holding =
        motion_execution_manager_->EnterHolding(terminal_evidence, now_sec);
    if (holding.state() != planning::MOTION_EXECUTION_HOLDING) {
      FailMotionExecution(now_sec, holding.reason());
      return;
    }
    if (!directive_rejected_this_cycle_) {
      latest_motion_execution_status_ = holding;
    }
  }
  latest_trajectory_.CopyFrom(evidence.reference);
  local_view_.mutable_trajectory()->CopyFrom(evidence.reference);
}

Status ControlComponent::ProduceControlCommand(ControlCommand *control_command,
                                               bool *used_previous_command) {
  last_goal_ = BuildControlCommandGoal(local_view_.trajectory());
  const auto* authorized = motion_execution_manager_->active_command();
  if (authorized != nullptr) {
    last_profile_ = strategy_orchestrator_.Resolve(*authorized);
  } else {
    last_profile_ = strategy_orchestrator_.Resolve(last_goal_);
  }

  // 2. Safety Pre-Check (Input Validation)
  // Checks timestamps, sensor health, and trajectory integrity.
  SafetyResult input_res = safety_manager_->PreCheck(local_view_);

  Status status = Status::OK();
  bool use_previous_cmd = false;

  if (!input_res.must_bypass) {
    // 3. Core Control Computation
    // Only run algorithms if system is relatively healthy.
    if (authorized == nullptr || !executor_arbiter_.has_owner() ||
        !last_profile_.supported) {
      status = Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                      "controller selection has no authorized executor");
    } else {
      status = controller_profiles_.Bind(
          last_profile_.parameter_profile, *authorized,
          injector_->vehicle_state().gear(),
          injector_->vehicle_state().linear_velocity());
      if (status.ok()) {
        status = controller_profiles_.ComputeControlCommand(
            &local_view_.localization(), &local_view_.chassis(),
            &local_view_.trajectory(), control_command);
      }
    }

    if (status.ok()) {
      // 4. Safety Post-Check (Output Validation)
      // Sanity check on computed commands (e.g., jerk, steering rate).
      SafetyResult output_res = safety_manager_->PostCheck(*control_command);
      if (output_res.need_freeze) {
        use_previous_cmd = true;
        status = Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                        "Output Limits Violated (Freeze)");
      }
    } else {
      // Logic failure (e.g., solver error) is treated as an internal fault.
      status = Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                      "Controller computation failed: " + status.error_message());
      use_previous_cmd = true;
    }
  } else {
    use_previous_cmd = true;
    status = Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                    "Input Physics Missing (Bypass)");
  }

  // 5. Freeze Strategy
  if (use_previous_cmd) {
    ProduceSafeStop(control_command);
    FailMotionExecution(Clock::NowInSeconds(), status.error_message());
  }

  if (used_previous_command != nullptr) {
    *used_previous_command = use_previous_cmd;
  }

  // 6. Apply Safety Policy (The Override)
  // This is the final authority. It overrides the command based on the FSM
  // state (Normal, SoftStop, HardEstop).
  safety_manager_->ApplySafetyPolicy(control_command);

  // 6. Housekeeping
  previous_cmd_ = *control_command;

  return status;
}

void ControlComponent::PublishRuntimeStatus(
    const ControlCommand &control_command, const Status &status,
    bool used_previous_command) {
  if (control_runtime_status_writer_ == nullptr) {
    return;
  }

  ControlRuntimeStatus runtime_status;
  common::util::FillHeader(node_->Name(), &runtime_status);

  const auto runtime_context =
      ExtractTrajectoryRuntimeContext(local_view_.trajectory());
  if (!runtime_context.mission_id.empty()) {
    runtime_status.set_mission_id(runtime_context.mission_id);
  }
  if (!runtime_context.command_id.empty()) {
    runtime_status.set_command_id(runtime_context.command_id);
  }
  runtime_status.set_active_scene(runtime_context.active_scene);
  runtime_status.set_active_mode(runtime_context.active_mode);
  runtime_status.set_active_shell(runtime_context.active_shell);
  runtime_status.set_driving_mode(local_view_.chassis().driving_mode());
  runtime_status.set_input_ready(local_view_.chassis().has_header() &&
                                 local_view_.localization().has_header());
  runtime_status.set_trajectory_available(
      local_view_.trajectory().has_header());
  runtime_status.set_trajectory_point_available(
      local_view_.trajectory().trajectory_point_size() > 0);
  runtime_status.set_using_previous_command(used_previous_command);
  runtime_status.set_estop_active(local_view_.trajectory().has_estop() &&
                                  local_view_.trajectory().estop().is_estop());
  runtime_status.set_manual_mode(local_view_.chassis().driving_mode() !=
                                 Chassis::COMPLETE_AUTO_DRIVE);
  runtime_status.set_parking_brake_applied(control_command.parking_brake());

  const SafetyState safety_state = safety_manager_ != nullptr
                                       ? safety_manager_->GetState()
                                       : SafetyState::kNormal;
  const auto control_safety_state = TranslateSafetyState(safety_state);
  runtime_status.set_safety_state(control_safety_state);
  if (latest_motion_execution_status_.has_identity()) {
    if (!runtime_status.has_command_id() &&
        latest_motion_execution_status_.identity().has_command_id()) {
      runtime_status.set_command_id(
          latest_motion_execution_status_.identity().command_id());
    }
  }
  if (latest_motion_execution_status_.has_authorized_mission_identity()) {
    const auto& mission =
        latest_motion_execution_status_.authorized_mission_identity();
    runtime_status.set_mission_id(mission.aggregate_id());
    runtime_status.set_command_id(mission.command_id());
  }

  if (runtime_context.has_control_intent) {
    runtime_status.set_tracking_mode(
        runtime_context.control_intent.tracking_mode());
    runtime_status.set_longitudinal_intent(
        runtime_context.control_intent.longitudinal_intent());
    runtime_status.set_lateral_intent(
        runtime_context.control_intent.lateral_intent());
    runtime_status.set_stop_class(runtime_context.control_intent.stop_class());
    runtime_status.set_primitive_type(
        runtime_context.control_intent.primitive_type());
    runtime_status.set_primitive_active(
        (executor_arbiter_.has_owner() &&
         motion_execution_manager_->status().execution_type() ==
             planning::MOTION_EXECUTION_TYPE_PRIMITIVE) ||
        runtime_context.control_intent.primitive_type() !=
            apollo::planning::CONTROL_PRIMITIVE_NONE);
    runtime_status.set_trajectory_optional(
        IsTrajectorylessControlPrimitive(local_view_.trajectory()));
    runtime_status.set_execution_channel(runtime_context.execution_channel);
    if (runtime_context.control_intent.has_stop_reason_code()) {
      runtime_status.set_stop_reason_code(
          runtime_context.control_intent.stop_reason_code());
    }
  } else if (runtime_context.execution_channel !=
             apollo::planning::EXECUTION_CHANNEL_UNKNOWN) {
    runtime_status.set_execution_channel(runtime_context.execution_channel);
  }
  if (latest_motion_execution_status_.has_state()) {
    runtime_status.mutable_motion_execution()->CopyFrom(
        latest_motion_execution_status_);
    runtime_status.set_motion_scope(reported_motion_scope_);
    runtime_status.set_executor_owner_active(executor_arbiter_.has_owner());
    const auto* active = motion_execution_manager_->active_command();
    if (executor_arbiter_.has_owner() && active != nullptr) {
      runtime_status.mutable_active_motion_identity()->CopyFrom(
          active->identity());
      runtime_status.mutable_active_mission_identity()->CopyFrom(
          active->authorized_mission_identity());
    }
    runtime_status.mutable_controller_selection()->CopyFrom(
        controller_profiles_.status());
  }

  std::string reason;
  if (!status.ok()) {
    reason = status.error_message();
  } else if (runtime_context.has_control_intent &&
             runtime_context.control_intent.has_reason()) {
    reason = runtime_context.control_intent.reason();
  }

  if (runtime_status.manual_mode()) {
    runtime_status.set_state(CONTROL_RUNTIME_MANUAL);
    if (reason.empty()) {
      reason = "chassis not in auto-drive";
    }
  } else if (control_safety_state == CONTROL_SAFETY_HARD_ESTOP ||
             control_safety_state == CONTROL_SAFETY_FATAL ||
             runtime_status.estop_active()) {
    runtime_status.set_state(CONTROL_RUNTIME_ESTOP);
    if (reason.empty()) {
      reason = "control safety estop active";
    }
  } else if (control_safety_state == CONTROL_SAFETY_SOFT_STOP) {
    runtime_status.set_state(CONTROL_RUNTIME_SOFT_STOP);
    if (reason.empty()) {
      reason = "control safety soft-stop active";
    }
  } else if (!runtime_status.input_ready() ||
             (!runtime_status.trajectory_point_available() &&
              !IsTrajectorylessControlPrimitive(local_view_.trajectory()))) {
    runtime_status.set_state(CONTROL_RUNTIME_WAITING_INPUT);
    if (reason.empty()) {
      reason = "control inputs not ready";
    }
  } else if (!status.ok() && used_previous_command) {
    runtime_status.set_state(CONTROL_RUNTIME_FAULTED);
  } else if (control_safety_state == CONTROL_SAFETY_WARNING) {
    runtime_status.set_state(CONTROL_RUNTIME_DEGRADED);
    if (reason.empty()) {
      reason = "control warning policy active";
    }
  } else if (runtime_context.has_control_intent &&
             IsHoldingIntent(runtime_context.control_intent)) {
    runtime_status.set_state(CONTROL_RUNTIME_HOLDING);
  } else {
    runtime_status.set_state(CONTROL_RUNTIME_RUNNING);
  }

  if (!reason.empty()) {
    runtime_status.set_reason(reason);
  } else if (!last_profile_.profile_key.empty()) {
    runtime_status.set_reason("profile=" + last_profile_.profile_key);
  }

  if (motion_event_sequence_ == 0) {
    auto fingerprint_status = runtime_status;
    fingerprint_status.clear_header();
    const std::string fingerprint = fingerprint_status.SerializeAsString();
    const double now = Clock::NowInSeconds();
    if (fingerprint == last_unscoped_status_fingerprint_ &&
        now - last_unscoped_status_submit_sec_ < 0.2) {
      FlushExecutionStatus();
      return;
    }
    last_unscoped_status_fingerprint_ = fingerprint;
    last_unscoped_status_submit_sec_ = now;
  }

  const auto motion_state = runtime_status.motion_execution().state();
  const bool heartbeat =
      motion_state == planning::MOTION_EXECUTION_EXECUTING_TRAJECTORY ||
      motion_state == planning::MOTION_EXECUTION_EXECUTING_PRIMITIVE ||
      motion_state == planning::MOTION_EXECUTION_HOLDING;
  const bool repeated_result =
      !pending_runtime_status_.empty() &&
      pending_runtime_status_.back().status.motion_execution().SerializeAsString() ==
          runtime_status.motion_execution().SerializeAsString();
  if ((heartbeat || repeated_result) && !pending_runtime_status_.empty() &&
      pending_runtime_status_.back().status.safety_state() ==
          runtime_status.safety_state() &&
      pending_runtime_status_.back().status.state() == runtime_status.state() &&
      pending_runtime_status_.back().status.executor_owner_active() ==
          runtime_status.executor_owner_active() &&
      pending_runtime_status_.back().status.controller_selection().SerializeAsString() ==
          runtime_status.controller_selection().SerializeAsString() &&
      pending_runtime_status_.back()
              .status.active_motion_identity()
              .SerializeAsString() ==
          runtime_status.active_motion_identity().SerializeAsString() &&
      pending_runtime_status_.back().mission_sequence == mission_event_sequence_ &&
      pending_runtime_status_.back().motion_sequence == motion_event_sequence_ &&
      pending_runtime_status_.back().status.motion_execution().identity().SerializeAsString() ==
          runtime_status.motion_execution().identity().SerializeAsString() &&
      pending_runtime_status_.back().status.motion_execution().state() ==
          runtime_status.motion_execution().state()) {
    pending_runtime_status_.back().status = runtime_status;
  } else if (pending_runtime_status_.size() < execution_state_sync::Client::kEventBatchSize) {
    pending_runtime_status_.push_back(
        {runtime_status, mission_event_sequence_, motion_event_sequence_});
  } else {
    execution_state_fault_ = true;
    FailMotionExecution(Clock::NowInSeconds(), "control status backlog exceeded capacity");
  }
  FlushExecutionStatus();
}

bool ControlComponent::Proc() {
  const auto start_time = Clock::Now();

  // 1. Data Observation (Lock-Free Read)
  chassis_reader_->Observe();
  localization_reader_->Observe();
  pad_msg_reader_->Observe();

  auto chassis_msg = chassis_reader_->GetLatestObserved();
  if (chassis_msg == nullptr) {
    AERROR_EVERY(100) << "Chassis msg is not ready!";
  } else {
    OnChassis(chassis_msg);
  }

  auto localization_msg = localization_reader_->GetLatestObserved();
  if (localization_msg) {
    OnLocalization(localization_msg);
  }

  auto pad_msg = pad_msg_reader_->GetLatestObserved();
  if (pad_msg) {
    OnPad(pad_msg);
  }

  // 2. Data Preparation (Critical Section)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.mutable_chassis()->CopyFrom(latest_chassis_);
    local_view_.mutable_trajectory()->CopyFrom(latest_trajectory_);
    local_view_.mutable_localization()->CopyFrom(latest_localization_);
    if (pad_msg != nullptr) {
      local_view_.mutable_pad_msg()->CopyFrom(pad_msg_);
    }
  }

  // 3. Main Control Loop
  ControlCommand control_command;
  Status status;
  bool used_previous_command = false;
  const double now_sec = Clock::NowInSeconds();
  directive_rejected_this_cycle_ = false;
  const auto state_status = injector_->UpdateVehicleState(
      local_view_.localization(), local_view_.chassis());
  const auto& chassis_header = local_view_.chassis().header();
  vehicle_state_ready_ = state_status.ok() &&
      chassis_header.has_timestamp_sec() &&
      std::isfinite(chassis_header.timestamp_sec()) &&
      chassis_header.timestamp_sec() <= now_sec &&
      now_sec - chassis_header.timestamp_sec() <= motion_state_max_age_sec_ &&
      !local_view_.localization().header().frame_id().empty();

  const bool sync_ready = PollExecutionState(now_sec);
  // Check driving mode
  if (local_view_.chassis().driving_mode() ==
      apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    if (!vehicle_state_ready_ || !sync_ready) {
      const std::string reason = state_status.ok()
          ? "motion requires fresh state, explicit frame and healthy committed authorization"
          : state_status.error_message();
      FailMotionExecution(now_sec, reason);
      ProduceSafeStop(&control_command);
      status = Status(ErrorCode::CONTROL_COMPUTE_ERROR, reason);
    } else {
      AdvanceMotionExecution(now_sec);
      if (executor_arbiter_.has_owner()) {
        status = ProduceControlCommand(&control_command, &used_previous_command);
        const auto safety = safety_manager_->GetState();
        if (!status.ok() || safety == SafetyState::kSoftStop ||
            safety == SafetyState::kHardEstop || safety == SafetyState::kFatal) {
          FailMotionExecution(now_sec, status.ok()
              ? "safety policy revoked normal executor" : status.error_message());
          ProduceSafeStop(&control_command);
        }
      } else {
        ProduceSafeStop(&control_command);
        status = Status::OK();
      }
    }
  } else {
    if (executor_arbiter_.has_owner()) {
      FailMotionExecution(now_sec, "manual mode revoked normal executor");
    }
    // In Manual Mode, reset algorithms and produce neutral command.
    ResetAndProduceZeroControlCommand(&control_command);
    safety_manager_->ApplySafetyPolicy(&control_command);
    status = Status::OK();

    // Note: Manual mode does not bypass SafetyManager state entirely,
    // but the Reset command ensures no actuator conflict.
  }

  if (pad_msg != nullptr) {
    control_command.mutable_pad_msg()->CopyFrom(pad_msg_);
  }

  // 4. Header & Diagnostics
  if (!status.ok()) {
    AERROR_EVERY(100) << "Control Error: " << status.error_message();
    control_command.mutable_header()->mutable_status()->set_msg(
        status.error_message());
  }

  // Fill timestamps
  control_command.mutable_header()->set_lidar_timestamp(
      local_view_.trajectory().header().lidar_timestamp());
  control_command.mutable_header()->set_camera_timestamp(
      local_view_.trajectory().header().camera_timestamp());
  control_command.mutable_header()->set_radar_timestamp(
      local_view_.trajectory().header().radar_timestamp());
  common::util::FillHeader(node_->Name(), &control_command);

  // Latency metrics
  const auto end_time = Clock::Now();
  const double time_diff_ms = (end_time - start_time).ToSecond() * 1e3;
  control_command.mutable_latency_stats()->set_total_time_ms(time_diff_ms);

  // 5. Publish
  PublishRuntimeStatus(control_command, status, used_previous_command);
  if (execution_state_fault_ &&
      local_view_.chassis().driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    FailMotionExecution(now_sec, "execution-state persistence failed");
    ProduceSafeStop(&control_command);
    common::util::FillHeader(node_->Name(), &control_command);
  }
  if (!control_conf_.is_control_test_mode()) {
    control_cmd_writer_->Write(control_command);
  }

  return true;
}

void ControlComponent::ProduceSafeStop(ControlCommand* command) {
  command->Clear();
  command->set_throttle(0.0);
  command->set_speed(0.0);
  command->set_acceleration(0.0);
  command->set_brake(control_conf_.soft_estop_brake());
  const double steering = local_view_.chassis().steering_percentage();
  command->set_steering_target(std::isfinite(steering) ? steering : 0.0);
  command->set_steering_rate(0.0);
  command->set_gear_location(local_view_.chassis().gear_location());
  safety_manager_->ApplySafetyPolicy(command);
}

void ControlComponent::ResetAndProduceZeroControlCommand(
    ControlCommand *control_command) {
  control_command->set_throttle(0.0);
  // Follow current steering angle to prevent sudden jerks during handover
  control_command->set_steering_target(latest_chassis_.steering_percentage());
  control_command->set_steering_rate(0.0);
  control_command->set_speed(0.0);
  control_command->set_brake(0.0);
  control_command->set_gear_location(Chassis::GEAR_DRIVE);

  const auto reset = controller_profiles_.Reset();
  if (!reset.ok()) {
    AERROR << "Manual controller reset failed: " << reset.error_message();
    execution_state_fault_ = true;
  }

  // Clear trajectory cache to prevent using stale data upon re-engaging
  latest_trajectory_.mutable_trajectory_point()->Clear();
}

}  // namespace control
}  // namespace apollo
