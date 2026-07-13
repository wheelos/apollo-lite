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

#include "absl/strings/str_cat.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/latency_recorder/latency_recorder.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/common/terminal_control_helper.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
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
  if (!controller_agent_.Init(injector_, &control_conf_).ok()) {
    monitor_logger_buffer_.ERROR("Control init controller agent failed!");
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

void ControlComponent::InitReaders() {
  cyber::ReaderConfig chassis_cfg;
  chassis_cfg.channel_name = FLAGS_chassis_topic;
  chassis_cfg.pending_queue_size = FLAGS_chassis_pending_queue_size;
  chassis_reader_ = node_->CreateReader<Chassis>(chassis_cfg, nullptr);

  cyber::ReaderConfig planning_cfg;
  planning_cfg.channel_name = FLAGS_planning_trajectory_topic;
  planning_cfg.pending_queue_size = FLAGS_planning_pending_queue_size;
  trajectory_reader_ =
      node_->CreateReader<ADCTrajectory>(planning_cfg, nullptr);

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

void ControlComponent::OnPlanning(
    const std::shared_ptr<ADCTrajectory> &trajectory) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_trajectory_.CopyFrom(*trajectory);
}

void ControlComponent::OnLocalization(
    const std::shared_ptr<LocalizationEstimate> &localization) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_localization_.CopyFrom(*localization);
}

Status ControlComponent::ProduceControlCommand(ControlCommand *control_command,
                                               bool *used_previous_command) {
  last_goal_ = BuildControlCommandGoal(local_view_.trajectory());
  EnsureControlIntentCompatibility(last_goal_, local_view_.mutable_trajectory());
  last_profile_ = strategy_orchestrator_.Resolve(last_goal_);
  strategy_orchestrator_.Apply(last_profile_, local_view_.mutable_trajectory());

  // 1. Update Vehicle State Estimation
  // This is a prerequisite for control computation.
  injector_->vehicle_state()->Update(local_view_.localization(),
                                     local_view_.chassis());

  // 2. Safety Pre-Check (Input Validation)
  // Checks timestamps, sensor health, and trajectory integrity.
  SafetyResult input_res = safety_manager_->PreCheck(local_view_);

  Status status = Status::OK();
  bool use_previous_cmd = false;

  if (!input_res.must_bypass) {
    // 3. Core Control Computation
    // Only run algorithms if system is relatively healthy.
    status = controller_agent_.ComputeControlCommand(
        &local_view_.localization(), &local_view_.chassis(),
        &local_view_.trajectory(), control_command);

    if (status.ok()) {
      // 4. Safety Post-Check (Output Validation)
      // Sanity check on computed commands (e.g., jerk, steering rate).
      SafetyResult output_res =
          safety_manager_->PostCheck(*control_command, previous_cmd_);
      if (output_res.need_freeze) {
        use_previous_cmd = true;
        status = Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                        "Output Limits Violated (Freeze)");
      }
    } else {
      // Logic failure (e.g., solver error) is treated as an internal fault.
      status = Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                      "Controller Agent Compute failed");
      use_previous_cmd = true;
    }
  } else {
    use_previous_cmd = true;
    status = Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                    "Input Physics Missing (Bypass)");
  }

  // 5. Freeze Strategy
  if (use_previous_cmd) {
    *control_command = previous_cmd_;
    controller_agent_.Reset();
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
        runtime_context.control_intent.primitive_type() !=
        apollo::planning::CONTROL_PRIMITIVE_NONE);
    runtime_status.set_trajectory_optional(
        IsTrajectorylessControlPrimitive(local_view_.trajectory()));
    if (runtime_context.control_intent.has_stop_reason_code()) {
      runtime_status.set_stop_reason_code(
          runtime_context.control_intent.stop_reason_code());
    }
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

  control_runtime_status_writer_->Write(runtime_status);
}

bool ControlComponent::Proc() {
  const auto start_time = Clock::Now();

  // 1. Data Observation (Lock-Free Read)
  chassis_reader_->Observe();
  trajectory_reader_->Observe();
  localization_reader_->Observe();
  pad_msg_reader_->Observe();

  auto chassis_msg = chassis_reader_->GetLatestObserved();
  if (chassis_msg == nullptr) {
    AERROR_EVERY(100) << "Chassis msg is not ready!";
    return false;
  }
  OnChassis(chassis_msg);

  auto trajectory_msg = trajectory_reader_->GetLatestObserved();
  if (trajectory_msg) {
    if (latest_trajectory_.header().sequence_num() !=
        trajectory_msg->header().sequence_num()) {
      OnPlanning(trajectory_msg);
    }
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

  // Check driving mode
  if (local_view_.chassis().driving_mode() ==
      apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    status = ProduceControlCommand(&control_command, &used_previous_command);
  } else {
    // In Manual Mode, reset algorithms and produce neutral command.
    ResetAndProduceZeroControlCommand(&control_command);
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
  if (!control_conf_.is_control_test_mode()) {
    control_cmd_writer_->Write(control_command);
  }

  return true;
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

  controller_agent_.Reset();

  // Clear trajectory cache to prevent using stale data upon re-engaging
  latest_trajectory_.mutable_trajectory_point()->Clear();
  trajectory_reader_->ClearData();
}

}  // namespace control
}  // namespace apollo
