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
#include "cyber/common/config_loader.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/latency_recorder/latency_recorder.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleStateProvider;
using apollo::cyber::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

ControlComponent::ControlComponent()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

bool ControlComponent::Init() {
  injector_ = std::make_shared<DependencyInjector>();
  init_time_ = Clock::Now();

  AINFO << "Control init, starting ...";

  ACHECK(cyber::common::GetProtoFromFileWithOverride(FLAGS_control_conf_file,
                                                      &control_conf_))
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

void ControlComponent::OnPad(const std::shared_ptr<PadMessage>& pad) {
  std::lock_guard<std::mutex> lock(mutex_);
  pad_msg_.CopyFrom(*pad);

  // Industrial Practice: Process Reset signal immediately via SafetyManager
  // instead of waiting for the next control cycle.
  if (safety_manager_) {
    safety_manager_->TryReset(pad_msg_);
  }
}

void ControlComponent::OnChassis(const std::shared_ptr<Chassis>& chassis) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_chassis_.CopyFrom(*chassis);
}

void ControlComponent::OnPlanning(
    const std::shared_ptr<ADCTrajectory>& trajectory) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_trajectory_.CopyFrom(*trajectory);
}

void ControlComponent::OnLocalization(
    const std::shared_ptr<LocalizationEstimate>& localization) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_localization_.CopyFrom(*localization);
}

Status ControlComponent::ProduceControlCommand(
    ControlCommand* control_command) {
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
      SafetyResult output_res = safety_manager_->PostCheck(*control_command);
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

  // 6. Apply Safety Policy (The Override)
  // This is the final authority. It overrides the command based on the FSM
  // state (Normal, SoftStop, HardEstop).
  safety_manager_->ApplySafetyPolicy(control_command);

  // 6. Housekeeping
  previous_cmd_ = *control_command;

  return status;
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

  // Check driving mode
  if (local_view_.chassis().driving_mode() ==
      apollo::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    status = ProduceControlCommand(&control_command);
  } else {
    // In Manual Mode, reset algorithms and produce neutral command.
    ResetAndProduceZeroControlCommand(&control_command);

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
  if (!control_conf_.is_control_test_mode()) {
    control_cmd_writer_->Write(control_command);
  }

  return true;
}

void ControlComponent::ResetAndProduceZeroControlCommand(
    ControlCommand* control_command) {
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
