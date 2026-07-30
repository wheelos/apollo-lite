// Copyright 2025 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-12-13
//  Author: daohu527

#include "modules/mission/mission_component.h"

#include <algorithm>
#include <filesystem>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/mission/common/mission_context.h"
#include "modules/mission/nodes/action/charge_node.h"
#include "modules/mission/nodes/action/move_to_node.h"
#include "modules/mission/nodes/action/park_in_node.h"
#include "modules/mission/nodes/action/station_wait_node.h"
#include "modules/mission/nodes/condition/check_battery.h"

namespace apollo {
namespace mission {

using apollo::canbus::Chassis;
using apollo::control::ControlRuntimeStatus;
using apollo::localization::LocalizationEstimate;
using apollo::mission::MissionRuntimeStatus;
using apollo::planning::MissionDirective;
using apollo::planning::PlanningRuntimeStatus;

namespace {

MissionCommandState ToProtoCommandState(CommandLifecycleState state) {
  switch (state) {
    case CommandLifecycleState::kDispatched:
      return MISSION_COMMAND_DISPATCHED;
    case CommandLifecycleState::kAccepted:
      return MISSION_COMMAND_ACCEPTED;
    case CommandLifecycleState::kRunning:
      return MISSION_COMMAND_RUNNING;
    case CommandLifecycleState::kHolding:
      return MISSION_COMMAND_HOLDING;
    case CommandLifecycleState::kQueued:
      return MISSION_COMMAND_QUEUED;
    case CommandLifecycleState::kCancelling:
      return MISSION_COMMAND_CANCELLING;
    case CommandLifecycleState::kCompleted:
      return MISSION_COMMAND_COMPLETED;
    case CommandLifecycleState::kFailed:
      return MISSION_COMMAND_FAILED;
    case CommandLifecycleState::kCancelled:
      return MISSION_COMMAND_CANCELLED;
    case CommandLifecycleState::kUnknown:
    default:
      return MISSION_COMMAND_UNKNOWN;
  }
}

MissionRecoveryAction ToProtoRecoveryAction(RecoveryAction action) {
  switch (action) {
    case RecoveryAction::kAcknowledge:
      return MISSION_RECOVERY_ACTION_ACKNOWLEDGE;
    case RecoveryAction::kResume:
      return MISSION_RECOVERY_ACTION_RESUME;
    case RecoveryAction::kRetry:
      return MISSION_RECOVERY_ACTION_RETRY;
    case RecoveryAction::kAbort:
      return MISSION_RECOVERY_ACTION_ABORT;
    case RecoveryAction::kEscalateMrm:
      return MISSION_RECOVERY_ACTION_ESCALATE_MRM;
    case RecoveryAction::kNone:
    default:
      return MISSION_RECOVERY_ACTION_NONE;
  }
}

}  // namespace

bool MissionComponent::Init() {
  AINFO << "MissionComponent Init start...";

  if (!GetProtoConfig(&mission_config_)) {
    AERROR << "Unable to load mission conf file: " << ConfigFilePath();
    return false;
  }
  AINFO << "Mission config loaded from: " << ConfigFilePath();
  MissionContext::Instance()->SetProducerEpoch(
      "mission-" +
      std::to_string(cyber::Time::Now().ToNanosecond()));

  // 2. Modular Initialization
  if (!RegisterBehaviorNodes()) {
    AERROR << "Failed to register BT nodes.";
    return false;
  }

  if (!InitCyberCommunication()) {
    AERROR << "Failed to init cyber communication.";
    return false;
  }

  // 3. Start the timer
  int interval = mission_config_.tick_interval_ms();
  tick_timer_.reset(new cyber::Timer(
      interval, [this]() { this->OnTimer(); }, false));
  return true;
}

bool MissionComponent::RegisterBehaviorNodes() {
  // 1. Register C++ node type
  try {
    factory_.registerNodeType<MoveToNode>("MoveTo");
    factory_.registerNodeType<ParkInNode>("ParkIn");
    factory_.registerNodeType<StationWaitNode>("StationWait");
    factory_.registerNodeType<CheckBatteryNode>("CheckBattery");
    factory_.registerNodeType<ChargeNode>("Charge");
  } catch (const std::exception& e) {
    AERROR << "Exception registering native nodes: " << e.what();
    return false;
  }

  std::string bt_dir = mission_config_.bt_tree_dir();
  if (bt_dir.empty()) {
    AERROR << "bt_tree_dir is empty in config!";
    return false;
  }

  std::error_code ec;
  if (!std::filesystem::exists(bt_dir, ec)) {
    AERROR << "BT tree directory does not exist: " << bt_dir;
    return false;
  }

  int loaded_count = 0;

  // 2. Load XML by traversing the directory
  for (const auto& entry : std::filesystem::directory_iterator(bt_dir)) {
    if (entry.is_regular_file() && entry.path().extension() == ".xml") {
      std::string file_path = entry.path().string();

      try {
        factory_.registerBehaviorTreeFromFile(file_path);
        AINFO << "Successfully cached BT XML: " << entry.path().filename();
        loaded_count++;
      } catch (const std::exception& e) {
        AERROR << "Failed to load XML [" << entry.path().filename()
               << "]: " << e.what();
      }
    }
  }

  if (loaded_count == 0) {
    AERROR << "No valid BT XML files were loaded from " << bt_dir;
    return false;
  }

  return true;
}

bool MissionComponent::InitCyberCommunication() {
  // 2.1 Chassis Reader
  chassis_reader_ = node_->CreateReader<Chassis>(
      FLAGS_chassis_topic, [](const std::shared_ptr<Chassis>& msg) {
        MissionContext::Instance()->UpdateChassis(msg);
      });

  // 2.2 Locating the Reader
  localization_reader_ = node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic,
      [](const std::shared_ptr<LocalizationEstimate>& msg) {
        MissionContext::Instance()->UpdateLocalization(msg);
      });

  auto mission_directive_writer = node_->CreateWriter<MissionDirective>(
      mission_config_.mission_directive_topic());
  MissionContext::Instance()->SetMissionDirectiveWriter(
      mission_directive_writer);

  planning_runtime_status_reader_ = node_->CreateReader<PlanningRuntimeStatus>(
      mission_config_.planning_runtime_status_topic(),
      [](const std::shared_ptr<PlanningRuntimeStatus>& msg) {
        MissionContext::Instance()->UpdatePlanningRuntimeStatus(msg);
      });

  control_runtime_status_reader_ = node_->CreateReader<ControlRuntimeStatus>(
      mission_config_.control_runtime_status_topic(),
      [](const std::shared_ptr<ControlRuntimeStatus>& msg) {
        MissionContext::Instance()->UpdateControlRuntimeStatus(msg);
      });

  mission_runtime_status_writer_ = node_->CreateWriter<MissionRuntimeStatus>(
      mission_config_.mission_runtime_status_topic());

  return true;
}

bool MissionComponent::Proc(const std::shared_ptr<MissionRequest>& request) {
  std::lock_guard<std::mutex> lock(mutex_);

  std::string incoming_id = request->mission_id();
  if (incoming_id.empty()) {
    AERROR << "Received request with empty mission_id";
    return false;
  }

  // 2. Check for duplicate requests
  std::string current_id = MissionContext::Instance()->GetCurrentMissionId();
  bool is_running = false;
  if (tree_.rootNode()) {
    is_running = (tree_.rootNode()->status() == BT::NodeStatus::RUNNING);
  }

  if (incoming_id == current_id && is_running) {
    return true;
  }

  AINFO << "New Mission Request. ID: " << incoming_id
        << ", Task: " << request->task_name();

  // 3. Stop the previous tree (if any).
  if (tree_.rootNode()) {
    tree_.haltTree();
    AINFO << "Previous tree halted.";
  }

  // Update the current Mission ID (for deduplication).
  MissionContext::Instance()->SetCurrentMissionId(incoming_id);
  MissionContext::Instance()->SetCurrentTaskName(request->task_name());

  // 4. Create a brand new Blackboard
  auto blackboard = BT::Blackboard::create();

  // 4.1 Injection Parameters
  for (const auto& param : request->parameters()) {
    if (param.has_string_value()) {
      blackboard->set(param.key(), param.string_value());
    } else if (param.has_double_value()) {
      blackboard->set(param.key(), param.double_value());
    } else if (param.has_int_value()) {
      blackboard->set(param.key(), param.int_value());
    } else if (param.has_bool_value()) {
      blackboard->set(param.key(), param.bool_value());
    }
  }

  // 4.2 Injecting Waypoints
  // In XML, you can use {Station_A} to get the PointENU object.
  for (const auto& wp : request->waypoints()) {
    // Save PointENU directly to the blackboard.
    blackboard->set(wp.name(), wp.pose());
    ADEBUG << "Injecting Waypoint: " << wp.name();
  }

  const std::string& task_name = request->task_name();

  // Check if the tree has been loaded
  const auto& known_trees = factory_.registeredBehaviorTrees();
  if (std::find(known_trees.begin(), known_trees.end(), task_name) ==
      known_trees.end()) {
    AERROR << "Requested task name not found in loaded XMLs: " << task_name;
    return false;
  }

  try {
    tree_ = factory_.createTree(task_name, blackboard);
    AINFO << "Instantiated tree: " << task_name;
  } catch (const std::exception& e) {
    AERROR << "Failed to create tree: " << e.what();
    return false;
  }

  return true;
}

void MissionComponent::OnTimer() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!tree_.rootNode()) {
    PublishMissionRuntimeStatus(BT::NodeStatus::IDLE);
    return;
  }

  // If the task has already ended (successfully or unsuccessfully), do not tick
  // again and wait for a new task.
  BT::NodeStatus prev_status = tree_.rootNode()->status();
  if (prev_status == BT::NodeStatus::SUCCESS ||
      prev_status == BT::NodeStatus::FAILURE) {
    PublishMissionRuntimeStatus(prev_status);
    return;
  }

  // Execute one Tick
  BT::NodeStatus status = tree_.tickOnce();
  PublishMissionRuntimeStatus(status);

  if (status == BT::NodeStatus::SUCCESS) {
    AINFO << "Mission [" << MissionContext::Instance()->GetCurrentMissionId()
          << "] Finished Successfully.";
  } else if (status == BT::NodeStatus::FAILURE) {
    AERROR << "Mission [" << MissionContext::Instance()->GetCurrentMissionId()
           << "] Failed.";
  }
}

void MissionComponent::PublishMissionRuntimeStatus(BT::NodeStatus tree_status) {
  if (mission_runtime_status_writer_ == nullptr) {
    return;
  }

  const auto snapshot = MissionContext::Instance()->GetMissionCommandSnapshot();
  MissionRuntimeStatus runtime_status;
  common::util::FillHeader(node_->Name(), &runtime_status);
  if (!snapshot.mission_id.empty()) {
    runtime_status.set_mission_id(snapshot.mission_id);
  }
  if (!snapshot.task_name.empty()) {
    runtime_status.set_task_name(snapshot.task_name);
  }
  if (!snapshot.active_command_id.empty()) {
    runtime_status.set_active_command_id(snapshot.active_command_id);
  }
  runtime_status.set_active_command_state(
      ToProtoCommandState(snapshot.active_command_status.state));
  runtime_status.set_operator_recovery_required(
      snapshot.operator_recovery_required);
  runtime_status.set_queued_command_count(snapshot.queued_command_ids.size());
  for (const auto& queued_command_id : snapshot.queued_command_ids) {
    runtime_status.add_queued_command_id(queued_command_id);
  }
  if (!snapshot.last_terminal_command_status.command_id.empty()) {
    runtime_status.set_last_terminal_command_id(
        snapshot.last_terminal_command_status.command_id);
    runtime_status.set_last_terminal_command_state(
        ToProtoCommandState(snapshot.last_terminal_command_status.state));
  }
  if (!snapshot.last_terminal_command_status.reason.empty()) {
    runtime_status.set_last_terminal_reason(
        snapshot.last_terminal_command_status.reason);
  }
  if (snapshot.active_command_status.dispatch_count > 0) {
    runtime_status.set_active_dispatch_count(
        snapshot.active_command_status.dispatch_count);
  }
  runtime_status.set_operator_acknowledged(
      snapshot.recovery_state.operator_acknowledged);
  if (!snapshot.recovery_state.reason.empty()) {
    runtime_status.set_recovery_reason(snapshot.recovery_state.reason);
  }
  runtime_status.set_retry_count(snapshot.recovery_state.retry_count);
  runtime_status.set_retry_budget_remaining(
      snapshot.recovery_state.retry_budget_remaining);
  if (snapshot.recovery_state.deadline_sec > 0.0) {
    runtime_status.set_recovery_deadline_sec(
        snapshot.recovery_state.deadline_sec);
  }
  runtime_status.set_recovery_timeout_expired(
      snapshot.recovery_state.timeout_expired);
  for (const auto action : snapshot.recovery_state.allowed_actions) {
    runtime_status.add_allowed_recovery_action(ToProtoRecoveryAction(action));
  }
  runtime_status.set_recommended_recovery_action(
      ToProtoRecoveryAction(snapshot.recovery_state.recommended_action));

  if (!snapshot.active_command_status.reason.empty()) {
    runtime_status.set_reason(snapshot.active_command_status.reason);
  } else if (!snapshot.last_terminal_command_status.reason.empty()) {
    runtime_status.set_reason(snapshot.last_terminal_command_status.reason);
  }

  if (snapshot.operator_recovery_required) {
    runtime_status.set_state(MISSION_RUNTIME_RECOVERY_REQUIRED);
  } else if (tree_status == BT::NodeStatus::FAILURE ||
             snapshot.active_command_status.state ==
                 CommandLifecycleState::kFailed) {
    runtime_status.set_state(MISSION_RUNTIME_FAILED);
  } else if (tree_status == BT::NodeStatus::SUCCESS &&
             snapshot.active_command_id.empty() &&
             snapshot.queued_command_ids.empty()) {
    runtime_status.set_state(MISSION_RUNTIME_COMPLETED);
  } else if (tree_status == BT::NodeStatus::RUNNING) {
    runtime_status.set_state(MISSION_RUNTIME_RUNNING);
  } else if (!snapshot.active_command_id.empty()) {
    runtime_status.set_state(MISSION_RUNTIME_RUNNING);
  } else if (!snapshot.queued_command_ids.empty()) {
    runtime_status.set_state(MISSION_RUNTIME_WAITING_COMMAND);
  } else {
    runtime_status.set_state(MISSION_RUNTIME_IDLE);
  }

  mission_runtime_status_writer_->Write(runtime_status);
}

}  // namespace mission
}  // namespace apollo
