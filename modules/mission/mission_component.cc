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
#include "modules/mission/nodes/action/station_wait_node.h"
#include "modules/mission/nodes/condition/check_battery.h"

namespace apollo {
namespace mission {

using apollo::canbus::Chassis;
using apollo::localization::LocalizationEstimate;
using apollo::routing::RoutingRequest;

bool MissionComponent::Init() {
  AINFO << "MissionComponent Init start...";

  if (!GetProtoConfig(&mission_config_)) {
    AERROR << "Unable to load mission conf file: " << ConfigFilePath();
    return false;
  }
  AINFO << "Mission config loaded from: " << ConfigFilePath();

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

  auto routing_writer =
      node_->CreateWriter<RoutingRequest>(FLAGS_routing_request_topic);
  MissionContext::Instance()->SetRoutingWriter(routing_writer);

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

  if (!tree_.rootNode()) return;

  // If the task has already ended (successfully or unsuccessfully), do not tick
  // again and wait for a new task.
  BT::NodeStatus prev_status = tree_.rootNode()->status();
  if (prev_status == BT::NodeStatus::SUCCESS ||
      prev_status == BT::NodeStatus::FAILURE) {
    return;
  }

  // Execute one Tick
  BT::NodeStatus status = tree_.tickOnce();

  if (status == BT::NodeStatus::SUCCESS) {
    AINFO << "Mission [" << MissionContext::Instance()->GetCurrentMissionId()
          << "] Finished Successfully.";
  } else if (status == BT::NodeStatus::FAILURE) {
    AERROR << "Mission [" << MissionContext::Instance()->GetCurrentMissionId()
           << "] Failed.";
  }
}

}  // namespace mission
}  // namespace apollo
