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

#include "modules/mission/nodes/action/move_to_node.h"

#include <sstream>

#include "cyber/common/log.h"
#include "cyber/time/time.h"

namespace apollo {
namespace mission {

namespace {

std::string BuildMoveToCommandId(const std::string& node_name) {
  const std::string mission_id =
      MissionContext::Instance()->GetCurrentMissionId();
  std::ostringstream oss;
  if (!mission_id.empty()) {
    oss << mission_id << "/";
  }
  oss << "move_to/" << node_name << "/"
      << apollo::cyber::Time::Now().ToNanosecond();
  return oss.str();
}

}  // namespace

BT::NodeStatus MoveToNode::onStart() {
  apollo::common::PointENU target_pose;

  // Retrieve data from the port, If the XML is written as <MoveTo
  // goal="{Station_A}" />, the value will be automatically retrieved from the
  // blackboard.
  if (!getInput<apollo::common::PointENU>("goal", target_pose)) {
    throw BT::RuntimeError("missing required input [goal]");
  }

  planning::PlanningCommand command;
  current_command_id_ = BuildMoveToCommandId(name());
  command.set_command_id(current_command_id_);
  command.set_action(planning::COMMAND_ACTIVATE);
  command.set_requested_scene(planning::SCENE_LANE_CRUISE);
  command.set_preferred_mode(planning::MODE_LANE_GRAPH);
  command.set_preemptible(true);
  *command.mutable_goal()->mutable_goal_pose() = target_pose;
  MissionContext::Instance()->SendPlanningCommand(command);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToNode::onRunning() {
  if (current_command_id_.empty()) {
    AERROR << "MoveToNode: Missing active command_id while running";
    return BT::NodeStatus::FAILURE;
  }
  const auto command_status =
      MissionContext::Instance()->GetCommandLifecycleStatus(
          current_command_id_);
  if (command_status.state == CommandLifecycleState::kCompleted) {
    AINFO << "MoveToNode: Completed by planning runtime status";
    return BT::NodeStatus::SUCCESS;
  }

  if (command_status.state == CommandLifecycleState::kFailed ||
      command_status.state == CommandLifecycleState::kCancelled) {
    AERROR << "MoveToNode: Command lifecycle failed: " << command_status.reason;
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void MoveToNode::onHalted() {
  AINFO << "MoveToNode: Halted (Task Cancelled)";
  if (current_command_id_.empty()) {
    AERROR << "MoveToNode: Missing active command_id on halt";
    return;
  }
  planning::PlanningCommand command;
  command.set_command_id(current_command_id_);
  command.set_action(planning::COMMAND_CANCEL);
  command.set_requested_scene(planning::SCENE_LANE_CRUISE);
  command.set_preferred_mode(planning::MODE_LANE_GRAPH);
  MissionContext::Instance()->SendPlanningCommand(command);
  current_command_id_.clear();
}

}  // namespace mission
}  // namespace apollo
