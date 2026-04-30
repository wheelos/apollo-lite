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

#include "modules/mission/nodes/action/park_in_node.h"

#include <sstream>

#include "cyber/common/log.h"
#include "cyber/time/time.h"

namespace apollo {
namespace mission {

namespace {

std::string BuildParkInCommandId(const std::string& parking_space_id) {
  const std::string mission_id =
      MissionContext::Instance()->GetCurrentMissionId();
  std::ostringstream oss;
  if (!mission_id.empty()) {
    oss << mission_id << "/";
  }
  oss << "park_in/" << parking_space_id << "/"
      << apollo::cyber::Time::Now().ToNanosecond();
  return oss.str();
}

bool IsParkInCommandDone(
    const std::shared_ptr<apollo::planning::PlanningRuntimeStatus>& status,
    const std::string& command_id) {
  if (status == nullptr || !status->has_command_id() ||
      status->command_id() != command_id) {
    return false;
  }
  if (status->has_completion() &&
      status->completion().has_command_completed() &&
      status->completion().command_completed()) {
    return true;
  }
  return status->has_state() &&
         status->state() == apollo::planning::RUNTIME_COMPLETED;
}

}  // namespace

BT::NodeStatus ParkInNode::onStart() {
  std::string parking_space_id;
  if (!getInput("parking_space_id", parking_space_id) ||
      parking_space_id.empty()) {
    throw BT::RuntimeError("missing required input [parking_space_id]");
  }

  planning::PlanningCommand command;
  current_command_id_ = BuildParkInCommandId(parking_space_id);
  command.set_command_id(current_command_id_);
  command.set_action(planning::COMMAND_ACTIVATE);
  command.set_requested_scene(planning::SCENE_PARK_IN);
  command.set_preemptible(false);

  bool whole_open_space_shell = false;
  if (getInput("whole_open_space_shell", whole_open_space_shell) &&
      whole_open_space_shell) {
    command.mutable_open_space()->set_navigation_type(
        planning::OPEN_SPACE_NAV_GLOBAL_GUIDED);
  }

  auto* parking_goal = command.mutable_goal()->mutable_parking_goal();
  parking_goal->set_parking_space_id(parking_space_id);

  common::PointENU parking_point;
  if (getInput("parking_point", parking_point)) {
    *parking_goal->mutable_parking_point() = parking_point;
  }

  bool parallel_parking = false;
  if (getInput("parallel_parking", parallel_parking) && parallel_parking) {
    parking_goal->set_parking_space_type(routing::PARALLEL_PARKING);
  } else {
    parking_goal->set_parking_space_type(routing::VERTICAL_PLOT);
  }

  double goal_heading = 0.0;
  if (getInput("goal_heading", goal_heading)) {
    parking_goal->set_heading(goal_heading);
    command.mutable_goal()->set_goal_heading(goal_heading);
  }

  common::PointENU left_bottom_corner;
  common::PointENU right_bottom_corner;
  if (getInput("left_bottom_corner", left_bottom_corner) &&
      getInput("right_bottom_corner", right_bottom_corner)) {
    auto* corner_polygon = parking_goal->mutable_corner_point();
    *corner_polygon->add_point() = left_bottom_corner;
    *corner_polygon->add_point() = right_bottom_corner;
  }

  MissionContext::Instance()->SendPlanningCommand(command);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ParkInNode::onRunning() {
  std::string parking_space_id;
  if (!getInput("parking_space_id", parking_space_id) ||
      parking_space_id.empty()) {
    AERROR << "ParkInNode: Missing required input [parking_space_id]";
    return BT::NodeStatus::FAILURE;
  }
  if (current_command_id_.empty()) {
    AERROR << "ParkInNode: Missing active command_id while running";
    return BT::NodeStatus::FAILURE;
  }

  const auto command_status =
      MissionContext::Instance()->GetCommandLifecycleStatus(
          current_command_id_);
  if (command_status.state == CommandLifecycleState::kFailed ||
      command_status.state == CommandLifecycleState::kCancelled) {
    AERROR << "ParkInNode: Command lifecycle failed: " << command_status.reason;
    return BT::NodeStatus::FAILURE;
  }
  if (command_status.state == CommandLifecycleState::kCompleted) {
    return BT::NodeStatus::SUCCESS;
  }

  auto runtime_status = MissionContext::Instance()->GetPlanningRuntimeStatus();
  if (IsParkInCommandDone(runtime_status, current_command_id_)) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ParkInNode::onHalted() {
  std::string parking_space_id;
  if (!getInput("parking_space_id", parking_space_id) ||
      parking_space_id.empty()) {
    AERROR << "ParkInNode: Missing required input [parking_space_id] on halt";
    return;
  }
  if (current_command_id_.empty()) {
    AERROR << "ParkInNode: Missing active command_id on halt";
    return;
  }

  planning::PlanningCommand command;
  command.set_command_id(current_command_id_);
  command.set_action(planning::COMMAND_CANCEL);
  command.set_requested_scene(planning::SCENE_PARK_IN);
  MissionContext::Instance()->SendPlanningCommand(command);
  current_command_id_.clear();
}

}  // namespace mission
}  // namespace apollo
