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

#include <cmath>

#include "cyber/common/log.h"

namespace apollo {
namespace mission {

namespace {

std::string BuildParkInCommandId(const std::string& parking_space_id) {
  const std::string mission_id = MissionContext::Instance()->GetCurrentMissionId();
  if (mission_id.empty()) {
    return "park_in/" + parking_space_id;
  }
  return mission_id + "/park_in/" + parking_space_id;
}

double GetPositionTolerance(const ParkInNode* node) {
  double tolerance = 1.0;
  node->getInput("position_tolerance_m", tolerance);
  return tolerance;
}

bool IsParkInCommandDone(const std::shared_ptr<apollo::planning::PlanningRuntimeStatus>& status,
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

bool IsParkInCommandFailed(
    const std::shared_ptr<apollo::planning::PlanningRuntimeStatus>& status,
    const std::string& command_id) {
  if (status == nullptr || !status->has_command_id() ||
      status->command_id() != command_id || !status->has_state()) {
    return false;
  }
  return status->state() == apollo::planning::RUNTIME_REJECTED ||
         status->state() == apollo::planning::RUNTIME_CANCELLED ||
         status->state() == apollo::planning::RUNTIME_FAILED;
}

}  // namespace

BT::NodeStatus ParkInNode::onStart() {
  std::string parking_space_id;
  if (!getInput("parking_space_id", parking_space_id) ||
      parking_space_id.empty()) {
    throw BT::RuntimeError("missing required input [parking_space_id]");
  }

  planning::PlanningCommand command;
  command.set_command_id(BuildParkInCommandId(parking_space_id));
  command.set_action(planning::COMMAND_ACTIVATE);
  command.set_requested_scene(planning::SCENE_PARK_IN);
  command.set_preferred_mode(planning::MODE_OPEN_SPACE);
  command.set_preemptible(false);

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

  const std::string command_id = BuildParkInCommandId(parking_space_id);
  auto runtime_status = MissionContext::Instance()->GetPlanningRuntimeStatus();
  if (IsParkInCommandFailed(runtime_status, command_id)) {
    AERROR << "ParkInNode: Planning rejected or failed park-in command";
    return BT::NodeStatus::FAILURE;
  }
  if (IsParkInCommandDone(runtime_status, command_id)) {
    return BT::NodeStatus::SUCCESS;
  }

  common::PointENU parking_point;
  if (!getInput("parking_point", parking_point)) {
    return BT::NodeStatus::RUNNING;
  }

  auto loc = MissionContext::Instance()->GetLocalization();
  if (!loc) {
    return BT::NodeStatus::RUNNING;
  }

  const double dx = loc->pose().position().x() - parking_point.x();
  const double dy = loc->pose().position().y() - parking_point.y();
  const double distance = std::sqrt(dx * dx + dy * dy);

  auto chassis = MissionContext::Instance()->GetChassis();
  const double speed =
      chassis != nullptr ? std::abs(chassis->speed_mps()) : 0.0;

  if (distance <= GetPositionTolerance(this) && speed < 0.2) {
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

  planning::PlanningCommand command;
  command.set_command_id(BuildParkInCommandId(parking_space_id));
  command.set_action(planning::COMMAND_CANCEL);
  command.set_requested_scene(planning::SCENE_PARK_IN);
  command.set_preferred_mode(planning::MODE_OPEN_SPACE);
  MissionContext::Instance()->SendPlanningCommand(command);
}

}  // namespace mission
}  // namespace apollo
