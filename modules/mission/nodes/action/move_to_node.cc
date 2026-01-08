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

#include "cyber/common/log.h"

namespace apollo {
namespace mission {

BT::NodeStatus MoveToNode::onStart() {
  apollo::common::PointENU target_pose;

  // Retrieve data from the port, If the XML is written as <MoveTo
  // goal="{Station_A}" />, the value will be automatically retrieved from the
  // blackboard.
  if (!getInput<apollo::common::PointENU>("goal", target_pose)) {
    throw BT::RuntimeError("missing required input [goal]");
  }

  MissionContext::Instance()->SendRoutingRequest(target_pose);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToNode::onRunning() {
  auto loc = MissionContext::Instance()->GetLocalization();
  if (!loc) {
    return BT::NodeStatus::RUNNING;
  }

  apollo::common::PointENU target_pose;
  if (!getInput("goal", target_pose)) {
    AERROR << "MoveToNode: Failed to get 'goal' from input port";
    return BT::NodeStatus::FAILURE;
  }

  double dx = loc->pose().position().x() - target_pose.x();
  double dy = loc->pose().position().y() - target_pose.y();
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist < 3.0) {
    AINFO << "MoveToNode: Arrived";
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void MoveToNode::onHalted() {
  AINFO << "MoveToNode: Halted (Task Cancelled)";
  // Clear Routing command here
}

}  // namespace mission
}  // namespace apollo
