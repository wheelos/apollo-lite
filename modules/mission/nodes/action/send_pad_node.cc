// Copyright 2025 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "modules/mission/nodes/action/send_pad_node.h"

#include "cyber/common/log.h"

namespace apollo {
namespace mission {

BT::NodeStatus SendPadNode::tick() {
  std::string action;
  if (!getInput("action", action)) {
    AERROR << "SendPadNode: Missing required input [action]";
    return BT::NodeStatus::FAILURE;
  }

  apollo::planning::PadMessage::DrivingAction pad_action;
  if (action == "PULL_OVER") {
    pad_action = apollo::planning::PadMessage::PULL_OVER;
  } else if (action == "STOP") {
    pad_action = apollo::planning::PadMessage::STOP;
  } else {
    AERROR << "SendPadNode: Unsupported action: " << action;
    return BT::NodeStatus::FAILURE;
  }

  if (!MissionContext::Instance()->SendPlanningPad(pad_action)) {
    AERROR << "SendPadNode: Failed to publish " << action
           << " to /apollo/planning/pad";
    return BT::NodeStatus::FAILURE;
  }

  AINFO << "SendPadNode: Published " << action << " to /apollo/planning/pad";
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mission
}  // namespace apollo
