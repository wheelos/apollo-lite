// Copyright 2025 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <string>

#include "behaviortree_cpp/action_node.h"

#include "modules/mission/common/mission_context.h"

namespace apollo {
namespace mission {

class SendPadNode : public BT::SyncActionNode {
 public:
  SendPadNode(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("action")};
  }

  BT::NodeStatus tick() override;
};

}  // namespace mission
}  // namespace apollo
