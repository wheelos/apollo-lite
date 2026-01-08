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

#include "modules/mission/nodes/action/charge_node.h"

#include "cyber/common/log.h"

namespace apollo {
namespace mission {

BT::NodeStatus ChargeNode::onStart() {
  double target_soc = 0.0;
  if (!getInput("target_soc", target_soc)) {
    AERROR << "ChargeNode: Missing required input [target_soc]";
    return BT::NodeStatus::FAILURE;
  }

  // TODO(zero): Canbus/BMS Start Charging
  // MissionContext::Instance()->SendChargeCommand(true);

  AINFO << "ChargeNode: Start charging, target SOC: " << target_soc << "%";
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ChargeNode::onRunning() {
  double target_soc = 0.0;
  getInput("target_soc", target_soc);

  // Get the latest chassis data from the Context
  auto chassis = MissionContext::Instance()->GetChassis();
  if (!chassis) {
    AWARN << "ChargeNode: Waiting for chassis data...";
    return BT::NodeStatus::RUNNING;
  }

  double current_soc = chassis->battery_soc_percentage();

  // Check if the target battery level has been reached.
  if (current_soc >= target_soc) {
    AINFO << "ChargeNode: Charging complete. Current SOC: " << current_soc
          << "%";
    // TODO(zero): Stop charging command
    // MissionContext::Instance()->SendChargeCommand(false);
    return BT::NodeStatus::SUCCESS;
  }

  ADEBUG << "Charging... " << current_soc << "/" << target_soc;

  return BT::NodeStatus::RUNNING;
}

void ChargeNode::onHalted() {
  AINFO << "ChargeNode: Halted (Stop Charging).";
  // TODO(zero): Send a stop charging command
  // MissionContext::Instance()->SendChargeCommand(false);
}

}  // namespace mission
}  // namespace apollo
