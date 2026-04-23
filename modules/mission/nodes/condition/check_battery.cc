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

#include "modules/mission/nodes/condition/check_battery.h"

namespace apollo {
namespace mission {

BT::NodeStatus CheckBatteryNode::tick() {
  double min_soc = 0.0;
  if (!getInput("min_percentage", min_soc)) {
    return BT::NodeStatus::FAILURE;
  }

  auto chassis = MissionContext::Instance()->GetChassis();
  if (!chassis) {
    return BT::NodeStatus::FAILURE;
  }

  if (chassis->battery_soc_percentage() < min_soc) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mission
}  // namespace apollo
