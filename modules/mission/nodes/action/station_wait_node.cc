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

#include "modules/mission/nodes/action/station_wait_node.h"

#include "cyber/common/log.h"

namespace apollo {
namespace mission {

BT::NodeStatus StationWaitNode::onStart() {
  double sec = 0.0;
  if (!getInput("seconds", sec)) {
    AERROR << "StationWaitNode: Missing seconds input";
    return BT::NodeStatus::FAILURE;
  }

  deadline_ = std::chrono::steady_clock::now() +
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>(sec));

  AINFO << "StationWaitNode: Waiting for " << sec << " seconds";
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StationWaitNode::onRunning() {
  if (std::chrono::steady_clock::now() >= deadline_) {
    AINFO << "StationWaitNode: Finished waiting";
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void StationWaitNode::onHalted() {}

}  // namespace mission
}  // namespace apollo
