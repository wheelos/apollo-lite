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

#pragma once

#include "behaviortree_cpp/behavior_tree.h"

#include "modules/mission/common/bt_type_converters.h"
#include "modules/mission/common/mission_context.h"

namespace apollo {
namespace mission {

class ParkInNode : public BT::StatefulActionNode {
 public:
  ParkInNode(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("parking_space_id"),
        BT::InputPort<apollo::common::PointENU>("parking_point"),
        BT::InputPort<bool>("parallel_parking"),
        BT::InputPort<double>("goal_heading"),
        BT::InputPort<apollo::common::PointENU>("left_bottom_corner"),
        BT::InputPort<apollo::common::PointENU>("right_bottom_corner"),
        BT::InputPort<bool>("whole_open_space_shell"),
        BT::InputPort<double>("position_tolerance_m"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

 private:
  std::string current_command_id_;
};

}  // namespace mission
}  // namespace apollo
