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

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_runtime_status.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/mission_msgs/mission_request.pb.h"
#include "modules/common_msgs/mission_msgs/mission_runtime_status.pb.h"
#include "modules/common_msgs/planning_msgs/planning_runtime_status.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/mission/proto/mission_config.pb.h"

#include "cyber/component/component.h"
#include "cyber/cyber.h"

namespace apollo {
namespace mission {

class MissionComponent : public ::apollo::cyber::Component<MissionRequest> {
 public:
  bool Init() override;

  bool Proc(const std::shared_ptr<MissionRequest>& request) override;

 private:
  bool RegisterBehaviorNodes();
  bool InitCyberCommunication();

  void OnTimer();
  void PublishMissionRuntimeStatus(BT::NodeStatus tree_status);

 private:
  MissionConfig mission_config_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;

  std::shared_ptr<apollo::cyber::Reader<apollo::canbus::Chassis>>
      chassis_reader_;
  std::shared_ptr<
      apollo::cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<
      apollo::cyber::Reader<apollo::planning::PlanningRuntimeStatus>>
      planning_runtime_status_reader_;
  std::shared_ptr<apollo::cyber::Reader<apollo::control::ControlRuntimeStatus>>
      control_runtime_status_reader_;
  std::shared_ptr<apollo::cyber::Writer<apollo::mission::MissionRuntimeStatus>>
      mission_runtime_status_writer_;

  std::unique_ptr<cyber::Timer> tick_timer_;
  std::mutex mutex_;
};

CYBER_REGISTER_COMPONENT(MissionComponent)

}  // namespace mission
}  // namespace apollo
