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

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/cyber.h"
#include "modules/common/util/message_util.h"
#include "modules/common_msgs/basic_msgs/geometry.pb.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_runtime_status.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "modules/common_msgs/planning_msgs/planning_runtime_status.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/mission/common/mission_command_supervisor.h"

namespace apollo {
namespace mission {

class MissionContext {
 public:
  void SetRoutingWriter(
      const std::shared_ptr<cyber::Writer<routing::RoutingRequest>>& writer);
  void SetPlanningCommandWriter(
      const std::shared_ptr<cyber::Writer<planning::PlanningCommand>>& writer);

  void UpdateChassis(const std::shared_ptr<canbus::Chassis>& msg);
  void UpdateLocalization(
      const std::shared_ptr<localization::LocalizationEstimate>& msg);
  void UpdatePlanningRuntimeStatus(
      const std::shared_ptr<planning::PlanningRuntimeStatus>& msg);
  void UpdateControlRuntimeStatus(
      const std::shared_ptr<control::ControlRuntimeStatus>& msg);
  void SaveWaypoint(const std::string& name, const common::PointENU& pose);

  std::shared_ptr<canbus::Chassis> GetChassis();
  std::shared_ptr<localization::LocalizationEstimate> GetLocalization();
  std::shared_ptr<planning::PlanningRuntimeStatus> GetPlanningRuntimeStatus();
  std::shared_ptr<control::ControlRuntimeStatus> GetControlRuntimeStatus();
  CommandLifecycleStatus GetCommandLifecycleStatus(
      const std::string& command_id) const;
  bool GetWaypoint(const std::string& name, common::PointENU* out_pose);

  void SendRoutingRequest(const common::PointENU& end_pose);
  void SendPlanningCommand(const planning::PlanningCommand& command);
  bool AcknowledgeRecovery();
  bool ResumeRecovery();
  bool RetryRecovery();
  bool AbortRecovery();

  void SetCurrentMissionId(const std::string& id);
  void SetCurrentTaskName(const std::string& task_name);
  std::string GetCurrentMissionId() const;
  MissionCommandSnapshot GetMissionCommandSnapshot() const;

 private:
  void PublishPlanningCommands(
      const std::shared_ptr<cyber::Writer<planning::PlanningCommand>>& writer,
      const std::vector<planning::PlanningCommand>& commands);

  mutable std::mutex mutex_;

  MissionCommandSupervisor command_supervisor_;
  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> routing_writer_;
  std::shared_ptr<cyber::Writer<planning::PlanningCommand>>
      planning_command_writer_;
  std::shared_ptr<canbus::Chassis> chassis_;
  std::shared_ptr<localization::LocalizationEstimate> localization_;
  std::shared_ptr<planning::PlanningRuntimeStatus> planning_runtime_status_;
  std::shared_ptr<control::ControlRuntimeStatus> control_runtime_status_;
  std::unordered_map<std::string, common::PointENU> waypoints_;

  DECLARE_SINGLETON(MissionContext)
};

}  // namespace mission
}  // namespace apollo
