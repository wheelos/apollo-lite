/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/control_msgs/pad_msg.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/control/proto/control_conf.pb.h"

#include "cyber/component/timer_component.h"
#include "cyber/time/time.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/control/common/dependency_injector.h"
#include "modules/control/controller/controller_agent.h"
#include "modules/control/safety/safety_manager.h"

namespace apollo {
namespace control {

/**
 * @class ControlComponent
 *
 * @brief Control module main class. It schedules the data flow:
 * 1. Reads inputs (Chassis, Localization, Planning, Pad).
 * 2. Delegates safety checks to SafetyManager.
 * 3. Invokes ControllerAgent for core computation.
 * 4. Applies Safety Overrides (Estop/Degradation).
 * 5. Publishes Control Command.
 */
class ControlComponent final : public apollo::cyber::TimerComponent {
 public:
  ControlComponent();
  bool Init() override;
  bool Proc() override;

 private:
  // Data Callbacks
  void OnPad(const std::shared_ptr<PadMessage> &pad);
  void OnChassis(const std::shared_ptr<apollo::canbus::Chassis> &chassis);
  void OnPlanning(
      const std::shared_ptr<apollo::planning::ADCTrajectory> &trajectory);
  void OnLocalization(
      const std::shared_ptr<apollo::localization::LocalizationEstimate>
          &localization);

  // Core Logic
  void InitReaders();
  common::Status ProduceControlCommand(ControlCommand *control_command);
  void ResetAndProduceZeroControlCommand(ControlCommand *control_command);

 private:
  apollo::cyber::Time init_time_;
  ControlConf control_conf_;
  std::mutex mutex_;

  // Data Buffers (Protected by mutex_)
  localization::LocalizationEstimate latest_localization_;
  canbus::Chassis latest_chassis_;
  planning::ADCTrajectory latest_trajectory_;
  PadMessage pad_msg_;

  // Modules
  ControllerAgent controller_agent_;
  std::shared_ptr<DependencyInjector> injector_;
  std::unique_ptr<SafetyManager> safety_manager_;
  common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  // Runtime State
  LocalView local_view_;
  ControlCommand previous_cmd_;
  bool pad_received_ = false;

  // Cyber RT Interfaces
  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<apollo::planning::ADCTrajectory>>
      trajectory_reader_;

  std::shared_ptr<cyber::Writer<ControlCommand>> control_cmd_writer_;
};

CYBER_REGISTER_COMPONENT(ControlComponent)

}  // namespace control
}  // namespace apollo
