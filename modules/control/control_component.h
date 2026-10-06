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
#include <deque>
#include <mutex>
#include <string>

#include "wheelos_msgs/chassis_msgs/chassis.pb.h"
#include "wheelos_msgs/control_msgs/control_cmd.pb.h"
#include "wheelos_msgs/control_msgs/control_runtime_status.pb.h"
#include "wheelos_msgs/control_msgs/safety_stop.pb.h"
#include "wheelos_msgs/control_msgs/pad_msg.pb.h"
#include "wheelos_msgs/localization_msgs/localization.pb.h"
#include "wheelos_msgs/planning_msgs/planning.pb.h"
#include "wheelos_msgs/planning_msgs/motion_execution.pb.h"
#include "modules/control/proto/control_conf.pb.h"

#include "cyber/component/timer_component.h"
#include "cyber/time/time.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/control/common/control_command_goal.h"
#include "modules/control/common/executor_arbiter.h"
#include "modules/control/common/motion_command_adapter.h"
#include "modules/control/common/motion_execution_manager.h"
#include "modules/control/common/motion_execution_monitor.h"
#include "modules/control/common/motion_primitive_executor.h"
#include "modules/control/common/dependency_injector.h"
#include "modules/control/common/strategy_orchestrator.h"
#include "modules/control/controller/controller_agent.h"
#include "modules/control/controller/controller_profile_manager.h"
#include "modules/control/safety/safety_manager.h"
#include "modules/execution_state_sync/client.h"

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
  void OnLocalization(
      const std::shared_ptr<apollo::localization::LocalizationEstimate>
          &localization);
  bool InitMotionExecutors();
  void ProcessMotionDirective(const planning::MotionDirective& directive,
                              double now_sec, bool authorized_stop_cleanup = false);
  void AdvanceMotionExecution(double now_sec);
  void ReleaseMotionExecutor();
  void FailMotionExecution(double now_sec, const std::string& reason);
  void ProduceSafeStop(ControlCommand* command);
  bool PollExecutionState(double now_sec);
  bool CheckMotionAuthorization(const execution_state_sync::Event& event,
                                const planning::MotionDirective& directive) const;
  void FlushExecutionStatus();
  void FlushSafetyStatus();
  bool ProcessSafetyOperation(const execution_state_sync::Event& event,
                              double now_sec);
  MotionExecutionVehicleState BuildMotionVehicleState() const;

  // Core Logic
  void InitReaders();
  common::Status ProduceControlCommand(ControlCommand *control_command,
                                       bool *used_previous_command);
  void PublishRuntimeStatus(const ControlCommand &control_command,
                            const common::Status &status,
                            bool used_previous_command);
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
  ControllerProfileManager controller_profiles_;
  std::shared_ptr<DependencyInjector> injector_;
  std::unique_ptr<SafetyManager> safety_manager_;
  common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  // Runtime State
  LocalView local_view_;
  ControlCommand previous_cmd_;
  ControlCommandGoal last_goal_;
  SemanticControlProfile last_profile_;
  StrategyOrchestrator strategy_orchestrator_;
  std::unique_ptr<MotionExecutionManager> motion_execution_manager_;
  MotionCommandAdapter motion_command_adapter_;
  std::unique_ptr<MotionPrimitiveExecutor> motion_primitive_executor_;
  std::unique_ptr<MotionExecutionMonitor> motion_execution_monitor_;
  ExecutorArbiter executor_arbiter_;
  planning::MotionExecutionStatus latest_motion_execution_status_;
  planning::MotionDirectiveScope active_motion_scope_ =
      planning::MOTION_SCOPE_UNKNOWN;
  planning::MotionDirectiveScope reported_motion_scope_ =
      planning::MOTION_SCOPE_UNKNOWN;
  std::unique_ptr<execution_state_sync::Client> execution_state_client_;
  uint64_t motion_event_sequence_ = 0;
  uint64_t mission_event_sequence_ = 0;
  uint64_t status_ticket_ = 0;
  uint64_t safety_status_ticket_ = 0;
  std::string control_epoch_;
  uint64_t active_cleanup_mission_sequence_ = 0;
  bool execution_state_fault_ = false;
  bool safety_latch_reconciled_ = false;
  bool directive_rejected_this_cycle_ = false;
  std::string last_unscoped_status_fingerprint_;
  double last_unscoped_status_submit_sec_ = 0.0;
  struct PendingRuntimeStatus {
    ControlRuntimeStatus status;
    uint64_t mission_sequence = 0;
    uint64_t motion_sequence = 0;
  };
  std::deque<PendingRuntimeStatus> pending_runtime_status_;
  std::deque<apollo::control::SafetyStopObservation> pending_safety_status_;
  bool pad_received_ = false;
  bool vehicle_state_ready_ = false;
  double motion_state_max_age_sec_ = 0.0;

  // Cyber RT Interfaces
  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Writer<ControlCommand>> control_cmd_writer_;
  std::shared_ptr<cyber::Writer<ControlRuntimeStatus>>
      control_runtime_status_writer_;
};

CYBER_REGISTER_COMPONENT(ControlComponent)

}  // namespace control
}  // namespace apollo
