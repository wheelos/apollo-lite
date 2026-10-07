/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "modules/planning/proto/learning_data.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "wheelos_msgs/chassis_msgs/chassis.pb.h"
#include "wheelos_msgs/control_msgs/control_runtime_status.pb.h"
#include "wheelos_msgs/localization_msgs/localization.pb.h"
#include "wheelos_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "wheelos_msgs/planning_msgs/mission_directive.pb.h"
#include "wheelos_msgs/planning_msgs/motion_execution.pb.h"
#include "wheelos_msgs/planning_msgs/pad_msg.pb.h"
#include "wheelos_msgs/planning_msgs/planning.pb.h"
#include "wheelos_msgs/planning_msgs/planning_command.pb.h"
#include "wheelos_msgs/planning_msgs/planning_runtime_status.pb.h"
#include "wheelos_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "wheelos_msgs/routing_msgs/routing.pb.h"
#include "wheelos_msgs/storytelling_msgs/story.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/message/raw_message.h"
#include "modules/planning/common/hybrid_maneuver_supervisor.h"
#include "modules/planning/common/message_process.h"
#include "modules/planning/common/motion_plan_builder.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_semantics.h"
#include "modules/planning/common/terminal_servo_guard.h"
#include "modules/planning/environment/capability_extractor.h"
#include "modules/planning/environment/environment_model_builder.h"
#include "modules/planning/planning_coordinator.h"
#include "modules/planning/planning_cycle_diagnostics.h"
#include "modules/planning/planning_cycle_result.h"
#include "modules/planning/planning_execution_state_transport.h"
#include "modules/planning/validation/validation_supervisor.h"
#include "modules/routing/routing.h"

namespace apollo {
namespace planning {

class PlanningComponent final
    : public cyber::Component<prediction::PredictionObstacles, canbus::Chassis,
                              localization::LocalizationEstimate> {
 public:
  PlanningComponent() = default;

  ~PlanningComponent() = default;

 public:
  bool Init() override;

  bool Proc(const std::shared_ptr<prediction::PredictionObstacles>&
                prediction_obstacles,
            const std::shared_ptr<canbus::Chassis>& chassis,
            const std::shared_ptr<localization::LocalizationEstimate>&
                localization_estimate) override;

 private:
  enum class PlanningCyclePreparation {
    kReady,
    kInputHold,
  };

  bool ServiceExecutionState(
      const localization::LocalizationEstimate& localization);
  PlanningCyclePreparation PreparePlanningCycle(
      const std::shared_ptr<prediction::PredictionObstacles>&
          prediction_obstacles,
      const std::shared_ptr<canbus::Chassis>& chassis,
      const std::shared_ptr<localization::LocalizationEstimate>&
          localization_estimate,
      PlanningCycleResult* result);
  bool ProcessLearningCycle(PlanningCycleResult* result);
  void RunPlanningCycle(PlanningCycleResult* result);
  void PrepareInputHoldResult(const std::string& reason,
                              PlanningCycleResult* result);
  bool CompletePlanningCycle(
      PlanningCycleResult* result, const canbus::Chassis* chassis,
      const localization::LocalizationEstimate* localization);

  void CheckRerouting();
  void UpdateRoutingForCommand(
      const localization::LocalizationEstimate& localization);
  void UpdateRoutingForMission(
      const localization::LocalizationEstimate& localization);
  void ApplyPendingMissionDirective(
      const localization::LocalizationEstimate& localization,
      bool superseded = false);
  void ApplyControlMotionStatus();
  bool PollExecutionState(
      const localization::LocalizationEstimate& localization);
  void DrainExecutionStateSubmissions();
  bool SubmitExecutionState(
      execution_state_sync::Channel channel, const std::string& payload,
      bool cleanup,
      execution_state_sync::PlanningStatusKind planning_status_kind =
          execution_state_sync::PlanningStatusKind::kRuntime);
  void PublishMotionPlan(const PlanningCoordinatorState& coordinator_state,
                         const PlanningSemanticSummary& semantic_summary,
                         const canbus::Chassis& chassis,
                         const localization::LocalizationEstimate& localization,
                         const ADCTrajectory& trajectory);
  void RefreshLocalView(
      const std::shared_ptr<prediction::PredictionObstacles>&
          prediction_obstacles,
      const std::shared_ptr<canbus::Chassis>& chassis,
      const std::shared_ptr<localization::LocalizationEstimate>&
          localization_estimate);
  void RefreshEnvironmentState();
  void ProcessLearningInputs();
  bool PublishLearningDataFrame();
  void FinalizeTrajectoryTiming(double original_start_time_sec,
                                ADCTrajectory* trajectory) const;
  RuntimeState InferCoordinatorRuntimeState() const;
  HybridManeuverSummary EvaluateHybridManeuver(
      const PlanningCoordinatorState& coordinator_state,
      RuntimeState runtime_state) const;
  PlanningExecutionContext ResolvePublishedExecutionContext(
      const PlanningCoordinatorState& coordinator_state,
      const ADCTrajectory& trajectory) const;
  std::string CheckInput(const PlanningCoordinatorState& preview_state) const;
  void FinalizePlanningResult(
      PlanningCycleResult* result, const canbus::Chassis* chassis,
      const localization::LocalizationEstimate* localization);
  void PopulateTrajectoryExecutionContext(
      const PlanningCoordinatorState& coordinator_state,
      const HybridManeuverSummary& hybrid_summary,
      ADCTrajectory* trajectory) const;
  void PublishRuntimeStatus(const PlanningSemanticSummary& semantic_summary,
                            const HybridManeuverSummary& hybrid_summary,
                            const ValidationResult& validation_result,
                            const PlanningCoordinatorState& coordinator_state,
                            const PlanningExecutionContext& execution,
                            const MissionCommandIdentity&
                                accepted_directive_identity,
                            const CapabilitySet* capability_set,
                            const std::string& reason = "");

 private:
  friend class PlanningMissionSyncTestPeer;

  std::shared_ptr<cyber::Reader<perception::TrafficLightDetection>>
      traffic_light_reader_;
  std::shared_ptr<cyber::Reader<planning::PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<planning::PlanningCommand>>
      planning_command_reader_;
  std::shared_ptr<cyber::Reader<relative_map::MapMsg>> relative_map_reader_;
  std::shared_ptr<cyber::Reader<storytelling::Stories>> story_telling_reader_;

  std::shared_ptr<cyber::Writer<ADCTrajectory>> planning_writer_;
  std::shared_ptr<cyber::Writer<PlanningRuntimeStatus>>
      planning_runtime_status_writer_;
  std::shared_ptr<cyber::Writer<MotionDirective>> motion_directive_writer_;
  std::shared_ptr<cyber::Writer<PlanningLearningData>>
      planning_learning_data_writer_;

  std::mutex mutex_;
  perception::TrafficLightDetection traffic_light_;
  routing::RoutingResponse routing_;
  planning::PadMessage pad_msg_;
  planning::PlanningCommand planning_command_;
  planning::MissionDirective mission_directive_;
  control::ControlRuntimeStatus control_runtime_status_;
  execution_state_sync::ControlStatusKind control_status_kind_ =
      execution_state_sync::ControlStatusKind::kOwnerRuntime;
  relative_map::MapMsg relative_map_;
  storytelling::Stories stories_;

  LocalView local_view_;

  std::unique_ptr<PlanningCoordinator> planning_coordinator_;
  std::unique_ptr<routing::RoutingService> routing_service_;
  std::shared_ptr<DependencyInjector> injector_;

  PlanningConfig config_;
  MessageProcess message_process_;
  CapabilityExtractor capability_extractor_;
  EnvironmentModelBuilder environment_model_builder_;
  HybridManeuverSupervisor hybrid_maneuver_supervisor_;
  ValidationSupervisor validation_supervisor_;
  TerminalServoSessionState terminal_servo_session_state_;
  MotionPlanBuilder motion_plan_builder_{"planning-runtime-v2"};
  PlanningCycleDiagnostics diagnostics_;
  std::unique_ptr<PlanningExecutionStateTransport> execution_state_transport_;
  uint64_t mission_event_sequence_ = 0;
  uint64_t motion_event_sequence_ = 0;
  uint64_t deferred_mission_ack_sequence_ = 0;
  size_t pending_mission_admissions_ = 0;
  bool execution_state_fault_ = false;
  std::string last_planning_status_fingerprint_;
  double last_planning_status_submit_sec_ = 0.0;
  std::string applied_control_motion_status_fingerprint_;
  std::string routed_command_fingerprint_;
  std::string applied_mission_directive_fingerprint_;
};

CYBER_REGISTER_COMPONENT(PlanningComponent)

}  // namespace planning
}  // namespace apollo
