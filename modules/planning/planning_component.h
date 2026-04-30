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

#include <memory>
#include <string>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/common_msgs/planning_msgs/pad_msg.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "modules/common_msgs/planning_msgs/planning_runtime_status.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/common_msgs/storytelling_msgs/story.pb.h"
#include "modules/planning/proto/learning_data.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/message/raw_message.h"
#include "modules/planning/common/hybrid_maneuver_supervisor.h"
#include "modules/planning/common/message_process.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_semantics.h"
#include "modules/planning/common/terminal_servo_guard.h"
#include "modules/planning/environment/capability_extractor.h"
#include "modules/planning/environment/environment_model_builder.h"
#include "modules/planning/planning_coordinator.h"
#include "modules/planning/validation/validation_supervisor.h"

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
  // PlanningCycleState owns single-cycle orchestration data only. It must not
  // persist across Proc() calls.
  struct PlanningCycleState {
    PlanningCoordinatorState preview_state;
    ValidationResult validation_result;
    PlanningSemanticSummary semantic_summary;
    HybridManeuverSummary hybrid_summary;
    RuntimeState runtime_state = RUNTIME_UNKNOWN;
    std::string publish_reason;
  };

  void CheckRerouting();
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
  bool CheckInput(const PlanningCoordinatorState& preview_state,
                  ValidationResult* validation_result);
  void PopulateTrajectoryExecutionContext(
      const PlanningCoordinatorState& coordinator_state,
      const PlanningSemanticSummary& semantic_summary,
      const HybridManeuverSummary& hybrid_summary,
      ADCTrajectory* trajectory) const;
  void PublishRuntimeStatus(const PlanningSemanticSummary& semantic_summary,
                            const HybridManeuverSummary& hybrid_summary,
                            const ValidationResult& validation_result,
                            const PlanningCoordinatorState& coordinator_state,
                            const PlanningExecutionContext& execution,
                            const std::string& reason = "");
  void LogPlanningCycle(const PlanningCoordinatorState& coordinator_state,
                        const PlanningSemanticSummary& semantic_summary,
                        const HybridManeuverSummary& hybrid_summary,
                        const std::string& reason);

 private:
  std::shared_ptr<cyber::Reader<perception::TrafficLightDetection>>
      traffic_light_reader_;
  std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_;
  std::shared_ptr<cyber::Reader<planning::PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<planning::PlanningCommand>>
      planning_command_reader_;
  std::shared_ptr<cyber::Reader<relative_map::MapMsg>> relative_map_reader_;
  std::shared_ptr<cyber::Reader<storytelling::Stories>> story_telling_reader_;

  std::shared_ptr<cyber::Writer<ADCTrajectory>> planning_writer_;
  std::shared_ptr<cyber::Writer<PlanningRuntimeStatus>>
      planning_runtime_status_writer_;
  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> rerouting_writer_;
  std::shared_ptr<cyber::Writer<PlanningLearningData>>
      planning_learning_data_writer_;

  std::mutex mutex_;
  perception::TrafficLightDetection traffic_light_;
  routing::RoutingResponse routing_;
  planning::PadMessage pad_msg_;
  planning::PlanningCommand planning_command_;
  relative_map::MapMsg relative_map_;
  storytelling::Stories stories_;

  LocalView local_view_;

  std::unique_ptr<PlanningCoordinator> planning_coordinator_;
  std::shared_ptr<DependencyInjector> injector_;

  PlanningConfig config_;
  MessageProcess message_process_;
  CapabilityExtractor capability_extractor_;
  EnvironmentModelBuilder environment_model_builder_;
  HybridManeuverSupervisor hybrid_maneuver_supervisor_;
  ValidationSupervisor validation_supervisor_;
  TerminalServoSessionState terminal_servo_session_state_;
  std::string last_logged_command_id_;
  PlanningMode last_logged_mode_ = MODE_UNKNOWN;
  PlanningShellType last_logged_shell_ = PLANNING_SHELL_UNKNOWN;
};

CYBER_REGISTER_COMPONENT(PlanningComponent)

}  // namespace planning
}  // namespace apollo
