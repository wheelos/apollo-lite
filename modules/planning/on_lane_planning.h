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
#include <vector>

#include "modules/planning/common/smoothers/smoother.h"
#include "modules/planning/on_lane_debug_exporter.h"
#include "modules/planning/planner/planner_selector.h"
#include "modules/planning/planning_base.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

class ReferenceLineInfo;

/**
 * @class planning
 *
 * @brief Planning module main class. It processes GPS and IMU as input,
 * to generate planning info.
 */
class OnLanePlanning : public PlanningBase {
 public:
  explicit OnLanePlanning(const std::shared_ptr<DependencyInjector>& injector)
      : PlanningBase(injector), debug_exporter_(injector) {}
  virtual ~OnLanePlanning();

  /**
   * @brief Planning name.
   */
  std::string Name() const override;
  PlanningMode Mode() const override { return MODE_LANE_GRAPH; }

  /**
   * @brief module initialization function
   * @return initialization status
   */
  common::Status Init(const PlanningConfig& config) override;

  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * timer.
   */
  void RunOnce(const LocalView& local_view,
               ADCTrajectory* const ptr_trajectory_pb) override;

  common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory) override;

 private:
  // PlanningCycleState owns only per-RunOnce orchestration data. It must not
  // outlive the current planning cycle.
  struct PlanningCycleState {
    double start_timestamp = 0.0;
    double start_system_timestamp = 0.0;
    common::VehicleState vehicle_state;
    uint32_t frame_num = 0;
    std::string replan_reason;
    std::vector<common::TrajectoryPoint> stitching_trajectory;
  };

  common::Status InitFrame(const uint32_t sequence_num,
                           const common::TrajectoryPoint& planning_start_point,
                           const common::VehicleState& vehicle_state);

  common::VehicleState AlignTimeStamp(const common::VehicleState& vehicle_state,
                                      const double curr_timestamp) const;
  void InitializeCycleState(PlanningCycleState* cycle_state) const;
  void PublishNotReadyStopTrajectory(const double start_timestamp,
                                     const common::Status& status,
                                     const std::string& reason,
                                     ADCTrajectory* ptr_trajectory_pb);
  bool UpdateVehicleStateForCycle(PlanningCycleState* cycle_state,
                                  ADCTrajectory* ptr_trajectory_pb);
  bool RefreshReferenceLineForCycle(const PlanningCycleState& cycle_state,
                                    ADCTrajectory* ptr_trajectory_pb);
  void ComputeStitchingTrajectory(PlanningCycleState* cycle_state);
  bool PrepareFrameForCycle(PlanningCycleState* cycle_state,
                            ADCTrajectory* ptr_trajectory_pb);
  void ApplyTrafficRulesToFrame();
  void FinalizeTrajectoryForCycle(const PlanningCycleState& cycle_state,
                                  const common::Status& plan_status,
                                  ADCTrajectory* ptr_trajectory_pb);
  void FinalizeFrameHistory(ADCTrajectory* ptr_trajectory_pb);
  void LogPlanningCycle(const PlanningCycleState& cycle_state,
                        const common::Status& plan_status,
                        const ADCTrajectory& trajectory_pb);
  void InitializePlannerDebug(
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* ptr_trajectory_pb);
  void PopulateOpenSpacePlanResult(ADCTrajectory* ptr_trajectory_pb);
  common::Status PopulateOnLanePlanResult(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* ptr_trajectory_pb);
  void BuildFallbackPathForNextCycle(
      const ReferenceLineInfo& best_ref_info,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory);
  void ExportOnLanePlanDebug(const ReferenceLineInfo& best_ref_info,
                             planning_internal::Debug* ptr_debug);
  bool CheckPlanningConfig(const PlanningConfig& config);
  void GenerateStopTrajectory(ADCTrajectory* ptr_trajectory_pb);

 private:
  routing::RoutingResponse last_routing_;
  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;
  Smoother planning_smoother_;
  OnLaneDebugExporter debug_exporter_;
};

}  // namespace planning
}  // namespace apollo
