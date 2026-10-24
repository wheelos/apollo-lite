/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
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

#include <cstdint>
#include <memory>

#include "wheelos_msgs/planning_msgs/planning.pb.h"
#include "wheelos_msgs/planning_msgs/planning_internal.pb.h"

namespace apollo {
namespace common {
class Status;
class TrajectoryPoint;
}  // namespace common

namespace planning {

class DependencyInjector;
class Frame;
class ReferenceLineInfo;

// OnLaneDebugExporter keeps chart generation and debug rendering out of the
// execution flow so OnLanePlanning can stay focused on runtime orchestration.
class OnLaneDebugExporter {
 public:
  explicit OnLaneDebugExporter(
      const std::shared_ptr<DependencyInjector>& injector);

  void RecordInputDebug(Frame* frame, planning_internal::Debug* debug,
                        LatencyStats* latency_stats) const;
  void InitializePlannerDebug(const common::TrajectoryPoint& stitching_point,
                              Frame* frame, planning_internal::Debug* debug,
                              LatencyStats* latency_stats) const;
  void LogPlanningCycle(uint32_t frame_num, const common::Status& plan_status,
                        const ADCTrajectory& trajectory,
                        const Frame* frame) const;
  void ExportReferenceLineDebug(Frame* frame,
                                planning_internal::Debug* debug) const;
  void ExportOnLanePlanDebug(Frame* frame,
                             const ReferenceLineInfo& best_ref_info,
                             planning_internal::Debug* debug,
                             LatencyStats* latency_stats) const;
  void ExportOpenSpacePlanDebug(Frame* frame, ADCTrajectory* trajectory) const;
  void ExportFailedLaneChangeSTChart(
      const planning_internal::Debug& debug_info,
      planning_internal::Debug* debug_chart) const;
  void ExportOnLaneChart(const planning_internal::Debug& debug_info,
                         planning_internal::Debug* debug_chart) const;
  void ExportOpenSpaceChart(Frame* frame,
                            const planning_internal::Debug& debug_info,
                            const ADCTrajectory& trajectory_pb,
                            planning_internal::Debug* debug_chart) const;
  void ExportPlanningReferenceLinePath(
      const ReferenceLineInfo& best_ref_info,
      planning_internal::Debug* ptr_debug) const;

 private:
  void AddOpenSpaceOptimizerResult(const Frame& frame,
                                   const planning_internal::Debug& debug_info,
                                   planning_internal::Debug* debug_chart) const;
  void AddPartitionedTrajectory(const Frame& frame,
                                const planning_internal::Debug& debug_info,
                                planning_internal::Debug* debug_chart) const;
  void AddStitchSpeedProfile(const Frame& frame,
                             planning_internal::Debug* debug_chart) const;
  void AddPublishedSpeed(const Frame& frame, const ADCTrajectory& trajectory_pb,
                         planning_internal::Debug* debug_chart) const;
  void AddPublishedAcceleration(const Frame& frame,
                                const ADCTrajectory& trajectory_pb,
                                planning_internal::Debug* debug_chart) const;

  std::shared_ptr<DependencyInjector> injector_;
};

}  // namespace planning
}  // namespace apollo
