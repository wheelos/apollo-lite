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

#include "modules/planning/planning_base.h"

#include "modules/common_msgs/planning_msgs/planning_internal.pb.h"

#include "cyber/time/clock.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/environment/capability_extractor.h"
#include "modules/planning/planning_runtime_context.h"
#include "modules/planning/tasks/task_factory.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

PlanningBase::PlanningBase(const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector) {}

PlanningBase::~PlanningBase() {}

Status PlanningBase::Init(const PlanningConfig& config) {
  injector_->planning_context()->Init();
  TaskFactory::Init(config, injector_);
  return Status::OK();
}

void PlanningBase::FillPlanningPb(const double timestamp,
                                  ADCTrajectory* const trajectory_pb) {
  trajectory_pb->mutable_header()->set_timestamp_sec(timestamp);
  if (local_view_.prediction_obstacles != nullptr &&
      local_view_.prediction_obstacles->has_header()) {
    trajectory_pb->mutable_header()->set_lidar_timestamp(
        local_view_.prediction_obstacles->header().lidar_timestamp());
    trajectory_pb->mutable_header()->set_camera_timestamp(
        local_view_.prediction_obstacles->header().camera_timestamp());
    trajectory_pb->mutable_header()->set_radar_timestamp(
        local_view_.prediction_obstacles->header().radar_timestamp());
  }
  if (local_view_.routing != nullptr) {
    trajectory_pb->mutable_routing_header()->CopyFrom(
        local_view_.routing->header());
  }

  auto* planning_data = trajectory_pb->mutable_debug()->mutable_planning_data();
  auto* runtime = planning_data->mutable_runtime();

  if (local_view_.planning_state != nullptr) {
    if (!local_view_.planning_state->mission_id.empty()) {
      runtime->set_mission_id(local_view_.planning_state->mission_id);
    }
    if (!local_view_.planning_state->command_id.empty()) {
      runtime->set_command_id(local_view_.planning_state->command_id);
    }
    runtime->set_active_scene(local_view_.planning_state->active_scene);
    runtime->set_requested_mode(local_view_.planning_state->requested_mode);
    runtime->set_resolved_mode(local_view_.planning_state->resolved_mode);
    if (!local_view_.planning_state->reason.empty()) {
      runtime->set_reason(local_view_.planning_state->reason);
    }
    for (const auto& blocker : local_view_.planning_state->blockers) {
      runtime->add_blockers(blocker);
    }
  }

  if (local_view_.capability_set != nullptr) {
    runtime->set_has_lane_graph(local_view_.capability_set->has_lane_graph);
    runtime->set_has_route_semantics(
        local_view_.capability_set->has_route_semantics);
    runtime->set_has_local_corridor(
        local_view_.capability_set->has_local_corridor);
    runtime->set_has_drivable_area(
        local_view_.capability_set->has_drivable_area);
    runtime->set_has_parking_roi(local_view_.capability_set->has_parking_roi);
    runtime->set_has_goal_pose(local_view_.capability_set->has_goal_pose);
    runtime->set_has_stop_target(local_view_.capability_set->has_stop_target);
    runtime->set_has_regulatory_context(
        local_view_.capability_set->has_regulatory_context);
    runtime->set_topology_confidence(
        local_view_.capability_set->topology_confidence);
    runtime->set_drivable_area_confidence(
        local_view_.capability_set->drivable_area_confidence);
    runtime->set_target_geometry_confidence(
        local_view_.capability_set->target_geometry_confidence);
  }
}
}  // namespace planning
}  // namespace apollo
