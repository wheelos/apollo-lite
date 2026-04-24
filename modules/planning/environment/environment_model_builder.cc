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

#include "modules/planning/environment/environment_model_builder.h"

namespace apollo {
namespace planning {

namespace {

int CountActiveStories(const storytelling::Stories& stories) {
  int count = 0;
  if (stories.has_close_to_clear_area()) {
    ++count;
  }
  if (stories.has_close_to_crosswalk()) {
    ++count;
  }
  if (stories.has_close_to_junction()) {
    ++count;
  }
  if (stories.has_close_to_signal()) {
    ++count;
  }
  if (stories.has_close_to_stop_sign()) {
    ++count;
  }
  if (stories.has_close_to_yield_sign()) {
    ++count;
  }
  return count;
}

}  // namespace

EnvironmentModel EnvironmentModelBuilder::Build(
    const LocalView& local_view) const {
  EnvironmentModel model;
  model.vehicle = BuildVehicleStateSnapshot(local_view);
  model.mission = BuildMissionContextSnapshot(local_view);
  model.route = BuildRouteContext(local_view);
  model.local_topology = BuildLocalTopology(local_view, model.route);
  model.drivable_area = BuildDrivableAreaModel(local_view);
  model.parking =
      BuildParkingContext(model.mission, model.route, model.local_topology);
  model.regulatory = BuildRegulatoryContext(local_view);
  model.objects = BuildDynamicObjectContext(local_view);
  model.source_health = BuildSourceHealth(local_view);
  return model;
}

VehicleStateSnapshot EnvironmentModelBuilder::BuildVehicleStateSnapshot(
    const LocalView& local_view) const {
  VehicleStateSnapshot snapshot;
  if (local_view.chassis != nullptr) {
    snapshot.has_chassis = true;
    snapshot.speed_mps = local_view.chassis->speed_mps();
    snapshot.gear = local_view.chassis->gear_location();
  }
  if (local_view.localization_estimate != nullptr &&
      local_view.localization_estimate->has_pose()) {
    snapshot.has_localization = true;
    const auto& pose = local_view.localization_estimate->pose();
    snapshot.position.set_x(pose.position().x());
    snapshot.position.set_y(pose.position().y());
    snapshot.position.set_z(pose.position().z());
    snapshot.heading = pose.heading();
  }
  return snapshot;
}

MissionContextSnapshot EnvironmentModelBuilder::BuildMissionContextSnapshot(
    const LocalView& local_view) const {
  MissionContextSnapshot snapshot;
  if (local_view.planning_command == nullptr) {
    return snapshot;
  }

  const auto& command = *local_view.planning_command;
  snapshot.has_command =
      command.has_header() || command.has_command_id() || command.has_action();
  if (command.has_mission_id()) {
    snapshot.mission_id = command.mission_id();
  }
  if (command.has_command_id()) {
    snapshot.command_id = command.command_id();
  }
  if (command.has_action()) {
    snapshot.action = command.action();
  }
  if (command.has_requested_scene()) {
    snapshot.requested_scene = command.requested_scene();
  }
  if (command.has_preferred_mode()) {
    snapshot.preferred_mode = command.preferred_mode();
  }
  if (command.has_preemptible()) {
    snapshot.preemptible = command.preemptible();
  }
  if (command.has_goal()) {
    const auto& goal = command.goal();
    if (goal.has_goal_pose()) {
      snapshot.has_goal_pose = true;
      snapshot.goal_pose = goal.goal_pose();
    }
    if (goal.has_parking_goal()) {
      snapshot.has_parking_goal = true;
      snapshot.parking_goal = goal.parking_goal();
    }
  }
  return snapshot;
}

RouteContext EnvironmentModelBuilder::BuildRouteContext(
    const LocalView& local_view) const {
  RouteContext context;
  if (local_view.routing == nullptr) {
    return context;
  }

  context.has_routing = true;
  context.has_route_header = local_view.routing->has_header();
  context.has_route_segments = local_view.routing->road_size() > 0;
  context.route_road_count = local_view.routing->road_size();
  if (local_view.routing->has_routing_request()) {
    const auto& routing_request = local_view.routing->routing_request();
    context.route_waypoint_count = routing_request.waypoint_size();
    if (routing_request.has_parking_info()) {
      context.has_parking_info = true;
      context.parking_info = routing_request.parking_info();
    }
  }
  return context;
}

LocalTopology EnvironmentModelBuilder::BuildLocalTopology(
    const LocalView& local_view, const RouteContext& route) const {
  LocalTopology topology;
  if (local_view.relative_map != nullptr) {
    topology.has_relative_map = local_view.relative_map->has_header();
    topology.navigation_path_count = local_view.relative_map->navigation_path_size();
    topology.has_navigation_path = topology.navigation_path_count > 0;
    topology.has_lane_markers = local_view.relative_map->has_lane_marker();
    topology.has_relative_localization =
        local_view.relative_map->has_localization();
  }

  if (route.has_route_segments && topology.has_navigation_path) {
    topology.topology_source = TopologySource::HYBRID;
  } else if (route.has_route_segments) {
    topology.topology_source = TopologySource::HDMAP_ROUTING;
  } else if (topology.has_navigation_path || topology.has_lane_markers) {
    topology.topology_source = TopologySource::WORLD_MODEL_RELATIVE_MAP;
  }

  return topology;
}

DrivableAreaModel EnvironmentModelBuilder::BuildDrivableAreaModel(
    const LocalView& local_view) const {
  DrivableAreaModel model;
  if (local_view.relative_map == nullptr) {
    return model;
  }
  model.has_map_geometry = local_view.relative_map->has_hdmap();
  model.has_lane_marker_geometry = local_view.relative_map->has_lane_marker();
  return model;
}

ParkingContext EnvironmentModelBuilder::BuildParkingContext(
    const MissionContextSnapshot& mission, const RouteContext& route,
    const LocalTopology& local_topology) const {
  ParkingContext context;
  context.has_parking_target =
      mission.has_parking_goal || route.has_parking_info;
  context.has_parking_roi_hint =
      local_topology.has_relative_map && local_topology.has_navigation_path;

  if (mission.has_parking_goal &&
      mission.parking_goal.has_parking_space_id()) {
    context.parking_space_id = mission.parking_goal.parking_space_id();
  } else if (route.has_parking_info &&
             route.parking_info.has_parking_space_id()) {
    context.parking_space_id = route.parking_info.parking_space_id();
  }

  return context;
}

RegulatoryContext EnvironmentModelBuilder::BuildRegulatoryContext(
    const LocalView& local_view) const {
  RegulatoryContext context;
  if (local_view.traffic_light != nullptr) {
    context.has_traffic_light =
        local_view.traffic_light->has_header() ||
        local_view.traffic_light->traffic_light_size() > 0;
    context.traffic_light_count = local_view.traffic_light->traffic_light_size();
  }
  if (local_view.stories != nullptr) {
    context.active_story_count = CountActiveStories(*local_view.stories);
    context.has_storytelling =
        local_view.stories->has_header() || context.active_story_count > 0;
  }
  return context;
}

DynamicObjectContext EnvironmentModelBuilder::BuildDynamicObjectContext(
    const LocalView& local_view) const {
  DynamicObjectContext context;
  if (local_view.prediction_obstacles == nullptr) {
    return context;
  }
  context.has_prediction =
      local_view.prediction_obstacles->has_header() ||
      local_view.prediction_obstacles->prediction_obstacle_size() > 0;
  context.obstacle_count =
      local_view.prediction_obstacles->prediction_obstacle_size();
  return context;
}

SourceHealth EnvironmentModelBuilder::BuildSourceHealth(
    const LocalView& local_view) const {
  SourceHealth health;
  health.chassis_ready =
      local_view.chassis != nullptr && local_view.chassis->has_header();
  health.localization_ready = local_view.localization_estimate != nullptr &&
                              local_view.localization_estimate->has_header();
  health.routing_ready =
      local_view.routing != nullptr && local_view.routing->has_header();
  health.relative_map_ready =
      local_view.relative_map != nullptr && local_view.relative_map->has_header();
  health.prediction_ready = local_view.prediction_obstacles != nullptr &&
                            local_view.prediction_obstacles->has_header();
  health.traffic_light_ready = local_view.traffic_light != nullptr &&
                               local_view.traffic_light->has_header();
  health.planning_command_ready =
      local_view.planning_command != nullptr &&
      local_view.planning_command->has_header() &&
      local_view.planning_command->has_command_id();
  health.storytelling_ready =
      local_view.stories != nullptr && local_view.stories->has_header();
  return health;
}

}  // namespace planning
}  // namespace apollo
