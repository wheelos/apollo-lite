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

#include <string>

#include "modules/common_msgs/basic_msgs/geometry.pb.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"

namespace apollo {
namespace planning {

enum class TopologySource {
  UNKNOWN = 0,
  HDMAP_ROUTING = 1,
  WORLD_MODEL_RELATIVE_MAP = 2,
  HYBRID = 3,
};

struct VehicleStateSnapshot {
  bool has_chassis = false;
  bool has_localization = false;
  common::PointENU position;
  double heading = 0.0;
  double speed_mps = 0.0;
  canbus::Chassis::GearPosition gear = canbus::Chassis::GEAR_NONE;
};

struct MissionContextSnapshot {
  bool has_command = false;
  std::string mission_id;
  std::string command_id;
  CommandAction action = COMMAND_UNKNOWN;
  PlanningSceneType requested_scene = SCENE_UNKNOWN;
  PlanningMode preferred_mode = MODE_UNKNOWN;
  bool preemptible = false;
  bool has_goal_pose = false;
  common::PointENU goal_pose;
  bool has_parking_goal = false;
  routing::ParkingInfo parking_goal;
};

struct RouteContext {
  bool has_routing = false;
  bool has_route_header = false;
  bool has_route_segments = false;
  int route_road_count = 0;
  int route_waypoint_count = 0;
  bool has_parking_info = false;
  routing::ParkingInfo parking_info;
};

struct LocalTopology {
  bool has_relative_map = false;
  bool has_navigation_path = false;
  bool has_lane_markers = false;
  bool has_relative_localization = false;
  int navigation_path_count = 0;
  TopologySource topology_source = TopologySource::UNKNOWN;
};

struct DrivableAreaModel {
  bool has_map_geometry = false;
  bool has_lane_marker_geometry = false;
};

struct ParkingContext {
  bool has_parking_target = false;
  bool has_parking_roi_hint = false;
  std::string parking_space_id;
};

struct RegulatoryContext {
  bool has_traffic_light = false;
  int traffic_light_count = 0;
  bool has_storytelling = false;
  int active_story_count = 0;
};

struct DynamicObjectContext {
  bool has_prediction = false;
  int obstacle_count = 0;
};

struct SourceHealth {
  bool chassis_ready = false;
  bool localization_ready = false;
  bool routing_ready = false;
  bool relative_map_ready = false;
  bool prediction_ready = false;
  bool traffic_light_ready = false;
  bool planning_command_ready = false;
  bool storytelling_ready = false;
};

struct EnvironmentModel {
  VehicleStateSnapshot vehicle;
  MissionContextSnapshot mission;
  RouteContext route;
  LocalTopology local_topology;
  DrivableAreaModel drivable_area;
  ParkingContext parking;
  RegulatoryContext regulatory;
  DynamicObjectContext objects;
  SourceHealth source_health;
};

}  // namespace planning
}  // namespace apollo
