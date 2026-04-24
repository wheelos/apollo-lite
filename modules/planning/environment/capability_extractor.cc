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

#include "modules/planning/environment/capability_extractor.h"

namespace apollo {
namespace planning {

CapabilitySet CapabilityExtractor::Extract(const EnvironmentModel& model) const {
  CapabilitySet capability;
  capability.has_route_semantics =
      model.route.has_route_segments || model.route.route_waypoint_count > 0;
  capability.has_lane_graph =
      capability.has_route_semantics && model.source_health.localization_ready &&
      model.source_health.chassis_ready;
  capability.has_local_corridor =
      model.local_topology.has_navigation_path ||
      model.local_topology.has_lane_markers;
  capability.has_drivable_area =
      model.drivable_area.has_map_geometry ||
      model.drivable_area.has_lane_marker_geometry;
  capability.has_parking_roi = model.parking.has_parking_roi_hint;
  capability.has_goal_pose =
      model.mission.has_goal_pose || model.mission.has_parking_goal ||
      model.route.has_parking_info;
  capability.has_stop_target =
      model.source_health.localization_ready && model.source_health.chassis_ready;
  capability.has_regulatory_context =
      model.regulatory.has_traffic_light || model.regulatory.has_storytelling;

  switch (model.local_topology.topology_source) {
    case TopologySource::HYBRID:
      capability.topology_confidence = 1.0;
      break;
    case TopologySource::HDMAP_ROUTING:
      capability.topology_confidence =
          model.source_health.routing_ready ? 0.95 : 0.7;
      break;
    case TopologySource::WORLD_MODEL_RELATIVE_MAP:
      capability.topology_confidence =
          model.local_topology.has_lane_markers ? 0.8 : 0.65;
      break;
    case TopologySource::UNKNOWN:
    default:
      capability.topology_confidence = 0.0;
      break;
  }

  if (model.drivable_area.has_map_geometry &&
      model.drivable_area.has_lane_marker_geometry) {
    capability.drivable_area_confidence = 1.0;
  } else if (model.drivable_area.has_map_geometry) {
    capability.drivable_area_confidence = 0.9;
  } else if (model.drivable_area.has_lane_marker_geometry) {
    capability.drivable_area_confidence = 0.7;
  }

  if (model.mission.has_parking_goal && model.parking.has_parking_target) {
    capability.target_geometry_confidence = 1.0;
  } else if (model.route.has_parking_info) {
    capability.target_geometry_confidence = 0.95;
  } else if (model.mission.has_goal_pose) {
    capability.target_geometry_confidence = 0.85;
  }

  return capability;
}

}  // namespace planning
}  // namespace apollo
