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

#include <cstddef>
#include <string>
#include <vector>

#include "modules/common_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/open_space/parking/parking_slot.h"

namespace apollo {
namespace planning {
namespace parking {

struct ParkingRoiBuildInput {
  std::vector<common::math::Vec2d> left_boundary;
  std::vector<common::math::Vec2d> right_boundary;
  std::vector<common::math::Vec2d> slot_side_connector_boundary;
  std::size_t connection_start_index = 0U;
  std::size_t connection_end_index = 0U;
  bool slot_on_left = false;
  bool allow_disconnected_roi_fallback = false;
  ParkingSlot slot;
};

struct ParkingRoiGeometry {
  std::vector<common::math::Vec2d> corridor_polygon;
  std::vector<common::math::Vec2d> bridge_polygon;
  std::vector<common::math::Vec2d> attachment_polygon;
  std::vector<common::math::Vec2d> slot_polygon;
  std::vector<common::math::Vec2d> connector_slice;
  std::vector<common::math::Vec2d> outer_bridge_slice;
  std::vector<common::math::Vec2d> union_polygon;
  std::vector<std::vector<common::math::Vec2d>> boundary_segments;
  std::vector<double> xy_boundary;
  double area = 0.0;
  double aisle_width = 0.0;
  std::size_t connection_start_index = 0U;
  std::size_t connection_end_index = 0U;
  std::size_t slot_side_boundary_size = 0U;
  std::size_t connector_boundary_size = 0U;
  bool slot_on_left = false;
};

bool BuildParkingRoiGeometry(const ParkingRoiBuildInput& input,
                             ParkingRoiGeometry* geometry,
                             std::string* error);

bool ExpandParkingRoiToIncludeVehicleFootprint(
    const common::math::Vec2d& vehicle_position, double vehicle_heading,
    const apollo::common::VehicleParam& vehicle_param,
    double start_pose_buffer, double start_escape_distance,
    ParkingRoiGeometry* geometry, std::string* error,
    bool allow_disconnected_roi_fallback = false);

bool ApplyStartGoalParkingRoiTemplate(
    const common::math::Vec2d& vehicle_position, double vehicle_heading,
    const ParkingSlot& slot, const std::vector<double>& goal_pose,
    const apollo::common::VehicleParam& vehicle_param, double template_buffer,
    ParkingRoiGeometry* geometry, std::string* error);

std::vector<common::math::Vec2d> BuildParkingEnvelopePolygon(
    const ParkingRoiGeometry& geometry);

double ComputeParkingEnvelopeOpeningExtension(
    const std::vector<common::math::Vec2d>& slot_polygon,
    const common::math::Box2d& ego_box);

std::vector<common::math::Vec2d> BuildExpandedParkingEnvelopePolygon(
    const ParkingRoiGeometry& geometry, double opening_extension);

}  // namespace parking
}  // namespace planning
}  // namespace apollo
