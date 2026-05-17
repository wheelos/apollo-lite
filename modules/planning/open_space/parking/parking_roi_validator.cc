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

#include "modules/planning/open_space/parking/parking_roi_validator.h"

#include <limits>
#include <memory>

#include "modules/common/math/box2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/planning/open_space/coarse_trajectory_generator/node3d.h"

namespace apollo {
namespace planning {
namespace parking {

using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

double ComputeBoxBoundaryClearance(const apollo::common::math::Box2d& ego_box,
                                   const Polygon2d& free_space_polygon) {
  std::vector<Vec2d> corners;
  ego_box.GetAllCorners(&corners);
  double min_clearance = std::numeric_limits<double>::infinity();
  for (const auto& corner : corners) {
    min_clearance = std::min(min_clearance,
                             free_space_polygon.DistanceToBoundary(corner));
  }
  return min_clearance;
}

ParkingRoiValidator::ParkingRoiValidator(const OpenSpaceRoiDeciderConfig& config)
    : config_(config) {}

ParkingRoiValidationResult ParkingRoiValidator::ValidateGeometryOnly(
    const ParkingRoiGeometry& geometry,
    const common::math::Vec2d& vehicle_position) const {
  ParkingRoiValidationResult result;
  if (geometry.union_polygon.size() < 3U) {
    result.reason = "parking roi polygon is empty";
    return result;
  }

  Polygon2d polygon(geometry.union_polygon);
  result.area = polygon.area();
  result.vehicle_inside = polygon.IsPointIn(vehicle_position);

  if (result.area < config_.roi_min_area()) {
    result.reason = "parking roi area is too small";
    return result;
  }
  if (!result.vehicle_inside) {
    result.reason = "vehicle outside parking roi";
    return result;
  }
  if (geometry.aisle_width < common::math::kMathEpsilon) {
    result.reason = "parking roi aisle width is invalid";
    return result;
  }
  result.valid = true;
  return result;
}

ParkingRoiValidationResult ParkingRoiValidator::ValidateGeometryOnly(
    const ParkingRoiGeometry& geometry,
    const common::math::Vec2d& vehicle_position,
    const double vehicle_heading,
    const apollo::common::VehicleParam& vehicle_param) const {
  ParkingRoiValidationResult result =
      ValidateGeometryOnly(geometry, vehicle_position);
  if (!result.valid) {
    return result;
  }

  const Polygon2d polygon(geometry.union_polygon);
  const auto ego_box = Node3d::GetBoundingBox(
      vehicle_param, vehicle_position.x(), vehicle_position.y(),
      vehicle_heading);
  const Polygon2d ego_polygon(ego_box);
  result.vehicle_inside = polygon.Contains(ego_polygon);
  if (!result.vehicle_inside) {
    result.valid = false;
    result.reason = "ego vehicle box outside parking roi";
    return result;
  }
  return result;
}

ParkingRoiValidationResult ParkingRoiValidator::Validate(
    const ParkingRoiGeometry& geometry,
    const common::math::Vec2d& vehicle_position,
    const std::vector<double>& goal_pose,
    const apollo::common::VehicleParam& vehicle_param) const {
  ParkingRoiValidationResult result =
      ValidateGeometryOnly(geometry, vehicle_position);
  if (!result.valid) {
    return result;
  }
  Polygon2d polygon(geometry.union_polygon);
  if (goal_pose.size() < 3U) {
    result.valid = false;
    result.reason = "goal pose unavailable";
    return result;
  }

  const auto parking_envelope = BuildParkingEnvelopePolygon(geometry);
  std::unique_ptr<Polygon2d> parking_envelope_polygon;
  if (parking_envelope.size() < 3U) {
    result.valid = false;
    result.reason = "parking envelope unavailable";
    return result;
  }
  if (parking_envelope.size() >= 3U) {
    parking_envelope_polygon = std::make_unique<Polygon2d>(parking_envelope);
  }
  const auto goal_box =
      Node3d::GetBoundingBox(vehicle_param, goal_pose[0], goal_pose[1],
                             goal_pose[2]);
  const Polygon2d goal_polygon(goal_box);
  result.goal_inside_roi = polygon.Contains(goal_polygon);
  std::unique_ptr<Polygon2d> expanded_envelope_polygon;
  const auto get_expanded_envelope_polygon = [&]() -> const Polygon2d* {
    if (geometry.slot_polygon.empty()) {
      return nullptr;
    }
    if (expanded_envelope_polygon == nullptr) {
      const double required_extension =
          ComputeParkingEnvelopeOpeningExtension(geometry.slot_polygon, goal_box);
      if (required_extension <= common::math::kMathEpsilon) {
        return parking_envelope_polygon.get();
      }
      const auto expanded_envelope = BuildExpandedParkingEnvelopePolygon(
          geometry, required_extension + 1e-3);
      if (expanded_envelope.size() < 3U) {
        return parking_envelope_polygon.get();
      }
      expanded_envelope_polygon =
          std::make_unique<Polygon2d>(expanded_envelope);
    }
    return expanded_envelope_polygon.get();
  };

  const Polygon2d* accepted_goal_boundary = nullptr;
  if (parking_envelope_polygon != nullptr &&
      parking_envelope_polygon->Contains(goal_polygon)) {
    accepted_goal_boundary = parking_envelope_polygon.get();
  } else if (const Polygon2d* expanded_polygon =
                 get_expanded_envelope_polygon();
             expanded_polygon != nullptr &&
             expanded_polygon->Contains(goal_polygon)) {
    accepted_goal_boundary = expanded_polygon;
  }
  result.goal_inside_envelope = accepted_goal_boundary != nullptr;
  if (accepted_goal_boundary != nullptr) {
    result.goal_clearance =
        ComputeBoxBoundaryClearance(goal_box, *accepted_goal_boundary);
  }

  result.goal_inside = result.goal_inside_roi;
  if (!result.goal_inside_roi) {
    result.goal_inside = false;
    result.valid = false;
    result.reason = "goal vehicle box outside parking roi";
    return result;
  }
  if (!result.goal_inside_envelope) {
    result.goal_clearance = ComputeBoxBoundaryClearance(goal_box, polygon);
  }
  result.valid = true;
  return result;
}

}  // namespace parking
}  // namespace planning
}  // namespace apollo
