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

#include "modules/common/math/box2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/planning/open_space/coarse_trajectory_generator/node3d.h"

namespace apollo {
namespace planning {
namespace parking {

using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

namespace {

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

}  // namespace

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
  if (parking_envelope.size() < 3U) {
    result.valid = false;
    result.reason = "parking envelope unavailable";
    return result;
  }
  Polygon2d parking_envelope_polygon(parking_envelope);
  const auto goal_box =
      Node3d::GetBoundingBox(vehicle_param, goal_pose[0], goal_pose[1],
                             goal_pose[2]);
  const Polygon2d goal_polygon(goal_box);
  result.goal_inside = parking_envelope_polygon.Contains(goal_polygon);
  result.goal_clearance =
      ComputeBoxBoundaryClearance(goal_box, parking_envelope_polygon);

  if (!result.goal_inside && !geometry.slot_polygon.empty() &&
      polygon.Contains(goal_polygon)) {
    const double required_extension =
        ComputeParkingEnvelopeOpeningExtension(geometry.slot_polygon, goal_box);
    if (required_extension > common::math::kMathEpsilon) {
      const auto expanded_envelope = BuildExpandedParkingEnvelopePolygon(
          geometry, required_extension + 1e-3);
      if (expanded_envelope.size() >= 3U) {
        const Polygon2d expanded_polygon(expanded_envelope);
        if (expanded_polygon.Contains(goal_polygon)) {
          result.goal_inside = true;
          result.goal_clearance =
              ComputeBoxBoundaryClearance(goal_box, expanded_polygon);
        }
      }
    }
  }

  if (!result.goal_inside) {
    result.valid = false;
    result.reason = "goal vehicle box outside parking envelope";
    return result;
  }
  result.valid = true;
  return result;
}

}  // namespace parking
}  // namespace planning
}  // namespace apollo
