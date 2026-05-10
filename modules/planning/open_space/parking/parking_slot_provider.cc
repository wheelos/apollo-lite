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

#include "modules/planning/open_space/parking/parking_slot_provider.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "modules/common/math/math_utils.h"

namespace apollo {
namespace planning {
namespace parking {

using apollo::common::math::Vec2d;

namespace {

double InferHeadingFromPolygon(const std::vector<Vec2d>& polygon_points) {
  double longest_edge_length = -1.0;
  double inferred_heading = 0.0;
  for (size_t index = 0; index < polygon_points.size(); ++index) {
    const size_t next_index = (index + 1U) % polygon_points.size();
    const Vec2d edge = polygon_points[next_index] - polygon_points[index];
    const double edge_length = edge.Length();
    if (edge_length > longest_edge_length) {
      longest_edge_length = edge_length;
      inferred_heading = edge.Angle();
    }
  }
  return inferred_heading;
}

double AverageDistance(const Vec2d& first_a, const Vec2d& first_b,
                       const Vec2d& second_a, const Vec2d& second_b) {
  return 0.5 * ((first_a - first_b).Length() + (second_a - second_b).Length());
}

bool BuildSemanticCornersFromEntranceEdge(const std::vector<Vec2d>& polygon_points,
                                          const hdmap::Path& nearby_path,
                                          ParkingSlotCorners* corners,
                                          std::string* error) {
  if (corners == nullptr) {
    if (error != nullptr) {
      *error = "parking slot corner output is null";
    }
    return false;
  }

  int entrance_index0 = -1;
  int entrance_index1 = -1;
  double min_lateral_distance = std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < polygon_points.size(); ++index) {
    const size_t next_index = (index + 1U) % polygon_points.size();
    const Vec2d midpoint = (polygon_points[index] + polygon_points[next_index]) * 0.5;
    double edge_s = 0.0;
    double edge_l = 0.0;
    if (!nearby_path.GetProjection(midpoint, &edge_s, &edge_l)) {
      continue;
    }
    const double lateral_distance = std::fabs(edge_l);
    if (lateral_distance < min_lateral_distance) {
      min_lateral_distance = lateral_distance;
      entrance_index0 = static_cast<int>(index);
      entrance_index1 = static_cast<int>(next_index);
    }
  }

  if (entrance_index0 < 0 || entrance_index1 < 0) {
    if (error != nullptr) {
      *error = "failed to identify parking slot entrance edge";
    }
    return false;
  }

  std::array<int, 2> entrance_indices = {entrance_index0, entrance_index1};
  std::array<int, 2> rear_indices = {-1, -1};
  int rear_count = 0;
  for (int index = 0; index < static_cast<int>(polygon_points.size()); ++index) {
    if (index == entrance_index0 || index == entrance_index1) {
      continue;
    }
    rear_indices[rear_count++] = index;
  }
  if (rear_count != 2) {
    if (error != nullptr) {
      *error = "parking slot polygon is not a quadrilateral";
    }
    return false;
  }

  auto sort_indices_by_path_s = [&nearby_path, &polygon_points](std::array<int, 2>* indices) {
    std::sort(indices->begin(), indices->end(),
              [&nearby_path, &polygon_points](int lhs, int rhs) {
                double lhs_s = 0.0;
                double lhs_l = 0.0;
                double rhs_s = 0.0;
                double rhs_l = 0.0;
                const bool lhs_ok =
                    nearby_path.GetProjection(polygon_points[lhs], &lhs_s, &lhs_l);
                const bool rhs_ok =
                    nearby_path.GetProjection(polygon_points[rhs], &rhs_s, &rhs_l);
                if (lhs_ok != rhs_ok) {
                  return lhs_ok;
                }
                if (!lhs_ok) {
                  return lhs < rhs;
                }
                if (std::fabs(lhs_s - rhs_s) > common::math::kMathEpsilon) {
                  return lhs_s < rhs_s;
                }
                return lhs < rhs;
              });
  };

  sort_indices_by_path_s(&entrance_indices);
  sort_indices_by_path_s(&rear_indices);

  corners->left_top = polygon_points[entrance_indices[0]];
  corners->right_top = polygon_points[entrance_indices[1]];
  corners->left_down = polygon_points[rear_indices[0]];
  corners->right_down = polygon_points[rear_indices[1]];
  return true;
}

bool BuildParkingSlotFromSemanticCorners(
    const std::vector<Vec2d>& polygon_points, const std::string& slot_id,
    const double slot_heading, const double lane_heading, const double slot_s,
    const double slot_l, const ParkingSlotCorners& corners,
    ParkingSlot* parking_slot, std::string* error) {
  if (parking_slot == nullptr) {
    if (error != nullptr) {
      *error = "parking slot output is null";
    }
    return false;
  }

  ParkingSlot slot;
  slot.id = slot_id;
  slot.raw_heading = common::math::NormalizeAngle(slot_heading);
  slot.lane_heading = common::math::NormalizeAngle(lane_heading);
  slot.lane_s = slot_s;
  slot.lane_l = slot_l;
  slot.on_left_lane_side =
      (std::fabs(slot_l) < common::math::kMathEpsilon)
          ? common::math::NormalizeAngle(slot_heading - lane_heading) >= 0.0
          : slot_l >= 0.0;
  slot.type = InferParkingSlotType(slot.raw_heading, slot.lane_heading);
  slot.corners = corners;
  slot.polygon = polygon_points;

  slot.center = Vec2d(0.0, 0.0);
  for (const auto& point : polygon_points) {
    slot.center += point;
  }
  slot.center /= static_cast<double>(polygon_points.size());
  slot.opening_center = (slot.corners.left_top + slot.corners.right_top) * 0.5;
  slot.rear_center = (slot.corners.left_down + slot.corners.right_down) * 0.5;
  slot.width = AverageDistance(slot.corners.left_top, slot.corners.right_top,
                               slot.corners.left_down,
                               slot.corners.right_down);
  slot.depth = AverageDistance(slot.corners.left_top, slot.corners.left_down,
                               slot.corners.right_top,
                               slot.corners.right_down);

  Vec2d primary_axis;
  if (slot.type == ParkingSlotType::kParallel) {
    primary_axis = slot.corners.right_top - slot.corners.left_top;
  } else {
    primary_axis = slot.rear_center - slot.opening_center;
  }
  if (primary_axis.Length() < common::math::kMathEpsilon) {
    primary_axis = Vec2d::CreateUnitVec2d(slot.raw_heading);
  } else {
    primary_axis.Normalize();
  }
  const Vec2d heading_direction = Vec2d::CreateUnitVec2d(slot.raw_heading);
  if (primary_axis.InnerProd(heading_direction) < 0.0) {
    primary_axis = primary_axis * -1.0;
  }
  slot.heading = primary_axis.Angle();

  *parking_slot = std::move(slot);
  return true;
}

}  // namespace

bool ParkingSlotProvider::BuildFromMap(
    const hdmap::ParkingSpaceInfoConstPtr& parking_space_info,
    const hdmap::Path& nearby_path, ParkingSlot* parking_slot,
    std::string* error) const {
  if (parking_space_info == nullptr) {
    if (error != nullptr) {
      *error = "target parking slot is null";
    }
    return false;
  }

  const auto& polygon_points = parking_space_info->polygon().points();
  if (polygon_points.size() != 4U) {
    if (error != nullptr) {
      *error = "parking slot polygon must contain exactly four points";
    }
    return false;
  }

  Vec2d center(0.0, 0.0);
  std::vector<Vec2d> points;
  points.reserve(polygon_points.size());
  for (const auto& point : polygon_points) {
    points.emplace_back(point);
    center += point;
  }
  center /= static_cast<double>(polygon_points.size());

  double slot_s = 0.0;
  double slot_l = 0.0;
  if (!nearby_path.GetProjection(center, &slot_s, &slot_l)) {
    if (error != nullptr) {
      *error = "failed to project parking slot center on nearby path";
    }
    return false;
  }

  double lane_heading = 0.0;
  if (!nearby_path.GetHeadingAlongPath(center, &lane_heading)) {
    lane_heading = nearby_path.GetSmoothPoint(slot_s).heading();
  }

  const auto& parking_space = parking_space_info->parking_space();
  const double slot_heading = parking_space.has_heading()
                                  ? parking_space.heading()
                                  : InferHeadingFromPolygon(points);
  ParkingSlotCorners semantic_corners;
  std::string semantic_error;
  if (BuildSemanticCornersFromEntranceEdge(points, nearby_path, &semantic_corners,
                                           &semantic_error) &&
      BuildParkingSlotFromSemanticCorners(points, parking_space.id().id(),
                                         slot_heading, lane_heading, slot_s,
                                         slot_l, semantic_corners, parking_slot,
                                         &semantic_error)) {
    return true;
  }

  if (!NormalizeParkingSlot(points, parking_space.id().id(), slot_heading,
                            lane_heading, slot_l, parking_slot, error)) {
    if (error != nullptr && !semantic_error.empty()) {
      *error = semantic_error + "; " + *error;
    }
    return false;
  }
  parking_slot->lane_s = slot_s;
  return true;
}

}  // namespace parking
}  // namespace planning
}  // namespace apollo
