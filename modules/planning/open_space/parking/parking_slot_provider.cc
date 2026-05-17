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

double NormalizeAcuteAngle(double angle) {
  double normalized = std::fabs(common::math::NormalizeAngle(angle));
  if (normalized > M_PI_2) {
    normalized = M_PI - normalized;
  }
  return normalized;
}

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

bool IsPointInPolygon(const std::vector<Vec2d>& polygon_points,
                      const Vec2d& point) {
  bool inside = false;
  for (size_t i = 0, j = polygon_points.size() - 1U;
       i < polygon_points.size(); j = i++) {
    const Vec2d& pi = polygon_points[i];
    const Vec2d& pj = polygon_points[j];
    if ((pi.y() > point.y()) != (pj.y() > point.y()) &&
        point.x() < (pj.x() - pi.x()) * (point.y() - pi.y()) /
                            (pj.y() - pi.y()) +
                        pi.x()) {
      inside = !inside;
    }
  }
  return inside;
}

bool BuildSemanticCornersFromEntranceEdge(const std::vector<Vec2d>& polygon_points,
                                           const hdmap::Path& nearby_path,
                                           const double slot_heading,
                                           const Vec2d& reference_point,
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
  bool found_lane_parallel_edge = false;
  double best_heading_alignment = std::numeric_limits<double>::infinity();
  double min_reference_distance = std::numeric_limits<double>::infinity();
  double best_direction_alignment = std::numeric_limits<double>::infinity();
  const bool reference_inside_slot =
      polygon_points.size() >= 3U &&
      IsPointInPolygon(polygon_points, reference_point);
  double reference_s = 0.0;
  double reference_l = 0.0;
  double reference_lane_heading = 0.0;
  if (nearby_path.GetProjection(reference_point, &reference_s, &reference_l)) {
    if (!nearby_path.GetHeadingAlongPath(reference_point,
                                         &reference_lane_heading)) {
      reference_lane_heading =
          nearby_path.GetSmoothPoint(reference_s).heading();
    }
  }
  const double reference_slot_lane_angle =
      NormalizeAcuteAngle(slot_heading - reference_lane_heading);
  const bool use_slot_heading_for_inside_reference =
      reference_inside_slot && reference_slot_lane_angle > M_PI / 6.0 &&
      reference_slot_lane_angle < 5.0 * M_PI / 12.0;
  for (size_t index = 0; index < polygon_points.size(); ++index) {
    const size_t next_index = (index + 1U) % polygon_points.size();
    const Vec2d midpoint = (polygon_points[index] + polygon_points[next_index]) * 0.5;
    double edge_s = 0.0;
    double edge_l = 0.0;
    if (!nearby_path.GetProjection(midpoint, &edge_s, &edge_l)) {
      continue;
    }
    double lane_heading = 0.0;
    if (!nearby_path.GetHeadingAlongPath(midpoint, &lane_heading)) {
      lane_heading = nearby_path.GetSmoothPoint(edge_s).heading();
    }
    const double heading_alignment = NormalizeAcuteAngle(
        (polygon_points[next_index] - polygon_points[index]).Angle() -
        lane_heading);
    const double reference_distance = (midpoint - reference_point).Length();
    double direction_alignment = std::numeric_limits<double>::infinity();
    if (polygon_points.size() == 4U) {
      const size_t opposite_index = (index + 2U) % polygon_points.size();
      const size_t opposite_next_index =
          (opposite_index + 1U) % polygon_points.size();
      const Vec2d opposite_midpoint =
          (polygon_points[opposite_index] +
           polygon_points[opposite_next_index]) *
          0.5;
      direction_alignment = std::fabs(common::math::NormalizeAngle(
          (opposite_midpoint - midpoint).Angle() - slot_heading));
    }
    const bool is_lane_parallel_edge = heading_alignment <= M_PI / 4.0;
    const double primary_score =
        use_slot_heading_for_inside_reference ? direction_alignment
                                              : reference_distance;
    const double best_primary_score =
        use_slot_heading_for_inside_reference ? best_direction_alignment
                                              : min_reference_distance;
    const double secondary_score =
        use_slot_heading_for_inside_reference ? reference_distance
                                              : heading_alignment;
    const double best_secondary_score =
        use_slot_heading_for_inside_reference ? min_reference_distance
                                              : best_heading_alignment;
    if ((!found_lane_parallel_edge && is_lane_parallel_edge) ||
        (is_lane_parallel_edge == found_lane_parallel_edge &&
         (primary_score < best_primary_score - common::math::kMathEpsilon ||
          (std::fabs(primary_score - best_primary_score) <=
               common::math::kMathEpsilon &&
           secondary_score + common::math::kMathEpsilon <
               best_secondary_score)))) {
      found_lane_parallel_edge = is_lane_parallel_edge;
      best_heading_alignment = heading_alignment;
      min_reference_distance = reference_distance;
      best_direction_alignment = direction_alignment;
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
  slot.heading = primary_axis.Angle();

  *parking_slot = std::move(slot);
  return true;
}

}  // namespace

bool ParkingSlotProvider::BuildFromMap(
    const hdmap::ParkingSpaceInfoConstPtr& parking_space_info,
    const hdmap::Path& nearby_path, const Vec2d& reference_point,
    ParkingSlot* parking_slot, std::string* error) const {
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
  if (BuildSemanticCornersFromEntranceEdge(points, nearby_path, slot_heading,
                                           reference_point, &semantic_corners,
                                           &semantic_error) &&
      BuildParkingSlotFromSemanticCorners(points, parking_space.id().id(),
                                          slot_heading, lane_heading, slot_s,
                                          slot_l, semantic_corners, parking_slot,
                                          &semantic_error)) {
    AINFO << "Built parking slot " << parking_slot->id
          << " type=" << ParkingSlotTypeName(parking_slot->type)
          << " lane_heading=" << parking_slot->lane_heading
          << " raw_heading=" << parking_slot->raw_heading
          << " normalized_heading=" << parking_slot->heading
          << " opening_center=(" << parking_slot->opening_center.x() << ", "
          << parking_slot->opening_center.y() << ")"
          << " rear_center=(" << parking_slot->rear_center.x() << ", "
          << parking_slot->rear_center.y() << ")"
          << " on_left_lane_side=" << parking_slot->on_left_lane_side;
    return true;
  }

  if (error != nullptr) {
    *error = semantic_error.empty()
                 ? "failed to normalize parking slot from ego-side entrance"
                 : semantic_error;
  }
  return false;
}

bool ParkingSlotProvider::BuildFromPerception(
    const PerceptionParkingSlotInput& input, ParkingSlot* parking_slot,
    std::string* error) const {
  if (parking_slot == nullptr) {
    if (error != nullptr) {
      *error = "parking slot output is null";
    }
    return false;
  }

  std::vector<Vec2d> points = input.polygon;
  if (points.empty()) {
    const auto corners = input.corners.AsArray();
    points.assign(corners.begin(), corners.end());
  }
  if (points.size() != 4U) {
    if (error != nullptr) {
      *error = "perception parking slot must provide exactly four corners";
    }
    return false;
  }

  const double slot_heading =
      input.has_slot_heading
          ? input.slot_heading
          : (input.corners.left_down + input.corners.right_down -
             input.corners.left_top - input.corners.right_top)
                .Angle();
  const double lane_heading =
      input.has_lane_context ? input.lane_heading : slot_heading + M_PI_2;
  const double lane_l = input.has_lane_context ? input.lane_l : 0.0;

  if (!BuildParkingSlotFromSemanticCorners(
          points, input.id, slot_heading, lane_heading, input.lane_s, lane_l,
          input.corners, parking_slot, error)) {
    return false;
  }
  return true;
}

}  // namespace parking
}  // namespace planning
}  // namespace apollo
