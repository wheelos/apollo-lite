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

#include "modules/planning/open_space/parking/parking_roi_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <boost/geometry/algorithms/append.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "modules/common/math/box2d.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/polygon2d.h"

namespace apollo {
namespace planning {
namespace parking {

using apollo::common::math::Box2d;
using apollo::common::math::LineSegment2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

namespace bg = boost::geometry;

namespace {

using BgPoint = bg::model::d2::point_xy<double>;
using BgPolygon = bg::model::polygon<BgPoint, false, true>;

bool IsValidPolylineRange(const std::vector<Vec2d>& boundary,
                          const std::size_t start_index,
                          const std::size_t end_index) {
  return !boundary.empty() && start_index <= end_index &&
         end_index < boundary.size();
}

bool NearlyEqual(const Vec2d& lhs, const Vec2d& rhs, const double tolerance) {
  return lhs.DistanceTo(rhs) <= tolerance;
}

double CrossProduct(const Vec2d& start, const Vec2d& end, const Vec2d& point) {
  return (end.x() - start.x()) * (point.y() - start.y()) -
         (end.y() - start.y()) * (point.x() - start.x());
}

bool ProperlyIntersects(const Vec2d& a, const Vec2d& b, const Vec2d& c,
                        const Vec2d& d) {
  const double ab_c = CrossProduct(a, b, c);
  const double ab_d = CrossProduct(a, b, d);
  const double cd_a = CrossProduct(c, d, a);
  const double cd_b = CrossProduct(c, d, b);
  return ((ab_c > common::math::kMathEpsilon &&
           ab_d < -common::math::kMathEpsilon) ||
          (ab_c < -common::math::kMathEpsilon &&
           ab_d > common::math::kMathEpsilon)) &&
         ((cd_a > common::math::kMathEpsilon &&
           cd_b < -common::math::kMathEpsilon) ||
          (cd_a < -common::math::kMathEpsilon &&
           cd_b > common::math::kMathEpsilon));
}

std::vector<Vec2d> NormalizePolygonVertices(
    const std::vector<Vec2d>& polygon_points) {
  std::vector<Vec2d> normalized;
  normalized.reserve(polygon_points.size());
  for (const auto& point : polygon_points) {
    if (!normalized.empty() && NearlyEqual(normalized.back(), point, 1e-6)) {
      continue;
    }
    normalized.push_back(point);
  }
  while (normalized.size() > 1U &&
         NearlyEqual(normalized.front(), normalized.back(), 1e-6)) {
    normalized.pop_back();
  }
  return normalized;
}

std::vector<Vec2d> RepairLocalPolygonSelfIntersections(
    std::vector<Vec2d> polygon_points) {
  polygon_points = NormalizePolygonVertices(polygon_points);
  bool changed = true;
  while (changed && polygon_points.size() >= 4U) {
    changed = false;
    const std::size_t point_count = polygon_points.size();
    for (std::size_t index = 0; index < point_count; ++index) {
      const std::size_t a = index;
      const std::size_t b = (index + 1U) % point_count;
      const std::size_t c = (index + 2U) % point_count;
      const std::size_t d = (index + 3U) % point_count;
      if (!ProperlyIntersects(polygon_points[a], polygon_points[b],
                              polygon_points[c], polygon_points[d])) {
        continue;
      }
      const double bc_length = polygon_points[b].DistanceTo(polygon_points[c]);
      const double cd_length = polygon_points[c].DistanceTo(polygon_points[d]);
      const std::size_t erase_index = bc_length <= cd_length ? c : b;
      polygon_points.erase(polygon_points.begin() + erase_index);
      polygon_points = NormalizePolygonVertices(polygon_points);
      changed = true;
      break;
    }
  }
  return polygon_points;
}

struct PolylineProjection {
  std::size_t segment_index = 0U;
  double ratio = 0.0;
  Vec2d point;

  double ordered_position() const {
    return static_cast<double>(segment_index) + ratio;
  }
};

bool ProjectPointToPolyline(const Vec2d& point, const std::vector<Vec2d>& polyline,
                            const std::size_t start_index,
                            const std::size_t end_index,
                            PolylineProjection* projection) {
  if (projection == nullptr || polyline.size() < 2U) {
    return false;
  }
  const std::size_t first_segment = std::min(start_index, polyline.size() - 2U);
  const std::size_t last_point_index =
      std::min(end_index, polyline.size() - 1U);
  if (last_point_index == 0U || first_segment >= last_point_index) {
    return false;
  }
  const std::size_t last_segment = last_point_index - 1U;

  bool found_projection = false;
  double min_distance_sqr = std::numeric_limits<double>::infinity();
  for (std::size_t index = first_segment; index <= last_segment; ++index) {
    const Vec2d segment_start = polyline[index];
    const Vec2d segment_end = polyline[index + 1U];
    const Vec2d segment = segment_end - segment_start;
    const double segment_length_sqr = segment.LengthSquare();
    if (segment_length_sqr < common::math::kMathEpsilon) {
      continue;
    }
    const double ratio =
        std::max(0.0, std::min(1.0, (point - segment_start).InnerProd(segment) /
                                        segment_length_sqr));
    const Vec2d projected_point = segment_start + segment * ratio;
    const double distance_sqr = (projected_point - point).LengthSquare();
    if (!found_projection || distance_sqr < min_distance_sqr) {
      found_projection = true;
      min_distance_sqr = distance_sqr;
      projection->segment_index = index;
      projection->ratio = ratio;
      projection->point = projected_point;
    }
  }
  return found_projection;
}

BgPolygon ToBoostPolygon(const std::vector<Vec2d>& polygon_points) {
  BgPolygon polygon;
  for (const auto& point : NormalizePolygonVertices(polygon_points)) {
    bg::append(polygon.outer(), BgPoint(point.x(), point.y()));
  }
  bg::correct(polygon);
  return polygon;
}

std::vector<Vec2d> FromBoostOuterRing(const BgPolygon& polygon) {
  std::vector<Vec2d> points;
  for (const auto& point : polygon.outer()) {
    points.emplace_back(point.x(), point.y());
  }
  if (!points.empty()) {
    points.pop_back();
  }
  return NormalizePolygonVertices(points);
}

bool UnionBoostPolygons(const BgPolygon& left_polygon,
                        const BgPolygon& right_polygon, BgPolygon* result,
                        std::string* error,
                        const std::string& disconnected_error) {
  if (result == nullptr) {
    if (error != nullptr) {
      *error = "parking roi boost polygon output is null";
    }
    return false;
  }
  std::vector<BgPolygon> union_result;
  bg::union_(left_polygon, right_polygon, union_result);
  if (union_result.size() != 1U) {
    if (error != nullptr) {
      *error = disconnected_error;
    }
    return false;
  }
  *result = std::move(union_result.front());
  return true;
}

bool UnionPolygons(const std::vector<Vec2d>& left,
                   const std::vector<Vec2d>& right,
                   std::vector<Vec2d>* result, std::string* error) {
  const BgPolygon left_polygon = ToBoostPolygon(left);
  const BgPolygon right_polygon = ToBoostPolygon(right);
  BgPolygon union_polygon;
  if (!UnionBoostPolygons(left_polygon, right_polygon, &union_polygon, error,
                          "parking roi polygons are disconnected after union")) {
    return false;
  }
  *result = FromBoostOuterRing(union_polygon);
  return true;
}

std::vector<Vec2d> BuildCorridorPolygon(const std::vector<Vec2d>& left_boundary,
                                        const std::vector<Vec2d>& right_boundary) {
  std::vector<Vec2d> corridor;
  corridor.reserve(left_boundary.size() + right_boundary.size());
  corridor.insert(corridor.end(), right_boundary.begin(), right_boundary.end());
  corridor.insert(corridor.end(), left_boundary.rbegin(), left_boundary.rend());
  return corridor;
}

std::vector<Vec2d> BoxPolygon(Box2d box, const double buffer) {
  if (buffer > common::math::kMathEpsilon) {
    box.LongitudinalExtend(2.0 * buffer);
    box.LateralExtend(2.0 * buffer);
  }
  std::vector<Vec2d> polygon;
  box.GetAllCorners(&polygon);
  return NormalizePolygonVertices(polygon);
}

bool UnionPatchIntoGeometry(const std::vector<Vec2d>& patch_polygon,
                            ParkingRoiGeometry* geometry, std::string* error) {
  if (geometry == nullptr || patch_polygon.size() < 3U) {
    return geometry != nullptr;
  }
  std::vector<Vec2d> expanded_union_polygon;
  if (!UnionPolygons(geometry->union_polygon, patch_polygon,
                     &expanded_union_polygon, error)) {
    return false;
  }
  geometry->union_polygon = std::move(expanded_union_polygon);
  return true;
}

bool InferOpeningEdge(const std::vector<Vec2d>& slot_polygon,
                     std::size_t* opening_edge_index,
                     Vec2d* inward_normal) {
  if (opening_edge_index == nullptr || inward_normal == nullptr ||
      slot_polygon.size() < 4U) {
    return false;
  }
  double min_midpoint_norm = std::numeric_limits<double>::infinity();
  std::size_t best_index = 0U;
  Vec2d best_opening_midpoint;
  Vec2d best_opposite_midpoint;
  for (std::size_t index = 0; index < slot_polygon.size(); ++index) {
    const Vec2d opening_midpoint =
        (slot_polygon[index] + slot_polygon[(index + 1U) % slot_polygon.size()]) *
        0.5;
    const Vec2d opposite_midpoint =
        (slot_polygon[(index + 2U) % slot_polygon.size()] +
         slot_polygon[(index + 3U) % slot_polygon.size()]) *
        0.5;
    const double midpoint_norm = opening_midpoint.Length();
    if (midpoint_norm < min_midpoint_norm) {
      min_midpoint_norm = midpoint_norm;
      best_index = index;
      best_opening_midpoint = opening_midpoint;
      best_opposite_midpoint = opposite_midpoint;
    }
  }
  const Vec2d opening_start = slot_polygon[best_index];
  const Vec2d opening_end = slot_polygon[(best_index + 1U) % slot_polygon.size()];
  const Vec2d opening_edge = opening_end - opening_start;
  if (opening_edge.Length() <= common::math::kMathEpsilon) {
    return false;
  }
  Vec2d normal_left(-opening_edge.y(), opening_edge.x());
  normal_left.Normalize();
  Vec2d normal_right(opening_edge.y(), -opening_edge.x());
  normal_right.Normalize();
  const Vec2d depth = best_opposite_midpoint - best_opening_midpoint;
  if (depth.Length() <= common::math::kMathEpsilon ||
      std::max(normal_left.InnerProd(depth), normal_right.InnerProd(depth)) <=
          common::math::kMathEpsilon) {
    return false;
  }
  *opening_edge_index = best_index;
  *inward_normal = normal_left.InnerProd(depth) >= normal_right.InnerProd(depth)
                       ? normal_left
                       : normal_right;
  return true;
}

std::vector<Vec2d> ExtractPolylineSlice(
    const std::vector<Vec2d>& polyline, const PolylineProjection& start_projection,
    const PolylineProjection& end_projection) {
  std::vector<Vec2d> slice;
  if (polyline.size() < 2U) {
    return slice;
  }
  slice.push_back(start_projection.point);
  for (std::size_t index = start_projection.segment_index + 1U;
       index <= end_projection.segment_index && index < polyline.size();
       ++index) {
    if (!NearlyEqual(polyline[index], slice.back(), 1e-6)) {
      slice.push_back(polyline[index]);
    }
  }
  if (slice.empty() || !NearlyEqual(end_projection.point, slice.back(), 1e-6)) {
    slice.push_back(end_projection.point);
  }
  return slice;
}

std::vector<Vec2d> BuildBridgePolygon(std::vector<Vec2d> boundary_slice,
                                      const ParkingSlot& slot) {
  std::vector<Vec2d> polygon;
  if (boundary_slice.empty()) {
    return polygon;
  }
  const double forward_cost =
      boundary_slice.front().DistanceTo(slot.corners.left_top) +
      boundary_slice.back().DistanceTo(slot.corners.right_top);
  const double reverse_cost =
      boundary_slice.front().DistanceTo(slot.corners.right_top) +
      boundary_slice.back().DistanceTo(slot.corners.left_top);
  if (reverse_cost + common::math::kMathEpsilon < forward_cost) {
    std::reverse(boundary_slice.begin(), boundary_slice.end());
  }
  polygon.reserve(boundary_slice.size() + 2U);
  polygon.insert(polygon.end(), boundary_slice.begin(), boundary_slice.end());
  polygon.push_back(slot.corners.right_top);
  polygon.push_back(slot.corners.left_top);
  return polygon;
}

bool BridgeBoundaryMatchesOpening(const std::vector<Vec2d>& boundary_slice,
                                  const ParkingSlot& slot) {
  if (boundary_slice.size() < 2U) {
    return true;
  }
  const double tolerance = 5e-2;
  std::vector<Vec2d> ordered_slice = boundary_slice;
  const bool forward_match =
      NearlyEqual(ordered_slice.front(), slot.corners.left_top, tolerance) &&
      NearlyEqual(ordered_slice.back(), slot.corners.right_top, tolerance);
  const bool reverse_match =
      NearlyEqual(ordered_slice.front(), slot.corners.right_top, tolerance) &&
      NearlyEqual(ordered_slice.back(), slot.corners.left_top, tolerance);
  if (!forward_match && !reverse_match) {
    return false;
  }
  if (reverse_match) {
    std::reverse(ordered_slice.begin(), ordered_slice.end());
  }

  const LineSegment2d opening_segment(slot.corners.left_top,
                                      slot.corners.right_top);
  for (const auto& point : ordered_slice) {
    if (opening_segment.DistanceTo(point) > tolerance) {
      return false;
    }
  }
  return true;
}

double ComputeAisleWidth(const ParkingSlot& slot,
                         const std::vector<Vec2d>& opposite_boundary) {
  if (opposite_boundary.size() < 2U) {
    return 0.0;
  }
  double min_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index + 1U < opposite_boundary.size(); ++index) {
    const LineSegment2d segment(opposite_boundary[index],
                                opposite_boundary[index + 1U]);
    min_distance = std::min(min_distance,
                            segment.DistanceTo(slot.opening_center));
  }
  return min_distance;
}

bool BoxOverlapsBoundarySegments(
    const Box2d& ego_box,
    const std::vector<std::vector<Vec2d>>& boundary_segments) {
  for (const auto& obstacle_vertices : boundary_segments) {
    if (obstacle_vertices.size() < 2U) {
      continue;
    }
    for (std::size_t index = 1; index < obstacle_vertices.size(); ++index) {
      const LineSegment2d line_segment(obstacle_vertices[index - 1U],
                                       obstacle_vertices[index]);
      if (ego_box.HasOverlap(line_segment)) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

bool BuildParkingRoiGeometry(const ParkingRoiBuildInput& input,
                             ParkingRoiGeometry* geometry,
                             std::string* error) {
  if (geometry == nullptr) {
    if (error != nullptr) {
      *error = "parking roi geometry output is null";
    }
    return false;
  }
  if (input.left_boundary.size() < 2U || input.right_boundary.size() < 2U) {
    if (error != nullptr) {
      *error = "parking roi boundaries are too short";
    }
    return false;
  }

  const auto& slot_side_outer_boundary =
      input.slot_on_left ? input.left_boundary : input.right_boundary;
  const bool has_connector_boundary =
      input.slot_side_connector_boundary.size() >= 2U;
  const auto& slot_side_bridge_boundary =
      has_connector_boundary ? input.slot_side_connector_boundary
                             : slot_side_outer_boundary;
  if (!IsValidPolylineRange(slot_side_bridge_boundary, input.connection_start_index,
                            input.connection_end_index)) {
    if (error != nullptr) {
      *error = "parking roi connection indices are invalid";
    }
    return false;
  }

  ParkingRoiGeometry roi_geometry;
  roi_geometry.slot_on_left = input.slot_on_left;
  const auto& corridor_left_boundary =
      input.slot_on_left && has_connector_boundary ? input.slot_side_connector_boundary
                                                   : input.left_boundary;
  const auto& corridor_right_boundary =
      !input.slot_on_left && has_connector_boundary ? input.slot_side_connector_boundary
                                                    : input.right_boundary;
  roi_geometry.corridor_polygon = RepairLocalPolygonSelfIntersections(
      BuildCorridorPolygon(corridor_left_boundary, corridor_right_boundary));
  const auto slot_vertices = input.slot.Vertices();
  roi_geometry.slot_polygon.assign(slot_vertices.begin(), slot_vertices.end());
  roi_geometry.slot_polygon =
      NormalizePolygonVertices(roi_geometry.slot_polygon);

  PolylineProjection left_outer_projection;
  PolylineProjection right_outer_projection;
  if (!ProjectPointToPolyline(input.slot.corners.left_top, slot_side_outer_boundary,
                              0U, slot_side_outer_boundary.size() - 1U,
                              &left_outer_projection) ||
      !ProjectPointToPolyline(input.slot.corners.right_top,
                              slot_side_outer_boundary, 0U,
                              slot_side_outer_boundary.size() - 1U,
                              &right_outer_projection)) {
    if (error != nullptr) {
      *error = "failed to project parking slot opening onto outer road boundary";
    }
    return false;
  }
  if (right_outer_projection.ordered_position() <
      left_outer_projection.ordered_position()) {
    std::swap(left_outer_projection, right_outer_projection);
  }
  roi_geometry.outer_bridge_slice = ExtractPolylineSlice(
      slot_side_outer_boundary, left_outer_projection, right_outer_projection);
  roi_geometry.attachment_polygon = RepairLocalPolygonSelfIntersections(
      BuildBridgePolygon(roi_geometry.outer_bridge_slice, input.slot));

  const std::size_t projection_start_index =
      input.connection_start_index > 0U ? input.connection_start_index - 1U : 0U;
  const std::size_t projection_end_index = std::min(
      slot_side_bridge_boundary.size() - 1U, input.connection_end_index + 1U);
  PolylineProjection left_bridge_projection;
  PolylineProjection right_bridge_projection;
  if (!ProjectPointToPolyline(input.slot.corners.left_top, slot_side_bridge_boundary,
                              projection_start_index, projection_end_index,
                              &left_bridge_projection) ||
      !ProjectPointToPolyline(input.slot.corners.right_top, slot_side_bridge_boundary,
                              projection_start_index, projection_end_index,
                              &right_bridge_projection)) {
    if (error != nullptr) {
      *error = "failed to project parking slot opening onto road boundary";
    }
    return false;
  }

  if (right_bridge_projection.ordered_position() <
      left_bridge_projection.ordered_position()) {
    std::swap(left_bridge_projection, right_bridge_projection);
  }
  roi_geometry.connector_slice = ExtractPolylineSlice(
      slot_side_bridge_boundary, left_bridge_projection, right_bridge_projection);
  roi_geometry.bridge_polygon = RepairLocalPolygonSelfIntersections(
      BuildBridgePolygon(roi_geometry.connector_slice, input.slot));

  std::vector<Vec2d> corridor_with_bridge;
  if (BridgeBoundaryMatchesOpening(roi_geometry.connector_slice, input.slot)) {
    corridor_with_bridge = roi_geometry.corridor_polygon;
    roi_geometry.bridge_polygon.clear();
  } else if (!UnionPolygons(roi_geometry.corridor_polygon,
                            roi_geometry.bridge_polygon, &corridor_with_bridge,
                            error)) {
    *geometry = roi_geometry;
    return false;
  }
  std::vector<Vec2d> corridor_with_attachment = corridor_with_bridge;
  if (!roi_geometry.attachment_polygon.empty() &&
      !UnionPolygons(corridor_with_bridge, roi_geometry.attachment_polygon,
                     &corridor_with_attachment, error)) {
    *geometry = roi_geometry;
    return false;
  }
  if (!UnionPolygons(corridor_with_attachment, roi_geometry.slot_polygon,
                     &roi_geometry.union_polygon, error)) {
    *geometry = roi_geometry;
    return false;
  }

  Polygon2d union_polygon(roi_geometry.union_polygon);
  roi_geometry.area = union_polygon.area();
  roi_geometry.xy_boundary = {union_polygon.min_x(), union_polygon.max_x(),
                              union_polygon.min_y(), union_polygon.max_y()};
  roi_geometry.aisle_width = ComputeAisleWidth(
      input.slot, input.slot_on_left ? input.right_boundary : input.left_boundary);

  for (std::size_t index = 0; index < roi_geometry.union_polygon.size(); ++index) {
    const Vec2d& start = roi_geometry.union_polygon[index];
    const Vec2d& end = roi_geometry.union_polygon[
        (index + 1U) % roi_geometry.union_polygon.size()];
    roi_geometry.boundary_segments.push_back({start, end});
  }
  *geometry = std::move(roi_geometry);
  return true;
}

bool ExpandParkingRoiToIncludeVehicleFootprint(
    const Vec2d& vehicle_position, const double vehicle_heading,
    const apollo::common::VehicleParam& vehicle_param,
    const double start_pose_buffer, const double start_escape_distance,
    ParkingRoiGeometry* geometry, std::string* error) {
  if (geometry == nullptr) {
    if (error != nullptr) {
      *error = "parking roi geometry output is null";
    }
    return false;
  }
  if (geometry->union_polygon.size() < 3U) {
    if (error != nullptr) {
      *error = "parking roi polygon is empty";
    }
    return false;
  }

  const double ego_length = vehicle_param.length();
  const double shift_distance =
      0.5 * ego_length - vehicle_param.back_edge_to_center();
  const Vec2d ego_box_center(vehicle_position.x() +
                                 shift_distance * std::cos(vehicle_heading),
                             vehicle_position.y() +
                                 shift_distance * std::sin(vehicle_heading));
  const Box2d ego_box(ego_box_center, vehicle_heading, ego_length,
                      vehicle_param.width());
  const Polygon2d union_polygon(geometry->union_polygon);
  const Polygon2d ego_polygon(ego_box);
  if (start_pose_buffer <= common::math::kMathEpsilon &&
      start_escape_distance <= common::math::kMathEpsilon &&
      union_polygon.Contains(ego_polygon) &&
      !BoxOverlapsBoundarySegments(ego_box, geometry->boundary_segments)) {
    return true;
  }

  if (!UnionPatchIntoGeometry(BoxPolygon(ego_box, start_pose_buffer), geometry,
                              error)) {
    return false;
  }
  if (start_escape_distance > common::math::kMathEpsilon) {
    const Vec2d heading_vec(std::cos(vehicle_heading), std::sin(vehicle_heading));
    const Vec2d forward_position =
        vehicle_position + heading_vec * start_escape_distance;
    const Vec2d reverse_position =
        vehicle_position - heading_vec * start_escape_distance;
    const Box2d forward_box(
        Vec2d(forward_position.x() + shift_distance * std::cos(vehicle_heading),
              forward_position.y() + shift_distance * std::sin(vehicle_heading)),
        vehicle_heading, ego_length, vehicle_param.width());
    if (!UnionPatchIntoGeometry(BoxPolygon(forward_box, start_pose_buffer),
                                geometry, error)) {
      return false;
    }
    const Box2d reverse_box(
        Vec2d(reverse_position.x() + shift_distance * std::cos(vehicle_heading),
              reverse_position.y() + shift_distance * std::sin(vehicle_heading)),
        vehicle_heading, ego_length, vehicle_param.width());
    if (!UnionPatchIntoGeometry(BoxPolygon(reverse_box, start_pose_buffer),
                                geometry, error)) {
      return false;
    }
  }

  Polygon2d expanded_union(geometry->union_polygon);
  geometry->area = expanded_union.area();
  geometry->xy_boundary = {expanded_union.min_x(), expanded_union.max_x(),
                           expanded_union.min_y(), expanded_union.max_y()};
  geometry->boundary_segments.clear();
  for (std::size_t index = 0; index < geometry->union_polygon.size(); ++index) {
    const Vec2d& start = geometry->union_polygon[index];
    const Vec2d& end =
        geometry->union_polygon[(index + 1U) % geometry->union_polygon.size()];
    geometry->boundary_segments.push_back({start, end});
  }
  return true;
}

std::vector<Vec2d> BuildParkingEnvelopePolygon(
    const ParkingRoiGeometry& geometry) {
  if (geometry.slot_polygon.empty()) {
    if (!geometry.attachment_polygon.empty()) {
      return geometry.attachment_polygon;
    }
    return geometry.union_polygon;
  }
  if (geometry.attachment_polygon.empty()) {
    return geometry.slot_polygon;
  }

  std::vector<Vec2d> envelope_polygon;
  std::string error;
  if (UnionPolygons(geometry.slot_polygon, geometry.attachment_polygon,
                    &envelope_polygon, &error)) {
    return envelope_polygon;
  }
  return geometry.slot_polygon;
}

double ComputeParkingEnvelopeOpeningExtension(
    const std::vector<Vec2d>& slot_polygon, const Box2d& ego_box) {
  std::size_t opening_edge_index = 0U;
  Vec2d inward_axis;
  if (!InferOpeningEdge(slot_polygon, &opening_edge_index, &inward_axis)) {
    return 0.0;
  }
  const Vec2d opening_midpoint =
      (slot_polygon[opening_edge_index] +
       slot_polygon[(opening_edge_index + 1U) % slot_polygon.size()]) *
      0.5;
  std::vector<Vec2d> corners;
  ego_box.GetAllCorners(&corners);
  double min_depth = std::numeric_limits<double>::infinity();
  for (const auto& corner : corners) {
    min_depth = std::min(min_depth, (corner - opening_midpoint).InnerProd(inward_axis));
  }
  return std::max(0.0, -min_depth);
}

std::vector<Vec2d> BuildExpandedParkingEnvelopePolygon(
    const ParkingRoiGeometry& geometry, const double opening_extension) {
  std::vector<Vec2d> base_envelope = BuildParkingEnvelopePolygon(geometry);
  if (opening_extension <= common::math::kMathEpsilon ||
      geometry.slot_polygon.size() < 4U) {
    return base_envelope;
  }

  std::size_t opening_edge_index = 0U;
  Vec2d inward_axis;
  if (!InferOpeningEdge(geometry.slot_polygon, &opening_edge_index, &inward_axis)) {
    return base_envelope;
  }

  std::vector<Vec2d> expanded_slot = geometry.slot_polygon;
  const Vec2d outward_offset = inward_axis * (-opening_extension);
  expanded_slot[opening_edge_index] += outward_offset;
  expanded_slot[(opening_edge_index + 1U) % expanded_slot.size()] += outward_offset;
  expanded_slot = NormalizePolygonVertices(expanded_slot);

  if (base_envelope.size() < 3U) {
    return expanded_slot;
  }

  std::vector<Vec2d> expanded_envelope;
  std::string error;
  if (UnionPolygons(base_envelope, expanded_slot, &expanded_envelope, &error)) {
    return expanded_envelope;
  }
  return expanded_slot;
}

}  // namespace parking
}  // namespace planning
}  // namespace apollo
