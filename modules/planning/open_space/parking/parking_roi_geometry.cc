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
#include <sstream>
#include <vector>

#include <boost/geometry/algorithms/append.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/union.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "cyber/common/log.h"
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
constexpr std::size_t kMaxClippedStartGoalTemplateVertices = 24U;

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
      std::ostringstream stream;
      stream << disconnected_error << " (components=" << union_result.size()
             << ")";
      *error = stream.str();
    }
    return false;
  }
  *result = std::move(union_result.front());
  return true;
}

bool IntersectBoostPolygons(const BgPolygon& left_polygon,
                            const BgPolygon& right_polygon, BgPolygon* result,
                            std::string* error,
                            const std::string& disconnected_error) {
  if (result == nullptr) {
    if (error != nullptr) {
      *error = "parking roi boost polygon output is null";
    }
    return false;
  }
  std::vector<BgPolygon> intersection_result;
  bg::intersection(left_polygon, right_polygon, intersection_result);
  if (intersection_result.size() != 1U) {
    if (error != nullptr) {
      std::ostringstream stream;
      stream << disconnected_error << " (components="
             << intersection_result.size() << ")";
      *error = stream.str();
    }
    return false;
  }
  *result = std::move(intersection_result.front());
  return true;
}

bool UnionPolygons(const std::vector<Vec2d>& left,
                   const std::vector<Vec2d>& right,
                   std::vector<Vec2d>* result, std::string* error,
                   const std::string& disconnected_error =
                       "parking roi polygons are disconnected after union") {
  const BgPolygon left_polygon = ToBoostPolygon(left);
  const BgPolygon right_polygon = ToBoostPolygon(right);
  BgPolygon union_polygon;
  if (!UnionBoostPolygons(left_polygon, right_polygon, &union_polygon, error,
                          disconnected_error)) {
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

bool PolygonContainsPatch(const std::vector<Vec2d>& polygon,
                          const std::vector<Vec2d>& patch_polygon) {
  if (polygon.size() < 3U || patch_polygon.size() < 3U) {
    return false;
  }
  return Polygon2d(polygon).Contains(Polygon2d(patch_polygon));
}

bool UnionPatchIntoGeometry(const std::vector<Vec2d>& patch_polygon,
                            ParkingRoiGeometry* geometry, std::string* error,
                            const std::string& disconnected_error =
                                "parking roi union polygon and warm-start patch "
                                "are disconnected after union") {
  if (geometry == nullptr || patch_polygon.size() < 3U) {
    return geometry != nullptr;
  }
  if (PolygonContainsPatch(geometry->union_polygon, patch_polygon)) {
    return true;
  }
  std::vector<Vec2d> expanded_union_polygon;
  if (!UnionPolygons(geometry->union_polygon, patch_polygon,
                     &expanded_union_polygon, error, disconnected_error)) {
    return false;
  }
  geometry->union_polygon = std::move(expanded_union_polygon);
  return true;
}

void UpdateUnionPolygonMetadata(ParkingRoiGeometry* geometry) {
  if (geometry == nullptr) {
    return;
  }
  const Polygon2d union_polygon(geometry->union_polygon);
  geometry->area = union_polygon.area();
  geometry->xy_boundary = {union_polygon.min_x(), union_polygon.max_x(),
                           union_polygon.min_y(), union_polygon.max_y()};
  geometry->boundary_segments.clear();
  for (std::size_t index = 0; index < geometry->union_polygon.size(); ++index) {
    const Vec2d& start = geometry->union_polygon[index];
    const Vec2d& end =
        geometry->union_polygon[(index + 1U) % geometry->union_polygon.size()];
    geometry->boundary_segments.push_back({start, end});
  }
}

std::vector<Vec2d> BuildBoundingRoiPolygon(
    const std::vector<std::vector<Vec2d>>& polygons, const double buffer) {
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  bool has_point = false;
  for (const auto& polygon : polygons) {
    for (const auto& point : polygon) {
      min_x = std::min(min_x, point.x());
      max_x = std::max(max_x, point.x());
      min_y = std::min(min_y, point.y());
      max_y = std::max(max_y, point.y());
      has_point = true;
    }
  }
  if (!has_point || min_x >= max_x || min_y >= max_y) {
    return {};
  }
  min_x -= buffer;
  max_x += buffer;
  min_y -= buffer;
  max_y += buffer;
  return {Vec2d(min_x, min_y), Vec2d(max_x, min_y), Vec2d(max_x, max_y),
          Vec2d(min_x, max_y)};
}

double EstimateSlotWidth(const ParkingSlot& slot,
                         const std::vector<Vec2d>& slot_polygon) {
  if (slot.width > common::math::kMathEpsilon) {
    return slot.width;
  }
  const double opening_width =
      slot.corners.left_top.DistanceTo(slot.corners.right_top);
  if (opening_width > common::math::kMathEpsilon) {
    return opening_width;
  }
  double max_distance = 0.0;
  for (std::size_t i = 0; i < slot_polygon.size(); ++i) {
    for (std::size_t j = i + 1U; j < slot_polygon.size(); ++j) {
      max_distance = std::max(max_distance,
                              slot_polygon[i].DistanceTo(slot_polygon[j]));
    }
  }
  return max_distance;
}

Vec2d NormalizeOrFallback(const Vec2d& value, const Vec2d& fallback) {
  if (value.Length() > common::math::kMathEpsilon) {
    return value / value.Length();
  }
  if (fallback.Length() > common::math::kMathEpsilon) {
    return fallback / fallback.Length();
  }
  return Vec2d(1.0, 0.0);
}

std::vector<Vec2d> BuildParkingPitRoiPolygon(
    const ParkingSlot& slot, const std::vector<std::vector<Vec2d>>& polygons,
    const apollo::common::VehicleParam& vehicle_param, const double buffer) {
  const Vec2d depth_axis = NormalizeOrFallback(
      slot.rear_center - slot.opening_center,
      Vec2d::CreateUnitVec2d(slot.heading));
  const Vec2d width_axis = NormalizeOrFallback(
      slot.corners.right_top - slot.corners.left_top,
      Vec2d(-depth_axis.y(), depth_axis.x()));
  const Vec2d origin = slot.opening_center;

  auto update_bounds = [&](const Vec2d& point, double* min_width,
                           double* max_width, double* min_depth,
                           double* max_depth) {
    const Vec2d delta = point - origin;
    const double width = delta.InnerProd(width_axis);
    const double depth = delta.InnerProd(depth_axis);
    *min_width = std::min(*min_width, width);
    *max_width = std::max(*max_width, width);
    *min_depth = std::min(*min_depth, depth);
    *max_depth = std::max(*max_depth, depth);
  };

  double min_width = std::numeric_limits<double>::infinity();
  double max_width = -std::numeric_limits<double>::infinity();
  double min_depth = std::numeric_limits<double>::infinity();
  double max_depth = -std::numeric_limits<double>::infinity();
  bool has_point = false;
  for (const auto& polygon : polygons) {
    for (const auto& point : polygon) {
      update_bounds(point, &min_width, &max_width, &min_depth, &max_depth);
      has_point = true;
    }
  }
  if (!has_point || min_width >= max_width || min_depth >= max_depth) {
    return {};
  }

  double slot_left = std::numeric_limits<double>::infinity();
  double slot_right = -std::numeric_limits<double>::infinity();
  double slot_front = std::numeric_limits<double>::infinity();
  double slot_rear = -std::numeric_limits<double>::infinity();
  for (const auto& point : slot.Vertices()) {
    update_bounds(point, &slot_left, &slot_right, &slot_front, &slot_rear);
  }
  if (slot_left >= slot_right || slot_front >= slot_rear) {
    return {};
  }

  const double min_maneuver_width =
      std::max(12.0, 2.5 * vehicle_param.length());
  const double min_aisle_depth =
      std::max(8.0, 1.5 * vehicle_param.length());
  const auto expand_interval_around = [](const double min_size,
                                          const double center,
                                          double* min_value,
                                          double* max_value) {
    const double current_size = *max_value - *min_value;
    if (current_size >= min_size) {
      return;
    }
    *min_value = center - 0.5 * min_size;
    *max_value = center + 0.5 * min_size;
  };
  expand_interval_around(min_maneuver_width,
                          0.5 * (slot_left + slot_right), &min_width,
                          &max_width);
  const double opening_depth = slot_front - buffer;
  const double neck_depth = std::min(
      slot_rear,
      opening_depth + std::max(0.5 * vehicle_param.length(), buffer));
  min_depth = std::min(min_depth - buffer, opening_depth - min_aisle_depth);
  min_width -= buffer;
  max_width += buffer;
  const double physical_slot_left = slot_left;
  const double physical_slot_right = slot_right;
  const double physical_slot_width = physical_slot_right - physical_slot_left;
  const double slot_center_width =
      0.5 * (physical_slot_left + physical_slot_right);
  constexpr double kSlotSideClearance = 0.1;
  const double side_clearance =
      physical_slot_width >= vehicle_param.width() + 2.0 * kSlotSideClearance
          ? kSlotSideClearance
          : 0.0;
  if (side_clearance <= common::math::kMathEpsilon) {
    AWARN << "Parking slot width " << physical_slot_width
          << " is too narrow for side clearance; ROI will not widen beyond "
          << "physical slot lines";
  }
  const double half_slot_roi_width =
      std::max(0.5 * physical_slot_width - side_clearance, 0.0);
  slot_left = slot_center_width - half_slot_roi_width;
  slot_right = slot_center_width + half_slot_roi_width;
  double mouth_band_half_width = half_slot_roi_width;
  for (const auto& polygon : polygons) {
    for (const auto& point : polygon) {
      const Vec2d delta = point - origin;
      const double width = delta.InnerProd(width_axis);
      const double depth = delta.InnerProd(depth_axis);
      if (depth + common::math::kMathEpsilon < opening_depth ||
          depth > neck_depth + common::math::kMathEpsilon) {
        continue;
      }
      mouth_band_half_width =
          std::max(mouth_band_half_width,
                   std::fabs(width - slot_center_width) + buffer);
    }
  }
  mouth_band_half_width =
      std::min(mouth_band_half_width,
               std::min(max_width - buffer - slot_center_width,
                        slot_center_width - min_width - buffer));
  const double mouth_band_left = slot_center_width - mouth_band_half_width;
  const double mouth_band_right = slot_center_width + mouth_band_half_width;
  const auto to_world = [&](const double width, const double depth) {
    return origin + width_axis * width + depth_axis * depth;
  };
  const double mouth_left = min_width + buffer;
  const double mouth_right = max_width - buffer;
  const std::vector<Vec2d> roi_polygon = NormalizePolygonVertices({
      to_world(min_width, min_depth),
      to_world(max_width, min_depth),
      to_world(max_width, opening_depth),
      to_world(mouth_right, opening_depth),
      to_world(mouth_band_right, opening_depth),
      to_world(mouth_band_right, neck_depth),
      to_world(slot_right, neck_depth),
      to_world(slot_right, slot_rear),
      to_world(slot_left, slot_rear),
      to_world(slot_left, neck_depth),
      to_world(mouth_band_left, neck_depth),
      to_world(mouth_band_left, opening_depth),
      to_world(mouth_left, opening_depth),
      to_world(min_width, opening_depth),
  });
  return NormalizePolygonVertices(roi_polygon);
}

bool ApplyDisconnectedRoiBoundingFallback(const std::string& reason,
                                          ParkingRoiGeometry* geometry,
                                          std::string* error) {
  if (geometry == nullptr) {
    return false;
  }
  const auto fallback_polygon = BuildBoundingRoiPolygon(
      {geometry->corridor_polygon, geometry->bridge_polygon,
       geometry->attachment_polygon, geometry->slot_polygon},
      0.5);
  if (fallback_polygon.size() < 3U) {
    return false;
  }
  AWARN << "Parking ROI fallback uses bounding polygon after: " << reason;
  if (error != nullptr) {
    *error = "parking roi disconnected-union fallback: " + reason;
  }
  geometry->union_polygon = fallback_polygon;
  UpdateUnionPolygonMetadata(geometry);
  return true;
}

bool ApplyExpansionBoundingFallback(const std::vector<Vec2d>& patch_polygon,
                                    const std::string& reason,
                                    ParkingRoiGeometry* geometry,
                                    std::string* error) {
  if (geometry == nullptr || patch_polygon.size() < 3U) {
    return false;
  }
  const auto fallback_polygon =
      BuildBoundingRoiPolygon({geometry->union_polygon, patch_polygon}, 0.5);
  if (fallback_polygon.size() < 3U) {
    return false;
  }
  AWARN << "Parking ROI expansion fallback after: " << reason;
  if (error != nullptr) {
    *error = "parking roi expansion fallback: " + reason;
  }
  geometry->union_polygon = fallback_polygon;
  UpdateUnionPolygonMetadata(geometry);
  return true;
}

bool ComputeConnectorTargetPoint(const Vec2d& point,
                                 const std::vector<double>& xy_boundary,
                                 Vec2d* target_point) {
  if (target_point == nullptr || xy_boundary.size() != 4U) {
    return false;
  }
  const double min_x = xy_boundary[0];
  const double max_x = xy_boundary[1];
  const double min_y = xy_boundary[2];
  const double max_y = xy_boundary[3];
  if (point.x() < min_x) {
    *target_point = Vec2d(min_x, std::clamp(point.y(), min_y, max_y));
    return true;
  }
  if (point.x() > max_x) {
    *target_point = Vec2d(max_x, std::clamp(point.y(), min_y, max_y));
    return true;
  }
  if (point.y() < min_y) {
    *target_point = Vec2d(std::clamp(point.x(), min_x, max_x), min_y);
    return true;
  }
  if (point.y() > max_y) {
    *target_point = Vec2d(std::clamp(point.x(), min_x, max_x), max_y);
    return true;
  }
  return false;
}

std::vector<Vec2d> BuildConnectorPatchPolygon(
    const Vec2d& start, const Vec2d& end,
    const apollo::common::VehicleParam& vehicle_param,
    const double start_pose_buffer) {
  const Vec2d delta = end - start;
  if (delta.Length() <= common::math::kMathEpsilon) {
    return {};
  }
  const Vec2d center = (start + end) * 0.5;
  const double heading = std::atan2(delta.y(), delta.x());
  const double length =
      delta.Length() + vehicle_param.length() + 2.0 * start_pose_buffer;
  const double width = vehicle_param.width() + 2.0 * start_pose_buffer;
  return BoxPolygon(Box2d(center, heading, length, width), 0.0);
}

void AppendExpansionDebugContext(const Box2d& ego_box,
                                 const ParkingRoiGeometry& geometry,
                                 std::string* error) {
  if (error == nullptr || geometry.xy_boundary.size() != 4U) {
    return;
  }
  std::ostringstream stream;
  stream << *error << "; ego_center=(" << ego_box.center().x() << ","
         << ego_box.center().y() << "); roi_xy=[" << geometry.xy_boundary[0]
         << "," << geometry.xy_boundary[1] << "," << geometry.xy_boundary[2]
         << "," << geometry.xy_boundary[3] << "]";
  *error = stream.str();
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
                            error,
                            "parking roi corridor and bridge are disconnected "
                            "after union")) {
    const std::string reason =
        error == nullptr || error->empty()
            ? "parking roi corridor and bridge are disconnected after union"
            : *error;
    if (input.allow_disconnected_roi_fallback &&
        ApplyDisconnectedRoiBoundingFallback(reason, &roi_geometry, error)) {
      roi_geometry.aisle_width = ComputeAisleWidth(
          input.slot,
          input.slot_on_left ? input.right_boundary : input.left_boundary);
      *geometry = std::move(roi_geometry);
      return true;
    }
    *geometry = roi_geometry;
    return false;
  }
  std::vector<Vec2d> corridor_with_attachment = corridor_with_bridge;
  if (!roi_geometry.attachment_polygon.empty() &&
      !UnionPolygons(corridor_with_bridge, roi_geometry.attachment_polygon,
                      &corridor_with_attachment, error,
                      "parking roi corridor and opening attachment are "
                      "disconnected after union")) {
    const std::string reason =
        error == nullptr || error->empty()
            ? "parking roi corridor and opening attachment are disconnected "
              "after union"
            : *error;
    if (input.allow_disconnected_roi_fallback &&
        ApplyDisconnectedRoiBoundingFallback(reason, &roi_geometry, error)) {
      roi_geometry.aisle_width = ComputeAisleWidth(
          input.slot,
          input.slot_on_left ? input.right_boundary : input.left_boundary);
      *geometry = std::move(roi_geometry);
      return true;
    }
    *geometry = roi_geometry;
    return false;
  }
  if (!UnionPolygons(corridor_with_attachment, roi_geometry.slot_polygon,
                     &roi_geometry.union_polygon, error,
                     "parking roi corridor and slot are disconnected after "
                     "union")) {
    const std::string reason =
        error == nullptr || error->empty()
            ? "parking roi corridor and slot are disconnected after union"
            : *error;
    if (input.allow_disconnected_roi_fallback &&
        ApplyDisconnectedRoiBoundingFallback(reason, &roi_geometry, error)) {
      roi_geometry.aisle_width = ComputeAisleWidth(
          input.slot,
          input.slot_on_left ? input.right_boundary : input.left_boundary);
      *geometry = std::move(roi_geometry);
      return true;
    }
    *geometry = roi_geometry;
    return false;
  }

  UpdateUnionPolygonMetadata(&roi_geometry);
  roi_geometry.aisle_width = ComputeAisleWidth(
      input.slot, input.slot_on_left ? input.right_boundary : input.left_boundary);
  *geometry = std::move(roi_geometry);
  return true;
}

bool ExpandParkingRoiToIncludeVehicleFootprint(
    const Vec2d& vehicle_position, const double vehicle_heading,
    const apollo::common::VehicleParam& vehicle_param,
    const double start_pose_buffer, const double start_escape_distance,
    ParkingRoiGeometry* geometry, std::string* error,
    const bool allow_disconnected_roi_fallback) {
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

  const auto ego_patch_polygon = BoxPolygon(ego_box, start_pose_buffer);
  const std::string ego_patch_error =
      "parking roi union polygon and ego footprint patch are disconnected "
      "after union";
  if (!UnionPatchIntoGeometry(ego_patch_polygon, geometry, error,
                              ego_patch_error)) {
    Vec2d connector_target;
    if (!ComputeConnectorTargetPoint(ego_box.center(), geometry->xy_boundary,
                                     &connector_target) ||
        !UnionPatchIntoGeometry(
            BuildConnectorPatchPolygon(ego_box.center(), connector_target,
                                       vehicle_param, start_pose_buffer),
            geometry, error,
            "parking roi union polygon and ego connector patch are "
            "disconnected after union") ||
        !UnionPatchIntoGeometry(
            ego_patch_polygon, geometry, error,
            "parking roi union polygon and ego footprint patch are disconnected "
            "after union")) {
      if (!allow_disconnected_roi_fallback ||
          !ApplyExpansionBoundingFallback(
              ego_patch_polygon,
              error == nullptr || error->empty() ? ego_patch_error : *error,
              geometry, error)) {
        AppendExpansionDebugContext(ego_box, *geometry, error);
        return false;
      }
    }
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
    const auto forward_patch = BoxPolygon(forward_box, start_pose_buffer);
    const std::string forward_patch_error =
        "parking roi union polygon and forward warm-start patch are "
        "disconnected after union";
    if (!UnionPatchIntoGeometry(forward_patch, geometry, error,
                                forward_patch_error) &&
        (!allow_disconnected_roi_fallback ||
          !ApplyExpansionBoundingFallback(
             forward_patch,
             error == nullptr || error->empty() ? forward_patch_error : *error,
             geometry, error))) {
      return false;
    }
    const Box2d reverse_box(
        Vec2d(reverse_position.x() + shift_distance * std::cos(vehicle_heading),
              reverse_position.y() + shift_distance * std::sin(vehicle_heading)),
        vehicle_heading, ego_length, vehicle_param.width());
    const auto reverse_patch = BoxPolygon(reverse_box, start_pose_buffer);
    const std::string reverse_patch_error =
        "parking roi union polygon and reverse warm-start patch are "
        "disconnected after union";
    if (!UnionPatchIntoGeometry(reverse_patch, geometry, error,
                                reverse_patch_error) &&
        (!allow_disconnected_roi_fallback ||
          !ApplyExpansionBoundingFallback(
             reverse_patch,
             error == nullptr || error->empty() ? reverse_patch_error : *error,
             geometry, error))) {
      return false;
    }
  }

  UpdateUnionPolygonMetadata(geometry);
  return true;
}

bool ApplyStartGoalParkingRoiTemplate(
    const Vec2d& vehicle_position, const double vehicle_heading,
    const ParkingSlot& slot, const std::vector<double>& goal_pose,
    const apollo::common::VehicleParam& vehicle_param,
    const double template_buffer, ParkingRoiGeometry* geometry,
    std::string* error) {
  if (geometry == nullptr) {
    if (error != nullptr) {
      *error = "parking roi geometry output is null";
    }
    return false;
  }
  if (geometry->slot_polygon.size() < 3U) {
    if (error != nullptr) {
      *error = "parking roi slot polygon is empty";
    }
    return false;
  }
  if (goal_pose.size() < 3U) {
    if (error != nullptr) {
      *error = "parking goal pose is incomplete";
    }
    return false;
  }

  const std::vector<Vec2d> map_union_polygon = geometry->union_polygon;

  const double ego_length = vehicle_param.length();
  const double shift_distance =
      0.5 * ego_length - vehicle_param.back_edge_to_center();
  const Vec2d ego_box_center(vehicle_position.x() +
                                 shift_distance * std::cos(vehicle_heading),
                             vehicle_position.y() +
                                 shift_distance * std::sin(vehicle_heading));
  const Box2d ego_box(ego_box_center, vehicle_heading, ego_length,
                      vehicle_param.width());
  const Vec2d vehicle_heading_vec(std::cos(vehicle_heading),
                                  std::sin(vehicle_heading));
  const double start_escape_distance =
      std::max(template_buffer,
               vehicle_param.wheel_base() > common::math::kMathEpsilon
                   ? vehicle_param.wheel_base()
                   : 0.5 * vehicle_param.length());
  const Vec2d forward_position =
      vehicle_position + vehicle_heading_vec * start_escape_distance;
  const Vec2d reverse_position =
      vehicle_position - vehicle_heading_vec * start_escape_distance;
  const Box2d forward_box(
      Vec2d(forward_position.x() + shift_distance * std::cos(vehicle_heading),
            forward_position.y() + shift_distance * std::sin(vehicle_heading)),
      vehicle_heading, ego_length, vehicle_param.width());
  const Box2d reverse_box(
      Vec2d(reverse_position.x() + shift_distance * std::cos(vehicle_heading),
            reverse_position.y() + shift_distance * std::sin(vehicle_heading)),
      vehicle_heading, ego_length, vehicle_param.width());
  const Vec2d goal_position(goal_pose[0], goal_pose[1]);
  const double goal_heading = goal_pose[2];
  const Vec2d goal_box_center(goal_position.x() +
                                  shift_distance * std::cos(goal_heading),
                              goal_position.y() +
                                  shift_distance * std::sin(goal_heading));
  const Box2d goal_box(goal_box_center, goal_heading, ego_length,
                       vehicle_param.width());

  const Vec2d corridor_delta = goal_box_center - ego_box_center;
  const double corridor_length = std::max(corridor_delta.Length(), 1.0);
  const double corridor_heading =
      corridor_delta.Length() > common::math::kMathEpsilon
          ? corridor_delta.Angle()
          : vehicle_heading;
  const double buffer = std::max(0.0, template_buffer);
  const double slot_width = EstimateSlotWidth(slot, geometry->slot_polygon);
  const double corridor_width =
      std::max({vehicle_param.width() + 2.0 * buffer,
                slot_width + 2.0 * buffer, 2.5 * vehicle_param.length(), 12.0});
  const Box2d corridor_box((ego_box_center + goal_box_center) * 0.5,
                            corridor_heading,
                            corridor_length + ego_length + 2.0 * buffer,
                            corridor_width);

  const auto ego_patch = BoxPolygon(ego_box, buffer);
  const auto forward_patch = BoxPolygon(forward_box, buffer);
  const auto reverse_patch = BoxPolygon(reverse_box, buffer);
  const auto goal_patch = BoxPolygon(goal_box, 0.0);
  const auto corridor_patch = BoxPolygon(corridor_box, 0.0);
  std::vector<Vec2d> template_polygon = BuildParkingPitRoiPolygon(
      slot, {ego_patch, forward_patch, reverse_patch, goal_patch,
             corridor_patch},
      vehicle_param, 0.25);
  if (template_polygon.size() < 3U) {
    if (error != nullptr) {
      *error = "failed to build parking roi start-goal template";
    }
    return false;
  }
  if (map_union_polygon.size() >= 3U) {
    const BgPolygon map_polygon = ToBoostPolygon(map_union_polygon);
    const BgPolygon raw_template_polygon = ToBoostPolygon(template_polygon);
    BgPolygon clipped_template_polygon;
    std::string clip_error;
    if (IntersectBoostPolygons(
            map_polygon, raw_template_polygon, &clipped_template_polygon,
            &clip_error,
            "parking roi start-goal template exceeds map roi envelope")) {
      const std::vector<Vec2d> clipped_polygon =
          FromBoostOuterRing(clipped_template_polygon);
      if (clipped_polygon.size() >= 3U &&
          clipped_polygon.size() <= kMaxClippedStartGoalTemplateVertices &&
          PolygonContainsPatch(clipped_polygon, ego_patch) &&
          PolygonContainsPatch(clipped_polygon, goal_patch)) {
        template_polygon = clipped_polygon;
      } else {
        AWARN << "Ignore map-clipped parking ROI template: vertices="
              << clipped_polygon.size()
              << " contains_ego="
              << PolygonContainsPatch(clipped_polygon, ego_patch)
              << " contains_goal="
              << PolygonContainsPatch(clipped_polygon, goal_patch)
              << "; use bounded start-goal-slot template instead";
      }
    } else {
      AWARN << "Unable to clip parking ROI template by map envelope: "
            << clip_error
            << "; use bounded start-goal-slot template instead";
    }
  }

  geometry->union_polygon = template_polygon;
  UpdateUnionPolygonMetadata(geometry);
  AINFO << "Parking pit ROI template vertices="
        << geometry->union_polygon.size() << " xmin=" << geometry->xy_boundary[0]
        << " xmax=" << geometry->xy_boundary[1]
        << " ymin=" << geometry->xy_boundary[2]
        << " ymax=" << geometry->xy_boundary[3];
  if (!UnionPatchIntoGeometry(
          ego_patch, geometry, error,
          "parking roi start-goal template and ego footprint are disconnected "
          "after union") ||
      !UnionPatchIntoGeometry(
          goal_patch, geometry, error,
          "parking roi start-goal template and goal footprint are disconnected "
          "after union")) {
    return false;
  } else {
    UpdateUnionPolygonMetadata(geometry);
  }
  if (!PolygonContainsPatch(geometry->union_polygon, ego_patch) ||
      !PolygonContainsPatch(geometry->union_polygon, goal_patch)) {
    if (error != nullptr) {
      *error = "parking roi start-goal template does not contain required "
               "footprints";
    }
    return false;
  }
  AINFO << "Direct parking ROI uses start-goal template xmin="
        << geometry->xy_boundary[0] << " xmax=" << geometry->xy_boundary[1]
        << " ymin=" << geometry->xy_boundary[2]
        << " ymax=" << geometry->xy_boundary[3]
        << " corridor_width=" << corridor_width
        << " start=(" << vehicle_position.x() << ", " << vehicle_position.y()
        << ", " << vehicle_heading << ") goal=(" << goal_pose[0] << ", "
        << goal_pose[1] << ", " << goal_pose[2] << ")";
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
