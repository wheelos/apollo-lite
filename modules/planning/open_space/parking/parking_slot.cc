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

#include "modules/planning/open_space/parking/parking_slot.h"

#include <algorithm>
#include <cmath>
#include <utility>

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

double AverageDistance(const Vec2d& first_a, const Vec2d& first_b,
                       const Vec2d& second_a, const Vec2d& second_b) {
  return 0.5 * ((first_a - first_b).Length() + (second_a - second_b).Length());
}

void TransformPoint(const Vec2d& origin_point, const double origin_heading,
                    Vec2d* point) {
  *point -= origin_point;
  point->SelfRotate(-origin_heading);
}

}  // namespace

std::array<Vec2d, 4> ParkingSlotCorners::AsArray() const {
  return {left_top, left_down, right_down, right_top};
}

std::array<Vec2d, 4> ParkingSlot::Vertices() const { return corners.AsArray(); }

ParkingSlotType InferParkingSlotType(double slot_heading, double lane_heading) {
  const double angle = NormalizeAcuteAngle(slot_heading - lane_heading);
  if (angle < M_PI / 6.0) {
    return ParkingSlotType::kParallel;
  }
  if (angle > 5.0 * M_PI / 12.0) {
    return ParkingSlotType::kPerpendicular;
  }
  return ParkingSlotType::kAngled;
}

const char* ParkingSlotTypeName(ParkingSlotType type) {
  switch (type) {
    case ParkingSlotType::kParallel:
      return "parallel";
    case ParkingSlotType::kPerpendicular:
      return "perpendicular";
    case ParkingSlotType::kAngled:
      return "angled";
    case ParkingSlotType::kUnknown:
    default:
      return "unknown";
  }
}

bool NormalizeParkingSlot(const std::vector<Vec2d>& polygon_points,
                          const std::string& slot_id, double slot_heading,
                          double lane_heading, double lane_l,
                          ParkingSlot* parking_slot, std::string* error) {
  if (parking_slot == nullptr) {
    if (error != nullptr) {
      *error = "parking slot output is null";
    }
    return false;
  }
  if (polygon_points.size() != 4U) {
    if (error != nullptr) {
      *error = "parking slot polygon must contain exactly four points";
    }
    return false;
  }

  ParkingSlot slot;
  slot.id = slot_id;
  slot.raw_heading = common::math::NormalizeAngle(slot_heading);
  slot.lane_heading = common::math::NormalizeAngle(lane_heading);
  slot.lane_l = lane_l;
  slot.on_left_lane_side =
      (std::fabs(lane_l) < common::math::kMathEpsilon)
          ? common::math::NormalizeAngle(slot_heading - lane_heading) >= 0.0
          : lane_l >= 0.0;
  slot.type = InferParkingSlotType(slot.raw_heading, slot.lane_heading);
  slot.polygon = polygon_points;

  slot.center = Vec2d(0.0, 0.0);
  for (const auto& point : polygon_points) {
    slot.center += point;
  }
  slot.center /= static_cast<double>(polygon_points.size());

  const Vec2d lane_direction = Vec2d::CreateUnitVec2d(slot.lane_heading);
  const Vec2d left_normal = Vec2d::CreateUnitVec2d(slot.lane_heading + M_PI_2);
  Vec2d aisle_direction =
      slot.on_left_lane_side ? left_normal * -1.0 : left_normal;

  Vec2d lateral_axis;
  Vec2d depth_axis;
  if (slot.type == ParkingSlotType::kParallel) {
    lateral_axis = Vec2d::CreateUnitVec2d(slot.raw_heading);
    if (lateral_axis.InnerProd(lane_direction) < 0.0) {
      lateral_axis = lateral_axis * -1.0;
    }
    depth_axis = aisle_direction * -1.0;
    slot.heading = lateral_axis.Angle();
  } else {
    depth_axis = Vec2d::CreateUnitVec2d(slot.raw_heading);
    if (depth_axis.InnerProd(aisle_direction * -1.0) < 0.0) {
      depth_axis = depth_axis * -1.0;
    }
    lateral_axis = Vec2d(-depth_axis.y(), depth_axis.x());
    if (lateral_axis.InnerProd(lane_direction) < 0.0) {
      lateral_axis = lateral_axis * -1.0;
    }
    slot.heading = depth_axis.Angle();
  }

  using LocalPoint = std::pair<Vec2d, Vec2d>;
  std::vector<LocalPoint> local_points;
  local_points.reserve(polygon_points.size());
  for (const auto& point : polygon_points) {
    const Vec2d delta = point - slot.center;
    local_points.emplace_back(
        Vec2d(delta.InnerProd(lateral_axis), delta.InnerProd(depth_axis)),
        point);
  }
  std::sort(local_points.begin(), local_points.end(),
            [](const LocalPoint& lhs, const LocalPoint& rhs) {
              if (std::fabs(lhs.first.y() - rhs.first.y()) >
                  common::math::kMathEpsilon) {
                return lhs.first.y() < rhs.first.y();
              }
              return lhs.first.x() < rhs.first.x();
            });

  std::array<LocalPoint, 2> front = {local_points[0], local_points[1]};
  std::array<LocalPoint, 2> rear = {local_points[2], local_points[3]};
  std::sort(front.begin(), front.end(),
            [](const LocalPoint& lhs, const LocalPoint& rhs) {
              return lhs.first.x() < rhs.first.x();
            });
  std::sort(rear.begin(), rear.end(),
            [](const LocalPoint& lhs, const LocalPoint& rhs) {
              return lhs.first.x() < rhs.first.x();
            });

  slot.corners.left_top = front[0].second;
  slot.corners.right_top = front[1].second;
  slot.corners.left_down = rear[0].second;
  slot.corners.right_down = rear[1].second;
  slot.opening_center = (slot.corners.left_top + slot.corners.right_top) * 0.5;
  slot.rear_center = (slot.corners.left_down + slot.corners.right_down) * 0.5;
  slot.width = AverageDistance(slot.corners.left_top, slot.corners.right_top,
                               slot.corners.left_down,
                               slot.corners.right_down);
  slot.depth = AverageDistance(slot.corners.left_top, slot.corners.left_down,
                               slot.corners.right_top,
                               slot.corners.right_down);

  *parking_slot = std::move(slot);
  return true;
}

ParkingSlot TransformParkingSlot(const ParkingSlot& parking_slot,
                                 const Vec2d& origin_point,
                                 const double origin_heading) {
  ParkingSlot transformed = parking_slot;
  TransformPoint(origin_point, origin_heading, &transformed.center);
  TransformPoint(origin_point, origin_heading, &transformed.opening_center);
  TransformPoint(origin_point, origin_heading, &transformed.rear_center);
  TransformPoint(origin_point, origin_heading, &transformed.corners.left_top);
  TransformPoint(origin_point, origin_heading, &transformed.corners.left_down);
  TransformPoint(origin_point, origin_heading, &transformed.corners.right_down);
  TransformPoint(origin_point, origin_heading, &transformed.corners.right_top);
  for (auto& point : transformed.polygon) {
    TransformPoint(origin_point, origin_heading, &point);
  }
  transformed.raw_heading =
      common::math::NormalizeAngle(transformed.raw_heading - origin_heading);
  transformed.heading =
      common::math::NormalizeAngle(transformed.heading - origin_heading);
  transformed.lane_heading =
      common::math::NormalizeAngle(transformed.lane_heading - origin_heading);
  return transformed;
}

}  // namespace parking
}  // namespace planning
}  // namespace apollo
