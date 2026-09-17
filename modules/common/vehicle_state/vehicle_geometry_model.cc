// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/common/vehicle_state/vehicle_geometry_model.h"

#include <utility>

namespace apollo {
namespace common {

VehicleGeometryModel::VehicleGeometryModel()
    : description_(VehicleDescription()) {}

VehicleGeometryModel::VehicleGeometryModel(
    const VehicleDescription& description)
    : description_(description) {}

VehicleBounds VehicleGeometryModel::Bounds(
    const ReferencePoint reference_point) const {
  VehicleBounds bounds;
  bounds.front = FrontEdgeDistance(reference_point);
  bounds.rear = BackEdgeDistance(reference_point);
  bounds.left = LeftEdgeDistance();
  bounds.right = RightEdgeDistance();
  return bounds;
}

Status VehicleGeometryModel::GetCenter(const VehicleState& vehicle_state,
                                       math::Vec2d* center) const {
  const math::Vec2d position(vehicle_state.x(), vehicle_state.y());
  return GetCenter(position, vehicle_state.heading(),
                   vehicle_state.reference_point(), center);
}

Status VehicleGeometryModel::GetCenter(const math::Vec2d& position,
                                       const double heading,
                                       const ReferencePoint reference_point,
                                       math::Vec2d* center) const {
  if (center == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "center is null");
  }
  const math::Vec2d center_offset = description_.CenterOffset(reference_point);
  *center = position + center_offset.rotate(heading);
  return Status::OK();
}

Status VehicleGeometryModel::BuildBox(const VehicleState& vehicle_state,
                                      math::Box2d* vehicle_box) const {
  if (vehicle_box == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "vehicle_box is null");
  }
  math::Vec2d center;
  const auto status = GetCenter(vehicle_state, &center);
  if (!status.ok()) {
    return status;
  }
  *vehicle_box = math::Box2d(center, vehicle_state.heading(),
                             description_.length(), description_.width());
  return Status::OK();
}

Status VehicleGeometryModel::BuildBox(const math::Vec2d& position,
                                      const double heading,
                                      const ReferencePoint reference_point,
                                      math::Box2d* vehicle_box) const {
  if (vehicle_box == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "vehicle_box is null");
  }
  math::Vec2d center;
  const auto status = GetCenter(position, heading, reference_point, &center);
  if (!status.ok()) {
    return status;
  }
  *vehicle_box =
      math::Box2d(center, heading, description_.length(), description_.width());
  return Status::OK();
}

Status VehicleGeometryModel::BuildBox(const PathPoint& path_point,
                                      const ReferencePoint reference_point,
                                      math::Box2d* vehicle_box) const {
  const math::Vec2d position(path_point.x(), path_point.y());
  return BuildBox(position, path_point.theta(), reference_point, vehicle_box);
}

Status VehicleGeometryModel::BuildBox(const TrajectoryPoint& trajectory_point,
                                      const ReferencePoint reference_point,
                                      math::Box2d* vehicle_box) const {
  return BuildBox(trajectory_point.path_point(), reference_point, vehicle_box);
}

math::Box2d VehicleGeometryModel::BuildBox(
    const VehicleState& vehicle_state) const {
  math::Box2d box;
  BuildBox(vehicle_state, &box);
  return box;
}

math::Box2d VehicleGeometryModel::BuildBox(
    const math::Vec2d& position, const double heading,
    const ReferencePoint reference_point) const {
  math::Box2d box;
  BuildBox(position, heading, reference_point, &box);
  return box;
}

math::Box2d VehicleGeometryModel::BuildBox(
    const PathPoint& path_point, const ReferencePoint reference_point) const {
  math::Box2d box;
  BuildBox(path_point, reference_point, &box);
  return box;
}

math::Box2d VehicleGeometryModel::BuildBox(
    const TrajectoryPoint& trajectory_point,
    const ReferencePoint reference_point) const {
  math::Box2d box;
  BuildBox(trajectory_point, reference_point, &box);
  return box;
}

math::Box2d VehicleGeometryModel::BuildBox(
    const PathPoint& path_point, const double lateral_buffer,
    const double longitudinal_buffer,
    const ReferencePoint reference_point) const {
  const math::Vec2d position(path_point.x(), path_point.y());
  math::Vec2d center;
  GetCenter(position, path_point.theta(), reference_point, &center);
  return math::Box2d(center, path_point.theta(),
                     description_.length() + 2.0 * longitudinal_buffer,
                     description_.width() + 2.0 * lateral_buffer);
}

Status VehicleGeometryModel::BuildFrontRegion(const VehicleState& vehicle_state,
                                              const double distance_threshold,
                                              const double buffer,
                                              math::Box2d* front_region) const {
  if (front_region == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "front_region is null");
  }
  math::Vec2d center;
  const auto status = GetCenter(vehicle_state, &center);
  if (!status.ok()) {
    return status;
  }
  const math::Vec2d unit_vec_heading =
      math::Vec2d::CreateUnitVec2d(vehicle_state.heading());
  const double impact_region_length =
      description_.length() + buffer + distance_threshold;
  *front_region =
      math::Box2d(center + unit_vec_heading * (distance_threshold / 2.0),
                  vehicle_state.heading(), impact_region_length,
                  description_.width() + buffer);
  return Status::OK();
}

double VehicleGeometryModel::FrontEdgeDistance(
    const ReferencePoint reference_point) const {
  return description_.front_edge_to_center() -
         description_.LongitudinalOffset(reference_point);
}

double VehicleGeometryModel::BackEdgeDistance(
    const ReferencePoint reference_point) const {
  return description_.back_edge_to_center() +
         description_.LongitudinalOffset(reference_point);
}

double VehicleGeometryModel::LeftEdgeDistance() const {
  return description_.left_edge_to_center();
}

double VehicleGeometryModel::RightEdgeDistance() const {
  return description_.right_edge_to_center();
}

bool VehicleGeometryModel::CheckCollision(
    const PathPoint& path_point, const math::Box2d& obstacle_box,
    const double lateral_buffer, const double longitudinal_buffer,
    const ReferencePoint reference_point) const {
  const math::Box2d vehicle_box = BuildBox(
      path_point, lateral_buffer, longitudinal_buffer, reference_point);
  return obstacle_box.HasOverlap(vehicle_box);
}

bool VehicleGeometryModel::CheckCollision(
    const VehicleState& vehicle_state, const math::Box2d& obstacle_box,
    const double lateral_buffer, const double longitudinal_buffer) const {
  PathPoint path_point;
  path_point.set_x(vehicle_state.x());
  path_point.set_y(vehicle_state.y());
  path_point.set_theta(vehicle_state.heading());
  return CheckCollision(path_point, obstacle_box, lateral_buffer,
                        longitudinal_buffer, vehicle_state.reference_point());
}

double VehicleGeometryModel::ComputeClearance(
    const PathPoint& path_point, const math::Box2d& obstacle_box,
    const ReferencePoint reference_point) const {
  const math::Box2d vehicle_box =
      BuildBox(path_point, 0.0, 0.0, reference_point);
  return vehicle_box.DistanceTo(obstacle_box);
}

double VehicleGeometryModel::ComputeClearance(
    const VehicleState& vehicle_state,
    const math::Box2d& obstacle_box) const {
  PathPoint path_point;
  path_point.set_x(vehicle_state.x());
  path_point.set_y(vehicle_state.y());
  path_point.set_theta(vehicle_state.heading());
  return ComputeClearance(path_point, obstacle_box,
                          vehicle_state.reference_point());
}

}  // namespace common
}  // namespace apollo
