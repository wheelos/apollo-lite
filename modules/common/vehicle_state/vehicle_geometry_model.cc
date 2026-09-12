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

namespace apollo {
namespace common {

VehicleGeometryModel::VehicleGeometryModel()
    : description_(VehicleDescription()) {}

VehicleGeometryModel::VehicleGeometryModel(
    const VehicleDescription& description)
    : description_(description) {}

Status VehicleGeometryModel::GetCenter(
    const ReferenceState& reference_state, math::Vec2d* center,
    const double center_of_mass_offset) const {
  const math::Vec2d position(reference_state.x(), reference_state.y());
  return GetCenter(position, reference_state.heading(),
                   reference_state.reference_point(), center,
                   center_of_mass_offset);
}

Status VehicleGeometryModel::GetCenter(
    const math::Vec2d& position, const double heading,
    const ReferencePoint reference_point, math::Vec2d* center,
    const double center_of_mass_offset) const {
  if (center == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "center is null");
  }
  const math::Vec2d center_offset =
      description_.CenterOffset(reference_point, center_of_mass_offset);
  *center = position + center_offset.rotate(heading);
  return Status::OK();
}

Status VehicleGeometryModel::BuildBox(
    const ReferenceState& reference_state, math::Box2d* vehicle_box,
    const double center_of_mass_offset) const {
  if (vehicle_box == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "vehicle_box is null");
  }
  math::Vec2d center;
  const auto status =
      GetCenter(reference_state, &center, center_of_mass_offset);
  if (!status.ok()) {
    return status;
  }
  *vehicle_box = math::Box2d(center, reference_state.heading(),
                             description_.length(), description_.width());
  return Status::OK();
}

Status VehicleGeometryModel::BuildBox(
    const math::Vec2d& position, const double heading,
    const ReferencePoint reference_point, math::Box2d* vehicle_box,
    const double center_of_mass_offset) const {
  if (vehicle_box == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "vehicle_box is null");
  }
  math::Vec2d center;
  const auto status = GetCenter(position, heading, reference_point, &center,
                                center_of_mass_offset);
  if (!status.ok()) {
    return status;
  }
  *vehicle_box = math::Box2d(center, heading, description_.length(),
                             description_.width());
  return Status::OK();
}

Status VehicleGeometryModel::BuildBox(
    const PathPoint& path_point, const ReferencePoint reference_point,
    math::Box2d* vehicle_box, const double center_of_mass_offset) const {
  const math::Vec2d position(path_point.x(), path_point.y());
  return BuildBox(position, path_point.theta(), reference_point, vehicle_box,
                  center_of_mass_offset);
}

Status VehicleGeometryModel::BuildBox(
    const TrajectoryPoint& trajectory_point,
    const ReferencePoint reference_point, math::Box2d* vehicle_box,
    const double center_of_mass_offset) const {
  return BuildBox(trajectory_point.path_point(), reference_point, vehicle_box,
                  center_of_mass_offset);
}

math::Box2d VehicleGeometryModel::BuildBox(
    const ReferenceState& reference_state,
    const double center_of_mass_offset) const {
  math::Box2d box;
  BuildBox(reference_state, &box, center_of_mass_offset);
  return box;
}

math::Box2d VehicleGeometryModel::BuildBox(
    const math::Vec2d& position, const double heading,
    const ReferencePoint reference_point,
    const double center_of_mass_offset) const {
  math::Box2d box;
  BuildBox(position, heading, reference_point, &box, center_of_mass_offset);
  return box;
}

math::Box2d VehicleGeometryModel::BuildBox(
    const PathPoint& path_point, const ReferencePoint reference_point,
    const double center_of_mass_offset) const {
  math::Box2d box;
  BuildBox(path_point, reference_point, &box, center_of_mass_offset);
  return box;
}

math::Box2d VehicleGeometryModel::BuildBox(
    const TrajectoryPoint& trajectory_point,
    const ReferencePoint reference_point,
    const double center_of_mass_offset) const {
  math::Box2d box;
  BuildBox(trajectory_point, reference_point, &box, center_of_mass_offset);
  return box;
}

math::Box2d VehicleGeometryModel::BuildBox(
    const PathPoint& path_point, const double lateral_buffer,
    const double longitudinal_buffer,
    const ReferencePoint reference_point,
    const double center_of_mass_offset) const {
  const math::Vec2d position(path_point.x(), path_point.y());
  math::Vec2d center;
  GetCenter(position, path_point.theta(), reference_point, &center,
            center_of_mass_offset);
  return math::Box2d(center, path_point.theta(),
                     description_.length() + 2.0 * longitudinal_buffer,
                     description_.width() + 2.0 * lateral_buffer);
}

Status VehicleGeometryModel::BuildFrontRegion(
    const ReferenceState& reference_state, const double distance_threshold,
    const double buffer, math::Box2d* front_region,
    const double center_of_mass_offset) const {
  if (front_region == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "front_region is null");
  }
  math::Vec2d center;
  const auto status =
      GetCenter(reference_state, &center, center_of_mass_offset);
  if (!status.ok()) {
    return status;
  }
  const math::Vec2d unit_vec_heading =
      math::Vec2d::CreateUnitVec2d(reference_state.heading());
  const double impact_region_length =
      description_.length() + buffer + distance_threshold;
  *front_region = math::Box2d(
      center + unit_vec_heading * (distance_threshold / 2.0),
      reference_state.heading(), impact_region_length,
      description_.width() + buffer);
  return Status::OK();
}

double VehicleGeometryModel::FrontEdgeDistance(
    const ReferencePoint reference_point,
    const double center_of_mass_offset) const {
  return description_.front_edge_to_center() -
         description_.LongitudinalOffset(reference_point, center_of_mass_offset);
}

double VehicleGeometryModel::BackEdgeDistance(
    const ReferencePoint reference_point,
    const double center_of_mass_offset) const {
  return description_.back_edge_to_center() +
         description_.LongitudinalOffset(reference_point, center_of_mass_offset);
}

double VehicleGeometryModel::LeftEdgeDistance() const {
  return description_.left_edge_to_center();
}

double VehicleGeometryModel::RightEdgeDistance() const {
  return description_.right_edge_to_center();
}

double VehicleGeometryModel::ComputeStopReferenceS(
    const double target_s, const double stop_margin,
    const TravelDirection travel_direction,
    const ReferencePoint reference_point,
    const double center_of_mass_offset) const {
  if (travel_direction == TravelDirection::TRAVEL_DIRECTION_REVERSE) {
    return target_s + stop_margin +
           BackEdgeDistance(reference_point, center_of_mass_offset);
  }
  return target_s - stop_margin -
         FrontEdgeDistance(reference_point, center_of_mass_offset);
}

std::pair<double, double> VehicleGeometryModel::GetOccupancySRange(
    const double ref_s, const ReferencePoint reference_point,
    const double center_of_mass_offset) const {
  const double s_min =
      ref_s - BackEdgeDistance(reference_point, center_of_mass_offset);
  const double s_max =
      ref_s + FrontEdgeDistance(reference_point, center_of_mass_offset);
  return {s_min, s_max};
}

std::pair<double, double> VehicleGeometryModel::GetOccupancyLRange(
    const double ref_l) const {
  const double l_min = ref_l - RightEdgeDistance();
  const double l_max = ref_l + LeftEdgeDistance();
  return {l_min, l_max};
}

bool VehicleGeometryModel::CheckCollision(
    const PathPoint& path_point, const math::Box2d& obstacle_box,
    const double lateral_buffer, const double longitudinal_buffer,
    const ReferencePoint reference_point,
    const double center_of_mass_offset) const {
  const math::Box2d vehicle_box =
      BuildBox(path_point, lateral_buffer, longitudinal_buffer, reference_point,
               center_of_mass_offset);
  return obstacle_box.HasOverlap(vehicle_box);
}

bool VehicleGeometryModel::CheckCollision(
    const ReferenceState& reference_state, const math::Box2d& obstacle_box,
    const double lateral_buffer, const double longitudinal_buffer,
    const double center_of_mass_offset) const {
  PathPoint path_point;
  path_point.set_x(reference_state.x());
  path_point.set_y(reference_state.y());
  path_point.set_theta(reference_state.heading());
  return CheckCollision(path_point, obstacle_box, lateral_buffer,
                        longitudinal_buffer, reference_state.reference_point(),
                        center_of_mass_offset);
}

double VehicleGeometryModel::ComputeClearance(
    const PathPoint& path_point, const math::Box2d& obstacle_box,
    const ReferencePoint reference_point,
    const double center_of_mass_offset) const {
  const math::Box2d vehicle_box =
      BuildBox(path_point, 0.0, 0.0, reference_point, center_of_mass_offset);
  return vehicle_box.DistanceTo(obstacle_box);
}

double VehicleGeometryModel::ComputeClearance(
    const ReferenceState& reference_state, const math::Box2d& obstacle_box,
    const double center_of_mass_offset) const {
  PathPoint path_point;
  path_point.set_x(reference_state.x());
  path_point.set_y(reference_state.y());
  path_point.set_theta(reference_state.heading());
  return ComputeClearance(path_point, obstacle_box,
                          reference_state.reference_point(),
                          center_of_mass_offset);
}

}  // namespace common
}  // namespace apollo
