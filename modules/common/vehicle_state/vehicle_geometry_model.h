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

#pragma once

#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/status/status.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common/vehicle_state/vehicle_description.h"
#include "wheelos_msgs/basic_msgs/pnc_point.pb.h"

namespace apollo {
namespace common {

class VehicleGeometryModel {
 public:
  VehicleGeometryModel();
  explicit VehicleGeometryModel(const VehicleDescription& description);

  const VehicleDescription& description() const { return description_; }

  Status BuildBox(const ReferenceState& reference_state,
                  math::Box2d* vehicle_box,
                  double center_of_mass_offset = 0.0) const;

  Status BuildBox(const math::Vec2d& position, double heading,
                  ReferencePoint reference_point,
                  math::Box2d* vehicle_box,
                  double center_of_mass_offset = 0.0) const;

  Status BuildBox(const PathPoint& path_point,
                  ReferencePoint reference_point,
                  math::Box2d* vehicle_box,
                  double center_of_mass_offset = 0.0) const;

  Status BuildBox(const TrajectoryPoint& trajectory_point,
                  ReferencePoint reference_point,
                  math::Box2d* vehicle_box,
                  double center_of_mass_offset = 0.0) const;

  math::Box2d BuildBox(const ReferenceState& reference_state,
                       double center_of_mass_offset = 0.0) const;

  math::Box2d BuildBox(const math::Vec2d& position, double heading,
                       ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER,
                       double center_of_mass_offset = 0.0) const;

  math::Box2d BuildBox(const PathPoint& path_point,
                       ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER,
                       double center_of_mass_offset = 0.0) const;

  math::Box2d BuildBox(const TrajectoryPoint& trajectory_point,
                       ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER,
                       double center_of_mass_offset = 0.0) const;

  math::Box2d BuildBox(const PathPoint& path_point,
                       double lateral_buffer,
                       double longitudinal_buffer = 0.0,
                       ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER,
                       double center_of_mass_offset = 0.0) const;

  Status GetCenter(const ReferenceState& reference_state,
                   math::Vec2d* center,
                   double center_of_mass_offset = 0.0) const;

  Status GetCenter(const math::Vec2d& position, double heading,
                   ReferencePoint reference_point,
                   math::Vec2d* center,
                   double center_of_mass_offset = 0.0) const;

  Status BuildFrontRegion(const ReferenceState& reference_state,
                          double distance_threshold, double buffer,
                          math::Box2d* front_region,
                          double center_of_mass_offset = 0.0) const;

  // --- Edge and clearance distances ---
  double FrontEdgeDistance(
      ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER,
      double center_of_mass_offset = 0.0) const;

  double BackEdgeDistance(
      ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER,
      double center_of_mass_offset = 0.0) const;

  double LeftEdgeDistance() const;
  double RightEdgeDistance() const;

  // --- Stop Alignment Semantics ---
  // Computes the reference point s so that the vehicle stops before target_s with stop_margin.
  double ComputeStopReferenceS(
      double target_s,
      double stop_margin = 0.0,
      TravelDirection travel_direction = TravelDirection::TRAVEL_DIRECTION_FORWARD,
      ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER,
      double center_of_mass_offset = 0.0) const;

  // --- Spatial Occupancy Semantics ---
  // Returns [s_min, s_max] on the reference line when the vehicle reference point is at ref_s.
  std::pair<double, double> GetOccupancySRange(
      double ref_s,
      ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER,
      double center_of_mass_offset = 0.0) const;

  // Returns [l_min, l_max] on the reference line when the vehicle reference point is at ref_l.
  std::pair<double, double> GetOccupancyLRange(double ref_l) const;

  // --- Collision & Clearance Semantics ---
  bool CheckCollision(
      const PathPoint& path_point,
      const math::Box2d& obstacle_box,
      double lateral_buffer = 0.0,
      double longitudinal_buffer = 0.0,
      ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER,
      double center_of_mass_offset = 0.0) const;

  bool CheckCollision(
      const ReferenceState& reference_state,
      const math::Box2d& obstacle_box,
      double lateral_buffer = 0.0,
      double longitudinal_buffer = 0.0,
      double center_of_mass_offset = 0.0) const;

  double ComputeClearance(
      const PathPoint& path_point,
      const math::Box2d& obstacle_box,
      ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER,
      double center_of_mass_offset = 0.0) const;

  double ComputeClearance(
      const ReferenceState& reference_state,
      const math::Box2d& obstacle_box,
      double center_of_mass_offset = 0.0) const;

 private:
  VehicleDescription description_;
};

}  // namespace common
}  // namespace apollo
