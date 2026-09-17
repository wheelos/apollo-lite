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

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "wheelos_msgs/basic_msgs/pnc_point.pb.h"

#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/status/status.h"
#include "modules/common/vehicle_state/vehicle_description.h"

namespace apollo {
namespace common {

// Signed distances from a longitudinal vehicle reference point to the
// footprint edges. All values are measured in the vehicle heading frame;
// they do not change when the vehicle travels in reverse.
struct VehicleBounds {
  double front = 0.0;
  double rear = 0.0;
  double left = 0.0;
  double right = 0.0;
};

// Provides geometry-only operations for vehicle footprint, collision, and
// clearance calculations.
//
// VehicleGeometryModel consumes the reference point declared by VehicleState
// and vehicle dimensions. For every operation that receives a VehicleState,
// the input position is interpreted at that point and converted to the
// geometric center using
// VehicleDescription::CenterOffset(). Therefore rear axle, front axle, and
// center-of-mass inputs describe the same physical footprint.
//
// It does not transform motion state, apply steering constraints, or predict
// future motion; those responsibilities belong to ReferencePointTransformer
// and VehicleModel respectively. Planning owns scene-specific geometry
// decisions; this class only computes reusable vehicle geometry.
class VehicleGeometryModel {
 public:
  VehicleGeometryModel();
  explicit VehicleGeometryModel(const VehicleDescription& description);

  const VehicleDescription& description() const { return description_; }

  // Returns the footprint distances relative to a center-line reference
  // point. ReferencePoint currently has longitudinal anchors only, so left
  // and right are independent of the selected reference point.
  VehicleBounds Bounds(
      ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER) const;

  Status BuildBox(const VehicleState& vehicle_state,
                  math::Box2d* vehicle_box) const;

  Status BuildBox(const math::Vec2d& position, double heading,
                  ReferencePoint reference_point,
                  math::Box2d* vehicle_box) const;

  Status BuildBox(const PathPoint& path_point, ReferencePoint reference_point,
                  math::Box2d* vehicle_box) const;

  Status BuildBox(const TrajectoryPoint& trajectory_point,
                  ReferencePoint reference_point,
                  math::Box2d* vehicle_box) const;

  math::Box2d BuildBox(const VehicleState& vehicle_state) const;

  math::Box2d BuildBox(
      const math::Vec2d& position, double heading,
      ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER) const;

  math::Box2d BuildBox(
      const PathPoint& path_point,
      ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER) const;

  math::Box2d BuildBox(
      const TrajectoryPoint& trajectory_point,
      ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER) const;

  math::Box2d BuildBox(
      const PathPoint& path_point, double lateral_buffer,
      double longitudinal_buffer = 0.0,
      ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER) const;

  Status BuildFrontRegion(const VehicleState& vehicle_state,
                          double distance_threshold, double buffer,
                          math::Box2d* front_region) const;

  // --- Edge and clearance distances ---
  double FrontEdgeDistance(
      ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER) const;

  double BackEdgeDistance(
      ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER) const;

  double LeftEdgeDistance() const;
  double RightEdgeDistance() const;

  // --- Collision & Clearance Semantics ---
  bool CheckCollision(
      const PathPoint& path_point, const math::Box2d& obstacle_box,
      double lateral_buffer = 0.0, double longitudinal_buffer = 0.0,
      ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER) const;

  bool CheckCollision(const VehicleState& vehicle_state,
                      const math::Box2d& obstacle_box,
                      double lateral_buffer = 0.0,
                      double longitudinal_buffer = 0.0) const;

  double ComputeClearance(
      const PathPoint& path_point, const math::Box2d& obstacle_box,
      ReferencePoint reference_point = ReferencePoint::REAR_AXLE_CENTER) const;

  double ComputeClearance(const VehicleState& vehicle_state,
                          const math::Box2d& obstacle_box) const;

 private:
  Status GetCenter(const VehicleState& vehicle_state,
                   math::Vec2d* center) const;

  Status GetCenter(const math::Vec2d& position, double heading,
                   ReferencePoint reference_point, math::Vec2d* center) const;

  VehicleDescription description_;
};

}  // namespace common
}  // namespace apollo
