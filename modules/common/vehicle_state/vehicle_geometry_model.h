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
// VehicleGeometryModel resolves two independent notions of "reference point":
//
// 1. VehicleState inputs carry their own reference_point() field, owned and
//    normalized by VehicleStateProvider. Every overload that receives a
//    VehicleState reads that field directly; callers never select it.
// 2. PathPoint / TrajectoryPoint / (position, heading) inputs describe
//    planning-generated trajectory points. These points do not carry a
//    reference-point field of their own; their anchor is a structural
//    property of whichever kinematic VehicleModel produced them (e.g. the
//    rear axle for an Ackermann bicycle model, the center of mass for a
//    four-wheel-steering model). This anchor is bound once at construction
//    time as trajectory_reference_point() and must be supplied by the
//    VehicleModel that owns this geometry model (see
//    VehicleModel::geometry_model()). Planning must never pass a
//    ReferencePoint into these overloads; doing so would let planning decide
//    something that belongs to the configured vehicle model.
//
// It does not transform motion state, apply steering constraints, or predict
// future motion; those responsibilities belong to ReferencePointTransformer
// and VehicleModel respectively. Planning owns scene-specific geometry
// decisions; this class only computes reusable vehicle geometry.
class VehicleGeometryModel {
 public:
  VehicleGeometryModel();
  explicit VehicleGeometryModel(const VehicleDescription& description);

  // trajectory_reference_point binds the anchor used to interpret every
  // PathPoint/TrajectoryPoint/(position, heading) overload below. It should
  // come from the canonical_reference_point() of the VehicleModel currently
  // configured for the vehicle; see VehicleModel::geometry_model().
  VehicleGeometryModel(const VehicleDescription& description,
                       ReferencePoint trajectory_reference_point);

  const VehicleDescription& description() const { return description_; }

  ReferencePoint trajectory_reference_point() const {
    return trajectory_reference_point_;
  }

  // Returns the footprint distances relative to trajectory_reference_point().
  // ReferencePoint currently has longitudinal anchors only, so left and right
  // are independent of the selected reference point.
  VehicleBounds Bounds() const;

  Status BuildBox(const VehicleState& vehicle_state,
                  math::Box2d* vehicle_box) const;

  Status BuildBox(const math::Vec2d& position, double heading,
                  math::Box2d* vehicle_box) const;

  Status BuildBox(const PathPoint& path_point, math::Box2d* vehicle_box) const;

  Status BuildBox(const TrajectoryPoint& trajectory_point,
                  math::Box2d* vehicle_box) const;

  math::Box2d BuildBox(const VehicleState& vehicle_state) const;

  math::Box2d BuildBox(const math::Vec2d& position, double heading) const;

  math::Box2d BuildBox(const PathPoint& path_point) const;

  math::Box2d BuildBox(const TrajectoryPoint& trajectory_point) const;

  math::Box2d BuildBox(const PathPoint& path_point, double lateral_buffer,
                       double longitudinal_buffer = 0.0) const;

  Status BuildFrontRegion(const VehicleState& vehicle_state,
                          double distance_threshold, double buffer,
                          math::Box2d* front_region) const;

  // --- Edge and clearance distances (relative to trajectory_reference_point()) ---
  double FrontEdgeDistance() const;
  double BackEdgeDistance() const;
  double LeftEdgeDistance() const;
  double RightEdgeDistance() const;

  // --- Collision & Clearance Semantics ---
  bool CheckCollision(const PathPoint& path_point,
                      const math::Box2d& obstacle_box,
                      double lateral_buffer = 0.0,
                      double longitudinal_buffer = 0.0) const;

  bool CheckCollision(const VehicleState& vehicle_state,
                      const math::Box2d& obstacle_box,
                      double lateral_buffer = 0.0,
                      double longitudinal_buffer = 0.0) const;

  double ComputeClearance(const PathPoint& path_point,
                          const math::Box2d& obstacle_box) const;

  double ComputeClearance(const VehicleState& vehicle_state,
                          const math::Box2d& obstacle_box) const;

 private:
  Status GetCenter(const VehicleState& vehicle_state,
                   math::Vec2d* center) const;

  Status GetCenter(const math::Vec2d& position, double heading,
                   ReferencePoint reference_point, math::Vec2d* center) const;

  VehicleDescription description_;
  ReferencePoint trajectory_reference_point_ = ReferencePoint::REAR_AXLE_CENTER;
};

}  // namespace common
}  // namespace apollo
