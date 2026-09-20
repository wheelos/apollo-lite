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

// A minimal, explicit 2D geometric pose: where the vehicle body is, and which
// physical reference point that position/heading describes. It carries no
// motion state and no trajectory semantics; it is only a value type used to
// invoke VehicleGeometryModel without fabricating a full VehicleState.
struct VehiclePose2d {
  VehiclePose2d(const math::Vec2d& position, double body_heading,
                ReferencePoint reference_point)
      : position(position),
        body_heading(body_heading),
        reference_point(reference_point) {}

  math::Vec2d position;
  double body_heading;
  ReferencePoint reference_point;
};

// Provides geometry-only operations for vehicle footprint, collision, and
// clearance calculations.
//
// VehicleGeometryModel is pure state -> geometry: every overload requires an
// explicit reference point, either via VehicleState::reference_point() or via
// VehiclePose2d::reference_point. It never stores or infers a reference point
// on its own, and it has no notion of "trajectory" or "canonical" reference
// points. Interpreting a PathPoint/TrajectoryPoint (i.e. deciding which
// physical point that trajectory point represents) is the caller's
// responsibility -- see PlanningGeometryAdapter for the planning-side
// binding.
//
// It does not transform motion state, apply steering constraints, or predict
// future motion; those responsibilities belong to ReferencePointTransformer
// and VehicleModel respectively.
class VehicleGeometryModel {
 public:
  // Uses the process-wide default vehicle description.
  [[deprecated("Use VehicleGeometryModel(VehicleDescription)")]]
  VehicleGeometryModel();

  explicit VehicleGeometryModel(const VehicleDescription& description);

  const VehicleDescription& description() const { return description_; }

  // Returns the footprint distances relative to reference_point.
  // ReferencePoint currently has longitudinal anchors only, so left and right
  // are independent of the selected reference point.
  Status GetBounds(ReferencePoint reference_point, VehicleBounds* bounds) const;

  Status BuildBox(const VehicleState& vehicle_state,
                  math::Box2d* vehicle_box) const;

  Status BuildBox(const VehiclePose2d& pose, math::Box2d* vehicle_box) const;

  [[deprecated("Use the Status-returning BuildBox overload")]]
  math::Box2d BuildBox(const VehicleState& vehicle_state) const;
  [[deprecated("Use the Status-returning BuildBox overload")]]
  math::Box2d BuildBox(const VehiclePose2d& pose) const;
  Status BuildBox(const VehiclePose2d& pose, double lateral_buffer,
                  double longitudinal_buffer, math::Box2d* vehicle_box) const;
  [[deprecated("Use the Status-returning buffered BuildBox overload")]]
  math::Box2d BuildBox(const VehiclePose2d& pose, double lateral_buffer,
                       double longitudinal_buffer = 0.0) const;

  Status BuildFrontRegion(const VehicleState& vehicle_state,
                          double distance_threshold, double lateral_buffer,
                          math::Box2d* front_region) const;
  // distance_threshold extends the region forward from the footprint front;
  // lateral_buffer expands both sides; longitudinal_buffer expands both
  // front and rear.
  Status BuildFrontRegion(const VehiclePose2d& pose, double distance_threshold,
                          double lateral_buffer, double longitudinal_buffer,
                          math::Box2d* front_region) const;

  // Edge distances are measured from reference_point along body heading.
  Status FrontEdgeDistance(ReferencePoint reference_point,
                           double* distance) const;
  Status RearEdgeDistance(ReferencePoint reference_point,
                          double* distance) const;
  double LeftEdgeDistance() const;
  double RightEdgeDistance() const;

  // Buffers are symmetric: lateral_buffer expands both sides and
  // longitudinal_buffer expands both front and rear. Clearance is zero when
  // the two boxes overlap or touch.
  Status CheckCollision(const VehiclePose2d& pose,
                        const math::Box2d& obstacle_box, bool* collision,
                        double lateral_buffer = 0.0,
                        double longitudinal_buffer = 0.0) const;
  Status CheckCollision(const VehicleState& vehicle_state,
                        const math::Box2d& obstacle_box, bool* collision,
                        double lateral_buffer = 0.0,
                        double longitudinal_buffer = 0.0) const;
  Status ComputeClearance(const VehiclePose2d& pose,
                          const math::Box2d& obstacle_box,
                          double* clearance) const;
  Status ComputeClearance(const VehicleState& vehicle_state,
                          const math::Box2d& obstacle_box,
                          double* clearance) const;

 private:
  Status ComputeFootprintCenter(const VehicleState& vehicle_state,
                                math::Vec2d* center) const;

  Status ComputeFootprintCenter(const VehiclePose2d& pose,
                                math::Vec2d* center) const;

  VehicleDescription description_;
};

}  // namespace common
}  // namespace apollo
