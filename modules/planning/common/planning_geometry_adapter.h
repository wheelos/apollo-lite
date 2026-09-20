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

#include "wheelos_msgs/basic_msgs/pnc_point.pb.h"

#include "cyber/common/log.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/vehicle_state/vehicle_geometry_model.h"

namespace apollo {
namespace planning {

// Binds a single planning reference point to a VehicleGeometryModel so that
// planning algorithms can build footprints directly from PathPoint /
// TrajectoryPoint without knowing or choosing any physical anchor.
//
// The bound reference point must come from the planning state snapshot that
// generated the trajectory being interpreted (typically
// ReferenceLineInfo::vehicle_state().reference_point()), not from the
// VehicleModel's internal canonical reference point. VehicleGeometryModel
// itself never stores or infers this binding; PlanningGeometryAdapter is the
// only place planning-side reference-point semantics are attached to
// PathPoint/TrajectoryPoint interpretation.
class PlanningGeometryAdapter {
 public:
  PlanningGeometryAdapter()
      : has_reference_point_(false),
        reference_point_(common::VehicleReferencePoint::REAR_AXLE_CENTER) {}

  PlanningGeometryAdapter(const common::VehicleGeometryModel& geometry_model,
                          common::VehicleReferencePoint reference_point)
      : geometry_model_(geometry_model),
        has_reference_point_(
            common::IsSupportedReferencePoint(reference_point)),
        reference_point_(reference_point) {
    CHECK(common::IsSupportedReferencePoint(reference_point_));
  }

  const common::VehicleGeometryModel& geometry_model() const {
    return geometry_model_;
  }

  common::VehicleReferencePoint reference_point() const { return reference_point_; }

  common::VehiclePose2d ToPose(const common::PathPoint& path_point) const {
    CHECK(IsBound());
    return common::VehiclePose2d(
        common::math::Vec2d(path_point.x(), path_point.y()), path_point.theta(),
        reference_point_);
  }

  common::VehiclePose2d ToPose(
      const common::TrajectoryPoint& trajectory_point) const {
    return ToPose(trajectory_point.path_point());
  }

  common::VehiclePose2d ToPose(const common::math::Vec2d& position,
                               double heading) const {
    CHECK(IsBound());
    return common::VehiclePose2d(position, heading, reference_point_);
  }

  common::math::Box2d BuildBox(const common::VehicleState& state) const {
    CHECK(IsBound());
    common::math::Box2d box;
    CHECK(geometry_model_.BuildBox(state, &box).ok());
    return box;
  }

  common::math::Box2d BuildBox(const common::PathPoint& path_point) const {
    common::math::Box2d box;
    CHECK(geometry_model_.BuildBox(ToPose(path_point), &box).ok());
    return box;
  }

  common::math::Box2d BuildBox(
      const common::TrajectoryPoint& trajectory_point) const {
    return BuildBox(trajectory_point.path_point());
  }

  common::math::Box2d BuildBox(const common::math::Vec2d& position,
                               double heading) const {
    common::math::Box2d box;
    CHECK(geometry_model_.BuildBox(ToPose(position, heading), &box).ok());
    return box;
  }

  common::math::Box2d BuildBox(const common::PathPoint& path_point,
                               double lateral_buffer,
                               double longitudinal_buffer = 0.0) const {
    common::math::Box2d box;
    CHECK(geometry_model_
              .BuildBox(ToPose(path_point), lateral_buffer, longitudinal_buffer,
                        &box)
              .ok());
    return box;
  }

  bool CheckCollision(const common::PathPoint& path_point,
                      const common::math::Box2d& obstacle_box,
                      double lateral_buffer = 0.0,
                      double longitudinal_buffer = 0.0) const {
    bool collision = false;
    CHECK(geometry_model_
              .CheckCollision(ToPose(path_point), obstacle_box, &collision,
                              lateral_buffer, longitudinal_buffer)
              .ok());
    return collision;
  }

  double ComputeClearance(const common::PathPoint& path_point,
                          const common::math::Box2d& obstacle_box) const {
    double clearance = 0.0;
    CHECK(geometry_model_
              .ComputeClearance(ToPose(path_point), obstacle_box, &clearance)
              .ok());
    return clearance;
  }

  double FrontEdgeDistance() const {
    CHECK(IsBound());
    double distance = 0.0;
    CHECK(geometry_model_.FrontEdgeDistance(reference_point_, &distance).ok());
    return distance;
  }

  double RearEdgeDistance() const {
    CHECK(IsBound());
    double distance = 0.0;
    CHECK(geometry_model_.RearEdgeDistance(reference_point_, &distance).ok());
    return distance;
  }

  double LeftEdgeDistance() const { return geometry_model_.LeftEdgeDistance(); }

  double RightEdgeDistance() const {
    return geometry_model_.RightEdgeDistance();
  }

  common::VehicleBounds Bounds() const {
    CHECK(IsBound());
    common::VehicleBounds bounds;
    CHECK(geometry_model_.GetBounds(reference_point_, &bounds).ok());
    return bounds;
  }

 private:
  bool IsBound() const {
    return has_reference_point_ &&
           common::IsSupportedReferencePoint(reference_point_);
  }

  common::VehicleGeometryModel geometry_model_;
  bool has_reference_point_ = true;
  common::VehicleReferencePoint reference_point_;
};

}  // namespace planning
}  // namespace apollo
