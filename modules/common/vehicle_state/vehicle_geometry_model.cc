// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/common/vehicle_state/vehicle_geometry_model.h"

#include <cmath>
#include <string>

#include "cyber/common/log.h"

namespace apollo {
namespace common {
namespace {

Status ValidateFinite(const double value, const char* name) {
  if (!std::isfinite(value)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  std::string(name) + " is not finite");
  }
  return Status::OK();
}

Status ValidateBuffer(const double value, const char* name) {
  if (!std::isfinite(value) || value < 0.0) {
    return Status(ErrorCode::PLANNING_ERROR,
                  std::string(name) + " must be finite and non-negative");
  }
  return Status::OK();
}

}  // namespace

VehicleGeometryModel::VehicleGeometryModel() : description_() {}

VehicleGeometryModel::VehicleGeometryModel(
    const VehicleDescription& description)
    : description_(description) {}

Status VehicleGeometryModel::GetBounds(const ReferencePoint reference_point,
                                       VehicleBounds* bounds) const {
  if (bounds == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "bounds is null");
  }
  if (!IsSupportedReferencePoint(reference_point)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "unsupported vehicle reference point");
  }
  if (!FrontEdgeDistance(reference_point, &bounds->front).ok() ||
      !RearEdgeDistance(reference_point, &bounds->rear).ok()) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "failed to compute vehicle bounds");
  }
  bounds->left = description_.left_edge_to_center();
  bounds->right = description_.right_edge_to_center();
  return Status::OK();
}

Status VehicleGeometryModel::ComputeFootprintCenter(
    const VehicleState& vehicle_state, math::Vec2d* center) const {
  if (center == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "center is null");
  }
  return ComputeFootprintCenter(
      VehiclePose2d(math::Vec2d(vehicle_state.x(), vehicle_state.y()),
                    vehicle_state.heading(), vehicle_state.reference_point()),
      center);
}

Status VehicleGeometryModel::ComputeFootprintCenter(const VehiclePose2d& pose,
                                                    math::Vec2d* center) const {
  if (center == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "center is null");
  }
  if (!IsSupportedReferencePoint(pose.reference_point)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "unsupported vehicle reference point");
  }
  if (!ValidateFinite(pose.position.x(), "pose.x").ok() ||
      !ValidateFinite(pose.position.y(), "pose.y").ok() ||
      !ValidateFinite(pose.body_heading, "pose.heading").ok()) {
    return Status(ErrorCode::PLANNING_ERROR, "pose contains non-finite data");
  }
  math::Vec2d offset;
  const auto status =
      description_.FootprintCenterOffset(pose.reference_point, &offset);
  if (!status.ok()) {
    return status;
  }
  *center = pose.position + offset.rotate(pose.body_heading);
  return Status::OK();
}

Status VehicleGeometryModel::BuildBox(const VehicleState& vehicle_state,
                                      math::Box2d* vehicle_box) const {
  if (vehicle_box == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "vehicle_box is null");
  }
  math::Vec2d center;
  const auto status = ComputeFootprintCenter(vehicle_state, &center);
  if (!status.ok()) {
    return status;
  }
  *vehicle_box = math::Box2d(center, vehicle_state.heading(),
                             description_.length(), description_.width());
  return Status::OK();
}

Status VehicleGeometryModel::BuildBox(const VehiclePose2d& pose,
                                      math::Box2d* vehicle_box) const {
  if (vehicle_box == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "vehicle_box is null");
  }
  math::Vec2d center;
  const auto status = ComputeFootprintCenter(pose, &center);
  if (!status.ok()) {
    return status;
  }
  *vehicle_box = math::Box2d(center, pose.body_heading, description_.length(),
                             description_.width());
  return Status::OK();
}

math::Box2d VehicleGeometryModel::BuildBox(
    const VehicleState& vehicle_state) const {
  math::Box2d box;
  const auto status = BuildBox(vehicle_state, &box);
  CHECK(status.ok()) << status;
  return box;
}

math::Box2d VehicleGeometryModel::BuildBox(const VehiclePose2d& pose) const {
  math::Box2d box;
  const auto status = BuildBox(pose, &box);
  CHECK(status.ok()) << status;
  return box;
}

Status VehicleGeometryModel::BuildBox(const VehiclePose2d& pose,
                                      const double lateral_buffer,
                                      const double longitudinal_buffer,
                                      math::Box2d* vehicle_box) const {
  if (vehicle_box == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "vehicle_box is null");
  }
  auto status = ValidateBuffer(lateral_buffer, "lateral buffer");
  if (!status.ok()) {
    return status;
  }
  status = ValidateBuffer(longitudinal_buffer, "longitudinal buffer");
  if (!status.ok()) {
    return status;
  }
  status = BuildBox(pose, vehicle_box);
  if (!status.ok()) {
    return status;
  }
  *vehicle_box = math::Box2d(vehicle_box->center(), vehicle_box->heading(),
                             description_.length() + 2.0 * longitudinal_buffer,
                             description_.width() + 2.0 * lateral_buffer);
  return Status::OK();
}

math::Box2d VehicleGeometryModel::BuildBox(
    const VehiclePose2d& pose, const double lateral_buffer,
    const double longitudinal_buffer) const {
  math::Box2d box;
  const auto status = BuildBox(pose, lateral_buffer, longitudinal_buffer, &box);
  CHECK(status.ok()) << status;
  return box;
}

Status VehicleGeometryModel::BuildFrontRegion(const VehicleState& vehicle_state,
                                              const double distance_threshold,
                                              const double lateral_buffer,
                                              math::Box2d* front_region) const {
  return BuildFrontRegion(
      VehiclePose2d(math::Vec2d(vehicle_state.x(), vehicle_state.y()),
                    vehicle_state.heading(), vehicle_state.reference_point()),
      distance_threshold, lateral_buffer, 0.0, front_region);
}

Status VehicleGeometryModel::BuildFrontRegion(const VehiclePose2d& pose,
                                              const double distance_threshold,
                                              const double lateral_buffer,
                                              const double longitudinal_buffer,
                                              math::Box2d* front_region) const {
  if (front_region == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "front_region is null");
  }
  auto status = ValidateBuffer(distance_threshold, "distance threshold");
  if (!status.ok()) {
    return status;
  }
  status = ValidateBuffer(lateral_buffer, "lateral buffer");
  if (!status.ok()) {
    return status;
  }
  status = ValidateBuffer(longitudinal_buffer, "longitudinal buffer");
  if (!status.ok()) {
    return status;
  }
  math::Vec2d center;
  status = ComputeFootprintCenter(pose, &center);
  if (!status.ok()) {
    return status;
  }
  const math::Vec2d unit_heading =
      math::Vec2d::CreateUnitVec2d(pose.body_heading);
  const double length =
      description_.length() + distance_threshold + 2.0 * longitudinal_buffer;
  const double width = description_.width() + 2.0 * lateral_buffer;
  *front_region =
      math::Box2d(center + unit_heading * (distance_threshold / 2.0),
                  pose.body_heading, length, width);
  return Status::OK();
}

Status VehicleGeometryModel::FrontEdgeDistance(const ReferencePoint point,
                                               double* distance) const {
  if (distance == nullptr || !IsSupportedReferencePoint(point)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "invalid reference point or distance output");
  }
  double offset = 0.0;
  const auto status = description_.LongitudinalOffset(point, &offset);
  if (!status.ok()) {
    return status;
  }
  *distance = description_.front_edge_to_center() - offset;
  return Status::OK();
}

Status VehicleGeometryModel::RearEdgeDistance(const ReferencePoint point,
                                              double* distance) const {
  if (distance == nullptr || !IsSupportedReferencePoint(point)) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "invalid reference point or distance output");
  }
  double offset = 0.0;
  const auto status = description_.LongitudinalOffset(point, &offset);
  if (!status.ok()) {
    return status;
  }
  *distance = description_.back_edge_to_center() + offset;
  return Status::OK();
}

double VehicleGeometryModel::LeftEdgeDistance() const {
  return description_.left_edge_to_center();
}

double VehicleGeometryModel::RightEdgeDistance() const {
  return description_.right_edge_to_center();
}

Status VehicleGeometryModel::CheckCollision(
    const VehiclePose2d& pose, const math::Box2d& obstacle_box, bool* collision,
    const double lateral_buffer, const double longitudinal_buffer) const {
  if (collision == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "collision output is null");
  }
  math::Box2d vehicle_box;
  const auto status =
      BuildBox(pose, lateral_buffer, longitudinal_buffer, &vehicle_box);
  if (!status.ok()) {
    return status;
  }
  *collision = obstacle_box.HasOverlap(vehicle_box);
  return Status::OK();
}

Status VehicleGeometryModel::CheckCollision(
    const VehicleState& vehicle_state, const math::Box2d& obstacle_box,
    bool* collision, const double lateral_buffer,
    const double longitudinal_buffer) const {
  return CheckCollision(
      VehiclePose2d(math::Vec2d(vehicle_state.x(), vehicle_state.y()),
                    vehicle_state.heading(), vehicle_state.reference_point()),
      obstacle_box, collision, lateral_buffer, longitudinal_buffer);
}

Status VehicleGeometryModel::ComputeClearance(const VehiclePose2d& pose,
                                              const math::Box2d& obstacle_box,
                                              double* clearance) const {
  if (clearance == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "clearance output is null");
  }
  math::Box2d vehicle_box;
  const auto status = BuildBox(pose, &vehicle_box);
  if (!status.ok()) {
    return status;
  }
  // Box2d::DistanceTo returns zero for overlapping boxes.
  *clearance = vehicle_box.DistanceTo(obstacle_box);
  return Status::OK();
}

Status VehicleGeometryModel::ComputeClearance(const VehicleState& vehicle_state,
                                              const math::Box2d& obstacle_box,
                                              double* clearance) const {
  return ComputeClearance(
      VehiclePose2d(math::Vec2d(vehicle_state.x(), vehicle_state.y()),
                    vehicle_state.heading(), vehicle_state.reference_point()),
      obstacle_box, clearance);
}

}  // namespace common
}  // namespace apollo
