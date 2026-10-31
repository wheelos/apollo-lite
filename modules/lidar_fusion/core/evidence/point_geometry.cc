// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/lidar_fusion/core/evidence/point_geometry.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "Eigen/Geometry"

namespace apollo {
namespace lidar_fusion {
namespace fusion_detail {

Eigen::Matrix3d Rotation(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

void SetVector(const Eigen::Vector3d& value, Vec3* output) {
  output->set_x(value.x());
  output->set_y(value.y());
  output->set_z(value.z());
}

Eigen::Vector3d Vector(const Vec3& value) {
  return {value.x(), value.y(), value.z()};
}

void SetBox(const OrientedBox& box, Box* output) {
  SetVector(box.center, output->mutable_center());
  SetVector(box.size, output->mutable_size());
  const Eigen::Quaterniond q(box.rotation);
  output->set_qx(q.x());
  output->set_qy(q.y());
  output->set_qz(q.z());
  output->set_qw(q.w());
}

bool InBox(const Eigen::Vector3d& point, const OrientedBox& box,
           double margin) {
  const Eigen::Vector3d local = box.rotation.transpose() * (point - box.center);
  return (local.array().abs() <= box.size.array() * 0.5 + margin).all();
}

CellKey Key(double x, double y, double size) {
  return {static_cast<int>(std::floor(x / size)),
          static_cast<int>(std::floor(y / size))};
}

bool InRoi(const Eigen::Vector3d& p, const FusionConfig& c) {
  return p.x() >= c.min_x() && p.x() < c.max_x() && p.y() >= c.min_y() &&
         p.y() < c.max_y();
}

bool InCorridor(const Eigen::Vector3d& p, const FusionConfig& c) {
  return p.x() >= c.corridor_min_x() && p.x() <= c.corridor_max_x() &&
         std::abs(p.y()) <= c.corridor_half_width();
}

std::pair<CellKey, CellKey> BoxRange(const OrientedBox& box,
                                     const FusionConfig& c, double margin) {
  const Eigen::Vector3d ext =
      box.rotation.cwiseAbs() *
      (box.size * 0.5 + Eigen::Vector3d::Constant(margin));
  const CellKey low =
      Key(std::max(c.min_x(), box.center.x() - ext.x()),
          std::max(c.min_y(), box.center.y() - ext.y()), c.resolution());
  const CellKey high =
      Key(std::min(c.max_x(), box.center.x() + ext.x()),
          std::min(c.max_y(), box.center.y() + ext.y()), c.resolution());
  return {low, high};
}

bool ObstacleHeight(const PointEvidence& p, const FusionConfig& c) {
  return !p.ground_valid() ||
         (p.height_above_ground() >= c.obstacle_min_height() &&
          p.height_above_ground() <= c.clearance_height());
}

}  // namespace fusion_detail
}  // namespace lidar_fusion
}  // namespace apollo
