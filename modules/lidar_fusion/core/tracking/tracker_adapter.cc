// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/lidar_fusion/core/tracking/temporal_internal.h"

namespace apollo {
namespace lidar_fusion {
namespace temporal_detail {

Vector V(const Vec3& p) { return {p.x(), p.y(), p.z()}; }

void Set(const Vector& value, Vec3* p) {
  p->set_x(value.x());
  p->set_y(value.y());
  p->set_z(value.z());
}

Eigen::Isometry3d Transform(const LocalPose& p) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = Eigen::Quaterniond(p.qw(), p.qx(), p.qy(), p.qz())
                   .normalized()
                   .toRotationMatrix();
  t.translation() = V(p.translation());
  return t;
}

Box TransformBox(const Box& box, const Eigen::Isometry3d& t) {
  Box result = box;
  Set(t * V(box.center()), result.mutable_center());
  const Eigen::Quaterniond q(
      t.rotation() * Eigen::Quaterniond(box.qw(), box.qx(), box.qy(), box.qz())
                         .toRotationMatrix());
  result.set_qx(q.x());
  result.set_qy(q.y());
  result.set_qz(q.z());
  result.set_qw(q.w());
  return result;
}

}  // namespace temporal_detail
}  // namespace lidar_fusion
}  // namespace apollo
