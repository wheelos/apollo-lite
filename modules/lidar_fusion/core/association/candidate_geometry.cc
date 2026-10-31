// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/lidar_fusion/core/association/candidate_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "Eigen/Geometry"

#include "modules/lidar_fusion/core/association/relation_support.h"

namespace apollo {
namespace lidar_fusion {
namespace candidate_detail {
namespace {
using XY = std::pair<double, double>;

bool ConflictingExtension(
    const FusionConfig& c,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    const ObjectCandidate& object, const ObjectSupport& support,
    const Eigen::Vector3d& extent, const PointIndex* index) {
  const auto& box = object.box();
  const Eigen::Vector3d center(box.center().x(), box.center().y(),
                               box.center().z());
  const Eigen::Vector3d half(box.size().x() * 0.5, box.size().y() * 0.5,
                             box.size().z() * 0.5);
  uint32_t same = 0, conflict = 0;
  auto visit = [&](int id) {
    const auto& p = evidence.Get(id);
    const auto& point = p.position();
    if (!p.finite() || p.semantic_probability() < c.semantic_min_confidence() ||
        p.is_ground_geo() ||
        (p.ground_valid() &&
         (p.height_above_ground() < c.obstacle_min_height() ||
          p.height_above_ground() > c.clearance_height())) ||
        point.x() < c.min_x() || point.x() >= c.max_x() ||
        point.y() < c.min_y() || point.y() >= c.max_y()) {
      return;
    }
    const bool compatible =
        p.group() == THING && p.unified_class() == object.unified_class();
    const bool background =
        p.group() == STATIC_STRUCT || p.group() == VEGETATION;
    if (!compatible && !background) {
      return;
    }
    const Eigen::Vector3d local =
        support.rotation * Eigen::Vector3d(point.x() - center.x(),
                                           point.y() - center.y(),
                                           point.z() - center.z());
    if (local.allFinite() &&
        (local.cwiseAbs().array() <= extent.array()).all() &&
        (local.cwiseAbs().array() > half.array()).any()) {
      same += compatible;
      conflict += background;
    }
  };
  const double guard = 4096 * std::numeric_limits<double>::epsilon() *
                       (1 + center.cwiseAbs().maxCoeff() + extent.sum());
  const Eigen::Vector3d radius =
      support.rotation.conjugate().toRotationMatrix().cwiseAbs() * extent +
      Eigen::Vector3d::Constant(guard);
  if (index == nullptr || !radius.allFinite()) {
    for (int id = 0; id < evidence.size(); ++id) {
      visit(id);
    }
  } else {
    const int min_x = static_cast<int>(std::floor(
        std::max(c.min_x(), center.x() - radius.x()) / c.resolution()));
    const int max_x = static_cast<int>(std::floor(
        std::min(c.max_x(), center.x() + radius.x()) / c.resolution()));
    const int min_y = static_cast<int>(std::floor(
        std::max(c.min_y(), center.y() - radius.y()) / c.resolution()));
    const int max_y = static_cast<int>(std::floor(
        std::min(c.max_y(), center.y() + radius.y()) / c.resolution()));
    for (int x = min_x; x <= max_x; ++x) {
      for (int y = min_y; y <= max_y; ++y) {
        const auto found = index->find({x, y});
        if (found != index->end()) {
          for (int id : found->second) {
            visit(id);
          }
        }
      }
    }
  }
  return conflict > 0 && static_cast<double>(conflict) / (same + conflict) >=
                             c.conflict_ratio();
}

void ExtendDetection(
    const FusionConfig& c,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    ObjectCandidate* object, const PointIndex* index) {
  if (c.quality().semantic_identity_unverified() || object->source() != DET ||
      object->group() != THING || object->consistency() != SUPPORT ||
      object->class_conflict() ||
      object->classification_use() != CLASSIFICATION_CANDIDATE ||
      std::find(object->reason().begin(), object->reason().end(),
                ASSOCIATED_SUPPORT_EXTENT) != object->reason().end()) {
    return;
  }
  const ObjectSupport support(*object);
  if (!support.bounds.valid) {
    return;
  }
  const auto& b = object->box();
  const Eigen::Vector3d center(b.center().x(), b.center().y(), b.center().z());
  const Eigen::Vector3d half(b.size().x() * 0.5, b.size().y() * 0.5,
                             b.size().z() * 0.5);
  if ((half.array() <= 0).any()) {
    return;
  }
  Eigen::Vector3d extent = half;
  uint32_t anchors = 0;
  for (uint32_t id : object->point_id()) {
    const auto& p = evidence.Get(id);
    if (!p.finite() || p.group() != THING ||
        p.unified_class() != object->unified_class() ||
        p.semantic_probability() < c.semantic_min_confidence()) {
      continue;
    }
    const Eigen::Vector3d local =
        support.rotation * Eigen::Vector3d(p.position().x() - center.x(),
                                           p.position().y() - center.y(),
                                           p.position().z() - center.z());
    if (!local.allFinite() ||
        (local.cwiseAbs().array() > half.array() + c.association_margin())
            .any()) {
      continue;
    }
    anchors += (local.cwiseAbs().array() <= half.array()).all();
    extent = extent.cwiseMax(local.cwiseAbs());
  }
  if (anchors < c.core_min_points() || (extent.array() == half.array()).all() ||
      ConflictingExtension(c, evidence, *object, support, extent, index)) {
    return;
  }
  auto* box = object->mutable_box();
  box->mutable_size()->set_x(2 * extent.x());
  box->mutable_size()->set_y(2 * extent.y());
  box->mutable_size()->set_z(2 * extent.z());
  object->add_reason(ASSOCIATED_SUPPORT_EXTENT);
}

double Cross(const XY& a, const XY& b, const XY& c) {
  return (b.first - a.first) * (c.second - a.second) -
         (b.second - a.second) * (c.first - a.first);
}

std::vector<XY> Hull(std::vector<XY> points) {
  std::sort(points.begin(), points.end());
  points.erase(std::unique(points.begin(), points.end()), points.end());
  if (points.size() < 3) {
    return points;
  }
  std::vector<XY> result;
  for (const auto& p : points) {
    while (result.size() >= 2 &&
           Cross(result[result.size() - 2], result.back(), p) <= 0) {
      result.pop_back();
    }
    result.push_back(p);
  }
  const auto lower = result.size();
  for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
    while (result.size() > lower &&
           Cross(result[result.size() - 2], result.back(), *it) <= 0) {
      result.pop_back();
    }
    result.push_back(*it);
  }
  result.pop_back();
  return result;
}

std::vector<XY> Points(
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    const google::protobuf::RepeatedField<uint32_t>& ids, double pad) {
  std::vector<XY> points;
  for (uint32_t id : ids) {
    const auto& p = evidence.Get(id).position();
    for (double dx : {-pad, pad}) {
      for (double dy : {-pad, pad}) {
        points.emplace_back(p.x() + dx, p.y() + dy);
      }
    }
  }
  return points;
}

void Fit(const FusionConfig& config,
         const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
         ObjectCandidate* object) {
  const auto points = Points(evidence, object->point_id(), 0);
  double best = std::numeric_limits<double>::infinity();
  double best_yaw = 0;
  XY low, high;
  // Bounded angular minimum-area reference, not a learned orientation estimate.
  for (int angle = 0; angle < 90; ++angle) {
    const double yaw = angle * std::acos(-1) / 180;
    const double c = std::cos(yaw), s = std::sin(yaw);
    XY a{std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity()};
    XY b{-a.first, -a.second};
    for (const auto& p : points) {
      const XY q{c * p.first + s * p.second, -s * p.first + c * p.second};
      a.first = std::min(a.first, q.first);
      a.second = std::min(a.second, q.second);
      b.first = std::max(b.first, q.first);
      b.second = std::max(b.second, q.second);
    }
    const double area = std::max(config.resolution(), b.first - a.first) *
                        std::max(config.resolution(), b.second - a.second);
    if (area < best) {
      best = area;
      best_yaw = yaw;
      low = a;
      high = b;
    }
  }
  auto* box = object->mutable_box();
  const double cx = (low.first + high.first) * 0.5;
  const double cy = (low.second + high.second) * 0.5;
  box->mutable_center()->set_x(std::cos(best_yaw) * cx -
                               std::sin(best_yaw) * cy);
  box->mutable_center()->set_y(std::sin(best_yaw) * cx +
                               std::cos(best_yaw) * cy);
  box->mutable_size()->set_x(
      std::max(config.resolution(), high.first - low.first));
  box->mutable_size()->set_y(
      std::max(config.resolution(), high.second - low.second));
  box->set_qx(0);
  box->set_qy(0);
  box->set_qz(std::sin(best_yaw * 0.5));
  box->set_qw(std::cos(best_yaw * 0.5));
  object->set_orientation_reliable(false);
}

}  // namespace

void RefineObjectGeometry(
    const FusionConfig& c,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    ObjectCandidate* object, const PointIndex* index) {
  if (c.candidates().extend_detection_support()) {
    ExtendDetection(c, evidence, object, index);
  }
  if (c.candidates().oriented_recall() && object->source() == SEG_RECALL &&
      !object->point_id().empty()) {
    Fit(c, evidence, object);
  }
}

void RefineUnknownGeometry(
    const FusionConfig& c,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    UnknownCandidate* unknown) {
  if (!c.candidates().convex_unknown() || unknown->point_id().empty()) {
    return;
  }
  const auto hull =
      Hull(Points(evidence, unknown->point_id(), c.resolution() * 0.5));
  unknown->clear_polygon();
  for (const auto& xy : hull) {
    auto* p = unknown->add_polygon();
    p->set_x(xy.first);
    p->set_y(xy.second);
    p->set_z(unknown->z_min());
  }
}

}  // namespace candidate_detail
}  // namespace lidar_fusion
}  // namespace apollo
