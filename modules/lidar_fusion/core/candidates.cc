// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_fusion/core/candidates.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "Eigen/Geometry"

namespace apollo {
namespace lidar_fusion {
namespace {
using XY = std::pair<double, double>;
using Voxel = std::tuple<int, int, int>;

struct PointBounds {
  Eigen::Vector3d low =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d high = -low;
  bool valid = false;
};

PointBounds ActualBounds(
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    const google::protobuf::RepeatedField<uint32_t>& ids) {
  PointBounds bounds;
  for (uint32_t id : ids) {
    const auto& p = evidence.Get(id).position();
    const Eigen::Vector3d point(p.x(), p.y(), p.z());
    if (!point.allFinite()) {
      return PointBounds{};
    }
    bounds.low = bounds.low.cwiseMin(point);
    bounds.high = bounds.high.cwiseMax(point);
    bounds.valid = true;
  }
  return bounds;
}

struct PointBlock {
  int begin = 0;
  int end = 0;
  PointBounds bounds;
};

std::vector<PointBlock> PointBlocks(
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    const UnknownCandidate& unknown) {
  std::vector<PointBlock> blocks;
  for (int begin = 0; begin < unknown.point_id_size(); begin += 128) {
    PointBlock block;
    block.begin = begin;
    block.end = std::min(begin + 128, unknown.point_id_size());
    bool finite = true;
    for (int i = block.begin; i < block.end; ++i) {
      const auto& p = evidence.Get(unknown.point_id(i)).position();
      const Eigen::Vector3d point(p.x(), p.y(), p.z());
      finite &= point.allFinite();
      block.bounds.low = block.bounds.low.cwiseMin(point);
      block.bounds.high = block.bounds.high.cwiseMax(point);
    }
    block.bounds.valid = finite;
    blocks.push_back(block);
  }
  return blocks;
}

struct ObjectSupport {
  explicit ObjectSupport(const ObjectCandidate& object)
      : rotation(Eigen::Quaterniond(object.box().qw(), object.box().qx(),
                                    object.box().qy(), object.box().qz())
                     .conjugate()),
        ids(object.point_id().begin(), object.point_id().end()) {
    const auto& box = object.box();
    const Eigen::Vector3d center(box.center().x(), box.center().y(),
                                 box.center().z());
    const Eigen::Vector3d half(box.size().x() * 0.5, box.size().y() * 0.5,
                               box.size().z() * 0.5);
    const double epsilon = std::numeric_limits<double>::epsilon();
    if (!center.allFinite() || !half.allFinite() || (half.array() < 0).any() ||
        !std::isfinite(rotation.squaredNorm()) ||
        std::abs(rotation.squaredNorm() - 1) > 32 * epsilon) {
      return;
    }
    const Eigen::Vector3d extent =
        rotation.conjugate().toRotationMatrix().cwiseAbs() * half;
    bounds.low = center - extent;
    bounds.high = center + extent;
    bounds.valid = bounds.low.allFinite() && bounds.high.allFinite();
    scale = center.cwiseAbs().maxCoeff() + extent.sum() + half.sum();
  }

  bool Separated(const PointBounds& points, double adjacent) const {
    if (!points.valid || !bounds.valid || !std::isfinite(adjacent) ||
        adjacent < 0) {
      return false;
    }
    const double magnitude =
        1 + scale + adjacent +
        points.low.cwiseAbs().cwiseMax(points.high.cwiseAbs()).maxCoeff();
    if (!std::isfinite(magnitude) ||
        magnitude > std::sqrt(std::numeric_limits<double>::max()) / 32) {
      return false;
    }
    // Cover quaternion norm error and all bound/transform rounding. Uncertain
    // or extreme inputs retain the original exact relation calculation.
    const double guard =
        4096 * std::numeric_limits<double>::epsilon() * magnitude;
    const Eigen::Vector3d gap = (bounds.low - points.high)
                                    .cwiseMax(points.low - bounds.high)
                                    .cwiseMax(0.);
    return gap.norm() > adjacent + guard;
  }

  Eigen::Quaterniond rotation;
  std::set<uint32_t> ids;
  PointBounds bounds;
  double scale = 0;
};

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

bool ValidateCandidates(const FusionConfig& c) {
  const auto& o = c.candidates();
  return std::isfinite(o.adjacent_distance()) && o.adjacent_distance() >= 0 &&
         std::isfinite(o.far_distance()) && o.far_distance() > 0 &&
         (!o.has_far_connectivity_size() ||
          (std::isfinite(o.far_connectivity_size()) &&
           o.far_connectivity_size() >= c.connectivity_size() &&
           o.far_connectivity_size() <= c.connectivity_size() * 4));
}

std::vector<int> CandidateComponents(
    const FusionConfig& c,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    const google::protobuf::RepeatedPtrField<PointDecision>& decisions) {
  const int count = evidence.size();
  std::vector<int> parent(count, -1);
  std::map<Voxel, std::vector<int>> voxels;
  const double size = c.connectivity_size();
  const auto& options = c.candidates();
  auto extent = [&](const PointEvidence& p) {
    return options.has_far_connectivity_size() &&
                   std::hypot(p.position().x(), p.position().y()) >=
                       options.far_distance()
               ? options.far_connectivity_size()
               : size;
  };
  auto root = [&](int id) {
    int start = id;
    while (parent[id] != id) {
      id = parent[id];
    }
    while (parent[start] != start) {
      const int next = parent[start];
      parent[start] = id;
      start = next;
    }
    return id;
  };
  for (int i = 0; i < count; ++i) {
    const auto& p = evidence.Get(i);
    const auto& q = p.position();
    const auto state = decisions.Get(i).explanation();
    if (!p.finite() || q.x() < c.min_x() || q.x() >= c.max_x() ||
        q.y() < c.min_y() || q.y() >= c.max_y() ||
        (p.ground_valid() &&
         (p.height_above_ground() < c.obstacle_min_height() ||
          p.height_above_ground() > c.clearance_height())) ||
        (state != UNEXPLAINED && state != EXPL_OBJECT_WEAK)) {
      continue;
    }
    parent[i] = i;
    const int x = std::floor(q.x() / size), y = std::floor(q.y() / size),
              z = std::floor(q.z() / size);
    const int radius = std::ceil((options.has_far_connectivity_size()
                                      ? options.far_connectivity_size()
                                      : size) /
                                 size);
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dz = -radius; dz <= radius; ++dz) {
          auto it = voxels.find({x + dx, y + dy, z + dz});
          if (it == voxels.end()) {
            continue;
          }
          for (int j : it->second) {
            const auto& other = evidence.Get(j);
            if (options.separate_semantic_instances() &&
                ((p.group() == THING) != (other.group() == THING) ||
                 (p.group() == THING &&
                  p.unified_class() != other.unified_class()))) {
              continue;
            }
            const auto& b = other.position();
            const double tolerance = std::min(extent(p), extent(other));
            if (std::max({std::abs(q.x() - b.x()), std::abs(q.y() - b.y()),
                          std::abs(q.z() - b.z())}) <= tolerance) {
              parent[root(i)] = root(j);
            }
          }
        }
      }
    }
    voxels[{x, y, z}].push_back(i);
  }
  for (int i = 0; i < count; ++i) {
    if (parent[i] >= 0) {
      parent[i] = root(i);
    }
  }
  return parent;
}

void RefineCandidates(
    const FusionConfig& c,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    CandidateResult* candidates, CandidateWork* work) {
  if (work != nullptr) {
    *work = CandidateWork{};
  }
  for (auto& object : candidates->objects) {
    if (c.candidates().oriented_recall() && object.source() == SEG_RECALL &&
        !object.point_id().empty()) {
      Fit(c, evidence, &object);
    }
  }
  if (candidates->unknowns.empty()) {
    return;
  }
  std::vector<ObjectSupport, Eigen::aligned_allocator<ObjectSupport>> support;
  support.reserve(candidates->objects.size());
  for (const auto& object : candidates->objects) {
    support.emplace_back(object);
  }
  for (auto& unknown : candidates->unknowns) {
    if (c.candidates().convex_unknown() && !unknown.point_id().empty()) {
      const auto hull =
          Hull(Points(evidence, unknown.point_id(), c.resolution() * 0.5));
      unknown.clear_polygon();
      for (const auto& xy : hull) {
        auto* p = unknown.add_polygon();
        p->set_x(xy.first);
        p->set_y(xy.second);
        p->set_z(unknown.z_min());
      }
    }
    unknown.clear_relation();
    std::set<uint32_t> references(unknown.related_object_id().begin(),
                                  unknown.related_object_id().end());
    const PointBounds points = ActualBounds(evidence, unknown.point_id());
    std::vector<PointBlock> blocks;
    for (int i = 0; i < candidates->objects.size(); ++i) {
      const auto& object = candidates->objects.Get(i);
      const auto& cached = support[i];
      if (references.count(object.id()) == 0 &&
          cached.Separated(points, c.candidates().adjacent_distance())) {
        continue;
      }
      const auto& b = object.box();
      if (work != nullptr) {
        ++work->pairs;
      }
      const bool referenced = references.count(object.id()) != 0;
      const bool duplicate =
          !object.class_conflict() && !unknown.point_id().empty() &&
          std::all_of(unknown.point_id().begin(), unknown.point_id().end(),
                      [&](uint32_t id) { return cached.ids.count(id) != 0; });
      bool inside = false;
      double distance = std::numeric_limits<double>::infinity();
      if (!(referenced && (object.class_conflict() || duplicate))) {
        if (blocks.empty()) {
          blocks = PointBlocks(evidence, unknown);
        }
        bool decided = false;
        for (const auto& block : blocks) {
          // A reference already establishes a relation; only inside support
          // can change its kind. Otherwise retain all possible near support.
          if (cached.Separated(
                  block.bounds,
                  referenced ? 0 : c.candidates().adjacent_distance())) {
            continue;
          }
          for (int j = block.begin; j < block.end; ++j) {
            const uint32_t id = unknown.point_id(j);
            if (work != nullptr) {
              ++work->point_transforms;
            }
            const auto& p = evidence.Get(id).position();
            const Eigen::Vector3d local =
                cached.rotation * Eigen::Vector3d(p.x() - b.center().x(),
                                                  p.y() - b.center().y(),
                                                  p.z() - b.center().z());
            const double dx =
                std::max(0., std::abs(local.x()) - b.size().x() * 0.5);
            const double dy =
                std::max(0., std::abs(local.y()) - b.size().y() * 0.5);
            const double dz =
                std::max(0., std::abs(local.z()) - b.size().z() * 0.5);
            distance =
                std::min(distance, std::sqrt(dx * dx + dy * dy + dz * dz));
            inside |= dx == 0 && dy == 0 && dz == 0;
            decided =
                inside || ((object.class_conflict() || duplicate) &&
                           distance <= c.candidates().adjacent_distance());
            if (decided) {
              break;
            }
          }
          if (decided) {
            break;
          }
        }
      }
      if (distance > c.candidates().adjacent_distance() && !referenced) {
        continue;
      }
      auto* relation = unknown.add_relation();
      relation->set_object_id(object.id());
      references.insert(object.id());
      if (object.class_conflict()) {
        relation->set_kind(RELATION_AMBIGUOUS);
      } else if (duplicate) {
        relation->set_kind(RELATION_DUPLICATE);
      } else if (inside) {
        relation->set_kind(RELATION_SUPPLEMENT);
      } else {
        relation->set_kind(RELATION_ADJACENT);
      }
    }
    unknown.clear_related_object_id();
    for (uint32_t id : references) {
      unknown.add_related_object_id(id);
    }
  }
}

}  // namespace lidar_fusion
}  // namespace apollo
