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
#include <unordered_map>
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

struct VoxelHash {
  std::size_t operator()(const Voxel& voxel) const {
    std::size_t value = std::hash<int>{}(std::get<0>(voxel));
    for (int coordinate : {std::get<1>(voxel), std::get<2>(voxel)}) {
      value ^= std::hash<int>{}(coordinate) + 0x9e3779b9 + (value << 6) +
               (value >> 2);
    }
    return value;
  }
};

struct ComponentVoxel {
  std::vector<int> points;
  PointBounds bounds;
  bool clique = true;
};

template <typename Point, typename State>
std::vector<int> Components(const FusionConfig& c, int count,
                            const Point& point, const State& state) {
  std::vector<int> parent(count, -1);
  std::unordered_map<Voxel, ComponentVoxel, VoxelHash> voxels;
  voxels.reserve(count);
  const double size = c.connectivity_size();
  const auto& options = c.candidates();
  const int radius = std::ceil((options.has_far_connectivity_size()
                                    ? options.far_connectivity_size()
                                    : size) /
                               size);
  auto extent = [&](const PointEvidence& p) {
    return options.has_far_connectivity_size() &&
                   std::hypot(p.position().x(), p.position().y()) >=
                       options.far_distance()
               ? options.far_connectivity_size()
               : size;
  };
  auto compatible = [&](const PointEvidence& a, const PointEvidence& b) {
    return !options.separate_semantic_instances() ||
           ((a.group() == THING) == (b.group() == THING) &&
            (a.group() != THING || a.unified_class() == b.unified_class()));
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
    const auto& p = point(i);
    const auto& q = p.position();
    const auto explanation = state(i);
    if (!p.finite() || q.x() < c.min_x() || q.x() >= c.max_x() ||
        q.y() < c.min_y() || q.y() >= c.max_y() ||
        (p.ground_valid() &&
         (p.height_above_ground() < c.obstacle_min_height() ||
          p.height_above_ground() > c.clearance_height())) ||
        (explanation != UNEXPLAINED && explanation != EXPL_OBJECT_WEAK)) {
      continue;
    }
    parent[i] = i;
    const int x = std::floor(q.x() / size), y = std::floor(q.y() / size),
              z = std::floor(q.z() / size);
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dz = -radius; dz <= radius; ++dz) {
          const auto it = voxels.find({x + dx, y + dy, z + dz});
          if (it == voxels.end()) {
            continue;
          }
          const auto& voxel = it->second;
          if (voxel.clique && root(i) == root(voxel.points.front())) {
            continue;
          }
          for (int j : voxel.points) {
            const auto& other = point(j);
            if (!compatible(p, other)) {
              continue;
            }
            const auto& b = other.position();
            const double tolerance = std::min(extent(p), extent(other));
            if (std::max({std::abs(q.x() - b.x()), std::abs(q.y() - b.y()),
                          std::abs(q.z() - b.z())}) <= tolerance) {
              parent[root(i)] = root(j);
              // Verified cliques share one root; later members cannot change
              // it.
              if (voxel.clique) {
                break;
              }
            }
          }
        }
      }
    }
    auto& voxel = voxels[{x, y, z}];
    const Eigen::Vector3d position(q.x(), q.y(), q.z());
    voxel.bounds.low = voxel.bounds.low.cwiseMin(position);
    voxel.bounds.high = voxel.bounds.high.cwiseMax(position);
    voxel.clique &=
        (voxel.bounds.high - voxel.bounds.low).maxCoeff() <= size &&
        (voxel.points.empty() || compatible(p, point(voxel.points.front())));
    voxel.points.push_back(i);
  }
  for (int i = 0; i < count; ++i) {
    if (parent[i] >= 0) {
      parent[i] = root(i);
    }
  }
  return parent;
}
}  // namespace

bool ValidateCandidates(const FusionConfig& c) {
  const auto& o = c.candidates();
  return std::isfinite(o.adjacent_distance()) && o.adjacent_distance() >= 0 &&
         std::isfinite(o.far_distance()) && o.far_distance() > 0 &&
         (!o.recall_confident_foreground() ||
          (!o.separate_semantic_instances() &&
           !o.has_far_connectivity_size())) &&
         (!o.has_far_connectivity_size() ||
          (std::isfinite(o.far_connectivity_size()) &&
           o.far_connectivity_size() >= c.connectivity_size() &&
           o.far_connectivity_size() <= c.connectivity_size() * 4));
}

std::vector<int> CandidateComponents(
    const FusionConfig& c,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    const google::protobuf::RepeatedPtrField<PointDecision>& decisions) {
  return Components(
      c, evidence.size(),
      [&](int i) -> const PointEvidence& { return evidence.Get(i); },
      [&](int i) { return decisions.Get(i).explanation(); });
}

void PartitionUnknowns(
    const FusionConfig& c,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    const google::protobuf::RepeatedPtrField<UnknownCandidate>& unknowns,
    google::protobuf::RepeatedPtrField<ShadowPartition>* partitions) {
  partitions->Clear();
  if (!c.candidates().shadow_semantic_partitions() ||
      c.quality().semantic_identity_unverified()) {
    return;
  }
  FusionConfig local = c;
  local.mutable_candidates()->clear_far_connectivity_size();
  local.mutable_candidates()->set_separate_semantic_instances(false);
  using Bucket = std::tuple<PartitionKind, Group, UnifiedClass>;
  const Bucket unexplained{PARTITION_UNEXPLAINED, GENERIC_OBS, CLASS_UNKNOWN};
  auto category = [&](const PointEvidence& p) -> Bucket {
    if (p.has_semantic_probability() &&
        p.semantic_probability() >= c.semantic_min_confidence()) {
      if (p.group() == THING && p.unified_class() != CLASS_UNKNOWN) {
        return {PARTITION_THING_SUPPORT, THING, p.unified_class()};
      }
      if (p.group() != THING && p.group() != GENERIC_OBS) {
        return {PARTITION_GROUP_SUPPORT, p.group(), CLASS_UNKNOWN};
      }
    }
    return unexplained;
  };
  for (const auto& unknown : unknowns) {
    std::vector<uint32_t> ids(unknown.point_id().begin(),
                              unknown.point_id().end());
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    if (ids.empty() || ids.back() >= static_cast<uint32_t>(evidence.size())) {
      continue;
    }
    std::map<Bucket, std::vector<uint32_t>> buckets;
    bool has_thing = false;
    for (uint32_t id : ids) {
      const auto key = category(evidence.Get(id));
      has_thing |= std::get<0>(key) == PARTITION_THING_SUPPORT;
      buckets[key].push_back(id);
    }
    if (!has_thing) {
      continue;
    }
    auto publish = [&](const Bucket& key, const std::vector<uint32_t>& points) {
      if (points.empty()) {
        return;
      }
      auto* partition = partitions->Add();
      partition->set_id(partitions->size());
      partition->set_parent_unknown_id(unknown.id());
      partition->set_kind(std::get<0>(key));
      partition->set_group(std::get<1>(key));
      partition->set_unified_class(std::get<2>(key));
      for (uint32_t id : points) {
        partition->add_point_id(id);
      }
    };
    auto residual = buckets[unexplained];
    for (const auto& entry : buckets) {
      if (entry.first == unexplained) {
        continue;
      }
      const auto components = Components(
          local, entry.second.size(),
          [&](int i) -> const PointEvidence& {
            return evidence.Get(entry.second[i]);
          },
          [](int) { return UNEXPLAINED; });
      std::map<int, std::vector<uint32_t>> connected;
      for (int i = 0; i < static_cast<int>(entry.second.size()); ++i) {
        if (components[i] < 0) {
          residual.push_back(entry.second[i]);
        } else {
          connected[components[i]].push_back(entry.second[i]);
        }
      }
      for (const auto& component : connected) {
        publish(entry.first, component.second);
      }
    }
    std::sort(residual.begin(), residual.end());
    publish(unexplained, residual);
  }
}

void RefineCandidates(
    const FusionConfig& c,
    const google::protobuf::RepeatedPtrField<PointEvidence>& evidence,
    CandidateResult* candidates, CandidateWork* work, const PointIndex* index) {
  if (work != nullptr) {
    *work = CandidateWork{};
  }
  for (auto& object : candidates->objects) {
    if (c.candidates().extend_detection_support()) {
      ExtendDetection(c, evidence, &object, index);
    }
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
