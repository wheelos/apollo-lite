// Copyright 2026 WheelOS. All Rights Reserved.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"

#include "modules/lidar_fusion/core/association/candidates.h"

namespace apollo {
namespace lidar_fusion {
namespace {

using Voxel = std::tuple<int, int, int>;

struct PointBounds {
  Eigen::Vector3d low =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d high = -low;
};

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

}  // namespace lidar_fusion
}  // namespace apollo
