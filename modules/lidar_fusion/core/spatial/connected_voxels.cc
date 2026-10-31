// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/lidar_fusion/core/spatial/connected_voxels.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace apollo {
namespace lidar_fusion {
namespace fusion_detail {

ConnectedVoxels::ConnectedVoxels(const std::vector<Eigen::Vector3d>& positions,
                                 const std::vector<bool>& eligible, double size)
    : count_(positions.size()) {
  std::vector<std::pair<VoxelKey, int>> sorted;
  sorted.reserve(positions.size());
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (eligible[i]) {
      const auto& p = positions[i];
      sorted.push_back({{static_cast<int>(std::floor(p.x() / size)),
                         static_cast<int>(std::floor(p.y() / size)),
                         static_cast<int>(std::floor(p.z() / size))},
                        i});
    }
  }
  std::sort(sorted.begin(), sorted.end());
  for (const auto& entry : sorted) {
    if (buckets_.empty() || buckets_.back().key != entry.first) {
      buckets_.push_back({entry.first, ids_.size(), ids_.size(), 0, 0});
    }
    ids_.push_back(entry.second);
    buckets_.back().end = ids_.size();
  }
  std::unordered_map<VoxelKey, int, Hash> lookup;
  lookup.reserve(buckets_.size());
  for (std::size_t i = 0; i < buckets_.size(); ++i) {
    lookup.emplace(buckets_[i].key, i);
  }
  std::vector<std::pair<int, int>> edges;
  edges.reserve(buckets_.size());
  for (std::size_t i = 0; i < buckets_.size(); ++i) {
    const auto [x, y, z] = buckets_[i].key;
    for (int dx = 0; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && (dy < 0 || (dy == 0 && dz <= 0))) {
            continue;
          }
          const auto found = lookup.find({x + dx, y + dy, z + dz});
          if (found != lookup.end()) {
            edges.emplace_back(i, found->second);
            ++buckets_[i].neighbor_end;
            ++buckets_[found->second].neighbor_end;
          }
        }
      }
    }
  }
  std::size_t neighbor_count = 0;
  for (auto& bucket : buckets_) {
    bucket.neighbor_begin = neighbor_count;
    neighbor_count += bucket.neighbor_end;
    bucket.neighbor_end = bucket.neighbor_begin;
  }
  neighbors_.resize(neighbor_count);
  for (const auto& edge : edges) {
    neighbors_[buckets_[edge.first].neighbor_end++] = edge.second;
    neighbors_[buckets_[edge.second].neighbor_end++] = edge.first;
  }
  // Query each undirected edge once, retaining the original neighbor order.
  for (auto& bucket : buckets_) {
    std::sort(neighbors_.begin() + bucket.neighbor_begin,
              neighbors_.begin() + bucket.neighbor_end);
  }
}

std::vector<int> ConnectedVoxels::Components(
    const std::vector<bool>& eligible) const {
  std::vector<int> labels(buckets_.size(), -1), result(count_, -1);
  for (std::size_t i = 0; i < buckets_.size(); ++i) {
    const auto& bucket = buckets_[i];
    for (std::size_t j = bucket.begin; j < bucket.end; ++j) {
      if (eligible[ids_[j]]) {
        labels[i] = -2;
        break;
      }
    }
  }
  int component = 0;
  std::vector<int> pending;
  pending.reserve(buckets_.size());
  for (std::size_t i = 0; i < buckets_.size(); ++i) {
    if (labels[i] != -2) {
      continue;
    }
    labels[i] = component;
    pending.clear();
    pending.push_back(i);
    for (std::size_t j = 0; j < pending.size(); ++j) {
      const auto& bucket = buckets_[pending[j]];
      for (std::size_t k = bucket.neighbor_begin; k < bucket.neighbor_end;
           ++k) {
        const int next = neighbors_[k];
        if (labels[next] == -2) {
          labels[next] = component;
          pending.push_back(next);
        }
      }
    }
    ++component;
  }
  for (std::size_t i = 0; i < buckets_.size(); ++i) {
    const auto& bucket = buckets_[i];
    for (std::size_t j = bucket.begin; j < bucket.end; ++j) {
      if (eligible[ids_[j]]) {
        result[ids_[j]] = labels[i];
      }
    }
  }
  return result;
}

void ConnectedVoxels::Count(FusionWork* work) const {
  if (work != nullptr) {
    work->voxels = buckets_.size();
    work->neighbor_edges += neighbors_.size();
  }
}

}  // namespace fusion_detail
}  // namespace lidar_fusion
}  // namespace apollo
