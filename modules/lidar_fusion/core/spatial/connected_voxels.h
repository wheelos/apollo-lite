// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include <cstddef>
#include <functional>
#include <tuple>
#include <vector>

#include "Eigen/Core"

#include "modules/lidar_fusion/core/runtime/single_frame_fusion.h"

namespace apollo {
namespace lidar_fusion {
namespace fusion_detail {

class ConnectedVoxels {
 public:
  ConnectedVoxels(const std::vector<Eigen::Vector3d>& positions,
                  const std::vector<bool>& eligible, double size);
  std::vector<int> Components(const std::vector<bool>& eligible) const;
  void Count(FusionWork* work) const;

 private:
  using VoxelKey = std::tuple<int, int, int>;
  struct Hash {
    std::size_t operator()(const VoxelKey& key) const {
      std::size_t result = 0;
      for (int value : {std::get<0>(key), std::get<1>(key), std::get<2>(key)}) {
        result ^= std::hash<int>{}(value) + 0x9e3779b9 + (result << 6) +
                  (result >> 2);
      }
      return result;
    }
  };
  struct Bucket {
    VoxelKey key;
    std::size_t begin;
    std::size_t end;
    std::size_t neighbor_begin;
    std::size_t neighbor_end;
  };

  std::size_t count_;
  std::vector<int> ids_;
  std::vector<Bucket> buckets_;
  std::vector<int> neighbors_;
};

}  // namespace fusion_detail
}  // namespace lidar_fusion
}  // namespace apollo
