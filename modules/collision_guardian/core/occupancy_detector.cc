#include "modules/collision_guardian/core/occupancy_detector.h"

#include <cmath>
#include <cstddef>
#include <unordered_set>

namespace apollo {
namespace collision_guardian {
namespace {

struct VoxelKey {
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;

  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  size_t operator()(const VoxelKey& key) const {
    size_t seed = std::hash<int64_t>{}(key.x);
    seed ^= std::hash<int64_t>{}(key.y) + 0x9e3779b9 + (seed << 6) +
            (seed >> 2);
    seed ^= std::hash<int64_t>{}(key.z) + 0x9e3779b9 + (seed << 6) +
            (seed >> 2);
    return seed;
  }
};

}  // namespace

OccupancyResult OccupancyDetector::Evaluate(
    const std::vector<Eigen::Vector3d>& points_in_vehicle) const {
  OccupancyResult result;
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels;

  for (const auto& point : points_in_vehicle) {
    if (!point.allFinite() || point.z() < config_.min_height_m ||
        point.z() > config_.max_height_m) {
      continue;
    }

    const double forward =
        config_.y_axis_is_forward ? point.y() : point.x();
    const double lateral =
        config_.y_axis_is_forward ? point.x() : point.y();
    if (forward <= config_.ego_forward_m &&
        forward >= -config_.ego_backward_m &&
        std::abs(lateral) <= config_.ego_half_width_m) {
      continue;
    }
    if (forward > config_.roi_forward_m ||
        forward < -config_.roi_backward_m ||
        std::abs(lateral) > config_.roi_half_width_m) {
      continue;
    }

    const VoxelKey key{
        static_cast<int64_t>(std::floor(point.x() / config_.voxel_size_x_m)),
        static_cast<int64_t>(std::floor(point.y() / config_.voxel_size_y_m)),
        static_cast<int64_t>(std::floor(point.z() / config_.voxel_size_z_m))};
    if (!occupied_voxels.insert(key).second) {
      continue;
    }

    const double distance = std::hypot(forward, lateral);
    if (distance < result.nearest_distance_m) {
      result.nearest_distance_m = distance;
      result.nearest_forward_m = forward;
      result.nearest_lateral_m = lateral;
    }
  }

  result.occupied_voxel_count =
      static_cast<uint32_t>(occupied_voxels.size());
  return result;
}

}  // namespace collision_guardian
}  // namespace apollo
