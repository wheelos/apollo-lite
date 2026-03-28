#include <cmath>
#include <unordered_set>

#include "modules/drivers/lidar/processor/policy/cpu_lidar_policy.h"
#include "modules/drivers/lidar/processor/policy/lidar_policy_common.h"

namespace apollo {
namespace drivers {
namespace lidar {

namespace {

struct VoxelKey {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  size_t operator()(const VoxelKey& key) const {
    const size_t h1 = std::hash<int>()(key.x);
    const size_t h2 = std::hash<int>()(key.y);
    const size_t h3 = std::hash<int>()(key.z);
    return h1 ^ (h2 << 1U) ^ (h3 << 2U);
  }
};

}  // namespace

bool CpuLidarFilterPolicy::Init(const LidarUnifiedComponentConfig& config) {
  config_ = config;
  return true;
}

size_t CpuLidarFilterPolicy::ApplyFilters(PointCloudBuffer* io_buffer,
                                          size_t* ego_filtered_count,
                                          size_t* voxel_filtered_count) {
  if (ego_filtered_count != nullptr) {
    *ego_filtered_count = 0;
  }
  if (voxel_filtered_count != nullptr) {
    *voxel_filtered_count = 0;
  }

  if (GetHostPoints(io_buffer) == nullptr || io_buffer->valid_count == 0) {
    return 0;
  }

  const size_t before_ego = io_buffer->valid_count;
  const size_t after_ego = ApplyEgoQueryFilter(io_buffer);
  if (ego_filtered_count != nullptr) {
    *ego_filtered_count = before_ego - after_ego;
  }

  const size_t before_voxel = io_buffer->valid_count;
  const size_t after_voxel = ApplyVoxelFilter(io_buffer);
  if (voxel_filtered_count != nullptr) {
    *voxel_filtered_count = before_voxel - after_voxel;
  }

  return io_buffer->valid_count;
}

size_t CpuLidarFilterPolicy::ApplyVoxelFilter(PointCloudBuffer* io_buffer) {
  PointXYZIT* points = GetHostPoints(io_buffer);
  if (points == nullptr || io_buffer->valid_count == 0) {
    return 0;
  }

  const float voxel_size = config_.voxel_size();
  if (voxel_size <= 1e-4f) {
    return io_buffer->valid_count;
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> visited;
  visited.reserve(io_buffer->valid_count);

  size_t write_idx = 0;
  for (size_t i = 0; i < io_buffer->valid_count; ++i) {
    const auto& point = points[i];
    const VoxelKey key{static_cast<int>(std::floor(point.x() / voxel_size)),
                       static_cast<int>(std::floor(point.y() / voxel_size)),
                       static_cast<int>(std::floor(point.z() / voxel_size))};
    if (visited.insert(key).second) {
      if (write_idx != i) {
        points[write_idx] = point;
      }
      ++write_idx;
    }
  }

  io_buffer->valid_count = write_idx;
  return write_idx;
}

size_t CpuLidarFilterPolicy::ApplyEgoQueryFilter(PointCloudBuffer* io_buffer) {
  PointXYZIT* points = GetHostPoints(io_buffer);
  if (points == nullptr || io_buffer->valid_count == 0 ||
      !config_.enable_ego_query_filter()) {
    return io_buffer == nullptr ? 0 : io_buffer->valid_count;
  }

  size_t write_idx = 0;
  for (size_t i = 0; i < io_buffer->valid_count; ++i) {
    const auto& point = points[i];
    const bool in_ego_box = point.x() < config_.ego_box_forward_x() &&
                            point.x() > config_.ego_box_backward_x() &&
                            point.y() < config_.ego_box_forward_y() &&
                            point.y() > config_.ego_box_backward_y();
    if (!in_ego_box) {
      if (write_idx != i) {
        points[write_idx] = point;
      }
      ++write_idx;
    }
  }

  io_buffer->valid_count = write_idx;
  return write_idx;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
