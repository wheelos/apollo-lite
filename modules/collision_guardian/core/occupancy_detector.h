#pragma once

#include <cstdint>
#include <limits>
#include <vector>

#include "Eigen/Core"

namespace apollo {
namespace collision_guardian {

struct OccupancyDetectorConfig {
  double ego_forward_m = 0.0;
  double ego_backward_m = 0.0;
  double ego_half_width_m = 0.0;
  double roi_forward_m = 0.0;
  double roi_backward_m = 0.0;
  double roi_half_width_m = 0.0;
  double min_height_m = 0.0;
  double max_height_m = 0.0;
  double voxel_size_x_m = 0.1;
  double voxel_size_y_m = 0.1;
  double voxel_size_z_m = 0.1;
  bool y_axis_is_forward = false;
};

struct OccupancyResult {
  uint32_t occupied_voxel_count = 0;
  double nearest_distance_m = std::numeric_limits<double>::infinity();
  double nearest_forward_m = 0.0;
  double nearest_lateral_m = 0.0;
};

class OccupancyDetector {
 public:
  explicit OccupancyDetector(const OccupancyDetectorConfig& config)
      : config_(config) {}

  OccupancyResult Evaluate(
      const std::vector<Eigen::Vector3d>& points_in_vehicle) const;

 private:
  OccupancyDetectorConfig config_;
};

}  // namespace collision_guardian
}  // namespace apollo
