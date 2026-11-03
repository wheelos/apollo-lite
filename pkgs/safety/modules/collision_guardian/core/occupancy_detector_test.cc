#include "modules/collision_guardian/core/occupancy_detector.h"

#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace collision_guardian {

TEST(OccupancyDetectorTest, RemovesEgoAndDeduplicatesVoxels) {
  OccupancyDetectorConfig config;
  config.ego_forward_m = 1.0;
  config.ego_backward_m = 1.0;
  config.ego_half_width_m = 0.5;
  config.roi_forward_m = 2.0;
  config.roi_backward_m = 2.0;
  config.roi_half_width_m = 1.0;
  config.min_height_m = -0.5;
  config.max_height_m = 1.0;
  config.voxel_size_x_m = 0.1;
  config.voxel_size_y_m = 0.1;
  config.voxel_size_z_m = 0.1;
  OccupancyDetector detector(config);

  const std::vector<Eigen::Vector3d> points = {
      {0.5, 0.0, 0.0},
      {1.2, 0.0, 0.0},
      {1.2, 0.0, 0.0},
      {3.0, 0.0, 0.0},
  };
  const auto result = detector.Evaluate(points);

  EXPECT_EQ(result.occupied_voxel_count, 1U);
  EXPECT_NEAR(result.nearest_distance_m, 1.2, 1.0e-6);
}

}  // namespace collision_guardian
}  // namespace apollo
