// Copyright 2026 WheelOS. All Rights Reserved.

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "modules/lidar_fusion/core/runtime/fusion_test_support.h"

namespace apollo {
namespace lidar_fusion {
using testing_support::Config;
using testing_support::FusionTest;

TEST_F(FusionTest, InstanceRejectionDoesNotClearMeasuredOrInferredOccupancy) {
  Obstacle(0);
  Box(0.15F);
  Run(&det, &seg);
  ASSERT_EQ(output.hypothesis_size(), 1);
  EXPECT_EQ(output.object_size(), 0);
  EXPECT_TRUE(std::any_of(output.space_map().cell().begin(),
                          output.space_map().cell().end(),
                          [](const auto& c) { return c.occupancy_obs(); }));
  EXPECT_TRUE(std::any_of(output.space_map().cell().begin(),
                          output.space_map().cell().end(),
                          [](const auto& c) { return c.occupancy_det(); }));
}

TEST_F(FusionTest, NearGroundEndpointCannotEraseSameCellMeasuredObstacle) {
  Add(2, 1.4, 0, 9);
  Sync();
  const auto ground_cloud = cloud;
  const auto ground_seg = seg;
  std::string space;
  for (bool obstacle_first : {true, false}) {
    cloud = ground_cloud;
    seg = ground_seg;
    for (double height :
         {obstacle_first ? 0.8 : 0.0625, obstacle_first ? 0.0625 : 0.8}) {
      Add(2, 1.4, height, 1);
    }
    Sync();
    Run(nullptr, &seg);
    const int low = ground_cloud.point_size() + (obstacle_first ? 1 : 0);
    EXPECT_TRUE(output.point_evidence(low).is_ground_geo());
    int observed = 0;
    for (const auto& cell : output.space_map().cell()) {
      if (cell.occupancy_obs()) {
        ++observed;
        EXPECT_TRUE(cell.ground_supported());
        EXPECT_EQ(cell.traversability(), OCCUPIED);
        EXPECT_FALSE(cell.reachable());
        EXPECT_FALSE(cell.physically_traversable());
      }
    }
    EXPECT_EQ(observed, 1);
    if (obstacle_first) {
      space = output.space_map().SerializeAsString();
    } else {
      EXPECT_EQ(output.space_map().SerializeAsString(), space);
    }
  }
}

TEST_F(FusionTest, ReadOnlyViewsPreserveEvidenceAndBackgroundOccupancy) {
  Obstacle(13);
  Run(nullptr, &seg);
  const auto original = output;
  config.mutable_spatial()->set_point_views(true);
  Run(nullptr, &seg);
  ASSERT_EQ(output.point_view_size(), 6);
  EXPECT_EQ(output.point_view(STRUCTURE_POINTS).point_id_size(), 6);
  EXPECT_EQ(output.point_view(NON_GROUND_POINTS).point_id_size(), 6);
  EXPECT_EQ(output.space_map().SerializeAsString(),
            original.space_map().SerializeAsString());
  EXPECT_EQ(output.object_size(), original.object_size());
  for (int i = 0; i < output.point_evidence_size(); ++i) {
    EXPECT_EQ(output.point_evidence(i).SerializeAsString(),
              original.point_evidence(i).SerializeAsString());
  }
  EXPECT_EQ(output.point_evidence(0).ground_seed_count(), 6U);
  EXPECT_NEAR(output.point_evidence(0).ground_slope(), 0, 1e-10);
}

}  // namespace lidar_fusion
}  // namespace apollo
