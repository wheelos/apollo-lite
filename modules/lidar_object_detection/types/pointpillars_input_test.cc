// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_object_detection/types/pointpillars_input.h"

#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace apollo::lidar_object_detection {
TEST(PointPillarsInput, SingleSweepXyziAndExplicitScale) {
  apollo::drivers::PointCloud cloud;
  cloud.set_frame_id("lidar");
  cloud.set_measurement_time(42);
  auto* point = cloud.add_point();
  point->set_x(1);
  point->set_y(2);
  point->set_z(3);
  point->set_intensity(128);
  std::vector<float> points;
  std::string error;
  ASSERT_TRUE(BuildPointPillarsInput(cloud, "lidar", 0.5F, &points, &error));
  EXPECT_EQ(points, (std::vector<float>{1, 2, 3, 64}));
  EXPECT_FALSE(BuildPointPillarsInput(cloud, "other", 0.5F, &points, &error));
  EXPECT_FALSE(BuildPointPillarsInput(cloud, "lidar", 0, &points, &error));
  cloud.mutable_point(0)->set_x(std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(BuildPointPillarsInput(cloud, "lidar", 1, &points, &error));
  EXPECT_EQ(points, (std::vector<float>{1, 2, 3, 64}));
}
TEST(PointPillarsInput, RejectsEmptyAndOversizeCloud) {
  apollo::drivers::PointCloud cloud;
  cloud.set_frame_id("lidar");
  cloud.set_measurement_time(42);
  std::vector<float> points;
  std::string error;
  EXPECT_FALSE(BuildPointPillarsInput(cloud, "lidar", 1, &points, &error));
  for (int i = 0; i <= 300000; ++i) cloud.add_point();
  EXPECT_FALSE(BuildPointPillarsInput(cloud, "lidar", 1, &points, &error));
}
}  // namespace apollo::lidar_object_detection
