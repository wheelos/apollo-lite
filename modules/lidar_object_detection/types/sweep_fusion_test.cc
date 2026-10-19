// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/lidar_object_detection/types/sweep_fusion.h"

#include <cstdint>
#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "modules/lidar_object_detection/proto/lidar_object_detection.pb.h"

#include "modules/lidar_object_detection/types/model_input.h"

namespace apollo {
namespace lidar_object_detection {

namespace {

std::shared_ptr<apollo::drivers::PointCloud> MakeCloud(double timestamp,
                                                       float x, float y) {
  auto cloud = std::make_shared<apollo::drivers::PointCloud>();
  cloud->set_measurement_time(timestamp);
  cloud->set_frame_id("lidar");
  auto* point = cloud->add_point();
  point->set_x(x);
  point->set_y(y);
  point->set_z(0.0F);
  point->set_intensity(17U);
  return cloud;
}

void AddPoint(float x, float y, apollo::drivers::PointCloud* cloud) {
  auto* point = cloud->add_point();
  point->set_x(x);
  point->set_y(y);
  point->set_z(0.0F);
  point->set_intensity(17U);
}

CenterPointModelConfig TestConfig() {
  CenterPointModelConfig config;
  config.add_input_feature()->set_source(POINT_FEATURE_X);
  config.add_input_feature()->set_source(POINT_FEATURE_Y);
  config.add_input_feature()->set_source(POINT_FEATURE_Z);
  config.add_input_feature()->set_source(POINT_FEATURE_INTENSITY);
  config.add_input_feature()->set_source(POINT_FEATURE_TIME_OFFSET_SECONDS);
  return config;
}

}  // namespace

TEST(PointCloudSweepFusionTest, AlignsPastFrameAndEncodesPositiveTimeLag) {
  PointCloudSweepFusion fusion;
  std::string error;
  ASSERT_TRUE(fusion.Init(2U, 300000U, &error)) << error;

  Eigen::Affine3d past_pose = Eigen::Affine3d::Identity();
  apollo::drivers::PointCloud fused;
  ASSERT_TRUE(
      fusion.AddFrame(MakeCloud(10.0, 2.0F, 2.0F), past_pose, &fused, &error))
      << error;

  Eigen::Affine3d current_pose = Eigen::Affine3d::Identity();
  current_pose.translation().x() = 1.0;
  ASSERT_TRUE(fusion.AddFrame(MakeCloud(10.1, 3.0F, 2.0F), current_pose, &fused,
                              &error))
      << error;
  ASSERT_EQ(fused.point_size(), 2);
  EXPECT_FLOAT_EQ(fused.point(0).x(), 3.0F);
  EXPECT_FLOAT_EQ(fused.point(1).x(), 1.0F);
  EXPECT_EQ(fused.point(0).timestamp(), 0U);
  EXPECT_EQ(fused.point(1).timestamp(), 100000000U);
  EXPECT_DOUBLE_EQ(fused.measurement_time(), 0.0);

  ModelInputBuilder builder;
  ASSERT_TRUE(builder.Init(TestConfig(), &error)) << error;
  ModelInput input;
  ASSERT_TRUE(builder.Build(fused, &input, &error)) << error;
  ASSERT_EQ(input.point_count(), 2U);
  EXPECT_FLOAT_EQ(input.points[4], 0.0F);
  EXPECT_NEAR(input.points[9], 0.1F, 1.0e-6F);
}

TEST(PointCloudSweepFusionTest, PadsSceneStartAndRemovesCloseHistoryPoints) {
  PointCloudSweepFusion fusion;
  std::string error;
  ASSERT_TRUE(fusion.Init(3U, 300000U, &error)) << error;

  apollo::drivers::PointCloud fused;
  ASSERT_TRUE(fusion.AddFrame(MakeCloud(1.0, 0.5F, 0.5F),
                              Eigen::Affine3d::Identity(), &fused, &error))
      << error;
  ASSERT_EQ(fused.point_size(), 1);
  EXPECT_FLOAT_EQ(fused.point(0).x(), 0.5F);
  EXPECT_EQ(fused.point(0).timestamp(), 0U);
}

TEST(PointCloudSweepFusionTest, RejectsOutOfOrderFramesAndPointOverflow) {
  PointCloudSweepFusion fusion;
  std::string error;
  ASSERT_TRUE(fusion.Init(2U, 1U, &error)) << error;

  apollo::drivers::PointCloud fused;
  ASSERT_TRUE(fusion.AddFrame(MakeCloud(2.0, 0.5F, 0.5F),
                              Eigen::Affine3d::Identity(), &fused, &error))
      << error;
  EXPECT_FALSE(fusion.AddFrame(MakeCloud(1.0, 2.0F, 0.0F),
                               Eigen::Affine3d::Identity(), &fused, &error));
  EXPECT_NE(error.find("strictly increasing"), std::string::npos);

  ASSERT_TRUE(fusion.Init(2U, 2U, &error)) << error;
  ASSERT_TRUE(fusion.AddFrame(MakeCloud(2.0, 2.0F, 2.0F),
                              Eigen::Affine3d::Identity(), &fused, &error))
      << error;
  auto too_many_points = MakeCloud(2.1, 2.0F, 2.0F);
  AddPoint(3.0F, 2.0F, too_many_points.get());
  EXPECT_FALSE(fusion.AddFrame(too_many_points, Eigen::Affine3d::Identity(),
                               &fused, &error));
  EXPECT_NE(error.find("point limit"), std::string::npos);
}

TEST(PointCloudSweepFusionTest, RejectsInvalidPose) {
  PointCloudSweepFusion fusion;
  std::string error;
  ASSERT_TRUE(fusion.Init(1U, 300000U, &error)) << error;

  Eigen::Affine3d invalid_pose = Eigen::Affine3d::Identity();
  invalid_pose.linear()(0, 0) = 2.0;
  apollo::drivers::PointCloud fused;
  EXPECT_FALSE(fusion.AddFrame(MakeCloud(1.0, 2.0F, 0.0F), invalid_pose, &fused,
                               &error));
  EXPECT_NE(error.find("pose is invalid"), std::string::npos);
}

}  // namespace lidar_object_detection
}  // namespace apollo
