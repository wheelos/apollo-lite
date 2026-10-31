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

#include "modules/lidar_object_detection/types/model_input.h"

#include <cstdint>
#include <limits>
#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace lidar_object_detection {

namespace {

CenterPointModelConfig TestConfig() {
  CenterPointModelConfig config;
  config.add_input_feature()->set_source(POINT_FEATURE_X);
  config.add_input_feature()->set_source(POINT_FEATURE_Y);
  config.add_input_feature()->set_source(POINT_FEATURE_Z);
  config.add_input_feature()->set_source(POINT_FEATURE_INTENSITY);
  config.add_input_feature()->set_source(POINT_FEATURE_TIME_OFFSET_SECONDS);
  return config;
}

void AddPoint(float x, float y, float z, uint32_t intensity,
              uint64_t timestamp_ns, apollo::drivers::PointCloud* cloud) {
  auto* point = cloud->add_point();
  point->set_x(x);
  point->set_y(y);
  point->set_z(z);
  point->set_intensity(intensity);
  point->set_timestamp(timestamp_ns);
}

}  // namespace

TEST(ModelInputBuilderTest, BuildsFiveFeatureRowsAndPreservesPointIndices) {
  ModelInputBuilder builder;
  std::string error;
  ASSERT_TRUE(builder.Init(TestConfig(), &error)) << error;

  apollo::drivers::PointCloud cloud;
  cloud.set_measurement_time(10.0);
  AddPoint(1.0F, 2.0F, 3.0F, 64U, 10050000000ULL, &cloud);
  AddPoint(4.0F, 5.0F, 6.0F, 128U, 9900000000ULL, &cloud);

  ModelInput input;
  ASSERT_TRUE(builder.Build(cloud, &input, &error)) << error;
  ASSERT_EQ(input.point_count(), 2U);
  ASSERT_EQ(input.points.size(), 10U);
  EXPECT_EQ(input.source_point_indices[0], 0U);
  EXPECT_EQ(input.source_point_indices[1], 1U);
  EXPECT_FLOAT_EQ(input.points[0], 1.0F);
  EXPECT_FLOAT_EQ(input.points[3], 64.0F);
  EXPECT_NEAR(input.points[4], 0.05F, 1e-6F);
  EXPECT_NEAR(input.points[9], -0.1F, 1e-6F);
}

TEST(ModelInputBuilderTest, RequiresCenterPointXYZFeatureOrder) {
  auto config = TestConfig();
  config.mutable_input_feature(0)->set_source(POINT_FEATURE_Y);
  ModelInputBuilder builder;
  std::string error;
  EXPECT_FALSE(builder.Init(config, &error));
  EXPECT_FALSE(error.empty());
}

TEST(ModelInputBuilderTest, SkipsPointsWithNonFiniteCoordinates) {
  ModelInputBuilder builder;
  std::string error;
  ASSERT_TRUE(builder.Init(TestConfig(), &error)) << error;

  apollo::drivers::PointCloud cloud;
  AddPoint(1.0F, 2.0F, 3.0F, 64U, 0U, &cloud);
  AddPoint(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 0U, 0U, &cloud);

  ModelInput input;
  ASSERT_TRUE(builder.Build(cloud, &input, &error)) << error;
  ASSERT_EQ(input.point_count(), 1U);
  EXPECT_EQ(input.source_point_indices[0], 0U);
}

}  // namespace lidar_object_detection
}  // namespace apollo
