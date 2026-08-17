/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/lidar/lib/pointcloud_preprocessor/pointcloud_preprocessor.h"

#include <Eigen/Geometry>

#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <string>

#include "gtest/gtest.h"

#include "modules/perception/base/point_cloud.h"
#include "modules/perception/lidar/common/lidar_frame.h"

namespace apollo {
namespace perception {
namespace lidar {

namespace {

template <typename T>
void EmitMetric(const std::string& name, const T& value) {
  std::ostringstream stream;
  stream << value;
  std::cout << "METRIC " << name << "=" << stream.str() << std::endl;
  ::testing::Test::RecordProperty(name, stream.str());
}

pipeline::StageConfig MakeStageConfig() {
  pipeline::StageConfig stage_config;
  stage_config.set_stage_type(pipeline::StageType::POINTCLOUD_PREPROCESSOR);
  stage_config.set_enabled(true);
  stage_config.set_type("PointCloudPreprocessor");

  auto* config = stage_config.mutable_pointcloud_preprocessor_config();
  config->set_filter_naninf_points(true);
  config->set_filter_nearby_box_points(true);
  config->set_box_forward_x(2.0);
  config->set_box_backward_x(-2.0);
  config->set_box_forward_y(2.0);
  config->set_box_backward_y(-2.0);
  config->set_filter_high_z_points(true);
  config->set_z_threshold(5.0);
  return stage_config;
}

}  // namespace

void MockPointcloud(base::PointFCloud* cloud) {
  cloud->resize(10);
  for (size_t i = 0; i < cloud->size(); ++i) {
    cloud->at(i).x = 5.f * i;
    cloud->at(i).y = 5.f * i;
    cloud->at(i).z = 0.f;
  }
  // 1. three nan points
  cloud->at(0).x = std::numeric_limits<float>::quiet_NaN();
  cloud->at(1).y = std::numeric_limits<float>::quiet_NaN();
  cloud->at(2).z = std::numeric_limits<float>::quiet_NaN();
  // 2. three inf points
  cloud->at(3).x = 10000.f;
  cloud->at(4).y = 10000.f;
  cloud->at(5).z = 10000.f;
  // 3. one box points
  cloud->at(6).x = cloud->at(6).y = 0.f;
  // 4. one large z points
  cloud->at(7).z = 10.f;
  // 5. two normal points
}

TEST(PointCloudPreprocessorStageTest, process_stage_metrics) {
  PointCloudPreprocessor preprocessor;
  auto stage_config = MakeStageConfig();
  EXPECT_TRUE(preprocessor.Init(stage_config));
  EXPECT_EQ(preprocessor.Name(), "PointCloudPreprocessor");

  pipeline::DataFrame data_frame{};
  LidarFrame frame;
  data_frame.lidar_frame = &frame;

  EXPECT_FALSE(preprocessor.Process(nullptr));
  EXPECT_FALSE(preprocessor.Process(&data_frame));

  frame.cloud.reset(new base::PointFCloud);
  frame.lidar2vehicle_extrinsics = Eigen::Affine3d::Identity();
  frame.lidar2world_pose = Eigen::Affine3d::Identity();
  frame.lidar2world_pose.pretranslate(Eigen::Vector3d(5.0, -2.0, 1.5));
  MockPointcloud(frame.cloud.get());

  const size_t input_points = frame.cloud->size();
  EXPECT_EQ(input_points, 10u);
  EXPECT_TRUE(preprocessor.Process(&data_frame));

  ASSERT_NE(frame.world_cloud, nullptr);
  EXPECT_EQ(frame.cloud->size(), 2u);
  EXPECT_EQ(frame.world_cloud->size(), 2u);

  std::set<std::pair<int, int>> remaining_xy;
  for (size_t i = 0; i < frame.cloud->size(); ++i) {
    const auto& pt = frame.cloud->at(i);
    const auto& world_pt = frame.world_cloud->at(i);
    remaining_xy.emplace(static_cast<int>(pt.x), static_cast<int>(pt.y));
    EXPECT_NEAR(world_pt.x - pt.x, 5.0, 1e-6);
    EXPECT_NEAR(world_pt.y - pt.y, -2.0, 1e-6);
    EXPECT_NEAR(world_pt.z - pt.z, 1.5, 1e-6);
    EXPECT_EQ(pt.intensity, world_pt.intensity);
  }

  std::set<std::pair<int, int>> expected_xy = {{40, 40}, {45, 45}};
  EXPECT_EQ(remaining_xy, expected_xy);

  EmitMetric("input_points", input_points);
  EmitMetric("removed_naninf_or_inf_points", 6);
  EmitMetric("removed_nearby_box_points", 1);
  EmitMetric("removed_high_z_points", 1);
  EmitMetric("output_points", frame.cloud->size());
  EmitMetric("output_ratio", static_cast<double>(frame.cloud->size()) /
                                 static_cast<double>(input_points));
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
