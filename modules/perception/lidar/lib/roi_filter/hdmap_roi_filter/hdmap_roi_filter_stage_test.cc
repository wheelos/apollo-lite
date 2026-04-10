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

//  Created Date: 2026-03-06
//  Author: daohu527

#include <iostream>
#include <sstream>
#include <string>

#include "gtest/gtest.h"

#include "modules/perception/base/hdmap_struct.h"
#include "modules/perception/lidar/common/lidar_point_label.h"
#include "modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/hdmap_roi_filter.h"

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
  stage_config.set_stage_type(pipeline::StageType::POINTCLOUD_ROI_FILTER);
  stage_config.set_enabled(true);
  stage_config.set_type("HdmapPointCloudRoiFilter");

  auto* config = stage_config.mutable_pointcloud_roi_filter_config();
  config->set_range(10.0);
  config->set_cell_size(0.25);
  config->set_extend_dist(0.0);
  config->set_no_edge_table(false);
  config->set_set_roi_service(false);
  return stage_config;
}

void AddPoint(LidarFrame* frame, float x, float y, float z) {
  base::PointF local_point;
  local_point.x = x;
  local_point.y = y;
  local_point.z = z;
  frame->cloud->push_back(local_point, 0.0);

  base::PointD world_point;
  world_point.x = x;
  world_point.y = y;
  world_point.z = z;
  frame->world_cloud->push_back(world_point, 0.0);
}

base::PolygonDType MakeSquarePolygon(double min_x, double min_y, double max_x,
                                     double max_y) {
  base::PolygonDType polygon;
  polygon.resize(4);

  polygon[0].x = min_x;
  polygon[0].y = min_y;
  polygon[0].z = 0.0;
  polygon[1].x = max_x;
  polygon[1].y = min_y;
  polygon[1].z = 0.0;
  polygon[2].x = max_x;
  polygon[2].y = max_y;
  polygon[2].z = 0.0;
  polygon[3].x = min_x;
  polygon[3].y = max_y;
  polygon[3].z = 0.0;
  return polygon;
}

}  // namespace

TEST(HdmapPointCloudRoiFilterStageTest, process_stage_metrics) {
  HdmapPointCloudRoiFilter filter;
  auto stage_config = MakeStageConfig();
  EXPECT_TRUE(filter.Init(stage_config));
  EXPECT_EQ(filter.Name(), "HdmapPointCloudRoiFilter");

  LidarFrame frame;
  frame.cloud.reset(new base::PointFCloud);
  frame.world_cloud.reset(new base::PointDCloud);
  frame.hdmap_struct.reset(new base::HdmapStruct);
  frame.lidar2world_pose = Eigen::Affine3d::Identity();
  frame.hdmap_struct->road_polygons.push_back(
      MakeSquarePolygon(0.0, 0.0, 4.0, 4.0));

  AddPoint(&frame, 1.0f, 1.0f, 0.0f);
  AddPoint(&frame, 3.0f, 1.0f, 0.0f);
  AddPoint(&frame, 2.0f, 3.0f, 0.0f);
  AddPoint(&frame, -1.0f, 1.0f, 0.0f);
  AddPoint(&frame, 5.0f, 5.0f, 0.0f);

  pipeline::DataFrame data_frame{};
  data_frame.lidar_frame = &frame;

  EXPECT_FALSE(filter.Process(nullptr));
  EXPECT_TRUE(filter.Process(&data_frame));

  ASSERT_EQ(frame.roi_indices.indices.size(), 3u);
  EXPECT_EQ(frame.roi_indices.indices[0], 0);
  EXPECT_EQ(frame.roi_indices.indices[1], 1);
  EXPECT_EQ(frame.roi_indices.indices[2], 2);

  const auto roi_label = static_cast<uint8_t>(LidarPointLabel::ROI);
  for (size_t i = 0; i < frame.cloud->size(); ++i) {
    if (i < 3) {
      EXPECT_EQ(frame.cloud->points_label(i), roi_label);
      EXPECT_EQ(frame.world_cloud->points_label(i), roi_label);
    } else {
      EXPECT_EQ(frame.cloud->points_label(i), 0u);
      EXPECT_EQ(frame.world_cloud->points_label(i), 0u);
    }
  }

  EmitMetric("input_points", frame.cloud->size());
  EmitMetric("roi_points", frame.roi_indices.indices.size());
  EmitMetric("roi_ratio",
             static_cast<double>(frame.roi_indices.indices.size()) /
                 static_cast<double>(frame.cloud->size()));
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
