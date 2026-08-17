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

//  Created Date: 2026-04-11
//  Author: daohu527

#include "modules/perception/lidar/lib/detector/graph_segmentation/graph_segmentation.h"

#include <limits>
#include <numeric>
#include <string>

#include "gtest/gtest.h"

#include "modules/perception/base/point_cloud.h"

namespace apollo {
namespace perception {
namespace lidar {

namespace {

pipeline::StageConfig MakeStageConfig() {
  pipeline::StageConfig stage_config;
  stage_config.set_stage_type(pipeline::StageType::GRAPH_CLUSTER_SEGMENTER);
  stage_config.set_enabled(true);
  stage_config.set_type("GraphClusterSegmenter");

  auto* config = stage_config.mutable_graph_cluster_segmenter_config();
  config->set_resolution(0.6f);
  config->set_threshold(2.0f);
  config->set_min_pt_number(5);
  config->set_search_radius(2);
  config->set_height_threshold(2.0f);
  config->set_xmin(-20.0f);
  config->set_xmax(20.0f);
  config->set_ymin(-20.0f);
  config->set_ymax(20.0f);
  config->set_semantic_cost(1.0f);
  config->set_car_xmin(-0.35f);
  config->set_car_xmax(0.35f);
  config->set_car_ymin(-0.45f);
  config->set_car_ymax(0.45f);
  config->set_car_zmax(0.56f);
  config->set_min_radius(0.5f);
  config->set_z_min_from_ground(0.05f);
  config->set_split_aspect_ratio(15.0f);
  config->set_split_distance(5.0f);
  return stage_config;
}

void AddPoint(LidarFrame* frame, float x, float y, float z) {
  base::PointF local_point;
  local_point.x = x;
  local_point.y = y;
  local_point.z = z;
  local_point.intensity = 1.0f;
  frame->cloud->push_back(local_point, 0.0);

  base::PointD world_point;
  world_point.x = x;
  world_point.y = y;
  world_point.z = z;
  world_point.intensity = 1.0f;
  frame->world_cloud->push_back(world_point, 0.0);
}

}  // namespace

class GraphSegmentationTest : public ::testing::Test {
 protected:
  void SetUp() override { segmentor_.reset(new GraphSegmentation()); }

  void EmitMetric(const std::string& key, double value) {
    ::testing::Test::RecordProperty(key, std::to_string(value));
  }

  std::unique_ptr<GraphSegmentation> segmentor_;
};

TEST_F(GraphSegmentationTest, ClusterTest) {
  LidarFrame frame;
  frame.cloud = base::PointFCloudPool::Instance().Get();
  frame.world_cloud = base::PointDCloudPool::Instance().Get();
  frame.lidar2vehicle_extrinsics = Eigen::Affine3d::Identity();

  // Create two distinct clusters
  for (int i = 0; i < 100; ++i) {
    AddPoint(&frame, static_cast<float>(i % 10) * 0.1f,
             static_cast<float>(i / 10) * 0.1f, 1.0f);
  }

  for (int i = 0; i < 100; ++i) {
    AddPoint(&frame, 10.0f + static_cast<float>(i % 10) * 0.1f,
             10.0f + static_cast<float>(i / 10) * 0.1f, 1.0f);
  }

  frame.cloud->mutable_points_height()->assign(
      frame.cloud->size(), std::numeric_limits<float>::max());
  frame.world_cloud->mutable_points_height()->assign(
      frame.world_cloud->size(), std::numeric_limits<float>::max());
  frame.non_ground_indices.indices.resize(frame.cloud->size());
  std::iota(frame.non_ground_indices.indices.begin(),
            frame.non_ground_indices.indices.end(), 0);

  auto stage_config = MakeStageConfig();
  EXPECT_TRUE(segmentor_->Init(stage_config));

  LidarDetectorOptions options;
  EXPECT_TRUE(segmentor_->Detect(options, &frame));

  EmitMetric("generated_cluster_count", frame.segmented_objects.size());
  EXPECT_GE(frame.segmented_objects.size(), 2u);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
