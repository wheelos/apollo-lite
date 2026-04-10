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

#include "modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector/spatio_temporal_ground_detector.h"

#include <limits>
#include <string>

#include "gtest/gtest.h"

#include "modules/perception/base/point_cloud.h"
#include "modules/perception/common/point_cloud_processing/common.h"
#include "modules/perception/lidar/common/lidar_point_label.h"

namespace apollo {
namespace perception {
namespace lidar {

namespace {

pipeline::StageConfig MakeStageConfig() {
  pipeline::StageConfig stage_config;
  stage_config.set_stage_type(pipeline::StageType::GROUND_SEGMENTER);
  stage_config.set_enabled(true);
  stage_config.set_type("SpatioTemporalGroundDetector");

  auto* config = stage_config.mutable_ground_segmenter_config();
  config->set_grid_size(32);
  config->set_ground_thres(0.25f);
  config->set_roi_rad_x(120.0f);
  config->set_roi_rad_y(120.0f);
  config->set_roi_rad_z(100.0f);
  config->set_nr_smooth_iter(5);
  config->set_use_roi(false);
  config->set_use_ground_service(false);
  config->set_sample_region_z_lower(-3.0f);
  config->set_sample_region_z_upper(-1.0f);
  config->set_roi_near_rad(32.0f);
  config->set_planefit_orien_threshold(5.0f);
  config->set_big_grid_size(256);
  config->set_small_grid_size(32);
  config->set_z_compare_thres(0.1f);
  config->set_smooth_z_thres(1.0f);
  config->set_planefit_dist_thres_near(0.1f);
  config->set_planefit_dist_thres_far(0.2f);
  config->set_inliers_min_threshold(6);
  config->set_near_range_dist(10.0f);
  config->set_near_range_ground_thres(0.25f);
  config->set_middle_range_dist(20.0f);
  config->set_middle_range_ground_thres(0.25f);
  return stage_config;
}

}  // namespace

class SpatioTemporalGroundDetectorTest : public ::testing::Test {
 protected:
  void SetUp() override { detector_.reset(new SpatioTemporalGroundDetector()); }

  void EmitMetric(const std::string& key, double value) {
    ::testing::Test::RecordProperty(key, std::to_string(value));
  }

  std::unique_ptr<SpatioTemporalGroundDetector> detector_;
};

TEST_F(SpatioTemporalGroundDetectorTest, FlatGroundTest) {
  LidarFrame frame;
  frame.cloud = base::PointFCloudPool::Instance().Get();
  frame.world_cloud = base::PointDCloudPool::Instance().Get();

  // Synthetic planar ground
  int total_ground = 500;
  for (int i = 0; i < total_ground; ++i) {
    base::PointF pt;
    pt.x = static_cast<float>(i % 20);
    pt.y = static_cast<float>(i / 20);
    // Add small noise
    pt.z = 0.05f * ((i % 3) - 1);
    frame.cloud->push_back(pt, 0.0);
  }

  // Non-ground points
  int total_non_ground = 50;
  for (int i = 0; i < total_non_ground; ++i) {
    base::PointF pt;
    pt.x = static_cast<float>(i);
    pt.y = static_cast<float>(i);
    pt.z = 2.0f;
    frame.cloud->push_back(pt, 0.0);
  }

  // Need to populate world cloud to avoid crash if it depends on it
  for (size_t i = 0; i < frame.cloud->size(); ++i) {
    base::PointD ptd;
    ptd.x = frame.cloud->at(i).x;
    ptd.y = frame.cloud->at(i).y;
    ptd.z = frame.cloud->at(i).z;
    frame.world_cloud->push_back(ptd, 0.0);
  }

  frame.cloud->mutable_points_height()->assign(
      frame.cloud->size(), std::numeric_limits<float>::max());
  frame.world_cloud->mutable_points_height()->assign(
      frame.world_cloud->size(), std::numeric_limits<float>::max());
  frame.cloud->mutable_points_label()->assign(frame.cloud->size(), 0u);
  frame.world_cloud->mutable_points_label()->assign(frame.world_cloud->size(),
                                                    0u);
  frame.lidar2vehicle_extrinsics = Eigen::Affine3d::Identity();
  frame.lidar2world_pose = Eigen::Affine3d::Identity();

  auto stage_config = MakeStageConfig();
  EXPECT_TRUE(detector_->Init(stage_config));

  GroundDetectorOptions options;
  EXPECT_TRUE(detector_->Detect(options, &frame));

  int ground_count = 0;
  int non_ground_count = 0;
  for (size_t i = 0; i < frame.cloud->size(); ++i) {
    if (frame.cloud->points_label(i) ==
        static_cast<uint8_t>(LidarPointLabel::GROUND)) {
      ground_count++;
    } else {
      non_ground_count++;
    }
  }

  EmitMetric("ground_points_count", ground_count);
  EmitMetric("non_ground_points_count", non_ground_count);
  EmitMetric("total_input_points", frame.cloud->size());
  EXPECT_GT(ground_count, 0);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
