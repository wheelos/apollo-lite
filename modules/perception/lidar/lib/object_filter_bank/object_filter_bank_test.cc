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

#include "modules/perception/lidar/lib/object_filter_bank/object_filter_bank.h"

#include <string>

#include "gtest/gtest.h"

#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/base/point_cloud.h"

namespace apollo {
namespace perception {
namespace lidar {

namespace {

pipeline::StageConfig MakeStageConfig() {
  pipeline::StageConfig stage_config;
  stage_config.set_stage_type(pipeline::StageType::OBJECT_POST_FILTER_BANK);
  stage_config.set_enabled(true);
  stage_config.set_type("ObjectPostFilterBank");
  return stage_config;
}

}  // namespace

class ObjectFilterBankTest : public ::testing::Test {
 protected:
  void SetUp() override { filter_bank_.reset(new ObjectFilterBank()); }

  void EmitMetric(const std::string& key, double value) {
    ::testing::Test::RecordProperty(key, std::to_string(value));
  }

  std::unique_ptr<ObjectFilterBank> filter_bank_;
};

TEST_F(ObjectFilterBankTest, EmptyPluginBankKeepsObjectsStable) {
  LidarFrame frame;
  frame.cloud = base::PointFCloudPool::Instance().Get();
  frame.world_cloud = base::PointDCloudPool::Instance().Get();

  // Create one valid object and one invalid (too small/noise)
  auto obj1 = base::ObjectPool::Instance().Get();
  obj1->id = 1;
  obj1->size = Eigen::Vector3f(2.0, 1.0, 1.0);  // Good size
  obj1->center = Eigen::Vector3d(5.0, 5.0, 0.0);
  frame.segmented_objects.push_back(obj1);

  auto obj2 = base::ObjectPool::Instance().Get();
  obj2->id = 2;
  obj2->size = Eigen::Vector3f(0.01, 0.01, 0.01);  // Too small
  obj2->center = Eigen::Vector3d(10.0, 10.0, 0.0);
  frame.segmented_objects.push_back(obj2);

  const size_t initial_count = frame.segmented_objects.size();

  auto stage_config = MakeStageConfig();
  EXPECT_TRUE(filter_bank_->Init(stage_config));

  pipeline::DataFrame data_frame{};
  data_frame.lidar_frame = &frame;
  EXPECT_TRUE(filter_bank_->Process(&data_frame));

  const size_t final_count = frame.segmented_objects.size();

  EmitMetric("objects_pre_filter", initial_count);
  EmitMetric("objects_post_filter", final_count);
  EmitMetric("objects_filtered_out", initial_count - final_count);
  EXPECT_EQ(final_count, initial_count);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
