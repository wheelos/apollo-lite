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
#include "modules/transform/components/static_transform_component.h"

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "tools/cpp/runfiles/runfiles.h"

#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/vehicle_calibration/test_bundle.h"
#include "modules/transform/buffer.h"
#include "modules/transform/tests/buffer_test_peer.h"

namespace apollo {
namespace transform {

TEST(TransformComponentTest, PublishesCompleteSelectedCalibration) {
  common::vehicle_calibration::TestBundle bundle;
  bundle.AddSensor(common::vehicle_calibration::IMU, "imu", "imu_link");
  bundle.AddSensor(common::vehicle_calibration::LIDAR, "lidar", "lidar_link");
  ASSERT_TRUE(bundle.Publish());
  StaticTransformComponent component;
  component.node_ = cyber::CreateNode("static_transform_publish_test");
  ASSERT_NE(component.node_, nullptr);
  EXPECT_TRUE(component.Init());
  EXPECT_NE(component.writer_, nullptr);
}

TEST(TransformComponentTest, RejectsBadExtrinsicsBeforeCreatingWriter) {
  common::vehicle_calibration::TestBundle bundle;
  bundle.AddSensor(common::vehicle_calibration::IMU, "imu", "imu_link");
  bundle.Write("imu_link.yaml", "invalid extrinsic\n");
  ASSERT_TRUE(bundle.Publish());
  StaticTransformComponent component;
  EXPECT_FALSE(component.Init());
  EXPECT_EQ(component.writer_, nullptr);
}

TEST(TransformComponentTest, RejectsInvalidSelectionBeforeCreatingWriter) {
  common::vehicle_calibration::TestBundle bundle;
  bundle.AddSensor(common::vehicle_calibration::IMU, "imu", "imu_link");
  ASSERT_TRUE(bundle.Publish());
  setenv("WHEELOS_VEHICLE_PROFILE_KEY", "missing-profile.pb.txt", 1);
  StaticTransformComponent component;
  EXPECT_FALSE(component.Init());
  EXPECT_EQ(component.writer_, nullptr);
}

TEST(TransformComponentTest, BufferUsesConfiguredTopicsAndRejectsInvalidOnes) {
  const auto dynamic_topic = FLAGS_tf_topic;
  const auto static_topic = FLAGS_tf_static_topic;
  FLAGS_tf_topic = "/transform_test/dynamic";
  FLAGS_tf_static_topic = "/transform_test/static";
  {
    Buffer buffer(false);
    EXPECT_EQ(buffer.Init(), cyber::SUCC);
    EXPECT_EQ(BufferTestPeer::DynamicTopic(buffer), FLAGS_tf_topic);
    EXPECT_EQ(BufferTestPeer::StaticTopic(buffer), FLAGS_tf_static_topic);
  }
  FLAGS_tf_static_topic = FLAGS_tf_topic;
  Buffer invalid(false);
  EXPECT_EQ(invalid.Init(), cyber::FAIL);
  EXPECT_FALSE(BufferTestPeer::HasNode(invalid));
  FLAGS_tf_topic = dynamic_topic;
  FLAGS_tf_static_topic = static_topic;
}

}  // namespace transform
}  // namespace apollo

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  std::string error;
  std::unique_ptr<bazel::tools::cpp::runfiles::Runfiles> runfiles(
      bazel::tools::cpp::runfiles::Runfiles::CreateForTest(
          BAZEL_CURRENT_REPOSITORY, &error));
  if (runfiles == nullptr) {
    std::fprintf(stderr, "Runfiles initialization failed: %s\n", error.c_str());
    return 1;
  }
  const std::string config = runfiles->Rlocation(
      "core/cyber/conf/cyber.pb.conf", BAZEL_CURRENT_REPOSITORY);
  const std::string suffix = "/conf/cyber.pb.conf";
  if (config.size() <= suffix.size()) {
    std::fprintf(stderr, "Cyber configuration is missing from runfiles.\n");
    return 1;
  }
  const auto cyber_path = config.substr(0, config.size() - suffix.size());
  setenv("CYBER_PATH", cyber_path.c_str(), 1);
  if (!apollo::cyber::Init("transform_component_test")) {
    return 1;
  }
  const int result = RUN_ALL_TESTS();
  std::fflush(nullptr);
  std::_Exit(result);
}
