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

#include "modules/bevfusion/types/bounded_synchronizer.h"

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace bevfusion {
namespace {

std::shared_ptr<const CameraPacket> MakeCameraPacket(std::size_t camera,
                                                     std::uint64_t timestamp) {
  auto storage = std::make_shared<std::vector<std::uint8_t>>(1, 0);
  auto packet = std::make_shared<CameraPacket>();
  packet->timestamp_ns = timestamp;
  packet->arrival_monotonic_ns = 100;
  packet->storage = storage;
  packet->image.camera_name = kCameraOrder[camera];
  packet->image.timestamp_ns = timestamp;
  packet->image.data = storage->data();
  packet->image.byte_count = storage->size();
  return packet;
}

std::shared_ptr<const LidarPacket> MakeLidarPacket(std::uint64_t timestamp) {
  auto points = std::make_shared<std::vector<float>>(kPointFeatureCount, 0.0F);
  auto packet = std::make_shared<LidarPacket>();
  packet->timestamp_ns = timestamp;
  packet->arrival_monotonic_ns = 100;
  packet->point_count = 1;
  packet->packed_points = points;
  return packet;
}

TEST(BoundedSynchronizerTest, EmitsOneCompleteOrderedBundle) {
  BoundedSynchronizer synchronizer;
  std::string error;
  ASSERT_TRUE(synchronizer.Init(2, 50, 100, &error)) << error;
  ASSERT_TRUE(synchronizer.PushLidar(MakeLidarPacket(1000), &error)) << error;
  for (std::size_t camera = 0; camera < kCameraCount; ++camera) {
    ASSERT_TRUE(
        synchronizer.PushCamera(camera, MakeCameraPacket(camera, 1010), &error))
        << error;
  }

  SynchronizedPackets packets;
  EXPECT_EQ(synchronizer.TryPop(199, &packets, &error), SyncResult::kReady);
  ASSERT_NE(packets.lidar, nullptr);
  for (std::size_t camera = 0; camera < kCameraCount; ++camera) {
    ASSERT_NE(packets.cameras[camera], nullptr);
    EXPECT_STREQ(packets.cameras[camera]->image.camera_name,
                 kCameraOrder[camera]);
  }
  EXPECT_EQ(synchronizer.TryPop(200, &packets, &error), SyncResult::kWaiting);
}

TEST(BoundedSynchronizerTest, RejectsTimedOutIncompleteBundle) {
  BoundedSynchronizer synchronizer;
  std::string error;
  ASSERT_TRUE(synchronizer.Init(1, 50, 100, &error)) << error;
  ASSERT_TRUE(synchronizer.PushLidar(MakeLidarPacket(1000), &error)) << error;

  SynchronizedPackets packets;
  EXPECT_EQ(synchronizer.TryPop(199, &packets, &error), SyncResult::kWaiting);
  EXPECT_EQ(synchronizer.TryPop(200, &packets, &error), SyncResult::kRejected);
  EXPECT_NE(error.find("timed out waiting for camera"), std::string::npos);
}

TEST(BoundedSynchronizerTest, RejectsCompleteBundleAfterWaitDeadline) {
  BoundedSynchronizer synchronizer;
  std::string error;
  ASSERT_TRUE(synchronizer.Init(2, 50, 100, &error)) << error;
  ASSERT_TRUE(synchronizer.PushLidar(MakeLidarPacket(1000), &error)) << error;
  for (std::size_t camera = 0; camera < kCameraCount; ++camera) {
    ASSERT_TRUE(
        synchronizer.PushCamera(camera, MakeCameraPacket(camera, 1010), &error))
        << error;
  }

  SynchronizedPackets packets;
  EXPECT_EQ(synchronizer.TryPop(200, &packets, &error), SyncResult::kRejected);
  EXPECT_NE(error.find("wait bound"), std::string::npos);
}

TEST(BoundedSynchronizerTest, RejectsUnsynchronizedSamplesAndQueueOverflow) {
  BoundedSynchronizer synchronizer;
  std::string error;
  ASSERT_TRUE(synchronizer.Init(1, 50, 100, &error)) << error;
  ASSERT_TRUE(synchronizer.PushLidar(MakeLidarPacket(1000), &error)) << error;
  for (std::size_t camera = 0; camera < kCameraCount; ++camera) {
    ASSERT_TRUE(
        synchronizer.PushCamera(camera, MakeCameraPacket(camera, 2000), &error))
        << error;
  }
  EXPECT_FALSE(synchronizer.PushLidar(MakeLidarPacket(2000), &error));
  EXPECT_NE(error.find("full"), std::string::npos);

  SynchronizedPackets packets;
  EXPECT_EQ(synchronizer.TryPop(200, &packets, &error), SyncResult::kRejected);
  EXPECT_NE(error.find("exceeded synchronization tolerance"),
            std::string::npos);
}

TEST(BoundedSynchronizerTest, RejectsOutOfOrderCameraAndWrongIndex) {
  BoundedSynchronizer synchronizer;
  std::string error;
  ASSERT_TRUE(synchronizer.Init(2, 50, 100, &error)) << error;
  EXPECT_FALSE(synchronizer.PushCamera(1, MakeCameraPacket(0, 1000), &error));
  EXPECT_NE(error.find("camera order"), std::string::npos);
  ASSERT_TRUE(synchronizer.PushCamera(0, MakeCameraPacket(0, 2000), &error))
      << error;
  EXPECT_FALSE(synchronizer.PushCamera(0, MakeCameraPacket(0, 1500), &error));
  EXPECT_NE(error.find("out-of-order"), std::string::npos);
}

}  // namespace
}  // namespace bevfusion
}  // namespace apollo
