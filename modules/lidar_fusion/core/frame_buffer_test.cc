// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_fusion/core/frame_buffer.h"

#include <memory>

#include "gtest/gtest.h"

namespace apollo {
namespace lidar_fusion {
namespace {

class FrameBufferTest : public testing::Test {
 protected:
  void SetUp() override {
    config.set_sensor_frame("lidar");
    config.set_pointcloud_channel("/cloud");
    config.set_wait_ms(100);
    config.set_queue_capacity(2);
    buffer = std::make_unique<FrameBuffer>(config);
  }

  std::shared_ptr<drivers::PointCloud> Cloud(int id) {
    auto cloud = std::make_shared<drivers::PointCloud>();
    cloud->set_frame_id("lidar");
    cloud->set_measurement_time(id);
    cloud->mutable_header()->set_sequence_num(id);
    cloud->mutable_header()->set_lidar_timestamp(id * 1000000000ULL);
    auto* p = cloud->add_point();
    p->set_x(1);
    p->set_y(0);
    p->set_z(1);
    return cloud;
  }

  std::shared_ptr<DetectionFrame> Detection(int id) {
    auto det = std::make_shared<DetectionFrame>();
    *det->mutable_header() = Cloud(id)->header();
    det->set_sensor_frame_id("lidar");
    return det;
  }

  std::shared_ptr<SemanticFrame> Semantic(int id) {
    auto seg = std::make_shared<SemanticFrame>();
    *seg->mutable_header() = Cloud(id)->header();
    seg->set_source_frame_id("lidar");
    seg->set_source_topic("/cloud");
    return seg;
  }

  FusionConfig config;
  std::unique_ptr<FrameBuffer> buffer;
  ReadyFrame ready;
};

TEST_F(FrameBufferTest, WaitsForBothResultsRegardlessOfArrivalOrder) {
  ASSERT_TRUE(buffer->PushSemantic(Semantic(1)));
  ASSERT_TRUE(buffer->PushCloud(Cloud(1), 10));
  EXPECT_FALSE(buffer->Pop(10.01, &ready));
  ASSERT_TRUE(buffer->PushDetection(Detection(1)));
  ASSERT_TRUE(buffer->Pop(10.02, &ready));
  EXPECT_EQ(ready.cloud->measurement_time(), 1);
  EXPECT_NE(ready.detection, nullptr);
  EXPECT_NE(ready.segmentation, nullptr);
  EXPECT_FALSE(buffer->Pop(10.03, &ready));
}

TEST_F(FrameBufferTest, UsesReceiveDeadlineAndNeverAttachesStaleResults) {
  ASSERT_TRUE(buffer->PushCloud(Cloud(1), 10));
  ASSERT_TRUE(buffer->PushDetection(Detection(1)));
  EXPECT_FALSE(buffer->Pop(10.09, &ready));
  ASSERT_TRUE(buffer->Pop(10.11, &ready));
  EXPECT_NE(ready.detection, nullptr);
  EXPECT_EQ(ready.segmentation, nullptr);
  ASSERT_TRUE(buffer->PushSemantic(Semantic(1)));
  ASSERT_TRUE(buffer->PushCloud(Cloud(2), 11));
  ASSERT_TRUE(buffer->Pop(11.11, &ready));
  EXPECT_EQ(ready.cloud->measurement_time(), 2);
  EXPECT_EQ(ready.detection, nullptr);
  EXPECT_EQ(ready.segmentation, nullptr);
}

TEST_F(FrameBufferTest, BoundedQueuesExposeDropsAndRejectDuplicateClouds) {
  ASSERT_TRUE(buffer->PushCloud(Cloud(1), 10));
  EXPECT_FALSE(buffer->PushCloud(Cloud(1), 10.01));
  ASSERT_TRUE(buffer->PushCloud(Cloud(2), 10.02));
  ASSERT_TRUE(buffer->PushCloud(Cloud(3), 10.03));
  EXPECT_EQ(buffer->dropped_clouds(), 2);
  EXPECT_EQ(buffer->queue_overflows(), 1);
  ASSERT_TRUE(buffer->PushDetection(Detection(1)));
  ASSERT_TRUE(buffer->PushDetection(Detection(2)));
  ASSERT_TRUE(buffer->PushDetection(Detection(3)));
  EXPECT_EQ(buffer->dropped_results(), 1);
  EXPECT_EQ(buffer->queue_overflows(), 2);
  ASSERT_TRUE(buffer->Pop(11, &ready));
  EXPECT_EQ(ready.cloud->measurement_time(), 2);
  ASSERT_TRUE(buffer->Pop(11, &ready));
  EXPECT_EQ(ready.cloud->measurement_time(), 3);
  EXPECT_FALSE(buffer->PushCloud(Cloud(2), 12));
}

TEST_F(FrameBufferTest, RejectsWrongSensorAndOversizedMessagesBeforeQueueing) {
  auto cloud = Cloud(1);
  cloud->set_frame_id("other");
  EXPECT_FALSE(buffer->PushCloud(cloud, 1));
  auto det = Detection(1);
  det->set_sensor_frame_id("other");
  EXPECT_FALSE(buffer->PushDetection(det));
  auto seg = Semantic(1);
  seg->set_source_topic("/other");
  EXPECT_FALSE(buffer->PushSemantic(seg));
  EXPECT_EQ(buffer->dropped_clouds(), 1);
  EXPECT_EQ(buffer->dropped_results(), 2);
  EXPECT_FALSE(buffer->Pop(100, &ready));
  config.set_max_points(1);
  config.set_max_detections(1);
  buffer = std::make_unique<FrameBuffer>(config);
  cloud = Cloud(1);
  cloud->add_point();
  EXPECT_FALSE(buffer->PushCloud(cloud, 1));
  det = Detection(1);
  det->add_detection();
  det->add_detection();
  EXPECT_FALSE(buffer->PushDetection(det));
  seg = Semantic(1);
  seg->add_point_label();
  seg->add_point_label();
  EXPECT_FALSE(buffer->PushSemantic(seg));
  EXPECT_EQ(buffer->queue_overflows(), 0);
}

}  // namespace
}  // namespace lidar_fusion
}  // namespace apollo
