// Copyright 2026 WheelOS. All Rights Reserved.
#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "modules/drivers/lidar/fusion/lidar_fusion_component.h"
#include "modules/drivers/lidar/velodyne/fusion/pri_sec_fusion_component.h"
#include "modules/transform/tests/buffer_test_peer.h"

namespace apollo {
namespace drivers {
namespace lidar {
namespace {

std::shared_ptr<transform::TransformStampeds> Edge(const std::string& parent,
                                                   const std::string& child,
                                                   double x) {
  auto message = std::make_shared<transform::TransformStampeds>();
  auto* edge = message->add_transforms();
  edge->mutable_header()->set_frame_id(parent);
  edge->mutable_header()->set_timestamp_sec(1);
  edge->set_child_frame_id(child);
  edge->mutable_transform()->mutable_translation()->set_x(x);
  edge->mutable_transform()->mutable_translation()->set_y(0);
  edge->mutable_transform()->mutable_translation()->set_z(0);
  edge->mutable_transform()->mutable_rotation()->set_qx(0);
  edge->mutable_transform()->mutable_rotation()->set_qy(0);
  edge->mutable_transform()->mutable_rotation()->set_qz(0);
  edge->mutable_transform()->mutable_rotation()->set_qw(1);
  return message;
}

TEST(LidarExtrinsicTest, BothFusionPathsRejectDynamicOnlyExtrinsics) {
  transform::Buffer buffer(false);
  transform::BufferTestPeer::ReceiveDynamic(buffer,
                                            Edge("base_link", "lidar", 7));
  ASSERT_TRUE(buffer.canTransform("base_link", "lidar", cyber::Time(0), 0));
  LidarFusionComponent fusion(&buffer);
  velodyne::PriSecFusionComponent legacy(&buffer);
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  EXPECT_FALSE(fusion.QueryPoseAffine(0, "base_link", "lidar", &pose));
  EXPECT_FALSE(legacy.QueryPoseAffine("base_link", "lidar", &pose));
  EXPECT_TRUE(pose.matrix().isIdentity());
}

TEST(LidarExtrinsicTest, ComposesStaticChainsAndCachesBothFrames) {
  transform::Buffer buffer(false);
  transform::BufferTestPeer::ReceiveStatic(buffer,
                                           Edge("base_link", "mount", 2));
  transform::BufferTestPeer::ReceiveStatic(buffer, Edge("mount", "lidar", 3));
  LidarFusionComponent fusion(&buffer);
  velodyne::PriSecFusionComponent legacy(&buffer);
  Eigen::Affine3d pose;
  ASSERT_TRUE(legacy.QueryPoseAffine("base_link", "lidar", &pose));
  EXPECT_DOUBLE_EQ(pose.translation().x(), 5);
  auto source = std::make_shared<PointCloud>();
  source->mutable_header()->set_frame_id("lidar");
  auto target = std::make_shared<PointCloud>();
  target->mutable_header()->set_frame_id("base_link");
  Eigen::Affine3f cached;
  ASSERT_TRUE(fusion.GetPoseAffine(target, source, &cached));
  EXPECT_FLOAT_EQ(cached.translation().x(), 5);
  target->mutable_header()->set_frame_id("mount");
  ASSERT_TRUE(fusion.GetPoseAffine(target, source, &cached));
  EXPECT_FLOAT_EQ(cached.translation().x(), 3);
}

}  // namespace
}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
