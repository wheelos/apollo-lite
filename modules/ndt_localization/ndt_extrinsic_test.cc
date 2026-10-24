// Copyright 2026 WheelOS. All Rights Reserved.
#include <memory>

#include "gtest/gtest.h"

#include "modules/ndt_localization/ndt_localization.h"
#include "modules/transform/tests/buffer_test_peer.h"

namespace apollo {
namespace localization {
namespace ndt {
namespace {

TEST(NdtExtrinsicTest, LoadsBaseFromLidarForPredictionAndBodyRecovery) {
  transform::Buffer buffer(false);
  auto message = std::make_shared<transform::TransformStampeds>();
  auto* edge = message->add_transforms();
  edge->mutable_header()->set_frame_id("base_link");
  edge->set_child_frame_id("lidar");
  auto* value = edge->mutable_transform();
  value->mutable_translation()->set_x(2);
  value->mutable_translation()->set_y(-1);
  value->mutable_translation()->set_z(1.5);
  const Eigen::Quaterniond rotation(
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  value->mutable_rotation()->set_qw(rotation.w());
  value->mutable_rotation()->set_qx(rotation.x());
  value->mutable_rotation()->set_qy(rotation.y());
  value->mutable_rotation()->set_qz(rotation.z());
  transform::BufferTestPeer::ReceiveStatic(buffer, message);

  NDTLocalization localization(&buffer);
  localization.tf_target_frame_id_ = "base_link";
  localization.calibration_lidar_frame_ = "lidar";
  ASSERT_TRUE(localization.UpdateLidarExtrinsic("lidar"));
  const Eigen::Affine3d base_from_lidar =
      Eigen::Translation3d(2, -1, 1.5) * rotation;
  EXPECT_TRUE(localization.velodyne_extrinsic_.isApprox(base_from_lidar));
  EXPECT_TRUE(localization.lidar_locator_.velodyne_extrinsic_.isApprox(
      base_from_lidar));
  const Eigen::Affine3d world_from_base =
      Eigen::Translation3d(20, 30, 4) *
      Eigen::Quaterniond(Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitY()));
  const Eigen::Vector3d lidar_point(3, 4, 1);
  const Eigen::Vector3d expected =
      world_from_base * (base_from_lidar * lidar_point);
  EXPECT_TRUE((world_from_base * localization.velodyne_extrinsic_ * lidar_point)
                  .isApprox(expected));
  const Eigen::Affine3d world_from_lidar = world_from_base * base_from_lidar;
  EXPECT_TRUE((world_from_lidar * localization.velodyne_extrinsic_.inverse())
                  .isApprox(world_from_base));
  EXPECT_FALSE(localization.UpdateLidarExtrinsic("other_lidar"));
}

}  // namespace
}  // namespace ndt
}  // namespace localization
}  // namespace apollo
