// Copyright 2026 WheelOS. All Rights Reserved.
#include "modules/transform/calibration/static_transform_loader.h"

#include <string>

#include "gtest/gtest.h"

#include "Eigen/Geometry"

#include "modules/common/vehicle_calibration/test_bundle.h"

namespace apollo {
namespace transform {
namespace {

namespace vc = common::vehicle_calibration;

TEST(StaticTransformLoaderTest, LoadsCompleteSelectedGraph) {
  vc::TestBundle bundle;
  bundle.AddSensor(vc::CAMERA, "front", "camera_front_link");
  bundle.AddSensor(vc::RADAR, "radar", "radar_link");
  ASSERT_TRUE(bundle.Publish());
  vc::Registry registry;
  ASSERT_TRUE(registry.LoadFromEnvironment());
  TransformStampeds transforms;
  ASSERT_TRUE(LoadStaticTransforms(registry, &transforms));
  ASSERT_EQ(transforms.transforms_size(), 3);
  EXPECT_EQ(transforms.transforms(1).header().frame_id(), "camera_front_link");
  EXPECT_EQ(transforms.transforms(1).child_frame_id(),
            "camera_front_link_optical");
}

TEST(StaticTransformLoaderTest, RejectsWrongFrameMalformedAndInvalidNumbers) {
  vc::TestBundle bundle;
  bundle.AddSensor(vc::IMU, "imu", "imu_link");
  const std::string cases[] = {
      "header: {frame_id: wrong}\nchild_frame_id: imu_link\n", "header: [",
      "header: {frame_id: base_link}\nchild_frame_id: imu_link\n"
      "transform:\n  translation: {x: .nan, y: 0, z: 0}\n"
      "  rotation: {x: 0, y: 0, z: 0, w: 1}\n",
      "header: {frame_id: base_link}\nchild_frame_id: imu_link\n"
      "transform:\n  translation: {x: 0, y: 0, z: 0}\n"
      "  rotation: {x: 0, y: 0, z: 0, w: 0}\n",
      "header: {frame_id: base_link}\nchild_frame_id: imu_link\n"
      "transform:\n  translation: {x: 0, y: 0, z: 0}\n"
      "  rotation: {x: 0, y: 0, z: 0.71, w: 0.71}\n"};
  for (const auto& yaml : cases) {
    bundle.Write("imu_link.yaml", yaml);
    ASSERT_TRUE(bundle.Publish());
    vc::Registry registry;
    ASSERT_TRUE(registry.LoadFromEnvironment());
    TransformStampeds transforms;
    transforms.add_transforms();
    EXPECT_FALSE(LoadStaticTransforms(registry, &transforms));
    EXPECT_EQ(transforms.transforms_size(), 0);
  }
}

TEST(StaticTransformLoaderTest, NormalizesAcceptedQuaternionRoundingError) {
  vc::TestBundle bundle;
  bundle.AddSensor(vc::IMU, "imu", "imu_link");
  bundle.Write("imu_link.yaml",
               "header: {frame_id: base_link}\nchild_frame_id: imu_link\n"
               "transform:\n  translation: {x: 1, y: 2, z: 3}\n"
               "  rotation: {x: 0, y: 0, z: 0.7074, w: 0.7074}\n");
  ASSERT_TRUE(bundle.Publish());
  vc::Registry registry;
  ASSERT_TRUE(registry.LoadFromEnvironment());
  TransformStampeds transforms;
  ASSERT_TRUE(LoadStaticTransforms(registry, &transforms));
  const auto& rotation = transforms.transforms(0).transform().rotation();
  const Eigen::Quaterniond quaternion(rotation.qw(), rotation.qx(),
                                      rotation.qy(), rotation.qz());
  EXPECT_NEAR(quaternion.norm(), 1.0, 1e-15);
  const Eigen::Matrix3d matrix = quaternion.toRotationMatrix();
  EXPECT_TRUE((matrix.transpose() * matrix).isIdentity(1e-12));
  EXPECT_NEAR(matrix.determinant(), 1.0, 1e-12);
  EXPECT_TRUE((matrix * Eigen::Vector3d(50, 0, 0))
                  .isApprox(Eigen::Vector3d(0, 50, 0), 1e-12));
}

}  // namespace
}  // namespace transform
}  // namespace apollo
