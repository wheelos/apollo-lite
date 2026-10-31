// Copyright 2026 WheelOS. All Rights Reserved.
#include "modules/perception/onboard/transform_wrapper/sensor_calibration.h"

#include <memory>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "modules/common/vehicle_calibration/test_bundle.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"
#include "modules/transform/buffer.h"
#include "modules/transform/calibration/static_transform_loader.h"
#include "modules/transform/tests/buffer_test_peer.h"

namespace apollo {
namespace perception {
namespace onboard {
namespace {

TEST(SensorCalibrationTest, ResolvesOpticalAliasesAndTiltedReferenceHeight) {
  namespace vc = apollo::common::vehicle_calibration;
  vc::TestBundle bundle;
  bundle.AddSensor(vc::LIDAR, "lidar", "lidar_link");
  bundle.AddSensor(vc::CAMERA, "front", "camera_front_link");
  bundle.Write("lidar_link.yaml",
               "header: {frame_id: base_link}\nchild_frame_id: lidar_link\n"
               "transform:\n  translation: {x: 1, y: 0, z: 1.5}\n"
               "  rotation: {x: 0, y: 0.7071067811865476, z: 0, "
               "w: 0.7071067811865476}\n");
  bundle.Write("camera_front_link.yaml",
               "header: {frame_id: base_link}\n"
               "child_frame_id: camera_front_link\n"
               "transform:\n  translation: {x: 2, y: 0, z: 1}\n"
               "  rotation: {x: 0, y: 0, z: 0, w: 1}\n");
  bundle.Write("camera_front_link_optical.yaml",
               "header: {frame_id: camera_front_link}\n"
               "child_frame_id: camera_front_link_optical\n"
               "transform:\n  translation: {x: 0.1, y: 0, z: 0.2}\n"
               "  rotation: {x: -0.5, y: 0.5, z: -0.5, w: 0.5}\n");
  ASSERT_TRUE(bundle.Publish());
  ASSERT_TRUE(common::SensorManager::Instance()->Init());
  transform::TransformStampeds transforms;
  ASSERT_TRUE(transform::LoadStaticTransforms(
      common::SensorManager::Instance()->calibration(), &transforms));
  transform::Buffer buffer(false);
  transform::BufferTestPeer::ReceiveStatic(
      buffer, std::make_shared<transform::TransformStampeds>(transforms));
  TransformWrapper wrapper(&buffer);
  for (const std::string camera :
       {"front", "camera_front_link", "camera_front_link_optical"}) {
    float height = 0;
    ASSERT_TRUE(SetCameraHeight(&wrapper, "lidar", camera, &height));
    EXPECT_NEAR(height, 1.5f, 1e-6);
    Eigen::Matrix4d matrix;
    ASSERT_TRUE(QueryStaticTransform(&wrapper, "base_link", camera, &matrix));
    EXPECT_NEAR(matrix(0, 3), 2.1, 1e-9);
    EXPECT_NEAR(matrix(2, 3), 1.2, 1e-9);
    EXPECT_NEAR(matrix(0, 2), 1.0, 1e-9);
  }
  float height = 9;
  EXPECT_FALSE(SetCameraHeight(&wrapper, "unknown", "front", &height));
  EXPECT_FALSE(SetCameraHeight(&wrapper, "lidar", "lidar", &height));
  EXPECT_FLOAT_EQ(height, 9);
  transform::Buffer empty(false);
  TransformWrapper missing(&empty);
  EXPECT_FALSE(SetCameraHeight(&missing, "lidar", "front", &height));
  EXPECT_FLOAT_EQ(height, 9);
}

TEST(SensorCalibrationTest, CommonImageSizeRejectsUnsupportedMixedResolutions) {
  namespace vc = apollo::common::vehicle_calibration;
  vc::TestBundle bundle;
  bundle.AddSensor(vc::CAMERA, "front", "camera_front_link");
  bundle.AddSensor(vc::CAMERA, "rear", "camera_rear_link");
  ASSERT_TRUE(bundle.Publish());
  vc::Registry registry;
  ASSERT_TRUE(registry.LoadFromEnvironment());
  common::SensorManager sensors(std::move(registry));
  ASSERT_TRUE(sensors.Init());
  int width = 0;
  int height = 0;
  ASSERT_TRUE(
      GetCommonCameraImageSize(sensors, {"front", "rear"}, &width, &height));
  EXPECT_EQ(width, 640);
  EXPECT_EQ(height, 480);
  EXPECT_FALSE(GetCommonCameraImageSize(sensors, {}, &width, &height));
  EXPECT_FALSE(GetCommonCameraImageSize(sensors, {"unknown"}, &width, &height));
  bundle.manifest.mutable_sensor(1)->set_image_width(1280);
  bundle.Write("rear_intrinsic.yaml",
               "width: 1280\nheight: 480\n"
               "K: [800, 0, 640, 0, 400, 240, 0, 0, 1]\nD: [0, 0, 0, 0, 0]\n");
  ASSERT_TRUE(bundle.Publish());
  ASSERT_TRUE(registry.LoadFromEnvironment());
  common::SensorManager mixed(std::move(registry));
  ASSERT_TRUE(mixed.Init());
  EXPECT_FALSE(
      GetCommonCameraImageSize(mixed, {"front", "rear"}, &width, &height));
  EXPECT_EQ(width, 640);
  EXPECT_EQ(height, 480);
}

}  // namespace
}  // namespace onboard
}  // namespace perception
}  // namespace apollo
