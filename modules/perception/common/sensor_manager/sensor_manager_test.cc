// Copyright 2026 WheelOS. All Rights Reserved.
#include "modules/perception/common/sensor_manager/sensor_manager.h"

#include <utility>

#include "gtest/gtest.h"

#include "modules/common/vehicle_calibration/test_bundle.h"

namespace apollo {
namespace perception {
namespace common {
namespace {

namespace vc = apollo::common::vehicle_calibration;

TEST(SensorManagerTest, LoadsSelectedMetadataAndCameraModel) {
  vc::TestBundle bundle;
  bundle.AddSensor(vc::CAMERA, "front", "camera_front_link");
  bundle.AddSensor(vc::LIDAR, "lidar", "lidar_front_link");
  bundle.AddSensor(vc::RADAR, "radar", "radar_front_link");
  ASSERT_TRUE(bundle.Publish());
  vc::Registry registry;
  ASSERT_TRUE(registry.LoadFromEnvironment());
  SensorManager manager(std::move(registry));
  ASSERT_TRUE(manager.Init());
  EXPECT_TRUE(manager.IsCamera("front"));
  EXPECT_TRUE(manager.IsLidar("lidar"));
  EXPECT_TRUE(manager.IsRadar("radar"));
  EXPECT_EQ(manager.GetFrameId("front"), "camera_front_link_optical");
  EXPECT_EQ(manager.GetFrameId("camera_front_link"),
            "camera_front_link_optical");
  EXPECT_EQ(manager.GetFrameId("camera_front_link_optical"),
            "camera_front_link_optical");
  EXPECT_EQ(manager.GetFrameId("lidar_front_link"), "lidar_front_link");
  const auto camera = manager.GetUndistortCameraModel("front");
  ASSERT_NE(camera, nullptr);
  EXPECT_EQ(camera->get_width(), 640);
  EXPECT_EQ(camera->get_height(), 480);
}

TEST(SensorManagerTest, RejectsResolutionTypeAndIdentityMismatch) {
  for (int variant = 0; variant < 3; ++variant) {
    vc::TestBundle bundle;
    auto* sensor = bundle.AddSensor(vc::CAMERA, "front", "camera_front_link");
    if (variant == 0) {
      sensor->set_image_width(1280);
    } else {
      bundle.Write(
          "front.pb.txt",
          variant == 1
              ? "name: \"front\"\nframe_id: \"camera_front_link_optical\"\n"
                "type: LONG_RANGE_RADAR\norientation: FRONT\n"
              : "name: \"another\"\nframe_id: \"camera_front_link_optical\"\n"
                "type: MONOCULAR_CAMERA\norientation: FRONT\n");
    }
    ASSERT_TRUE(bundle.Publish());
    vc::Registry registry;
    ASSERT_TRUE(registry.LoadFromEnvironment());
    SensorManager manager(std::move(registry));
    EXPECT_FALSE(manager.Init());
    EXPECT_FALSE(manager.IsSensorExist("front"));
  }
}

}  // namespace
}  // namespace common
}  // namespace perception
}  // namespace apollo
