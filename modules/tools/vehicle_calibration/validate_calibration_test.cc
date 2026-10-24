// Copyright 2026 WheelOS. All Rights Reserved.
#include <sys/wait.h>
#include <unistd.h>

#include <cstdlib>
#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "tools/cpp/runfiles/runfiles.h"

#include "modules/common/vehicle_calibration/test_bundle.h"

namespace apollo {
namespace tools {
namespace {

namespace vc = apollo::common::vehicle_calibration;

class CalibrationValidatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::string error;
    std::unique_ptr<bazel::tools::cpp::runfiles::Runfiles> runfiles(
        bazel::tools::cpp::runfiles::Runfiles::CreateForTest(
            BAZEL_CURRENT_REPOSITORY, &error));
    ASSERT_NE(runfiles, nullptr) << error;
    executable_ = runfiles->Rlocation(
        "apollo/modules/tools/vehicle_calibration/validate_calibration",
        BAZEL_CURRENT_REPOSITORY);
    ASSERT_EQ(access(executable_.c_str(), X_OK), 0) << executable_;
  }

  int Validate() const {
    const pid_t child = fork();
    if (child == 0) {
      execl(executable_.c_str(), executable_.c_str(), nullptr);
      std::_Exit(127);
    }
    int status = 0;
    if (child < 0 || waitpid(child, &status, 0) != child ||
        !WIFEXITED(status)) {
      return -1;
    }
    return WEXITSTATUS(status);
  }

 private:
  std::string executable_;
};

TEST_F(CalibrationValidatorTest, AcceptsCompleteSelectedBundle) {
  vc::TestBundle bundle;
  bundle.AddSensor(vc::IMU, "imu", "imu_link");
  bundle.AddSensor(vc::LIDAR, "lidar", "lidar_link");
  bundle.AddSensor(vc::RADAR, "radar", "radar_link");
  bundle.AddSensor(vc::CAMERA, "front", "camera_front_link");
  ASSERT_TRUE(bundle.Publish());
  EXPECT_EQ(Validate(), 0);
}

TEST_F(CalibrationValidatorTest, RejectsTamperedAssetsAndInvalidCameraModel) {
  vc::TestBundle bundle;
  bundle.AddSensor(vc::CAMERA, "front", "camera_front_link");
  ASSERT_TRUE(bundle.Publish());
  bundle.Write("front_intrinsic.yaml", "not an intrinsic calibration\n");
  EXPECT_EQ(Validate(), 1);
  ASSERT_TRUE(bundle.Publish());
  EXPECT_EQ(Validate(), 1);
}

TEST_F(CalibrationValidatorTest, RejectsInvalidExtrinsicAndHeight) {
  vc::TestBundle bundle;
  bundle.AddSensor(vc::LIDAR, "lidar", "lidar_link");
  bundle.Write("lidar_link.yaml", "invalid extrinsic\n");
  ASSERT_TRUE(bundle.Publish());
  EXPECT_EQ(Validate(), 1);
  bundle.Write("lidar_link.yaml",
               "header: {frame_id: base_link}\nchild_frame_id: lidar_link\n"
               "transform:\n  translation: {x: 0, y: 0, z: 0}\n"
               "  rotation: {x: 0, y: 0, z: 0, w: 1}\n");
  bundle.Write("lidar_height.yaml",
               "vehicle:\n  parameters: {height: -1, height_var: 0}\n");
  ASSERT_TRUE(bundle.Publish());
  EXPECT_EQ(Validate(), 1);
}

TEST_F(CalibrationValidatorTest, ValidatesSchemaOneGlobalLidarHeight) {
  vc::TestBundle bundle;
  bundle.AddSensor(vc::IMU, "imu", "imu_link");
  bundle.AddSensor(vc::LIDAR, "lidar", "lidar_top_link");
  bundle.manifest.set_schema_version(1);
  bundle.manifest.clear_static_transform();
  bundle.manifest.set_lidar_height_file("lidar_height.yaml");
  for (auto& sensor : *bundle.manifest.mutable_sensor()) {
    sensor.set_parent_frame_id("base_link");
    sensor.set_extrinsic_file(sensor.frame_id() + ".yaml");
    sensor.clear_lidar_height_file();
  }
  ASSERT_TRUE(bundle.Publish());
  EXPECT_EQ(Validate(), 0);
  bundle.Write("lidar_height.yaml",
               "vehicle:\n  parameters: {height: -1, height_var: 0}\n");
  ASSERT_TRUE(bundle.Publish());
  EXPECT_EQ(Validate(), 1);
  bundle.Write("lidar_height.yaml", "not height calibration\n");
  ASSERT_TRUE(bundle.Publish());
  EXPECT_EQ(Validate(), 1);
}

}  // namespace
}  // namespace tools
}  // namespace apollo
