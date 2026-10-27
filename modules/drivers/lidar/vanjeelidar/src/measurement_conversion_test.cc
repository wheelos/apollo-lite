// Copyright 2026 The WheelOS Authors. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include "modules/drivers/lidar/vanjeelidar/src/measurement_conversion.h"

#include <cmath>
#include <limits>
#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace lidar {
namespace {

VanjeePointCloud MakeCloud(double frame_time, double first, double second) {
  VanjeePointCloud cloud;
  cloud.timestamp = frame_time;
  cloud.is_dense = true;
  for (const double time : {first, second}) {
    ::vanjee::lidar::PointXYZIRT point{};
    point.x = 1.0F;
    point.y = 2.0F;
    point.z = 3.0F;
    point.intensity = 42.0F;
    point.timestamp = time;
    cloud.points.push_back(point);
  }
  return cloud;
}

TEST(VanjeePointTimeTest, ConvertsBeginAndEndRelativeOffsets) {
  for (const auto& cloud :
       {MakeCloud(100.0, 0.0, 0.125), MakeCloud(100.125, -0.125, 0.0)}) {
    PointCloud output;
    std::string error;
    ASSERT_TRUE(ConvertVanjeePointCloud(cloud, true, "lidar", &output, &error))
        << error;
    ASSERT_EQ(output.point_size(), 2);
    EXPECT_EQ(output.point(0).timestamp(), 100000000000ULL);
    EXPECT_EQ(output.point(1).timestamp(), 100125000000ULL);
    EXPECT_DOUBLE_EQ(output.measurement_time(), 100.125);
    EXPECT_EQ(output.header().lidar_timestamp(), 100125000000ULL);
    EXPECT_EQ(output.frame_id(), "lidar");
    EXPECT_EQ(output.header().frame_id(), "lidar");
    EXPECT_EQ(output.width(), 2);
    EXPECT_EQ(output.height(), 1);
    EXPECT_FLOAT_EQ(output.point(0).x(), 1.0F);
    EXPECT_EQ(output.point(0).intensity(), 42U);
  }
}

TEST(VanjeePointTimeTest, DoesNotAddAnchorToAbsoluteTimeOrAssumePointOrder) {
  PointCloud output;
  std::string error;
  ASSERT_TRUE(ConvertVanjeePointCloud(MakeCloud(100.125, 100.125, 100.0), false,
                                      "lidar", &output, &error))
      << error;
  EXPECT_EQ(output.point(0).timestamp(), 100125000000ULL);
  EXPECT_EQ(output.point(1).timestamp(), 100000000000ULL);
  EXPECT_DOUBLE_EQ(output.measurement_time(), 100.125);
  EXPECT_EQ(output.header().lidar_timestamp(), 100125000000ULL);
}

TEST(VanjeePointTimeTest, PreservesSubMicrosecondOffsetsAtEpochScale) {
  PointCloud output;
  std::string error;
  ASSERT_TRUE(ConvertVanjeePointCloud(MakeCloud(1800000000.0, 0.0, 1.0e-7),
                                      true, "lidar", &output, &error))
      << error;
  EXPECT_EQ(output.point(1).timestamp() - output.point(0).timestamp(), 100U);
}

TEST(VanjeePointTimeTest, RejectsInvalidTimeWithoutPublishingPartialCloud) {
  PointCloud output;
  output.set_measurement_time(7.0);
  const std::string before = output.SerializeAsString();
  std::string error;
  auto empty = MakeCloud(100.0, 0.0, 0.125);
  empty.points.clear();
  EXPECT_FALSE(ConvertVanjeePointCloud(empty, true, "lidar", &output, &error));
  EXPECT_EQ(output.SerializeAsString(), before);
  for (const double time : {-101.0, std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity(), 2.0e10}) {
    EXPECT_FALSE(ConvertVanjeePointCloud(MakeCloud(100.0, 0.0, time), true,
                                         "lidar", &output, &error));
    EXPECT_FALSE(error.empty());
    EXPECT_EQ(output.SerializeAsString(), before);
  }
  auto invalid_intensity = MakeCloud(100.0, 0.0, 0.125);
  invalid_intensity.points[1].intensity =
      std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(ConvertVanjeePointCloud(invalid_intensity, true, "lidar",
                                       &output, &error));
  EXPECT_EQ(output.SerializeAsString(), before);
}

::vanjee::lidar::ImuPacket MakeImu() {
  ::vanjee::lidar::ImuPacket imu;
  imu.timestamp = 100.125;
  imu.seq = 19;
  imu.linear_acce = {1.0, -2.0, 9.81};
  imu.angular_voc = {0.1, -0.2, 0.3};
  imu.orientation.fill(std::numeric_limits<double>::quiet_NaN());
  imu.orientation_covariance.fill(std::numeric_limits<double>::quiet_NaN());
  imu.linear_acce_covariance.fill(std::numeric_limits<double>::quiet_NaN());
  imu.angular_voc_covariance.fill(std::numeric_limits<double>::quiet_NaN());
  return imu;
}

TEST(VanjeeImuTest, PreservesSdkSiUnitsFrameSequenceAndMeasurementClock) {
  const auto imu = MakeImu();
  for (const bool use_lidar_clock : {false, true}) {
    vanjee::VanjeeImu output;
    std::string error;
    ASSERT_TRUE(ConvertVanjeeImu(imu, "sdk_imu", use_lidar_clock, 101.0,
                                 &output, &error))
        << error;
    EXPECT_EQ(output.header().frame_id(), "sdk_imu");
    EXPECT_EQ(output.header().sequence_num(), 19U);
    EXPECT_DOUBLE_EQ(output.header().timestamp_sec(), 101.0);
    EXPECT_DOUBLE_EQ(output.measurement_time(), 100.125);
    EXPECT_EQ(output.use_lidar_clock(), use_lidar_clock);
    EXPECT_DOUBLE_EQ(output.linear_acceleration().x(), 1.0);
    EXPECT_DOUBLE_EQ(output.linear_acceleration().y(), -2.0);
    EXPECT_DOUBLE_EQ(output.linear_acceleration().z(), 9.81);
    EXPECT_DOUBLE_EQ(output.angular_velocity().x(), 0.1);
    EXPECT_DOUBLE_EQ(output.angular_velocity().y(), -0.2);
    EXPECT_DOUBLE_EQ(output.angular_velocity().z(), 0.3);
  }
}

TEST(VanjeeImuTest, RejectsInvalidMeasurementsAndRecoversOnNextSample) {
  vanjee::VanjeeImu output;
  output.set_measurement_time(7.0);
  const std::string before = output.SerializeAsString();
  std::string error;
  auto imu = MakeImu();
  imu.angular_voc[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ConvertVanjeeImu(imu, "imu", true, 101.0, &output, &error));
  EXPECT_EQ(output.SerializeAsString(), before);
  imu = MakeImu();
  imu.timestamp = -1.0;
  EXPECT_FALSE(ConvertVanjeeImu(imu, "imu", true, 101.0, &output, &error));
  EXPECT_EQ(output.SerializeAsString(), before);
  imu = MakeImu();
  EXPECT_FALSE(ConvertVanjeeImu(imu, "", true, 101.0, &output, &error));
  EXPECT_EQ(output.SerializeAsString(), before);
  EXPECT_TRUE(ConvertVanjeeImu(imu, "imu", true, 101.0, &output, &error));
  EXPECT_DOUBLE_EQ(output.measurement_time(), 100.125);
}

}  // namespace
}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
