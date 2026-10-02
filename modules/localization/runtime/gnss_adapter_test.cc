// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/runtime/gnss_adapter.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

GlobalEstimatorConfig Config() {
  GlobalEstimatorConfig config;
  auto* map = config.mutable_map();
  map->set_map_id("site");
  map->set_version("1");
  map->set_frame_id("map");
  map->set_calibration_id("extrinsics");
  map->set_pcd_path("");
  map->set_georeferenced(true);
  map->set_origin_latitude(30.0);
  map->set_origin_longitude(120.0);
  map->set_origin_height(50.0);
  config.set_gnss_to_unix_offset(0.0);
  config.set_heading_to_unix_offset(0.0);
  config.set_heading_mount_yaw(0.0);
  config.set_use_gnss_heading(true);
  return config;
}

drivers::gnss::GnssBestPose Position() {
  drivers::gnss::GnssBestPose message;
  message.set_measurement_time(10.0);
  message.set_sol_status(drivers::gnss::SOL_COMPUTED);
  message.set_sol_type(drivers::gnss::NARROW_INT);
  message.set_datum_id(drivers::gnss::WGS84);
  message.set_latitude(30.0);
  message.set_longitude(120.0);
  message.set_height_msl(40.0);
  message.set_undulation(10.0);
  message.set_longitude_std_dev(0.02);
  message.set_latitude_std_dev(0.03);
  message.set_height_std_dev(0.05);
  return message;
}

drivers::gnss::Heading Heading(double time) {
  drivers::gnss::Heading message;
  message.set_measurement_time(time);
  message.set_solution_status(drivers::gnss::SOL_COMPUTED);
  message.set_position_type(drivers::gnss::NARROW_INT);
  message.set_heading(90.0);
  message.set_heading_std_dev(1.0);
  return message;
}

TEST(GnssAdapterTest, UsesEllipsoidHeightHeadingAndLeverArm) {
  GnssAdapter adapter(Config(), Eigen::Vector3d(1.0, 0.0, 0.0));
  ASSERT_TRUE(adapter.ValidateConfig().ok());
  LocalState local;
  local.stamp = {10.0, 10.0, 1, "unix"};
  local.epoch = {"s", 1};
  local.covariance = Matrix15d::Identity() * 0.001;
  GlobalObservation observation;
  ASSERT_TRUE(adapter.AddHeading(Heading(10.0), 10.0).ok());
  ASSERT_TRUE(adapter.Observe(Position(), 10.0, local, &observation).ok());
  EXPECT_NEAR(-1.0, observation.pose.translation().x(), 1e-6);
  EXPECT_NEAR(0.0, observation.pose.translation().z(), 1e-6);
  EXPECT_TRUE(ValidCovariance(observation.covariance));
  EXPECT_TRUE(observation.georeferenced);
}

TEST(GnssAdapterTest, FutureHeadingCannotPairWithEarlierPosition) {
  GnssAdapter adapter(Config(), Eigen::Vector3d::Zero());
  ASSERT_TRUE(adapter.AddHeading(Heading(10.01), 10.01).ok());
  LocalState local;
  local.stamp.time = 10.0;
  GlobalObservation observation;
  ASSERT_TRUE(adapter.Observe(Position(), 10.01, local, &observation).ok());
  EXPECT_EQ(GlobalObservation::Kind::POSITION, observation.kind);
  EXPECT_TRUE((observation.covariance.bottomRightCorner<3, 3>().isZero()));
}

TEST(GnssAdapterTest, PositionOnlyKeepsAntennaReferenceWithoutInventingYaw) {
  auto config = Config();
  config.set_use_gnss_heading(false);
  config.clear_heading_to_unix_offset();
  config.clear_heading_mount_yaw();
  GnssAdapter adapter(config, Eigen::Vector3d(1.0, 0.0, 0.0));
  ASSERT_TRUE(adapter.ValidateConfig().ok());
  LocalState local;
  local.stamp.time = 10.0;
  GlobalObservation observation;
  ASSERT_TRUE(adapter.Observe(Position(), 10.0, local, &observation).ok());
  EXPECT_EQ(GlobalObservation::Kind::POSITION, observation.kind);
  EXPECT_NEAR(0.0, observation.pose.translation().norm(), 1e-6);
  EXPECT_DOUBLE_EQ(1.0, observation.point_in_base.x());
  EXPECT_TRUE((observation.covariance.bottomRightCorner<3, 3>().isZero()));
}

TEST(GnssAdapterTest, FixTypeAloneDoesNotProveCovariance) {
  GnssAdapter adapter(Config(), Eigen::Vector3d::Zero());
  ASSERT_TRUE(adapter.AddHeading(Heading(10.0), 10.0).ok());
  auto position = Position();
  position.clear_height_std_dev();
  LocalState local;
  local.stamp.time = 10.0;
  GlobalObservation observation;
  EXPECT_EQ(Reason::INVALID_INPUT,
            adapter.Observe(position, 10.0, local, &observation).reason);
}

TEST(GnssAdapterTest, GeographicReferenceIsNotRequiredForIndoorMap) {
  auto config = Config();
  config.mutable_map()->set_georeferenced(false);
  GnssAdapter adapter(config, Eigen::Vector3d::Zero());
  EXPECT_EQ(Reason::CONFIG_INVALID, adapter.ValidateConfig().reason);
  Eigen::Vector3d position;
  EXPECT_EQ(Reason::CONFIG_INVALID,
            GeodeticToMap(config.map(), 30.0, 120.0, 50.0, &position).reason);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace apollo
