// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "modules/localization/localization_adapter.h"

#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "modules/common/map/map_selection.h"
#include "modules/common/vehicle_calibration/test_bundle.h"
#include "modules/localization/localization_config.h"
#include "modules/transform/tests/buffer_test_peer.h"

namespace apollo::localization::openloc {
namespace {

LocalizationConfig Template() {
  return LoadDeploymentConfiguration(
      "modules/localization/conf/localization.pb.txt");
}

host::NativeGlobalPose Identity();

LocalizationConfig Config() {
  auto config = Template();
  config.set_deployment_configured(true);
  config.set_imu_frame("imu");
  config.set_vertical_datum("synthetic-height");
  config.set_utm_origin_height_m(30);
  config.set_imu_sensor("imu");
  config.set_lidar_sensor("lidar");
  config.set_gnss_sensor("gnss");
  *config.mutable_imu_from_base() = Identity();
  config.mutable_imu_from_base()->set_x(-1);
  *config.mutable_imu_from_vrp_flu() = config.imu_from_base();
  auto* local = config.mutable_local();
  local->set_lidar_topic("/test/lidar");
  local->set_lidar_frame("lidar");
  local->set_calibration_id("synthetic-calibration");
  local->mutable_imu_from_lidar_translation()->set_x(0);
  local->mutable_imu_from_lidar_translation()->set_y(0);
  local->mutable_imu_from_lidar_translation()->set_z(0);
  local->mutable_imu_from_lidar_rotation()->set_qx(0);
  local->mutable_imu_from_lidar_rotation()->set_qy(0);
  local->mutable_imu_from_lidar_rotation()->set_qz(0);
  local->mutable_imu_from_lidar_rotation()->set_qw(1);
  local->set_session("synthetic-session");
  local->set_journal("local.csv");
  local->mutable_graph()->set_motion_error_model_id("synthetic-model");
  local->mutable_graph()->set_imu_sample_period_s(0.01);
  auto* global = config.mutable_global();
  global->set_calibration_id(local->calibration_id());
  global->set_reference_id("synthetic-map");
  *global->mutable_online_map()->mutable_imu_from_lidar() = Identity();
  global->set_journal("global.csv");
  global->mutable_target_graph()->set_motion_error_model_id("synthetic-model");
  global->mutable_target_graph()->set_motion_error_model_qualified(true);
  global->mutable_online_map()->set_cloud_topic(local->lidar_topic());
  global->mutable_online_map()->set_lidar_frame(local->lidar_frame());
  global->mutable_online_map()->set_hypothesis_odometry_session(
      local->session());
  global->mutable_structured_gnss()->set_origin_easting_m(500000);
  global->mutable_structured_gnss()->set_origin_northing_m(4000000);
  global->mutable_structured_gnss()->set_lever_x(0);
  global->mutable_structured_gnss()->set_lever_y(0);
  global->mutable_structured_gnss()->set_lever_z(0);
  auto* qualification =
      global->mutable_structured_gnss()->mutable_position_graph_qualification();
  qualification->set_uncertainty_model_id("synthetic-white");
  qualification->set_independence_evidence_id("synthetic-independent");
  qualification->set_independent_of_local(true);
  qualification->set_independent_of_graph_sources(true);
  return config;
}

host::NativeGlobalPose Identity() {
  host::NativeGlobalPose pose;
  pose.set_x(0);
  pose.set_y(0);
  pose.set_z(0);
  pose.set_qx(0);
  pose.set_qy(0);
  pose.set_qz(0);
  pose.set_qw(1);
  return pose;
}

host::NativeGlobalDiagnostic Global() {
  host::NativeGlobalDiagnostic global;
  global.set_selected_owner("fixed_lag");
  global.set_session("synthetic-session");
  global.set_generation(1);
  global.set_odometry_sequence(12);
  global.set_measurement_ns(100000000000);
  global.set_usable(true);
  global.set_full_pose_available(true);
  global.set_target_graph_available(true);
  global.set_new_measurement(true);
  *global.mutable_map_from_imu() = Identity();
  global.mutable_map_from_imu()->set_x(10);
  global.mutable_map_from_imu()->set_y(20);
  global.mutable_map_from_imu()->set_z(3);
  *global.mutable_map_from_odom() = Identity();
  return global;
}

native::LocalState Local() {
  native::LocalState local;
  local.stamp = {100, 100, 12, "unix"};
  local.epoch = {"synthetic-session", 1};
  local.valid = true;
  local.velocity = {2, 0, 0};
  local.angular_velocity = {0, 0, 1};
  return local;
}

TEST(LocalizationAdapter, RefusesTemplateAndMismatchedLocalGlobalContracts) {
  EXPECT_THROW(ValidateConfiguration(Template()), std::invalid_argument);
  auto config = Config();
  EXPECT_NO_THROW(ValidateConfiguration(config));
  config.mutable_global()
      ->mutable_online_map()
      ->mutable_imu_from_lidar()
      ->set_x(1);
  EXPECT_THROW(ValidateConfiguration(config), std::invalid_argument);
  config = Config();
  config.mutable_global()->clear_target_graph();
  EXPECT_THROW(ValidateConfiguration(config), std::invalid_argument);
  config = Config();
  config.mutable_local()->set_recorded_receive_time_from_header(true);
  EXPECT_THROW(ValidateConfiguration(config), std::invalid_argument);
  config = Config();
  config.set_status_topic(config.localization_topic());
  EXPECT_THROW(ValidateConfiguration(config), std::invalid_argument);
}

TEST(LocalizationAdapter, RequiresInstantaneousFluImuWithExplicitClock) {
  auto config = Config();
  config.mutable_local()->set_imu_measurement_to_unix_offset(10);
  drivers::gnss::Imu input;
  input.mutable_header()->set_frame_id("imu");
  input.set_measurement_time(90);
  input.mutable_linear_acceleration()->set_x(1);
  input.mutable_linear_acceleration()->set_y(2);
  input.mutable_linear_acceleration()->set_z(9.8);
  input.mutable_angular_velocity()->set_x(0.1);
  input.mutable_angular_velocity()->set_y(0.2);
  input.mutable_angular_velocity()->set_z(0.3);
  const auto sample = DecodeImu(input, config, 100.01, 3);
  EXPECT_DOUBLE_EQ(sample.stamp.time, 100);
  EXPECT_DOUBLE_EQ(sample.acceleration.y(), 2);
  EXPECT_DOUBLE_EQ(sample.angular_velocity.z(), 0.3);
  input.set_measurement_span(0.01);
  EXPECT_THROW(DecodeImu(input, config, 100.01, 3), std::invalid_argument);
  input.set_measurement_span(0);
  input.mutable_header()->set_frame_id("rfu");
  EXPECT_THROW(DecodeImu(input, config, 100.01, 3), std::invalid_argument);
  input.mutable_header()->set_frame_id("imu");
  input.mutable_linear_acceleration()->clear_z();
  EXPECT_THROW(DecodeImu(input, config, 100.01, 3), std::invalid_argument);
}

TEST(LocalizationAdapter, PreservesTimedRawCloudAndRejectsInvalidPoints) {
  const auto config = Config();
  drivers::PointCloud cloud;
  cloud.set_frame_id("lidar");
  cloud.set_measurement_time(100);
  for (int i = 0; i < 5; ++i) {
    auto* point = cloud.add_point();
    point->set_x(i);
    point->set_y(0);
    point->set_z(0);
    point->set_timestamp(99960000000 + i * 10000000);
  }
  const auto scan = DecodeCloud(cloud, config, 2, 100.01, 7);
  ASSERT_EQ(scan.points.size(), 2);
  EXPECT_DOUBLE_EQ(scan.points[0].position.x(), 1);
  EXPECT_DOUBLE_EQ(scan.points[1].position.x(), 3);
  EXPECT_DOUBLE_EQ(scan.points[0].time, 99.97);
  cloud.mutable_point(2)->set_timestamp(99900000000);
  EXPECT_THROW(DecodeCloud(cloud, config, 2, 100.01, 7), std::invalid_argument);
  cloud.mutable_point(2)->set_timestamp(99980000000);
  cloud.mutable_point(2)->set_x(std::numeric_limits<float>::quiet_NaN());
  EXPECT_THROW(DecodeCloud(cloud, config, 2, 100.01, 7), std::invalid_argument);
}

TEST(LocalizationAdapter, PublishesUtmRearAxleRfuAndRigidBodyVelocity) {
  const auto local = Local();
  const auto output = MakePublication(Config(), Global(), &local, 100.01, 4);
  ASSERT_TRUE(output.pose);
  ASSERT_TRUE(output.transform);
  const auto& pose = output.pose->pose();
  EXPECT_DOUBLE_EQ(pose.position().x(), 500009);
  EXPECT_DOUBLE_EQ(pose.position().y(), 4000020);
  EXPECT_DOUBLE_EQ(pose.position().z(), 33);
  EXPECT_NEAR(pose.heading(), 0, 1e-12);
  EXPECT_DOUBLE_EQ(pose.linear_velocity().x(), 2);
  EXPECT_DOUBLE_EQ(pose.linear_velocity().y(), -1);
  EXPECT_DOUBLE_EQ(pose.angular_velocity_vrf().z(), 1);
  EXPECT_DOUBLE_EQ(output.pose->measurement_time(), 100);
  EXPECT_DOUBLE_EQ(output.transform->header().timestamp_sec(), 100);
  EXPECT_DOUBLE_EQ(output.transform->transform().translation().x(), 500009);
  EXPECT_EQ(output.transform->header().frame_id(), "map");
  EXPECT_EQ(output.transform->child_frame_id(), "base_link");
  EXPECT_DOUBLE_EQ(output.transform->transform().rotation().qw(), 1);
  EXPECT_DOUBLE_EQ(output.transform->transform().rotation().qz(), 0);
  EXPECT_NEAR(pose.orientation().qz(), -std::sqrt(0.5), 1e-12);
  EXPECT_EQ(output.status.fusion_status(), MeasureState::WARNNING);
  EXPECT_EQ(output.pose->msf_status().local_lidar_quality(),
            LocalLidarQuality::MSF_LOCAL_LIDAR_BAD);
  EXPECT_FALSE(output.pose->has_uncertainty());
  EXPECT_FALSE(pose.has_linear_acceleration());
}

TEST(LocalizationAdapter, RotatesKinematicsAndRearAxleIntoAbsoluteFrame) {
  auto global = Global();
  const double half = std::sqrt(0.5);
  global.mutable_map_from_imu()->set_qw(half);
  global.mutable_map_from_imu()->set_qz(half);
  global.mutable_map_from_odom()->set_qw(half);
  global.mutable_map_from_odom()->set_qz(half);
  const auto local = Local();
  const auto result = MakePublication(Config(), global, &local, 100.01, 1);
  ASSERT_TRUE(result.pose);
  const auto& pose = result.pose->pose();
  EXPECT_NEAR(pose.position().x(), 500010, 1e-9);
  EXPECT_NEAR(pose.position().y(), 4000019, 1e-9);
  EXPECT_NEAR(pose.linear_velocity().x(), 1, 1e-12);
  EXPECT_NEAR(pose.linear_velocity().y(), 2, 1e-12);
  EXPECT_NEAR(pose.heading(), std::acos(-1) / 2, 1e-12);
}

TEST(LocalizationAdapter, KeepsDistinctBodyAndRearAxleReferencePoints) {
  auto config = Config();
  config.set_vrp_frame("rear_axle");
  config.mutable_imu_from_vrp_flu()->set_x(-2);
  const auto local = Local();
  const auto result = MakePublication(config, Global(), &local, 100.01, 1);
  ASSERT_TRUE(result.pose);
  ASSERT_TRUE(result.transform);
  EXPECT_DOUBLE_EQ(result.pose->pose().position().x(), 500008);
  EXPECT_DOUBLE_EQ(result.pose->pose().linear_velocity().y(), -2);
  EXPECT_DOUBLE_EQ(result.transform->transform().translation().x(), 500009);
}

TEST(LocalizationAdapter, SuppressesPartialStaleResetReplayAndUnmatchedPoses) {
  auto local = Local();
  auto global = Global();
  const auto config = Config();
  const auto rejected = [&] {
    const auto output = MakePublication(config, global, &local, 100.01, 1);
    EXPECT_FALSE(output.pose);
    EXPECT_FALSE(output.transform);
    EXPECT_EQ(output.status.fusion_status(), MeasureState::ERROR);
  };
  global.set_full_pose_available(false);
  rejected();
  global = Global();
  global.set_reset_pending(true);
  rejected();
  global = Global();
  global.set_recorded_input(true);
  rejected();
  global = Global();
  local.epoch.generation = 2;
  rejected();
  local = Local();
  local.stamp.time = 99.99;
  rejected();
  local = Local();
  const auto stale = MakePublication(config, global, &local, 101, 1);
  EXPECT_FALSE(stale.pose);
  EXPECT_EQ(stale.status.fusion_status(), MeasureState::ERROR);
  global.set_new_measurement(false);
  const auto held = MakePublication(config, global, &local, 100.01, 1);
  EXPECT_FALSE(held.pose);
  EXPECT_FALSE(held.transform);
  EXPECT_EQ(held.status.fusion_status(), MeasureState::WARNNING);
}

class SharedConfigurationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    namespace vc = common::vehicle_calibration;
    bundle_.AddSensor(vc::IMU, "imu", "imu_link");
    bundle_.AddSensor(vc::LIDAR, "lidar", "lidar_link");
    bundle_.AddSensor(vc::GNSS, "gnss", "gnss_antenna");
    bundle_.AddSensor(vc::GNSS, "gnss2", "gnss_secondary");
    AddEdge("imu_link", 1);
    AddEdge("lidar_link", 4);
    AddEdge("gnss_antenna", 2);
    AddEdge("gnss_secondary", 3);
    ASSERT_TRUE(bundle_.Publish());
    ASSERT_TRUE(registry_.LoadFromEnvironment());
    map_directory_ = bundle_.path("selected_map");
    std::filesystem::create_directories(map_directory_);
    std::ofstream(map_directory_ + "/cloud.pcd") << "map-path fixture";
    common::MapSelection::SetTestMapDirectory(map_directory_);
  }

  void TearDown() override { common::MapSelection::ClearTestMapDirectory(); }

  void AddEdge(const std::string& frame, double x) {
    bundle_.Write(frame + ".yaml",
                  "header: {frame_id: base_link}\nchild_frame_id: " + frame +
                      "\ntransform:\n  translation: {x: " + std::to_string(x) +
                      ", y: 0, z: 0}\n"
                      "  rotation: {x: 0, y: 0, z: 0, w: 1}\n");
    auto message = std::make_shared<transform::TransformStampeds>();
    auto* edge = message->add_transforms();
    edge->mutable_header()->set_frame_id("base_link");
    edge->mutable_header()->set_timestamp_sec(1);
    edge->set_child_frame_id(frame);
    edge->mutable_transform()->mutable_translation()->set_x(x);
    edge->mutable_transform()->mutable_translation()->set_y(0);
    edge->mutable_transform()->mutable_translation()->set_z(0);
    edge->mutable_transform()->mutable_rotation()->set_qx(0);
    edge->mutable_transform()->mutable_rotation()->set_qy(0);
    edge->mutable_transform()->mutable_rotation()->set_qz(0);
    edge->mutable_transform()->mutable_rotation()->set_qw(1);
    transform::BufferTestPeer::ReceiveStatic(buffer_, message);
  }

  LocalizationConfig Deployment() {
    auto config = Config();
    config.set_static_tf_timeout_s(0);
    config.clear_imu_frame();
    config.clear_imu_from_base();
    config.clear_imu_from_vrp_flu();
    auto* local = config.mutable_local();
    local->clear_lidar_frame();
    local->clear_calibration_id();
    local->clear_imu_from_lidar_translation();
    local->clear_imu_from_lidar_rotation();
    auto* global = config.mutable_global();
    global->clear_reference_id();
    global->clear_calibration_id();
    global->mutable_online_map()->clear_lidar_frame();
    global->mutable_online_map()->clear_imu_from_lidar();
    global->mutable_online_map()->set_map_pcd("cloud.pcd");
    global->mutable_structured_gnss()->clear_lever_x();
    global->mutable_structured_gnss()->clear_lever_y();
    global->mutable_structured_gnss()->clear_lever_z();
    return config;
  }

  common::vehicle_calibration::TestBundle bundle_;
  common::vehicle_calibration::Registry registry_;
  transform::Buffer buffer_{false};
  transform::TransformQuery query_{&buffer_};
  std::string map_directory_;
};

TEST_F(SharedConfigurationTest, UsesSelectedMapRegistryAndComposedStaticTf) {
  const auto deployment = Deployment();
  const auto config = ResolveConfiguration(deployment, registry_, query_);
  EXPECT_EQ(config.global().reference_id(), "selected_map");
  EXPECT_EQ(config.global().online_map().map_pcd(),
            map_directory_ + "/cloud.pcd");
  EXPECT_EQ(config.local().calibration_id(), registry_.manifest_sha256());
  EXPECT_EQ(config.imu_frame(), "imu_link");
  EXPECT_EQ(config.local().lidar_frame(), "lidar_link");
  EXPECT_DOUBLE_EQ(config.local().imu_from_lidar_translation().x(), 3);
  EXPECT_DOUBLE_EQ(config.global().online_map().imu_from_lidar().x(), 3);
  EXPECT_DOUBLE_EQ(config.global().structured_gnss().lever_x(), 1);
  EXPECT_DOUBLE_EQ(config.imu_from_base().x(), -1);
  EXPECT_DOUBLE_EQ(config.imu_from_vrp_flu().x(), -1);
  EXPECT_EQ(config.parent_frame(), "map");
  EXPECT_EQ(config.child_frame(), "base_link");
  EXPECT_TRUE(config.IsInitialized());
  EXPECT_FALSE(deployment.IsInitialized());

  const auto other = bundle_.path("other_map");
  std::filesystem::create_directories(other);
  std::ofstream(other + "/cloud.pcd") << "other map";
  common::MapSelection::SetTestMapDirectory(other);
  const auto changed = ResolveConfiguration(deployment, registry_, query_);
  EXPECT_EQ(changed.global().reference_id(), "other_map");
  EXPECT_EQ(changed.global().online_map().map_pcd(), other + "/cloud.pcd");
}

TEST_F(SharedConfigurationTest,
       RejectsManualOverridesMissingTfAndWrongSensors) {
  auto deployment = Deployment();
  deployment.mutable_global()->mutable_structured_gnss()->set_lever_x(99);
  EXPECT_THROW(ResolveConfiguration(deployment, registry_, query_),
               std::invalid_argument);
  deployment = Deployment();
  deployment.set_gnss_sensor("lidar");
  EXPECT_THROW(ResolveConfiguration(deployment, registry_, query_),
               std::invalid_argument);
  deployment = Deployment();
  transform::Buffer empty(false);
  transform::TransformQuery query(&empty);
  EXPECT_THROW(ResolveConfiguration(deployment, registry_, query),
               std::invalid_argument);
  deployment.mutable_global()->clear_reference_version();
  EXPECT_THROW(ResolveConfiguration(deployment, registry_, query_),
               std::invalid_argument);
}

TEST_F(SharedConfigurationTest, RejectsAbsoluteMissingAndEscapingMapFiles) {
  auto deployment = Deployment();
  for (const auto& path :
       {map_directory_ + "/cloud.pcd", std::string("../cloud.pcd"),
        std::string("missing.pcd")}) {
    deployment.mutable_global()->mutable_online_map()->set_map_pcd(path);
    EXPECT_THROW(ResolveConfiguration(deployment, registry_, query_),
                 std::exception);
  }
  const auto outside = bundle_.path("outside.pcd");
  std::ofstream(outside) << "outside selected map";
  std::filesystem::create_symlink(outside, map_directory_ + "/escape.pcd");
  deployment.mutable_global()->mutable_online_map()->set_map_pcd("escape.pcd");
  EXPECT_THROW(ResolveConfiguration(deployment, registry_, query_),
               std::invalid_argument);
}

TEST_F(SharedConfigurationTest,
       ResolvesDualAntennaBaselineFromSelectedSensors) {
  auto deployment = Deployment();
  auto* heading =
      deployment.mutable_global()->mutable_structured_gnss()->mutable_heading();
  heading->set_topic("/test/heading");
  heading->set_true_north_yaw_rad(M_PI_2);
  heading->set_maximum_std_rad(0.035);
  heading->set_maximum_baseline_error_m(0.02);
  heading->set_minimum_satellites(12);
  heading->set_fixed_stability_ns(2000000000);
  heading->set_maximum_gap_ns(1500000000);
  EXPECT_THROW(ResolveConfiguration(deployment, registry_, query_),
               std::invalid_argument);
  deployment.set_secondary_gnss_sensor("gnss2");
  const auto config = ResolveConfiguration(deployment, registry_, query_);
  EXPECT_DOUBLE_EQ(config.global().structured_gnss().lever_x(), 1);
  EXPECT_DOUBLE_EQ(config.global().structured_gnss().heading().baseline_x(), 1);
  EXPECT_DOUBLE_EQ(config.global().structured_gnss().heading().baseline_y(), 0);
}

}  // namespace
}  // namespace apollo::localization::openloc
