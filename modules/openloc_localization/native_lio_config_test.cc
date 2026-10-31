// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_lio_config.h"

#include <cstdlib>
#include <fstream>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

#include "google/protobuf/text_format.h"

namespace apollo::localization::openloc_diagnostic {
namespace {

NativeLioHostConfig Profile() {
  const std::string path = std::string(std::getenv("TEST_SRCDIR")) + "/" +
                           std::getenv("TEST_WORKSPACE") +
                           "/testdata/native_lio_profile.pb.txt";
  std::ifstream input(path);
  const std::string text((std::istreambuf_iterator<char>(input)), {});
  NativeLioHostConfig profile;
  if (!input || !google::protobuf::TextFormat::ParseFromString(text, &profile))
    throw std::runtime_error("cannot load synthetic profile fixture");
  return profile;
}

TEST(NativeLioConfigTest, RejectsMissingDuplicateUnknownAndNonfiniteKeys) {
  auto profile = Profile();
  EXPECT_NO_THROW(LocalConfiguration(profile));
  profile.mutable_parameter()->RemoveLast();
  EXPECT_THROW(LocalConfiguration(profile), std::invalid_argument);
  profile = Profile();
  *profile.add_parameter() = profile.parameter(0);
  EXPECT_THROW(LocalConfiguration(profile), std::invalid_argument);
  profile = Profile();
  profile.mutable_parameter(0)->set_name("typo");
  EXPECT_THROW(LocalConfiguration(profile), std::invalid_argument);
  profile = Profile();
  profile.mutable_parameter(0)->set_value(
      std::numeric_limits<double>::quiet_NaN());
  EXPECT_THROW(LocalConfiguration(profile), std::invalid_argument);
}

TEST(NativeLioConfigTest, RejectsTruncatedCountsAndUncalibratedTransforms) {
  auto profile = Profile();
  bool changed = false;
  for (auto& parameter : *profile.mutable_parameter())
    if (parameter.name() == "max_points") {
      parameter.set_value(12000.5);
      changed = true;
    }

  ASSERT_TRUE(changed);
  EXPECT_THROW(LocalConfiguration(profile), std::invalid_argument);
  profile = Profile();
  profile.mutable_imu_from_lidar_rotation()->set_qw(2);
  EXPECT_THROW(LocalConfiguration(profile), std::invalid_argument);
  profile = Profile();
  profile.mutable_imu_from_lidar_translation()->clear_z();
  EXPECT_THROW(LocalConfiguration(profile), std::invalid_argument);
  profile = Profile();
  profile.set_lidar_commit_delay_ms(1);
  EXPECT_THROW(LocalConfiguration(profile), std::invalid_argument);
  profile.set_lidar_commit_delay_ms(101);
  EXPECT_THROW(LocalConfiguration(profile), std::invalid_argument);
  profile.set_lidar_commit_delay_ms(50);
  EXPECT_NO_THROW(LocalConfiguration(profile));
  profile.set_startup_task_timeout_ms(501);
  EXPECT_THROW(LocalConfiguration(profile), std::invalid_argument);
  profile.set_startup_task_timeout_ms(500);
  EXPECT_NO_THROW(LocalConfiguration(profile));
  profile.set_require_live_source_sequences(true);
  EXPECT_NO_THROW(LocalConfiguration(profile));
  profile.set_recorded_receive_time_from_header(true);
  EXPECT_THROW(LocalConfiguration(profile), std::invalid_argument);
}

TEST(NativeLioConfigTest, JointExportIsExplicitBoundedAndPreservesNumericProfile) {
  auto profile = Profile();
  auto* graph = profile.mutable_graph();
  graph->set_output_topic("/openloc/diagnostic/joint");
  graph->set_motion_error_model_id("native-frozen-linearization-v1");
  graph->set_imu_sample_period_s(0.01);
  graph->set_maximum_error_dimension(1024);
  graph->set_maximum_memory_rows(4096);
  graph->set_maximum_snapshots(64);
  graph->set_history_duration_s(0.1);
  graph->set_odometry_stride(5);
  const auto config = LocalConfiguration(profile);
  EXPECT_TRUE(config.enable_native_joint_error);
  EXPECT_DOUBLE_EQ(config.native_imu_sample_period, 0.01);
  graph->set_recorded_source_clock(true);
  EXPECT_THROW(LocalConfiguration(profile), std::invalid_argument);
  profile.set_recorded_receive_time_from_header(true);
  EXPECT_NO_THROW(LocalConfiguration(profile));
  graph->set_odometry_stride(20);
  EXPECT_THROW(LocalConfiguration(profile), std::invalid_argument);
  graph->set_odometry_stride(5);
  graph->set_maximum_error_dimension(2049);
  EXPECT_THROW(LocalConfiguration(profile), std::invalid_argument);
  profile.clear_graph();
  EXPECT_FALSE(LocalConfiguration(profile).enable_native_joint_error);
}

}  // namespace
}  // namespace apollo::localization::openloc_diagnostic
