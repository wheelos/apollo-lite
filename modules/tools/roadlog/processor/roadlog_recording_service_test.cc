/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/tools/roadlog/processor/roadlog_recording_service.h"

#include <cstdlib>
#include <string>

#include "gtest/gtest.h"

#include "modules/common/vehicle_calibration/test_bundle.h"
#include "modules/tools/roadlog/common/roadlog_layout.h"

namespace apollo {
namespace data {
namespace {

TEST(RoadlogRecordingServiceTest, BuildsRecorderOptionsFromPolicy) {
  SmartRecordTrigger trigger_conf;
  trigger_conf.mutable_segment_setting()->set_time_segment(5);
  trigger_conf.mutable_segment_setting()->set_size_segment(64);
  auto* recorder_policy = trigger_conf.mutable_recorder_policy();
  recorder_policy->add_include_channels("/apollo/planning");
  recorder_policy->add_exclude_channels("/apollo/debug");
  auto* limit = recorder_policy->add_channel_rate_limits();
  limit->set_channel_name("/apollo/sensor/lidar/fusion/PointCloud2");
  limit->set_max_rate_hz(1.0);
  auto* large_message_policy = recorder_policy->mutable_large_message_policy();
  large_message_policy->set_drop_message_size_bytes(1ULL * 1024ULL * 1024ULL);

  RoadlogRecordingService::RecorderOptions options;
  std::string error;
  ASSERT_TRUE(RoadlogRecordingService::BuildRecorderOptions(
      trigger_conf, "/tmp/test.record", &options, &error));
  EXPECT_TRUE(error.empty());
  EXPECT_EQ("/tmp/test.record", options.output_path);
  ASSERT_EQ(1U, options.include_channels.size());
  EXPECT_EQ("/apollo/planning", options.include_channels.front());
  ASSERT_EQ(1U, options.exclude_channels.size());
  EXPECT_EQ("/apollo/debug", options.exclude_channels.front());
  ASSERT_EQ(1U, options.rate_limited_channels.size());
  EXPECT_EQ("/apollo/sensor/lidar/fusion/PointCloud2",
            options.rate_limited_channels.front());
  EXPECT_EQ(5ULL * 1000000000ULL, options.header.segment_interval());
  EXPECT_EQ(64ULL * 1024ULL * 1024ULL, options.header.segment_raw_size());
  const auto& recorder_config = options.recorder_config_bundle;
  EXPECT_EQ(1U, recorder_config.version);
  EXPECT_EQ(apollo::cyber::record::SelectorAction::kExclude,
            recorder_config.subscription.default_action);
  ASSERT_EQ(2U, recorder_config.subscription.selectors.size());
  EXPECT_EQ(1U, recorder_config.policies.rules.size());
  EXPECT_DOUBLE_EQ(1.0,
                   recorder_config.policies.rules.front().policy.max_rate_hz);
  EXPECT_EQ(1ULL * 1024ULL * 1024ULL,
            recorder_config.policies.default_policy.drop_message_size_bytes);
}

TEST(RoadlogRecordingServiceTest, RecordsSelectedCalibrationIdentity) {
  namespace vc = apollo::common::vehicle_calibration;
  vc::TestBundle bundle;
  bundle.AddSensor(vc::LIDAR, "lidar", "lidar_link");
  ASSERT_TRUE(bundle.Publish());
  std::string digest;
  ASSERT_TRUE(vc::Registry::FileSha256(
      bundle.path("calibration_manifest.pb.txt"), &digest));
  RoadlogRecordingService::RecorderOptions options;
  std::string error;
  ASSERT_TRUE(RoadlogRecordingService::BuildRecorderOptions(
      SmartRecordTrigger(), "/tmp/test.record", &options, &error));
  const auto& metadata = options.header.metadata();
  EXPECT_EQ(metadata.at("wheelos.calibration.status"), "selected");
  EXPECT_EQ(metadata.at("wheelos.calibration.vehicle_id"), "test-vehicle");
  EXPECT_EQ(metadata.at("wheelos.calibration.version"), "v2");
  EXPECT_EQ(metadata.at("wheelos.calibration.manifest_sha256"), digest);
  EXPECT_EQ(metadata.at("wheelos.calibration.schema_version"), "2");
  EXPECT_EQ(metadata.at("wheelos.calibration.test_only"), "true");
}

TEST(RoadlogRecordingServiceTest, RejectsInvalidSelectedCalibration) {
  namespace vc = apollo::common::vehicle_calibration;
  vc::TestBundle bundle;
  bundle.AddSensor(vc::LIDAR, "lidar", "lidar_link");
  ASSERT_TRUE(bundle.Publish());
  setenv("WHEELOS_CALIBRATION_MANIFEST_SHA256", "wrong", 1);
  RoadlogRecordingService::RecorderOptions options;
  std::string error;
  EXPECT_FALSE(RoadlogRecordingService::BuildRecorderOptions(
      SmartRecordTrigger(), "/tmp/test.record", &options, &error));
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(options.header.metadata().empty());
  unsetenv("WHEELOS_VEHICLE_PROFILE_KEY");
  EXPECT_FALSE(RoadlogRecordingService::BuildRecorderOptions(
      SmartRecordTrigger(), "/tmp/test.record", &options, &error));
  EXPECT_TRUE(options.header.metadata().empty());
}

TEST(RoadlogRecordingServiceTest, ExplicitlyMarksUnselectedCalibration) {
  apollo::common::vehicle_calibration::TestBundle environment;
  unsetenv("WHEELOS_VEHICLE_PROFILE_KEY");
  unsetenv("WHEELOS_CALIBRATION_MANIFEST_SHA256");
  RoadlogRecordingService::RecorderOptions options;
  std::string error;
  ASSERT_TRUE(RoadlogRecordingService::BuildRecorderOptions(
      SmartRecordTrigger(), "/tmp/test.record", &options, &error));
  EXPECT_EQ(options.header.metadata().size(), 1);
  EXPECT_EQ(options.header.metadata().at("wheelos.calibration.status"),
            "not_selected");
}

TEST(RoadlogRecordingServiceTest, RejectsDuplicateRateRule) {
  SmartRecordTrigger trigger_conf;
  auto* recorder_policy = trigger_conf.mutable_recorder_policy();
  auto* first = recorder_policy->add_channel_rate_limits();
  first->set_channel_name("/apollo/lidar");
  first->set_max_rate_hz(1.0);
  auto* second = recorder_policy->add_channel_rate_limits();
  second->set_channel_name("/apollo/lidar");
  second->set_max_rate_hz(2.0);

  RoadlogRecordingService::RecorderOptions options;
  std::string error;
  EXPECT_FALSE(RoadlogRecordingService::BuildRecorderOptions(
      trigger_conf, "/tmp/test.record", &options, &error));
  EXPECT_FALSE(error.empty());
}

TEST(RoadlogRecordingServiceTest, RejectsLegacyMessageSizePolicy) {
  SmartRecordTrigger trigger_conf;
  auto* recorder_policy = trigger_conf.mutable_recorder_policy();
  recorder_policy->set_legacy_message_size_filter_policy("throttle=256@2");

  RoadlogRecordingService::RecorderOptions options;
  std::string error;
  EXPECT_FALSE(RoadlogRecordingService::BuildRecorderOptions(
      trigger_conf, "/tmp/test.record", &options, &error));
  EXPECT_FALSE(error.empty());
}

TEST(RoadlogRecordingServiceTest, RejectsConflictingIncludeAndExcludeChannels) {
  SmartRecordTrigger trigger_conf;
  auto* recorder_policy = trigger_conf.mutable_recorder_policy();
  recorder_policy->add_include_channels(
      "/apollo/sensor/lidar/fusion/PointCloud2");
  recorder_policy->add_exclude_channels(
      "/apollo/sensor/lidar/fusion/PointCloud2");

  RoadlogRecordingService::RecorderOptions options;
  std::string error;
  EXPECT_FALSE(RoadlogRecordingService::BuildRecorderOptions(
      trigger_conf, "/tmp/test.record", &options, &error));
  EXPECT_FALSE(error.empty());
}

TEST(RoadlogRecordingServiceTest, RejectsPoliciesThatDropTriggerChannels) {
  SmartRecordTrigger trigger_conf;
  auto* recorder_policy = trigger_conf.mutable_recorder_policy();
  recorder_policy->add_exclude_channels("/apollo/canbus/chassis");

  RoadlogRecordingService service(
      BuildRoadlogLayout("/tmp/roadlog_recording_service_test"));
  EXPECT_FALSE(service.Init(trigger_conf, {"/apollo/canbus/chassis"}));
}

}  // namespace
}  // namespace data
}  // namespace apollo
