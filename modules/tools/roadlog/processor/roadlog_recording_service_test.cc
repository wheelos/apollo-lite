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

#include "gtest/gtest.h"

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
  EXPECT_FALSE(options.all_channels);
  ASSERT_EQ(1U, options.include_channels.size());
  EXPECT_EQ("/apollo/planning", options.include_channels.front());
  ASSERT_EQ(1U, options.exclude_channels.size());
  EXPECT_EQ("/apollo/debug", options.exclude_channels.front());
  ASSERT_EQ(1U, options.channel_rate_filter_config.rules.size());
  EXPECT_EQ("/apollo/sensor/lidar/fusion/PointCloud2",
            options.channel_rate_filter_config.rules.front().channel_name);
  EXPECT_DOUBLE_EQ(
      1.0, options.channel_rate_filter_config.rules.front().max_rate_hz);
  EXPECT_EQ(5ULL * 1000000000ULL, options.header.segment_interval());
  EXPECT_EQ(64ULL * 1024ULL * 1024ULL, options.header.segment_raw_size());
  EXPECT_EQ(0ULL,
            options.message_size_filter_config.throttle_message_size_bytes);
  EXPECT_DOUBLE_EQ(0.0, options.message_size_filter_config.throttle_rate_hz);
  EXPECT_EQ(1ULL * 1024ULL * 1024ULL,
            options.message_size_filter_config.drop_message_size_bytes);
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

TEST(RoadlogRecordingServiceTest,
     RejectsMixedLegacyAndExplicitLargeMessagePolicy) {
  SmartRecordTrigger trigger_conf;
  auto* recorder_policy = trigger_conf.mutable_recorder_policy();
  recorder_policy->set_legacy_message_size_filter_policy(
      "throttle=256@2,drop=1m");
  recorder_policy->mutable_large_message_policy()->set_drop_message_size_bytes(
      1ULL * 1024ULL * 1024ULL);

  RoadlogRecordingService::RecorderOptions options;
  std::string error;
  EXPECT_FALSE(RoadlogRecordingService::BuildRecorderOptions(
      trigger_conf, "/tmp/test.record", &options, &error));
  EXPECT_FALSE(error.empty());
}

TEST(RoadlogRecordingServiceTest, RejectsLegacyThrottlePolicy) {
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
