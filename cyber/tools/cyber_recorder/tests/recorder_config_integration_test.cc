// Copyright 2026 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cyber/tools/cyber_recorder/record/core/recorder_config.h"

#include <cstdio>
#include <fstream>

#include <unistd.h>

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace record {
namespace {

std::string WriteTempYaml(const std::string& content,
                          const std::string& suffix) {
  const std::string path =
      "/tmp/cyber_recorder_" + suffix + "_" + std::to_string(::getpid()) +
      ".yaml";
  std::ofstream out(path);
  out << content;
  out.close();
  return path;
}

void RemoveFile(const std::string& path) { std::remove(path.c_str()); }

TEST(RecorderConfigIntegrationTest, LoadsYamlConfigWithSubscriptionAndPolicies) {
  const std::string yaml = R"(
version: 3
subscription:
  default_action: include
  selectors:
    - name: exclude_video
      match: { type: regex, pattern: "^/video/.*" }
      action: exclude
policies:
  rules:
    - name: lidar_front_rate
      match: { type: channel, value: "/lidar/front" }
      policy:
        sample_rate: 10
        compression: zstd
  default:
    policy:
      drop_message_size_bytes: 1024
      compression: none
)";
  const std::string path = WriteTempYaml(yaml, "config_ok");

  RecorderConfigBundle config;
  std::string error;
  ASSERT_TRUE(LoadRecorderConfigFromYamlFile(path, &config, &error)) << error;
  RemoveFile(path);

  EXPECT_EQ(3U, config.version);
  EXPECT_EQ(1U, config.subscription.selectors.size());
  EXPECT_EQ(1U, config.policies.rules.size());
  EXPECT_EQ(1024ULL, config.policies.default_policy.drop_message_size_bytes);
  EXPECT_DOUBLE_EQ(10.0, config.policies.rules[0].policy.max_rate_hz);
}

TEST(RecorderConfigIntegrationTest, RejectsPolicyWithBothDropAndRate) {
  const std::string yaml = R"(
version: 1
subscription:
  default_action: include
policies:
  default:
    policy:
      drop_message_size_bytes: 100
      sample_rate: 10
)";
  const std::string path = WriteTempYaml(yaml, "config_bad");

  RecorderConfigBundle config;
  std::string error;
  EXPECT_FALSE(LoadRecorderConfigFromYamlFile(path, &config, &error));
  EXPECT_NE(std::string::npos,
            error.find("either drop_message_size_bytes or sample_rate"));
  RemoveFile(path);
}

}  // namespace
}  // namespace record
}  // namespace cyber
}  // namespace apollo
