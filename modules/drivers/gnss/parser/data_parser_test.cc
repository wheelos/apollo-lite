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

#include "modules/drivers/gnss/parser/data_parser.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "modules/drivers/gnss/proto/gnss_status.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/global_data.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/time_conversion.h"
#include "modules/drivers/gnss/parser/novatel/novatel_parser.h"

namespace apollo {
namespace drivers {
namespace gnss {
namespace {

TEST(DataParserTest, NovatelReplayPreservesMeasurementContracts) {
  const std::string root = std::string(std::getenv("TEST_SRCDIR")) + "/_main/";
  config::Config config;
  ASSERT_TRUE(cyber::common::GetProtoFromFile(
      root + "modules/drivers/gnss/conf/gnss_conf.pb.txt", &config));
  config.set_imu_type(config::ISA100C);
  config.mutable_novatel_config()->set_corrimu_rate_hz(10);
  std::ifstream input(root + "modules/drivers/gnss/test_data/novatel.bin",
                      std::ios::binary);
  ASSERT_TRUE(input.is_open());
  const std::string bytes((std::istreambuf_iterator<char>(input)), {});

  std::vector<Imu> raw;
  std::vector<localization::CorrectedImu> corrected;
  std::vector<localization::Gps> odometry;
  std::vector<GnssBestPose> positions;
  std::vector<GnssStatus> statuses;
  std::shared_ptr<cyber::Node> node(
      cyber::CreateNode("novatel_publication_regression"));
  auto raw_reader = node->CreateReader<Imu>(
      FLAGS_raw_imu_topic,
      [&](const std::shared_ptr<Imu>& message) { raw.push_back(*message); });
  auto corrected_reader = node->CreateReader<localization::CorrectedImu>(
      FLAGS_imu_topic,
      [&](const std::shared_ptr<localization::CorrectedImu>& message) {
        corrected.push_back(*message);
      });
  auto gps_reader = node->CreateReader<localization::Gps>(
      FLAGS_gps_topic, [&](const std::shared_ptr<localization::Gps>& message) {
        odometry.push_back(*message);
      });
  auto position_reader = node->CreateReader<GnssBestPose>(
      FLAGS_gnss_best_pose_topic,
      [&](const std::shared_ptr<GnssBestPose>& message) {
        positions.push_back(*message);
      });
  auto status_reader = node->CreateReader<GnssStatus>(
      FLAGS_gnss_status_topic, [&](const std::shared_ptr<GnssStatus>& message) {
        statuses.push_back(*message);
      });
  ASSERT_NE(nullptr, raw_reader);
  ASSERT_NE(nullptr, corrected_reader);
  ASSERT_NE(nullptr, gps_reader);
  ASSERT_NE(nullptr, position_reader);
  ASSERT_NE(nullptr, status_reader);
  DataParser publisher(config, node);
  ASSERT_TRUE(publisher.Init());
  for (size_t offset = 0; offset < bytes.size(); offset += 127) {
    publisher.ParseRawData(bytes.substr(offset, 127));
  }
  ASSERT_EQ(15953u, raw.size());
  ASSERT_EQ(796u, corrected.size());
  ASSERT_EQ(796u, odometry.size());
  ASSERT_EQ(80u, positions.size());
  ASSERT_EQ(81u,
            statuses.size());  // Initial status + 80 positions, not velocity.
  EXPECT_FALSE(statuses.front().solution_completed());

  // Compare publication against the parser contract, catching extra rotations,
  // missing metadata, host timestamps, stale fields and duplicate odometry.
  NovatelParser reference(config);
  reference.AppendData(bytes);
  size_t raw_index = 0;
  size_t corrected_index = 0;
  size_t pva_index = 0;
  size_t position_index = 0;
  size_t status_index = 1;
  for (const auto& parsed : reference.ParseAllMessages()) {
    const auto message = std::get<Parser::ProtoMessagePtr>(parsed.second);
    if (parsed.first == Parser::MessageType::IMU) {
      const auto expected = std::dynamic_pointer_cast<Imu>(message);
      ASSERT_NE(nullptr, expected);
      ASSERT_LT(raw_index, raw.size());
      const auto& actual = raw[raw_index++];
      EXPECT_EQ(expected->linear_acceleration().SerializeAsString(),
                actual.linear_acceleration().SerializeAsString());
      EXPECT_EQ(expected->angular_velocity().SerializeAsString(),
                actual.angular_velocity().SerializeAsString());
      EXPECT_DOUBLE_EQ(expected->measurement_time(), actual.measurement_time());
      EXPECT_FLOAT_EQ(expected->measurement_span(), actual.measurement_span());
      EXPECT_DOUBLE_EQ(
          common::util::GpsToUnixSeconds(expected->measurement_time()),
          actual.header().timestamp_sec());
    } else if (parsed.first == Parser::MessageType::CORRECTED_IMU) {
      const auto expected = std::dynamic_pointer_cast<Ins>(message);
      ASSERT_NE(nullptr, expected);
      ASSERT_LT(corrected_index, corrected.size());
      const auto& actual = corrected[corrected_index++];
      EXPECT_FALSE(actual.imu().has_euler_angles());
      EXPECT_DOUBLE_EQ(-expected->linear_acceleration().y(),
                       actual.imu().linear_acceleration().x());
      EXPECT_DOUBLE_EQ(expected->linear_acceleration().x(),
                       actual.imu().linear_acceleration().y());
      EXPECT_DOUBLE_EQ(expected->linear_acceleration().z(),
                       actual.imu().linear_acceleration().z());
      EXPECT_DOUBLE_EQ(-expected->angular_velocity().y(),
                       actual.imu().angular_velocity().x());
      EXPECT_DOUBLE_EQ(expected->angular_velocity().x(),
                       actual.imu().angular_velocity().y());
      EXPECT_DOUBLE_EQ(expected->angular_velocity().z(),
                       actual.imu().angular_velocity().z());
      EXPECT_DOUBLE_EQ(
          common::util::GpsToUnixSeconds(expected->measurement_time()),
          actual.header().timestamp_sec());
    } else if (parsed.first == Parser::MessageType::INS) {
      const auto expected = std::dynamic_pointer_cast<Ins>(message);
      ASSERT_NE(nullptr, expected);
      ASSERT_LT(pva_index, odometry.size());
      const auto& actual = odometry[pva_index++];
      EXPECT_EQ(expected->linear_velocity().SerializeAsString(),
                actual.localization().linear_velocity().SerializeAsString());
      EXPECT_DOUBLE_EQ(
          common::util::GpsToUnixSeconds(expected->measurement_time()),
          actual.header().timestamp_sec());
    } else if (parsed.first == Parser::MessageType::BEST_GNSS_POS) {
      const auto expected = std::dynamic_pointer_cast<GnssBestPose>(message);
      ASSERT_NE(nullptr, expected);
      ASSERT_LT(position_index, positions.size());
      EXPECT_DOUBLE_EQ(
          common::util::GpsToUnixSeconds(expected->measurement_time()),
          positions[position_index++].header().timestamp_sec());
    } else if (parsed.first == Parser::MessageType::GNSS) {
      const auto expected = std::dynamic_pointer_cast<Gnss>(message);
      ASSERT_NE(nullptr, expected);
      if (expected->has_solution_status()) {
        ASSERT_LT(status_index, statuses.size());
        const auto& actual = statuses[status_index++];
        EXPECT_EQ(expected->solution_status(), actual.solution_status());
        EXPECT_EQ(expected->position_type(), actual.position_type());
        EXPECT_EQ(expected->num_sats(), actual.num_sats());
        EXPECT_EQ(expected->solution_status() == 0 && expected->has_position(),
                  actual.solution_completed());
      }
    }
  }
  EXPECT_EQ(raw.size(), raw_index);
  EXPECT_EQ(corrected.size(), corrected_index);
  EXPECT_EQ(odometry.size(), pva_index);
  EXPECT_EQ(positions.size(), position_index);
  EXPECT_EQ(statuses.size(), status_index);
}

}  // namespace
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  const char* runfiles = std::getenv("TEST_SRCDIR");
  if (runfiles == nullptr) {
    std::fprintf(
        stderr, "Run this regression with bazel test (TEST_SRCDIR required)\n");
    return EXIT_FAILURE;
  }
  const std::string root(runfiles);
  setenv("CYBER_PATH", (root + "/wheelos_core~/cyber").c_str(), 1);
  setenv("PROJ_DATA", (root + "/proj~/data").c_str(), 1);
  if (!apollo::cyber::Init(argv[0])) {
    return EXIT_FAILURE;
  }
  // Deterministic publication test, not a DDS transport or hardware test.
  apollo::cyber::common::GlobalData::Instance()->EnableSimulationMode();
  const int result = RUN_ALL_TESTS();
  apollo::cyber::Clear();
  return result;
}
