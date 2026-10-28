// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_fusion/component/lidar_fusion_component.h"

#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "google/protobuf/text_format.h"

#include "cyber/cyber.h"

namespace apollo {
namespace lidar_fusion {
namespace {

template <typename Predicate>
bool Wait(Predicate predicate) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
}

class ComponentTest : public testing::TestWithParam<bool> {
 protected:
  void TearDown() override {
    if (component != nullptr) {
      component->Shutdown();
    }
    if (reader != nullptr) {
      reader->Shutdown();
    }
    if (!config_path.empty()) {
      EXPECT_EQ(std::remove(config_path.c_str()), 0);
    }
  }

  std::shared_ptr<drivers::PointCloud> Cloud(int id) {
    auto cloud = std::make_shared<drivers::PointCloud>();
    cloud->set_frame_id("fixture_lidar");
    cloud->set_measurement_time(100 + id);
    cloud->mutable_header()->set_frame_id("fixture_lidar");
    cloud->mutable_header()->set_sequence_num(id);
    cloud->mutable_header()->set_lidar_timestamp((100ULL + id) * 1000000000ULL);
    auto* p = cloud->add_point();
    p->set_x(2);
    p->set_y(1);
    p->set_z(0.8);
    return cloud;
  }

  std::shared_ptr<LidarFusionComponent> component;
  std::shared_ptr<cyber::Reader<FusionOutput>> reader;
  std::string config_path;
  std::mutex mutex;
  std::vector<FusionOutput> results;
};

TEST_P(ComponentTest, PublishesAlignedAndTimedOutFramesOnActualCyberChannels) {
  const char* configured_root = std::getenv("APOLLO_ROOT_DIR");
  const std::string root =
      configured_root != nullptr ? configured_root : "/apollo";
  FusionConfig config;
  ASSERT_TRUE(cyber::common::GetProtoFromFile(
      root + "/modules/lidar_fusion/conf/lidar_fusion.pb.txt", &config));
  SingleFrameFusion core;
  std::string error;
  EXPECT_FALSE(
      core.Init(config, &error));  // Deployment template is fail-closed.
  const std::string channel = "/test/lidar_fusion_" + std::to_string(getpid());
  config.set_pointcloud_channel(channel + "/cloud");
  config.set_detection_channel(channel + "/det");
  config.set_segmentation_channel(channel + "/seg");
  config.set_output_channel(channel + "/output");
  config.set_sensor_frame("fixture_lidar");
  config.mutable_sensor_to_base()->set_version("fixture-extrinsic");
  config.set_wait_ms(100);
  if (GetParam()) {
    auto* temporal = config.mutable_temporal();
    temporal->set_enabled(true);
    temporal->set_pose_channel(channel + "/pose");
    temporal->set_odom_frame("fixture_odom");
    temporal->set_max_dt(2);
    temporal->set_confirmation_hits(1);
    temporal->set_feedback(true);
    config.set_core_min_points(1);
  }
  ASSERT_TRUE(core.Init(config, &error)) << error;
  std::string text;
  ASSERT_TRUE(google::protobuf::TextFormat::PrintToString(config, &text));
  config_path = root + "/.lidar-fusion-component-XXXXXX";
  const int descriptor = mkstemp(config_path.data());
  ASSERT_GE(descriptor, 0);
  close(descriptor);
  {
    std::ofstream file(config_path);
    file << text;
    ASSERT_TRUE(file.good());
  }
  component = std::make_shared<LidarFusionComponent>();
  cyber::proto::TimerComponentConfig timer;
  timer.set_name("lidar_fusion_fixture_" + std::to_string(getpid()));
  timer.set_config_file_path(config_path);
  timer.set_interval(5);
  ASSERT_TRUE(component->Initialize(timer));
  auto node = cyber::CreateNode("lidar_fusion_io_" + std::to_string(getpid()));
  ASSERT_NE(node, nullptr);
  reader = node->CreateReader<FusionOutput>(
      config.output_channel(),
      [this](const std::shared_ptr<FusionOutput>& output) {
        std::lock_guard<std::mutex> lock(mutex);
        results.push_back(*output);
      });
  auto clouds =
      node->CreateWriter<drivers::PointCloud>(config.pointcloud_channel());
  auto detections =
      node->CreateWriter<DetectionFrame>(config.detection_channel());
  auto semantics =
      node->CreateWriter<SemanticFrame>(config.segmentation_channel());
  auto poses =
      GetParam()
          ? node->CreateWriter<LocalPose>(config.temporal().pose_channel())
          : nullptr;
  ASSERT_NE(reader, nullptr);
  ASSERT_NE(clouds, nullptr);
  ASSERT_NE(detections, nullptr);
  ASSERT_NE(semantics, nullptr);
  ASSERT_TRUE(Wait([&] {
    return clouds->HasReader() && detections->HasReader() &&
           semantics->HasReader() && reader->HasWriter() &&
           (!GetParam() || poses->HasReader());
  }));
  auto pose = [&](double stamp) {
    if (poses != nullptr) {
      auto message = std::make_shared<LocalPose>();
      message->set_stamp(stamp);
      message->set_base_frame(config.base_frame());
      message->set_odom_frame(config.temporal().odom_frame());
      message->set_epoch(1);
      message->set_valid(true);
      message->mutable_translation()->set_x(0);
      EXPECT_TRUE(poses->Write(message));
      // Isolate pose delivery from the independent cloud/result callback order.
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  };
  pose(101);
  const auto cloud = Cloud(1);
  auto detection = std::make_shared<DetectionFrame>();
  *detection->mutable_header() = cloud->header();
  detection->set_sensor_frame_id("fixture_lidar");
  detection->set_backend(config.detection_backend());
  detection->set_model_revision("synthetic-publication-fixture");
  auto* box = detection->add_detection();
  box->set_center_x(2);
  box->set_center_y(1);
  box->set_center_z(0.8);
  box->set_size_x(1);
  box->set_size_y(1);
  box->set_size_z(1);
  box->set_yaw(0);
  box->set_class_id(0);
  box->set_score(0.9);
  auto semantic = std::make_shared<SemanticFrame>();
  *semantic->mutable_header() = cloud->header();
  semantic->set_source_frame_id("fixture_lidar");
  semantic->set_source_topic(config.pointcloud_channel());
  semantic->set_point_count(1);
  auto* label = semantic->add_point_label();
  label->set_point_index(0);
  label->set_semantic_label(1);
  label->set_confidence(0.95);
  // Real callbacks see independently scheduled sources, including result-first.
  ASSERT_TRUE(semantics->Write(semantic));
  ASSERT_TRUE(clouds->Write(cloud));
  ASSERT_TRUE(detections->Write(detection));
  ASSERT_TRUE(Wait([&] {
    std::lock_guard<std::mutex> lock(mutex);
    return !results.empty();
  }));
  pose(102);
  ASSERT_TRUE(clouds->Write(Cloud(2)));
  ASSERT_TRUE(semantics->Write(semantic));  // Late result from the old sweep.
  ASSERT_TRUE(Wait([&] {
    std::lock_guard<std::mutex> lock(mutex);
    return results.size() >= 2;
  }));
  component->Shutdown();
  reader->Shutdown();
  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_EQ(results.size(), 2);
  EXPECT_EQ(results[0].health().mode(), NORMAL);
  ASSERT_EQ(results[0].object_size(), 1);
  EXPECT_EQ(results[0].header().frame_id(), "base_link");
  EXPECT_EQ(results[0].point_evidence(0).raw_semantic_label(), 1);
  EXPECT_EQ(results[1].health().mode(), GEO_ONLY);
  EXPECT_EQ(results[1].header().sequence_num(), 2);
  if (GetParam()) {
    EXPECT_TRUE(results[0].health().temporal_enabled());
    EXPECT_TRUE(results[1].health().temporal_enabled());
    ASSERT_EQ(results[1].object_size(), 1);
    EXPECT_EQ(results[1].object(0).source(), TRACK_PRIOR);
    EXPECT_EQ(results[1].object(0).track_id(), results[0].object(0).track_id());
    EXPECT_TRUE(results[1].object(0).track_confirmed());
  } else {
    EXPECT_EQ(results[1].unknown_size(), 1);
    EXPECT_FALSE(results[1].health().temporal_enabled());
  }
  EXPECT_FALSE(results[1].point_evidence(0).has_raw_semantic_label());
}

INSTANTIATE_TEST_SUITE_P(TemporalOptIn, ComponentTest,
                         testing::Values(false, true));

}  // namespace
}  // namespace lidar_fusion
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  apollo::cyber::Init(argv[0]);
  const int result = RUN_ALL_TESTS();
  apollo::cyber::Clear();
  return result;
}
