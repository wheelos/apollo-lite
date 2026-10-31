// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_semantic_segmentation/component/cylinder3d_component.h"

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
namespace lidar_semantic_segmentation {
namespace {

template <typename Predicate>
bool Wait(Predicate predicate) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return predicate();
}

class CylinderComponentTest : public testing::Test {
 protected:
  void TearDown() override {
    if (component_) {
      component_->Shutdown();
    }
    if (reader_) {
      reader_->Shutdown();
    }
    if (!config_path_.empty()) {
      EXPECT_EQ(std::remove(config_path_.c_str()), 0);
    }
  }

  std::shared_ptr<Cylinder3DComponent> component_;
  std::shared_ptr<cyber::Reader<LidarSemanticSegmentationResult>> reader_;
  std::string config_path_;
  std::mutex mutex_;
  std::vector<LidarSemanticSegmentationResult> results_;
};

TEST_F(CylinderComponentTest, NativeInferencePublishesOnActualCyberChannels) {
  const char* fixture = std::getenv("CYLINDER3D_TEST_FIXTURE");
  if (fixture == nullptr) {
    GTEST_SKIP() << "Generate the upstream dense reference fixture first";
  }
  const std::string channel = "/test/cylinder3d_" + std::to_string(getpid());
  Cylinder3DComponentConfig config;
  config.set_model_path(std::string(fixture) + "/model.ts");
  config.set_source_topic(channel + "/cloud");
  config.set_output_channel(channel + "/labels");
  config.set_sensor_name("fixture_lidar");
  config.set_max_points(2);
  config.set_device_id(std::getenv("CYLINDER3D_TEST_CUDA") == nullptr ? -1 : 0);
  config.set_use_spconv(std::getenv("CYLINDER3D_TEST_SPCONV") != nullptr);
  std::string text;
  ASSERT_TRUE(google::protobuf::TextFormat::PrintToString(config, &text));
  config_path_ = std::string(fixture) + "/component-XXXXXX";
  const int descriptor = mkstemp(config_path_.data());
  ASSERT_GE(descriptor, 0);
  close(descriptor);
  {
    std::ofstream file(config_path_);
    file << text;
    ASSERT_TRUE(file.good());
  }
  component_ = std::make_shared<Cylinder3DComponent>();
  cyber::proto::ComponentConfig component_config;
  component_config.set_name("cylinder3d_fixture_" + std::to_string(getpid()));
  component_config.set_config_file_path(config_path_);
  auto* input_reader = component_config.add_readers();
  input_reader->set_channel(config.source_topic());
  input_reader->set_pending_queue_size(1);
  ASSERT_TRUE(component_->Initialize(component_config));
  auto node = cyber::CreateNode("cylinder3d_io_" + std::to_string(getpid()));
  ASSERT_NE(node, nullptr);
  reader_ = node->CreateReader<LidarSemanticSegmentationResult>(
      config.output_channel(),
      [this](const std::shared_ptr<LidarSemanticSegmentationResult>& output) {
        std::lock_guard<std::mutex> lock(mutex_);
        results_.push_back(*output);
      });
  auto writer = node->CreateWriter<drivers::PointCloud>(config.source_topic());
  ASSERT_NE(reader_, nullptr);
  ASSERT_NE(writer, nullptr);
  ASSERT_TRUE(
      Wait([&] { return writer->HasReader() && reader_->HasWriter(); }));
  auto cloud = std::make_shared<drivers::PointCloud>();
  cloud->set_frame_id("fixture_lidar");
  cloud->mutable_header()->set_sequence_num(1);
  cloud->mutable_header()->set_timestamp_sec(123.25);
  auto* point = cloud->add_point();
  point->set_x(2);
  point->set_y(1);
  point->set_z(-1);
  point->set_intensity(128);
  ASSERT_TRUE(writer->Write(cloud));
  ASSERT_TRUE(Wait([&] {
    std::lock_guard<std::mutex> lock(mutex_);
    return results_.size() == 1;
  }));
  auto invalid = std::make_shared<drivers::PointCloud>(*cloud);
  invalid->add_point();
  invalid->add_point();
  EXPECT_FALSE(component_->Proc(invalid));
  EXPECT_FALSE(component_->Proc(nullptr));
  cloud->mutable_header()->set_sequence_num(2);
  ASSERT_TRUE(writer->Write(cloud));
  ASSERT_TRUE(Wait([&] {
    std::lock_guard<std::mutex> lock(mutex_);
    return results_.size() >= 2;
  }));
  component_->Shutdown();
  reader_->Shutdown();
  std::lock_guard<std::mutex> lock(mutex_);
  ASSERT_EQ(results_.size(), 2U);
  EXPECT_EQ(results_[0].header().sequence_num(), 1U);
  EXPECT_EQ(results_[1].header().sequence_num(), 2U);
  EXPECT_EQ(results_[0].source_topic(), config.source_topic());
  EXPECT_EQ(results_[0].source_frame_id(), "fixture_lidar");
  EXPECT_EQ(results_[0].sensor_name(), config.sensor_name());
  ASSERT_EQ(results_[0].point_label_size(), 1);
  EXPECT_EQ(results_[0].point_label(0).point_index(), 0U);
  EXPECT_LT(results_[0].point_label(0).semantic_label(), 20U);
  EXPECT_GT(results_[0].point_label(0).confidence(), 0.0F);
}

}  // namespace
}  // namespace lidar_semantic_segmentation
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  if (!apollo::cyber::Init(argv[0])) {
    return 1;
  }
  const int result = RUN_ALL_TESTS();
  apollo::cyber::Clear();
  return result;
}
