// Copyright 2026 The WheelOS Authors. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include "modules/drivers/lidar/vanjeelidar/src/vanjeelidar_component.h"

#include <chrono>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

#include "gtest/gtest.h"

namespace apollo {
namespace drivers {
namespace lidar {

class VanjeelidarComponentTest : public ::testing::Test {
 protected:
  void Configure(VanjeelidarComponent* component) {
    component->conf_.set_imu_channel("/test/vanjee/imu");
    component->conf_.set_imu_frame_id("sdk_imu");
    component->conf_.mutable_driver_param()
        ->mutable_decoder_param()
        ->set_use_lidar_clock(true);
    component->node_ = cyber::CreateNode("vanjee_imu_output_test");
    component->imu_writer_ = component->node_->CreateWriter<vanjee::VanjeeImu>(
        component->conf_.imu_channel());
  }

  bool HasReader(const VanjeelidarComponent& component) {
    return component.imu_writer_ != nullptr &&
           component.imu_writer_->HasReader();
  }
};

TEST_F(VanjeelidarComponentTest, PublishesSdkImuAndRejectsInvalidSample) {
  std::mutex mutex;
  std::condition_variable received;
  std::shared_ptr<vanjee::VanjeeImu> result;
  int count = 0;
  auto reader_node = cyber::CreateNode("vanjee_imu_output_reader");
  ASSERT_NE(reader_node, nullptr);
  auto reader = reader_node->CreateReader<vanjee::VanjeeImu>(
      "/test/vanjee/imu",
      [&](const std::shared_ptr<vanjee::VanjeeImu>& message) {
        std::lock_guard<std::mutex> lock(mutex);
        result = message;
        ++count;
        received.notify_one();
      });
  ASSERT_NE(reader, nullptr);
  VanjeelidarComponent component;
  Configure(&component);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!HasReader(component) && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  ASSERT_TRUE(HasReader(component));
  auto packet = std::make_shared<::vanjee::lidar::ImuPacket>();
  packet->timestamp = 100.125;
  packet->seq = 27;
  packet->linear_acce = {1.0, -2.0, 9.81};
  packet->angular_voc = {0.1, -0.2, std::numeric_limits<double>::quiet_NaN()};
  component.VanjeeImuPacketPutCallback(packet);
  packet->angular_voc[2] = 0.3;
  component.VanjeeImuPacketPutCallback(packet);
  packet->linear_acce[0] = 999.0;
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(received.wait_for(lock, std::chrono::seconds(5),
                                  [&] { return count > 0; }));
    ASSERT_EQ(count, 1);
    ASSERT_NE(result, nullptr);
    EXPECT_EQ(result->header().frame_id(), "sdk_imu");
    EXPECT_EQ(result->header().sequence_num(), 27U);
    EXPECT_GT(result->header().timestamp_sec(), result->measurement_time());
    EXPECT_DOUBLE_EQ(result->measurement_time(), 100.125);
    EXPECT_TRUE(result->use_lidar_clock());
    EXPECT_DOUBLE_EQ(result->linear_acceleration().x(), 1.0);
    EXPECT_DOUBLE_EQ(result->linear_acceleration().z(), 9.81);
    EXPECT_DOUBLE_EQ(result->angular_velocity().y(), -0.2);
    EXPECT_DOUBLE_EQ(result->angular_velocity().z(), 0.3);
  }
  reader->Shutdown();
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  apollo::cyber::Init(argv[0]);
  const int result = RUN_ALL_TESTS();
  apollo::cyber::Clear();
  return result;
}
