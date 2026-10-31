// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_cyber_publisher.h"

#include <algorithm>
#include <condition_variable>
#include <map>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "openloc/localization/local/native_odometry.h"

namespace apollo::localization::openloc_diagnostic {
namespace {

TEST(NativeCyberPublisherTest, DeliversNativeSourceHeldStaleAndEpochOverCyber) {
  using Clock = NativePublication::Clock;
  std::shared_ptr<cyber::Node> node =
      cyber::CreateNode("openloc_native_publication_test");
  ASSERT_TRUE(node);
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<NativeOdometryDiagnostic> received;
  auto reader = node->CreateReader<NativeOdometryDiagnostic>(
      "/openloc/diagnostic/odometry",
      [&](const std::shared_ptr<NativeOdometryDiagnostic>& value) {
        if (value->session() != "native-cyber-test") return;
        {
          std::lock_guard<std::mutex> lock(mutex);
          received.push_back(*value);
        }
        condition.notify_all();
      });
  ASSERT_TRUE(reader);
  NativeCyberPublisher publisher(node, 0.2);
  ::openloc::localization::unified::LocalConfig config;
  config.session = "native-cyber-test";
  config.allow_imu_only_initialization = true;
  config.stationary_samples = 3;
  config.stationary_duration = 0.015;
  ::openloc::localization::NativeOdometry owner(config);
  std::map<uint64_t, NativePublication::State> source_states;
  ASSERT_TRUE(owner.ValidateConfig().ok());
  for (uint64_t index = 1; index <= 8; ++index) {
    const auto arrival = Clock::now();
    const double now = cyber::Time::Now().ToSecond();
    ::openloc::localization::unified::ImuSample imu;
    imu.stamp = {now, now, index, "unix"};
    imu.acceleration.z() = 9.80665;
    owner.AddImu(imu);
    auto state = owner.state();
    // The owner's initial invalid state has no committed source frontier.
    if (state.stamp.sequence) {
      source_states.emplace(state.stamp.sequence, state);
      publisher.Update(state, owner.EvaluateIntegrity(now), arrival);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(owner.state().valid);
  const auto source = owner.state();
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::any_of(received.begin(), received.end(), [&](const auto& p) {
        return !p.usable() && p.reason() == "IMU_STALE" &&
               p.source_sequence() == source.stamp.sequence;
      });
    }));
    bool new_seen = false;
    bool held_seen = false;
    for (const auto& message : received) {
      EXPECT_FALSE(message.trusted());
      EXPECT_TRUE(message.diagnostic_only());
      if (!message.usable()) {
        EXPECT_FALSE(message.has_odom_from_imu());
        EXPECT_FALSE(message.has_native_error_state());
        continue;
      }
      ASSERT_TRUE(message.has_native_error_state());
      const auto& actual = message.native_error_state();
      const auto& original = source_states.at(message.source_sequence());
      ASSERT_EQ(actual.covariance_size(), 225);
      for (int row = 0; row < 15; ++row)
        for (int column = 0; column < 15; ++column)
          EXPECT_DOUBLE_EQ(actual.covariance(15 * row + column),
                           original.covariance(row, column));
      EXPECT_EQ(actual.covariance_model_valid(), original.covariance_model_valid);
      EXPECT_EQ(actual.uncertainty_qualified(), original.uncertainty_qualified);
      new_seen |= message.new_measurement();
      held_seen |= !message.new_measurement();
      EXPECT_DOUBLE_EQ(message.odom_from_imu().header().timestamp_sec(),
                       message.measurement_time());
      EXPECT_DOUBLE_EQ(message.odom_from_imu().transform().translation().x(),
                       0.0);
    }
    EXPECT_TRUE(new_seen);
    EXPECT_TRUE(held_seen);
  }
  auto reset = source;
  reset.epoch.generation++;
  reset.stamp.time = reset.stamp.receive_time = cyber::Time::Now().ToSecond();
  reset.stamp.sequence++;
  publisher.Update(reset, {}, Clock::now());
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::any_of(received.begin(), received.end(), [](const auto& p) {
        return p.usable() && p.epoch_changed() && p.generation() == 2;
      });
    }));
  }
  publisher.Invalidate(
      {::openloc::localization::unified::Reason::INVALID_INPUT,
       "original raw source continuity failed"});
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::any_of(received.begin(), received.end(), [](const auto& p) {
        return p.reason() == "INVALID_INPUT";
      });
    }));
    const auto failed = std::find_if(
        received.begin(), received.end(),
        [](const auto& p) { return p.reason() == "INVALID_INPUT"; });
    EXPECT_FALSE(failed->usable());
    EXPECT_FALSE(failed->has_odom_from_imu());
    EXPECT_FALSE(failed->has_velocity_in_odom());
    EXPECT_FALSE(failed->has_native_error_state());
    EXPECT_EQ(failed->source_sequence(), reset.stamp.sequence);
    EXPECT_DOUBLE_EQ(failed->measurement_time(), reset.stamp.time);
    EXPECT_DOUBLE_EQ(failed->receive_time(), reset.stamp.receive_time);
    EXPECT_EQ(failed->detail(), "original raw source continuity failed");
  }
  publisher.Finish();
  reader->Shutdown();
}

}  // namespace
}  // namespace apollo::localization::openloc_diagnostic

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (!apollo::cyber::Init(argv[0])) return 1;
  const int result = RUN_ALL_TESTS();
  apollo::cyber::Clear();
  return result;
}
