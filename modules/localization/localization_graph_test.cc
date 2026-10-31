// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "modules/localization/localization_adapter.h"
#include "modules/openloc_localization/native_graph_test_input.h"

namespace apollo::localization::openloc {
namespace {

TEST(LocalizationGraph, ActualNativeSharedMapAndGnssProduceAbsoluteApolloPose) {
  const char* directory = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR");
  ASSERT_NE(directory, nullptr);
  const auto prefix =
      std::string(directory) + "/absolute-" + std::to_string(getpid());
  auto config = host::test::Config(prefix + ".csv");
  host::test::EnableSharedMap(&config, prefix + ".pcd");
  // Synthetic source-clock driver, not a live-sensor/deadline claim.
  config.set_recorded_input(false);
  const auto frames = host::test::ActualDelayedNativeFrames();
  ASSERT_EQ(frames.size(), 17);
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<host::NativeGlobalProcessed> decisions;
  host::NativeGlobalWorker worker(
      config, [&](const host::NativeGlobalProcessed& decision) {
        std::lock_guard<std::mutex> lock(mutex);
        decisions.push_back(decision);
        condition.notify_all();
      });
  const auto wait = [&](const std::string& kind, uint64_t sequence) {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, std::chrono::seconds(10), [&] {
      return std::any_of(decisions.begin(), decisions.end(),
                         [&](const auto& d) {
                           return d.kind() == kind && d.sequence() == sequence;
                         });
    });
  };
  host::NativePublication publisher(0.2);
  for (const auto& frame : frames) {
    const auto arrival = host::NativePublication::Clock::now();
    publisher.Update(frame.nominal, frame.integrity, arrival);
    auto source = publisher.Sample(frame.nominal.stamp.time, arrival);
    source.set_recorded_input(false);
    auto packet = host::EncodeNativeGraph(
        frame, source, config.target_graph().motion_error_model_id());
    ASSERT_TRUE(packet.model_available()) << packet.model_detail();
    worker.AddGraphOdometry(std::move(packet), arrival);
    ASSERT_TRUE(wait("joint_odom", source.publication_sequence()));
    if (frame.scan) {
      ASSERT_TRUE(wait("map_match", frame.scan->reference.sequence));
    }
    if (frame.nominal.stamp.sequence == 19) {
      auto gnss = host::test::Gnss(source);
      gnss.set_recorded_input(false);
      gnss.set_x(0);
      gnss.set_y(0);
      gnss.set_gnss_map_construction(host::NativeGlobalGnssInput::ABSENT);
      auto* qualification = gnss.mutable_graph_qualification();
      qualification->set_uncertainty_model_id("synthetic-independent-white");
      qualification->set_independence_evidence_id(
          "synthetic-fixed-map-and-imu");
      qualification->set_quality_valid(true);
      qualification->set_uncertainty_qualified(false);
      qualification->set_independent_of_local(true);
      qualification->set_independent_of_graph_sources(true);
      worker.AddGnss(gnss, host::NativePublication::Clock::now());
      ASSERT_TRUE(wait("gnss", gnss.sequence()));
    }
  }
  worker.Check();
  const auto& local = frames.back().nominal;
  const auto global = worker.Sample(local.stamp.time);
  ASSERT_TRUE(global.usable()) << global.reason();
  ASSERT_TRUE(global.full_pose_available()) << global.reason();
  ASSERT_TRUE(global.target_graph_available());
  EXPECT_EQ(global.map_tasks_completed(), 4);
  LocalizationConfig output_config;
  *output_config.mutable_global() = config;
  output_config.mutable_global()
      ->mutable_structured_gnss()
      ->set_origin_easting_m(500000);
  output_config.mutable_global()
      ->mutable_structured_gnss()
      ->set_origin_northing_m(4000000);
  output_config.set_utm_origin_height_m(30);
  *output_config.mutable_imu_from_base() = config.online_map().imu_from_lidar();
  *output_config.mutable_imu_from_vrp_flu() =
      config.online_map().imu_from_lidar();
  const auto result =
      MakePublication(output_config, global, &local, local.stamp.time, 1);
  ASSERT_TRUE(result.pose);
  ASSERT_TRUE(result.transform);
  EXPECT_NEAR(result.pose->pose().position().x(), 500000, 0.001);
  EXPECT_NEAR(result.pose->pose().position().y(), 4000000, 0.001);
  EXPECT_NEAR(result.pose->pose().position().z(), 30, 0.001);
  EXPECT_NEAR(result.pose->pose().heading(), 0, 0.001);
  EXPECT_EQ(result.status.fusion_status(), MeasureState::WARNNING);
  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_FALSE(decisions.empty());
    EXPECT_EQ(decisions.back().kind(), "gnss");
    EXPECT_EQ(decisions.back().status(), 0);
  }
  const auto stale =
      MakePublication(output_config, global, &local, local.stamp.time + 2, 2);
  EXPECT_FALSE(stale.pose);
  EXPECT_FALSE(stale.transform);
  EXPECT_EQ(stale.status.fusion_status(), MeasureState::ERROR);
  worker.Finish();
}

}  // namespace
}  // namespace apollo::localization::openloc
