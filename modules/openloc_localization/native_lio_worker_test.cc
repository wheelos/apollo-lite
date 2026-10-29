// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_lio_worker.h"

#include <filesystem>
#include <algorithm>
#include <fstream>
#include <future>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

namespace apollo::localization::openloc_diagnostic {
namespace {
namespace u = ::openloc::localization::unified;

u::LocalConfig Config() {
  u::LocalConfig config;
  config.session = "native-lio-worker-test";
  config.allow_imu_only_initialization = true;
  config.enable_motion_initialization = true;
  config.stationary_samples = 3;
  config.stationary_duration = 0.015;
  config.enable_lidar = true;
  config.lidar_frame = "lidar";
  config.lidar_calibration_id = "test-lidar";
  config.lidar_point_noise_std = 0.01;
  config.lidar_plane_noise_std = 0.01;
  config.lidar_voxel_size = 1;
  config.lidar_max_voxels = 100;
  config.lidar_max_scans = 3;
  config.lidar_min_points_per_plane = 3;
  config.lidar_min_correspondences = 2;
  config.lidar_max_points = 1000;
  config.lidar_max_correspondence_distance = 0.3;
  config.lidar_observed_singular_value = 1;
  config.lidar_innovation_gate = 100;
  config.lidar_max_scan_age = 0.1;
  config.lidar_future_tolerance = 0.001;
  config.lidar_max_translation_correction_rate = 0.2;
  config.lidar_max_rotation_correction_rate = 0.1;
  config.lidar_max_velocity_correction_acceleration = 0.3;
  config.lidar_anchor_variance_scale = 1;
  config.lidar_correction_variance_scale = 2;
  config.lidar_max_translation_correction = 2;
  config.lidar_max_rotation_correction = 0.5;
  config.lidar_max_velocity_correction = 2;
  config.lidar_max_update_iterations = 2;
  config.lidar_process_noise_psd_tolerance = 1e-12;
  config.lidar_rotation_length = 1;
  return config;
}

u::ImuSample Imu(uint64_t sequence) {
  u::ImuSample sample;
  sample.stamp = {1 + 0.01 * (sequence - 1), 1 + 0.01 * (sequence - 1),
                  sequence, "unix"};
  sample.acceleration.z() = 9.80665;
  return sample;
}

u::LidarScan Scan(uint64_t sequence, double time, double offset) {
  u::LidarScan scan;
  scan.stamp = {time, time, sequence, "unix"};
  scan.frame_id = "lidar";
  scan.calibration_id = "test-lidar";
  for (double x : {0.1, 0.3, 0.5})
    for (double z : {0.1, 0.3, 0.5})
      for (double side : {-1.0, 1.0})
        scan.points.push_back({{x, side + offset, z}, time});
  return scan;
}

std::string Journal(const std::string& name) {
  return (::testing::TempDir() + name + ".csv");
}

std::string Read(const std::string& path) {
  std::ifstream stream(path);
  return std::string(std::istreambuf_iterator<char>(stream), {});
}

TEST(NativeLioWorkerTest, RawScanFrontierAndLioMatchTheSameSerialOwner) {
  const auto path = Journal("native-lio");
  std::filesystem::remove(path);
  std::vector<u::LocalState> states;
  std::vector<u::LocalState> expected;
  const auto config = Config();
  ::openloc::localization::NativeOdometry direct(config);
  NativeLioWorker worker(
      config, 0.2, path,
      [&](const auto& state, const auto&, auto) { states.push_back(state); });
  for (uint64_t index = 1; index <= 8; ++index) {
    const auto sample = Imu(index);
    if (index == 4 || index == 5)
      worker.AddLidar(Scan(index - 3, sample.stamp.time, index == 5 ? 0.1 : 0),
                      NativePublication::Clock::now());
    worker.AddImu(sample, NativePublication::Clock::now());
    direct.AddImu(sample);
    if (direct.state().stamp.sequence) expected.push_back(direct.state());
    if (index == 4 || index == 5) {
      auto scan = Scan(index - 3, sample.stamp.time, index == 5 ? 0.1 : 0);
      scan.epoch = direct.state().epoch;
      const auto result = direct.AddLidar(scan);
      EXPECT_EQ(result.reason,
                index == 4 ? u::Reason::GEOMETRY_UNAVAILABLE : u::Reason::NONE);
    }

  }
  worker.Finish();
  ASSERT_FALSE(states.empty());
  ASSERT_EQ(states.size(), expected.size());
  for (std::size_t index = 0; index < states.size(); ++index) {
    EXPECT_EQ(states[index].stamp.sequence, expected[index].stamp.sequence);
    EXPECT_TRUE(states[index].position.isApprox(expected[index].position, 0));
    EXPECT_TRUE(states[index].velocity.isApprox(expected[index].velocity, 0));
    EXPECT_EQ(states[index].covariance_model_valid,
              expected[index].covariance_model_valid);
  }
  EXPECT_FALSE(states.back().covariance_model_valid);
  const auto journal = Read(path);
  EXPECT_NE(journal.find("GEOMETRY_UNAVAILABLE"), std::string::npos);
  EXPECT_NE(journal.find(",NONE,"), std::string::npos);
}

TEST(NativeLioWorkerTest, JointExportFollowsLioWithoutReplacingPublishedSnapshots) {
  auto config = Config();
  config.enable_native_joint_error = true;
  config.native_imu_sample_period = 0.01;
  config.native_joint_max_dimension = 1024;
  config.history_duration = 0.1;
  const auto path = Journal("native-lio-joint-export");
  std::filesystem::remove(path);
  std::vector<u::LocalState> nominal;
  std::vector<NativeGraphFrame> frames;
  NativeLioWorker worker(
      config, 0.2, path,
      [&](const auto& state, const auto&, auto) { nominal.push_back(state); },
      std::chrono::milliseconds(0), std::chrono::milliseconds(0), {},
      [&](const auto& frame, auto) { frames.push_back(frame); return true; }, 5);
  for (uint64_t index = 1; index <= 12; ++index) {
    const auto imu = Imu(index);
    if (index == 4 || index == 8)
      worker.AddLidar(Scan(index / 4, imu.stamp.time, 0.03713),
                      NativePublication::Clock::now());
    worker.AddImu(imu, NativePublication::Clock::now());
  }
  worker.Finish();
  ASSERT_FALSE(frames.empty());
  int shared_scans = 0;
  NativePublication graph_publication(0.2);
  for (const auto& frame : frames) {
    ASSERT_TRUE(frame.assessment.ok()) << frame.assessment.message;
    const auto published = std::find_if(nominal.begin(), nominal.end(),
        [&](const auto& state) { return state.stamp.sequence == frame.nominal.stamp.sequence; });
    ASSERT_NE(published, nominal.end());
    EXPECT_TRUE(published->position.isApprox(frame.nominal.position, 0));
    EXPECT_TRUE(published->orientation.isApprox(frame.nominal.orientation, 0));
    EXPECT_TRUE(published->covariance.isApprox(frame.nominal.covariance, 0));
    const auto now = NativePublication::Clock::now();
    graph_publication.Update(frame.nominal, frame.integrity, now);
    const auto packet = EncodeNativeGraph(
        frame, graph_publication.Sample(10, now), "worker-native-joint");
    const auto decoded = DecodeNativeGraph(packet, "worker-native-joint", 1024);
    EXPECT_EQ(decoded.odometry.sequence, frame.nominal.stamp.sequence);
    if (decoded.scan) {
      ++shared_scans;
      EXPECT_EQ(decoded.scan->points_in_reference_base.size(), 18);
      EXPECT_EQ(decoded.scan->conditioning_state.stamp.sequence,
                frame.nominal.stamp.sequence);
    }
  }
  EXPECT_EQ(shared_scans, 2);
  EXPECT_LT(frames.size(), nominal.size());
}

TEST(NativeLioWorkerTest, PendingScanAtEndIsExplicitAndNeverAppliesFutureData) {
  const auto path = Journal("native-lio-pending");
  std::filesystem::remove(path);
  NativeLioWorker worker(Config(), 0.2, path,
                         [](const auto&, const auto&, auto) {});
  worker.AddLidar(Scan(1, 2.0, 0), NativePublication::Clock::now());
  worker.AddImu(Imu(1), NativePublication::Clock::now());
  worker.Finish();
  EXPECT_NE(Read(path).find("End of input before a causal IMU frontier"),
            std::string::npos);
}

TEST(NativeLioWorkerTest, WithdrawnGraphFrameDoesNotAdvanceMotionEndpoint) {
  auto config = Config();
  config.enable_native_joint_error = true;
  config.native_imu_sample_period = 0.01;
  config.native_joint_max_dimension = 1024;
  config.history_duration = 0.1;
  const auto path = Journal("native-lio-joint-withdrawal");
  std::filesystem::remove(path);
  std::vector<NativeGraphFrame> frames;
  NativeLioWorker worker(
      config, 0.2, path, [](const auto&, const auto&, auto) {},
      std::chrono::milliseconds(0), std::chrono::milliseconds(0), {},
      [&](const auto& frame, auto) {
        frames.push_back(frame);
        return frames.size() != 2;
      });
  for (uint64_t index = 1; index <= 8; ++index)
    worker.AddImu(Imu(index), NativePublication::Clock::now());
  worker.Finish();
  ASSERT_GE(frames.size(), 3);
  ASSERT_TRUE(frames[0].assessment.ok());
  ASSERT_TRUE(frames[2].assessment.ok()) << frames[2].assessment.message;
  ASSERT_TRUE(frames[2].motion);
  EXPECT_EQ(frames[2].motion->start.sequence, frames[0].joint.stamp.sequence);
  EXPECT_NE(frames[2].motion->start.sequence, frames[1].joint.stamp.sequence);
}

TEST(NativeLioWorkerTest, CorrectionEnvelopeWithdrawsPoseWithoutResettingOdom) {
  auto config = Config();
  config.lidar_max_translation_correction = 0.001;
  const auto path = Journal("native-lio-output-envelope");
  std::filesystem::remove(path);
  NativePublication publication(0.2);
  std::vector<u::LocalState> actual, expected;
  std::vector<NativeOdometryDiagnostic> messages;
  ::openloc::localization::NativeOdometry direct(config);
  NativeLioWorker worker(
      config, 0.2, path,
      [&](const auto& state, const auto& assessment, auto arrival) {
        actual.push_back(state);
        publication.Update(state, assessment, arrival);
        messages.push_back(publication.Sample(100));
      });
  for (uint64_t index = 1; index <= 150; ++index) {
    const auto imu = Imu(index);
    if (index == 4 || index == 50)
      worker.AddLidar(Scan(index == 4 ? 1 : 2, imu.stamp.time,
                          index == 50 ? 0.1 : 0),
                      NativePublication::Clock::now());
    worker.AddImu(imu, NativePublication::Clock::now());
    direct.AddImu(imu);
    if (direct.state().stamp.sequence) expected.push_back(direct.state());
    if (index == 4 || index == 50) {
      auto scan = Scan(index == 4 ? 1 : 2, imu.stamp.time,
                       index == 50 ? 0.1 : 0);
      scan.epoch = direct.state().epoch;
      const auto updated = direct.AddLidar(scan);
      EXPECT_EQ(updated.reason, index == 4 ? u::Reason::GEOMETRY_UNAVAILABLE
                                           : u::Reason::NONE);
    }
  }
  worker.Finish();
  ASSERT_EQ(actual.size(), expected.size());
  ASSERT_EQ(messages.size(), actual.size());
  bool exceeded = false, recovered = false;
  for (std::size_t i = 0; i < actual.size(); ++i) {
    EXPECT_TRUE(actual[i].position.isApprox(expected[i].position, 0));
    EXPECT_TRUE(actual[i].velocity.isApprox(expected[i].velocity, 0));
    EXPECT_TRUE(actual[i].orientation.coeffs().isApprox(
        expected[i].orientation.coeffs(), 0));
    EXPECT_TRUE(actual[i].covariance.isApprox(expected[i].covariance, 0));
    EXPECT_EQ(actual[i].epoch, expected[i].epoch);
    EXPECT_EQ(actual[i].stamp.sequence, expected[i].stamp.sequence);
    if (messages[i].reason() == "CORRECTION_BUDGET_EXCEEDED") {
      exceeded = true;
      EXPECT_FALSE(messages[i].usable());
      EXPECT_FALSE(messages[i].has_odom_from_imu());
      EXPECT_FALSE(messages[i].has_velocity_in_odom());
      EXPECT_NE(messages[i].detail().find("translation_limit_m=0.001"),
                std::string::npos);
      EXPECT_EQ(messages[i].source_sequence(), actual[i].stamp.sequence);
      EXPECT_DOUBLE_EQ(messages[i].measurement_time(), actual[i].stamp.time);
    } else if (exceeded && messages[i].usable()) {
      recovered = true;
      EXPECT_TRUE(messages[i].new_measurement());
      EXPECT_TRUE(messages[i].has_odom_from_imu());
    }
  }
  EXPECT_TRUE(exceeded);
  EXPECT_TRUE(recovered);
}

TEST(NativeLioWorkerTest, QueueOverflowAndWorkerFailureReachTheCaller) {
  const auto path = Journal("native-lio-overflow");
  std::filesystem::remove(path);
  std::promise<void> entered;
  std::promise<void> release;
  const auto released = release.get_future().share();
  bool blocked = false;
  NativeLioWorker worker(Config(), 0.2, path,
                         [&](const auto& state, const auto&, auto) {
                           if (!blocked && state.valid) {
                             blocked = true;
                             entered.set_value();
                             released.wait();
                           }
                         });
  for (uint64_t index = 1; index <= 3; ++index)
    worker.AddImu(Imu(index), NativePublication::Clock::now());
  const auto ready = entered.get_future().wait_for(std::chrono::seconds(5));
  if (ready != std::future_status::ready) {
    release.set_value();
    worker.Finish();
    FAIL() << "Owner did not initialize before bounded overflow test";
  }
  try {
    for (uint64_t index = 4; index < 260; ++index)
      worker.AddImu(Imu(index), NativePublication::Clock::now());
  } catch (...) {
    release.set_value();
    throw;
  }
  EXPECT_THROW(worker.AddImu(Imu(260), NativePublication::Clock::now()),
               std::runtime_error);
  release.set_value();
  EXPECT_THROW(worker.Finish(), std::runtime_error);
}

TEST(NativeLioWorkerTest,
     BootstrapCannotBypassRawCalibrationAndGeometryChecks) {
  const auto path = Journal("native-lio-invalid-source");
  std::filesystem::remove(path);
  NativeLioWorker worker(Config(), 0.2, path,
                         [](const auto&, const auto&, auto) {});
  auto scan = Scan(1, 1.1, 0);
  scan.frame_id = "wrong-frame";
  EXPECT_THROW(worker.AddLidar(scan, NativePublication::Clock::now()),
               std::invalid_argument);
  scan = Scan(1, 1.1, 0);
  scan.calibration_id = "wrong-calibration";
  EXPECT_THROW(worker.AddLidar(scan, NativePublication::Clock::now()),
               std::invalid_argument);
  scan = Scan(1, 1.1, 0);
  scan.base_from_lidar.linear()(0, 0) = 2;
  EXPECT_THROW(worker.AddLidar(scan, NativePublication::Clock::now()),
               std::invalid_argument);
  scan = Scan(1, 1.1, 0);
  scan.points[0].position.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(worker.AddLidar(scan, NativePublication::Clock::now()),
               std::invalid_argument);
  scan = Scan(1, 1.1, 0);
  scan.points[0].time = 1.2;
  EXPECT_THROW(worker.AddLidar(scan, NativePublication::Clock::now()),
               std::invalid_argument);
  scan = Scan(1, 1.1, 0);
  scan.points[1].time = 1.09;
  EXPECT_THROW(worker.AddLidar(scan, NativePublication::Clock::now()),
               std::invalid_argument);
  worker.Finish();
}

TEST(NativeLioWorkerTest, StartupGeometryCannotBlockImuAndShutdownCannotCommitIt) {
  auto config = Config();
  config.allow_imu_only_initialization = false;
  config.lidar_max_scan_age = 0.5;
  const auto path = Journal("native-startup-nonblocking");
  std::filesystem::remove(path);
  std::filesystem::remove(path + ".startup.csv");
  std::promise<void> entered;
  std::promise<void> release;
  const auto gate = release.get_future().share();
  std::mutex mutex;
  std::condition_variable condition;
  uint64_t processed = 0;
  bool valid = false;
  NativeLioWorker worker(config, 0.2, path,
      [&](const auto& state, const auto&, auto) {
        std::lock_guard<std::mutex> lock(mutex);
        processed = state.stamp.sequence;
        valid = valid || state.valid;
        condition.notify_all();
      }, std::chrono::milliseconds(0), std::chrono::milliseconds(500),
      [&](auto* task) {
        entered.set_value();
        gate.wait();
        return task->Compute();
      });
  struct Release {
    std::promise<void>* promise;
    bool done = false;
    ~Release() { if (!done) promise->set_value(); }
  } cleanup{&release};
  worker.AddImu(Imu(1), NativePublication::Clock::now());
  worker.AddLidar(Scan(1, Imu(2).stamp.time, 0), NativePublication::Clock::now());
  worker.AddImu(Imu(2), NativePublication::Clock::now());
  ASSERT_EQ(entered.get_future().wait_for(std::chrono::seconds(5)),
            std::future_status::ready);
  for (uint64_t i = 3; i <= 8; ++i)
    worker.AddImu(Imu(i), NativePublication::Clock::now());
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5),
                                  [&] { return processed == 8; }));
    EXPECT_FALSE(valid);
  }
  release.set_value();
  cleanup.done = true;
  worker.Finish();
  EXPECT_NE(Read(path + ".startup.csv").find("end_of_input_uncommitted"),
            std::string::npos);
}

TEST(NativeLioWorkerTest, StartupDeadlineAndAsyncExceptionsHaveExplicitOutcomes) {
  auto config = Config();
  config.allow_imu_only_initialization = false;
  const auto path = Journal("native-startup-deadline");
  std::filesystem::remove(path);
  std::filesystem::remove(path + ".startup.csv");
  std::promise<void> entered;
  std::promise<void> release;
  const auto gate = release.get_future().share();
  std::promise<void> advanced;
  NativeLioWorker worker(config, 0.2, path,
      [&](const auto& state, const auto&, auto) {
        if (state.stamp.sequence == 8) advanced.set_value();
      }, std::chrono::milliseconds(0), std::chrono::milliseconds(20),
      [&](auto* task) {
        entered.set_value();
        gate.wait();
        return task->Compute();
      });
  struct Release {
    std::promise<void>* promise;
    bool done = false;
    ~Release() { if (!done) promise->set_value(); }
  } cleanup{&release};
  worker.AddImu(Imu(1), NativePublication::Clock::now());
  worker.AddLidar(Scan(1, Imu(2).stamp.time, 0), NativePublication::Clock::now());
  worker.AddImu(Imu(2), NativePublication::Clock::now());
  ASSERT_EQ(entered.get_future().wait_for(std::chrono::seconds(5)),
            std::future_status::ready);
  for (uint64_t i = 3; i <= 8; ++i)
    worker.AddImu(Imu(i), NativePublication::Clock::now());
  ASSERT_EQ(advanced.get_future().wait_for(std::chrono::seconds(5)),
            std::future_status::ready);
  release.set_value();
  cleanup.done = true;
  worker.Finish();
  EXPECT_NE(Read(path + ".startup.csv").find("deadline_or_buffer"), std::string::npos);
  const auto failed_path = path + ".failure";
  std::filesystem::remove(failed_path);
  std::filesystem::remove(failed_path + ".startup.csv");
  NativeLioWorker failed(config, 0.2, failed_path,
      [](const auto&, const auto&, auto) {},
      std::chrono::milliseconds(0), std::chrono::milliseconds(500),
      [](auto*) -> u::Result { throw std::runtime_error("injected startup failure"); });
  failed.AddImu(Imu(1), NativePublication::Clock::now());
  failed.AddLidar(Scan(1, Imu(2).stamp.time, 0), NativePublication::Clock::now());
  failed.AddImu(Imu(2), NativePublication::Clock::now());
  std::string error;
  const auto deadline = NativePublication::Clock::now() + std::chrono::seconds(5);
  while (error.empty() && NativePublication::Clock::now() < deadline) {
    try {
      failed.Check();
    } catch (const std::runtime_error& value) {
      error = value.what();
    }
    if (error.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_NE(error.find("injected startup failure"), std::string::npos);
  EXPECT_THROW(failed.Finish(), std::runtime_error);
}

TEST(NativeLioWorkerTest,
     DelayedSnapshotsCatchUpWithoutSkippingImuOrRewritingOutput) {
  const auto path = Journal("native-lio-delayed");
  std::filesystem::remove(path);
  std::filesystem::remove(path + ".tasks.csv");
  std::vector<u::LocalState> states;
  NativeLioWorker worker(
      Config(), 0.2, path,
      [&](const auto& state, const auto&, auto) { states.push_back(state); },
      std::chrono::milliseconds(50));
  for (uint64_t index = 1; index <= 32; ++index) {
    if (index == 4 || index == 16)
      worker.AddLidar(
          Scan(index == 4 ? 1 : 2, Imu(index).stamp.time, index == 4 ? 0 : 0.1),
          NativePublication::Clock::now());
    worker.AddImu(Imu(index), NativePublication::Clock::now());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  worker.Finish();
  ASSERT_EQ(states.size(), 32);
  for (std::size_t index = 0; index < states.size(); ++index) {
    EXPECT_EQ(states[index].stamp.sequence, index + 1);
    EXPECT_DOUBLE_EQ(states[index].stamp.time, Imu(index + 1).stamp.time);
  }
  const auto tasks = Read(path + ".tasks.csv");
  EXPECT_NE(tasks.find("committed,GEOMETRY_UNAVAILABLE"), std::string::npos)
      << tasks;
  EXPECT_NE(tasks.find("committed,NONE"), std::string::npos) << tasks;
  EXPECT_EQ(tasks.find("deadline_missed"), std::string::npos) << tasks;
  EXPECT_FALSE(states.back().covariance_model_valid);
  EXPECT_NE(Read(path).find("snapshot queued"), std::string::npos);
}

TEST(NativeLioWorkerTest, EndOfInputNeverCommitsSnapshotWithoutAnImuFrontier) {
  const auto path = Journal("native-lio-delayed-end");
  std::filesystem::remove(path);
  std::filesystem::remove(path + ".tasks.csv");
  NativeLioWorker worker(
      Config(), 0.2, path, [](const auto&, const auto&, auto) {},
      std::chrono::milliseconds(50));
  for (uint64_t index = 1; index <= 4; ++index) {
    if (index == 4)
      worker.AddLidar(Scan(1, Imu(index).stamp.time, 0),
                      NativePublication::Clock::now());
    worker.AddImu(Imu(index), NativePublication::Clock::now());
  }
  worker.Finish();
  const auto tasks = Read(path + ".tasks.csv");
  EXPECT_NE(tasks.find("end_of_input_uncommitted"), std::string::npos) << tasks;
  EXPECT_EQ(tasks.find(",committed,"), std::string::npos) << tasks;
}

}  // namespace
}  // namespace apollo::localization::openloc_diagnostic
