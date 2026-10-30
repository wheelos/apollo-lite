// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <unistd.h>

#include <filesystem>
#include <algorithm>
#include <atomic>

#include "gtest/gtest.h"
#include "native_map_test_input.h"

namespace apollo::localization::openloc_diagnostic {
namespace {
using Clock = NativeGlobalWorker::Clock;

class NativeMapTest : public ::testing::Test {
 protected:
  void SetUp() override {
    prefix = ::testing::TempDir() + "online-map-" + std::to_string(getpid()) +
        ::testing::UnitTest::GetInstance()->current_test_info()->name();
    config = test::Config(prefix + ".csv");
    test::EnableOnlineMap(&config, prefix + ".pcd");
  }
  void TearDown() override {
    std::filesystem::remove(prefix + ".pcd");
    std::filesystem::remove(prefix + ".csv");
    std::filesystem::remove(prefix + ".region.csv");
  }
  NativeMapJob Job(int index) {
    const auto packet = test::MapCloud(index);
    NativeMapJob job;
    job.cloud = packet.cloud;
    job.receive_ns = packet.receive_ns;
    job.session = "synthetic-global-test";
    job.generation = 1;
    job.history.push_back({packet.receive_ns, Eigen::Isometry3d::Identity()});
    return job;
  }
  void Regional() {
    auto* online = config.mutable_online_map();
    online->clear_initial_map_from_odom();
    online->clear_hypothesis_minimum_ns();
    online->clear_hypothesis_maximum_ns();
    online->clear_hypothesis_odometry_session();
    online->clear_hypothesis_odometry_generation();
    const auto path = prefix + ".region.csv";
    std::ofstream file(path);
    file << "name,value\nfield_min_x,-7\nfield_min_y,-7\nfield_min_z,-2\n"
            "field_max_x,7\nfield_max_y,7\nfield_max_z,5\n"
            "search_min_x,-1\nsearch_min_y,-1\nsearch_min_z,-0.5\n"
            "search_max_x,1\nsearch_max_y,1\nsearch_max_z,0.5\n"
            "observed_min_x,-7\nobserved_min_y,-7\nobserved_min_z,-2\n"
            "observed_max_x,7\nobserved_max_y,7\nobserved_max_z,5\n"
            "yaw_min_rad,-0.1\nyaw_max_rad,0.1\ntilt_qx,0\ntilt_qy,0\n"
            "tilt_qz,0\ntilt_qw,1\nfield_resolution_m,0.5\n"
            "distance_sigma_m,1\ninlier_distance_m,1.5\n";
    file.close();
    if (!file) throw std::runtime_error("cannot write synthetic regional policy");
    online->set_regional_policy_csv(path);
    online->set_regional_policy_sha256(NativeMapMatcher::Sha256(path));
  }
  std::string prefix;
  NativeGlobalHostConfig config;
};

TEST_F(NativeMapTest, ActualMatcherDeskewsFromCopiedOdometryWithoutInventedFields) {
  NativeMapMatcher matcher(config);
  auto job = Job(1);
  job.history.push_front({job.receive_ns - 10000000, Eigen::Isometry3d::Identity()});
  job.history.back().odom_from_imu.translation().x() = 0.1;
  // Returns were acquired before the reference pose translated forward.
  for (auto& point : *job.cloud.mutable_point())
    point.set_timestamp(job.receive_ns - 10000000);
  const auto original = job.cloud.SerializeAsString();
  const auto result = matcher.Match(job);
  ASSERT_TRUE(result.observation) << result.reason;
  const auto& input = *result.observation;
  EXPECT_TRUE(input.IsInitialized());
  EXPECT_EQ(input.kind(), NativeGlobalMapInput::FULL_POSE);
  EXPECT_NEAR(input.map_from_imu().x(), 0.45, 0.05);
  EXPECT_NEAR(input.map_from_imu().y(), -0.22, 0.05);
  EXPECT_NEAR(input.map_from_imu().z(), 0.18, 0.05);
  EXPECT_EQ(input.odometry_deskew(), NativeGlobalGnssInput::PRESENT);
  EXPECT_EQ(input.shared_local_scans(), NativeGlobalGnssInput::PRESENT);
  EXPECT_EQ(input.gnss_map_construction(), NativeGlobalGnssInput::UNKNOWN);
  EXPECT_EQ(input.measurement_ns(), job.receive_ns);
  EXPECT_EQ(input.receive_ns(), job.receive_ns);
  EXPECT_TRUE(input.diagnostic_only());
  EXPECT_FALSE(input.recovery());
  EXPECT_EQ(job.cloud.SerializeAsString(), original);
}

TEST_F(NativeMapTest, MissingHistoryExpiredHypothesisAndInvalidPointsCannotMatch) {
  NativeMapMatcher matcher(config);
  auto job = Job(0);
  job.history.clear();
  auto result = matcher.Match(job);
  EXPECT_FALSE(result.observation);
  EXPECT_EQ(result.reason, "map_deskew_reference_unavailable");
  job = Job(101);
  result = matcher.Match(job);
  EXPECT_FALSE(result.observation);
  EXPECT_EQ(result.reason, "explicit_map_hypothesis_expired");
  job = Job(0);
  job.generation = 2;
  result = matcher.Match(job);
  EXPECT_FALSE(result.observation);
  EXPECT_EQ(result.reason, "explicit_map_hypothesis_epoch_mismatch");
  job = Job(0);
  job.cloud.mutable_point(0)->clear_timestamp();
  EXPECT_THROW(matcher.Match(job), std::invalid_argument);
  job = Job(0);
  job.cloud.set_frame_id("other");
  EXPECT_THROW(matcher.Match(job), std::invalid_argument);
}

TEST_F(NativeMapTest, SlowTaskDoesNotBlockOdometryOrResetAndCannotCommitAfterReset) {
  test::EnableReset(&config);
  NativeMapMatcher actual(config);
  std::promise<void> release;
  const auto gate = release.get_future().share();
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<NativeGlobalProcessed> processed;
  NativeGlobalWorker worker(config, [&](const auto& value) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      processed.push_back(value);
    }
    condition.notify_all();
  }, [&](const auto& job) {
    gate.wait();
    return actual.Match(job);
  });
  // Release before destruction even if an assertion exits the test early.
  struct Release {
    std::promise<void>* promise;
    bool done = false;
    ~Release() { if (!done) promise->set_value(); }
  } cleanup{&release};
  const auto wait = [&](const char* kind, uint64_t sequence) {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::any_of(processed.begin(), processed.end(), [&](const auto& p) {
        return p.kind() == kind && p.sequence() == sequence;
      });
    });
  };
  worker.AddOdometry(test::MapOdometry(0), Clock::now());
  ASSERT_TRUE(wait("odom", 1));
  worker.AddRawMap(test::MapCloud(0), Clock::now());
  ASSERT_TRUE(wait("raw_map", 1));
  worker.AddRawMap(test::MapCloud(1), Clock::now());
  ASSERT_TRUE(wait("raw_map", 2));
  worker.AddOdometry(test::MapOdometry(1), Clock::now());
  ASSERT_TRUE(wait("odom", 2));
  EXPECT_EQ(worker.Sample(200).odometry_sequence(), 2);
  worker.AddReset(test::Reset(), Clock::now());
  ASSERT_TRUE(wait("reset", 1));
  EXPECT_TRUE(worker.Sample(200).reset_pending());
  release.set_value();
  cleanup.done = true;
  ASSERT_TRUE(wait("map_match", 1));
  worker.Finish();
  const auto output = worker.Sample(200);
  EXPECT_FALSE(output.has_map_from_imu());
  EXPECT_EQ(output.map_tasks_completed(), 1);
  std::ifstream file(prefix + ".csv");
  const std::string journal((std::istreambuf_iterator<char>(file)), {});
  EXPECT_NE(journal.find("map_matcher_busy"), std::string::npos);
  EXPECT_NE(journal.find("map_task_alignment_retired"), std::string::npos);
}

TEST_F(NativeMapTest, RetiredLocalTaskIsJournaledWithoutCorrection) {
  NativeMapMatcher actual(config);
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<NativeGlobalProcessed> processed;
  std::promise<void> release;
  const auto gate = release.get_future().share();
  NativeGlobalWorker worker(config, [&](const auto& value) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      processed.push_back(value);
    }
    condition.notify_all();
  }, [&](const auto& job) {
    gate.wait();
    return actual.Match(job);
  });
  struct Release {
    std::promise<void>* promise;
    bool done = false;
    ~Release() { if (!done) promise->set_value(); }
  } cleanup{&release};
  const auto wait = [&](const char* kind, uint64_t sequence) {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::any_of(processed.begin(), processed.end(), [&](const auto& p) {
        return p.kind() == kind && p.sequence() == sequence;
      });
    });
  };
  worker.AddOdometry(test::MapOdometry(0), Clock::now());
  ASSERT_TRUE(wait("odom", 1));
  worker.AddRawMap(test::MapCloud(0), Clock::now());
  ASSERT_TRUE(wait("raw_map", 1));
  worker.AddOdometry(test::MapOdometry(1, 2), Clock::now());
  ASSERT_TRUE(wait("odom", 2));
  release.set_value();
  cleanup.done = true;
  ASSERT_TRUE(wait("map_match", 1));
  EXPECT_FALSE(worker.Sample(200).has_map_from_imu());
  worker.Finish();
  std::ifstream file(prefix + ".csv");
  const std::string journal((std::istreambuf_iterator<char>(file)), {});
  EXPECT_NE(journal.find("map_task_local_epoch_retired"), std::string::npos);
}

TEST_F(NativeMapTest, ExpiredTaskIsDiscardedAndTaskExceptionsSurfaceAtFinish) {
  config.mutable_online_map()->set_task_deadline_ms(1);
  NativeMapMatcher actual(config);
  {
    NativeGlobalWorker worker(config, {}, [&](const auto& job) {
      // Deliberately miss the deadline while exercising the real matcher.
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      return actual.Match(job);
    });
    worker.AddOdometry(test::MapOdometry(0), Clock::now());
    worker.AddRawMap(test::MapCloud(0), Clock::now());
    worker.Finish();
    const auto output = worker.Sample(200);
    EXPECT_EQ(output.map_tasks_completed(), 1);
    EXPECT_FALSE(output.has_map_from_imu());
  }
  std::ifstream file(prefix + ".csv");
  const std::string journal((std::istreambuf_iterator<char>(file)), {});
  EXPECT_NE(journal.find("map_task_deadline_expired"), std::string::npos);
  file.close();
  std::filesystem::remove(prefix + ".csv");
  NativeGlobalWorker worker(config, {}, [](const NativeMapJob&) -> NativeMapMatch {
    throw std::runtime_error("explicit test matcher failure");
  });
  worker.AddOdometry(test::MapOdometry(0), Clock::now());
  worker.AddRawMap(test::MapCloud(0), Clock::now());
  EXPECT_THROW(worker.Finish(), std::runtime_error);
  EXPECT_THROW(worker.Sample(200), std::runtime_error);
}

TEST_F(NativeMapTest, FullRawScanIsValidatedBeforeBoundedSampling) {
  std::atomic<int> matched_points{0};
  NativeGlobalWorker worker(config, {}, [&](const NativeMapJob& job) {
    matched_points = job.cloud.point_size();
    for (const auto& point : job.cloud.point())
      if (point.x() == 0 && point.y() == 0 && point.z() == 0)
        throw std::runtime_error("no-return sentinel survived sampling");
    NativeMapMatch result;
    result.sequence = job.cloud.header().sequence_num();
    result.measurement_ns = job.receive_ns;
    result.receive_ns = job.receive_ns;
    result.reason = "test_sampling_only";
    return result;
  });
  auto packet = test::MapCloud(0);
  const auto first = packet.cloud.point(0);
  while (packet.cloud.point_size() < 24001) *packet.cloud.add_point() = first;
  auto* sentinel = packet.cloud.mutable_point(0);
  sentinel->set_x(0); sentinel->set_y(0); sentinel->set_z(0);
  auto bad = packet;
  bad.cloud.mutable_point(1)->clear_timestamp();
  EXPECT_THROW(worker.AddRawMap(std::move(bad), Clock::now()), std::invalid_argument);
  worker.AddOdometry(test::MapOdometry(0), Clock::now());
  worker.AddRawMap(std::move(packet), Clock::now());
  worker.Finish();
  EXPECT_EQ(matched_points.load(), 12000);
  std::ifstream file(prefix + ".csv");
  const std::string journal((std::istreambuf_iterator<char>(file)), {});
  EXPECT_NE(journal.find("source_points=24001 no_returns=1 sample_stride=2"),
            std::string::npos);
}

TEST_F(NativeMapTest, ScanWaitsForOdometryFrontierAndShutdownReportsMissingFrontier) {
  std::atomic<int64_t> frontier{0};
  {
    NativeGlobalWorker worker(config, {}, [&](const NativeMapJob& job) {
      frontier = job.history.back().time_ns;
      NativeMapMatch match;
      match.sequence = job.cloud.header().sequence_num();
      match.measurement_ns = job.reference_ns;
      match.receive_ns = job.receive_ns;
      match.reason = "test_frontier_only";
      return match;
    });
    worker.AddOdometry(test::MapOdometry(0), Clock::now());
    worker.AddRawMap(test::MapCloud(1), Clock::now());
    worker.AddOdometry(test::MapOdometry(1), Clock::now());
    worker.Finish();
    EXPECT_EQ(frontier.load(), 100010000000LL);
    EXPECT_EQ(worker.Sample(200).map_tasks_completed(), 1);
  }
  std::filesystem::remove(prefix + ".csv");
  NativeGlobalWorker worker(config);
  worker.AddOdometry(test::MapOdometry(0), Clock::now());
  worker.AddRawMap(test::MapCloud(1), Clock::now());
  worker.Finish();
  EXPECT_EQ(worker.Sample(200).map_tasks_completed(), 1);
  std::ifstream file(prefix + ".csv");
  const std::string journal((std::istreambuf_iterator<char>(file)), {});
  EXPECT_NE(journal.find("map_task_shutdown_without_odometry_frontier"),
            std::string::npos);
}

TEST_F(NativeMapTest, IntegerScanEndpointSurvivesDoubleWireRoundingOnly) {
  auto packet = test::MapCloud(0);
  const int64_t exact = 1791773623508556843LL;
  packet.cloud.set_measurement_time(exact / 1e9);
  for (auto& point : *packet.cloud.mutable_point()) point.set_timestamp(exact);
  const auto reference = NativeMapMatcher::Reference(packet.cloud);
  EXPECT_GE(reference, exact);
  EXPECT_LT(reference - exact, 256);
  packet.cloud.mutable_point(packet.cloud.point_size() - 1)->set_timestamp(exact + 10000);
  EXPECT_THROW(NativeMapMatcher::Reference(packet.cloud), std::invalid_argument);
  const int64_t twice_rounded = 1791773624500324449LL;
  packet.cloud.set_measurement_time(twice_rounded / 1e9);
  packet.cloud.mutable_point(packet.cloud.point_size() - 1)->set_timestamp(twice_rounded);
  EXPECT_EQ(NativeMapMatcher::Reference(packet.cloud), twice_rounded);
}

TEST_F(NativeMapTest, RegionalAcquisitionNeedsConsistentScansAndRestartsAfterEpoch) {
  Regional();
  NativeMapMatcher matcher(config);
  int observations = 0;
  for (int i = 0; i < 6; ++i) {
    auto job = Job(i * 13);
    const auto result = matcher.Match(job);
    if (i < 2) {
      EXPECT_FALSE(result.observation) << result.search_detail;
    }
    if (result.observation) {
      ++observations;
      EXPECT_TRUE(result.observation->recovery());
      EXPECT_NEAR(result.observation->map_from_imu().x(), 0.35, 0.05);
    }
  }
  EXPECT_GT(observations, 0);
  auto reset = Job(80);
  reset.generation = 2;
  const auto retired = matcher.Match(reset);
  EXPECT_FALSE(retired.observation);
  EXPECT_NE(retired.search_detail.find("attempt=1"), std::string::npos);
  for (int i = 0; i < 6; ++i) {
    auto next = Job(93 + i * 13);
    next.generation = 2;
    const auto reacquired = matcher.Match(next);
    if (i == 5) {
      EXPECT_TRUE(reacquired.observation);
    }
  }
  auto global_reset = Job(180);
  global_reset.generation = 2;
  global_reset.alignment_session = "new-global-owner";
  global_reset.alignment_generation = 2;
  EXPECT_FALSE(matcher.Match(global_reset).observation);
  auto tracking = global_reset;
  tracking.map_from_odom = Eigen::Isometry3d::Identity();
  tracking.map_from_odom->translation() = Eigen::Vector3d(0.35, -0.22, 0.18);
  const auto tracked = matcher.Match(tracking);
  ASSERT_TRUE(tracked.observation);
  EXPECT_FALSE(tracked.observation->recovery());
  auto lease_expired = Job(193);
  lease_expired.generation = 2;
  lease_expired.alignment_session = global_reset.alignment_session;
  lease_expired.alignment_generation = global_reset.alignment_generation;
  EXPECT_FALSE(matcher.Match(lease_expired).observation);
  auto bad = config;
  *bad.mutable_online_map()->mutable_initial_map_from_odom() =
      test::Map(test::Odometry(0)).map_from_imu();
  EXPECT_THROW(NativeMapMatcher{bad}, std::invalid_argument);
}

TEST_F(NativeMapTest, RegionalPolicyRejectsCorruptionAndInvalidKeysButAcceptsCrlf) {
  Regional();
  const std::string path = config.online_map().regional_policy_csv();
  std::ifstream file(path);
  const std::string original{std::istreambuf_iterator<char>(file),
                             std::istreambuf_iterator<char>()};
  const auto write = [&](const std::string& text, bool repin = true) {
    std::ofstream out(path, std::ios::binary);
    out << text;
    out.close();
    ASSERT_TRUE(out);
    if (repin) config.mutable_online_map()->set_regional_policy_sha256(
        NativeMapMatcher::Sha256(path));
  };
  write(original + "unexpected,1\n", false);
  EXPECT_THROW(NativeMapMatcher{config}, std::invalid_argument);
  write(original + "unexpected,1\n");
  EXPECT_THROW(NativeMapMatcher{config}, std::invalid_argument);
  write(original + "yaw_min_rad,-0.1\n");
  EXPECT_THROW(NativeMapMatcher{config}, std::invalid_argument);
  write(original.substr(original.find('\n') + 1));
  EXPECT_THROW(NativeMapMatcher{config}, std::invalid_argument);
  auto invalid = original;
  invalid.replace(invalid.find("distance_sigma_m,1"), 18, "distance_sigma_m,nan");
  write(invalid);
  EXPECT_THROW(NativeMapMatcher{config}, std::invalid_argument);
  std::string crlf;
  for (const auto c : original) {
    if (c == '\n') crlf.push_back('\r');
    crlf.push_back(c);
  }
  write(crlf);
  EXPECT_NO_THROW(NativeMapMatcher{config});
}

TEST_F(NativeMapTest, MapHashHeaderAndModeAreCheckedBeforeAnySubscription) {
  auto bad = config;
  bad.mutable_online_map()->set_map_sha256(std::string(64, '0'));
  EXPECT_THROW(NativeMapMatcher{bad}, std::invalid_argument);
  bad = config;
  bad.set_map_topic("/prepared-map");
  EXPECT_THROW(NativeGlobalWorker{bad}, std::invalid_argument);
  bad = config;
  bad.mutable_online_map()->set_cloud_topic(bad.output_topic() + "/processed");
  EXPECT_THROW(NativeGlobalWorker{bad}, std::invalid_argument);
  {
    std::ofstream file(prefix + ".pcd");
    file << "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            "WIDTH 2000001\nHEIGHT 1\nPOINTS 2000001\nDATA binary\n";
  }
  config.mutable_online_map()->set_map_sha256(
      NativeMapMatcher::Sha256(prefix + ".pcd"));
  EXPECT_THROW(NativeMapMatcher{config}, std::invalid_argument);
}

TEST_F(NativeMapTest, RawScansCommitThroughOneOwnerAndEpochRetiresFullPose) {
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<NativeGlobalProcessed> processed;
  NativeGlobalWorker worker(config, [&](const auto& value) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      processed.push_back(value);
    }
    condition.notify_all();
  });
  const auto wait = [&](const char* kind, uint64_t sequence) {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::any_of(processed.begin(), processed.end(), [&](const auto& p) {
        return p.kind() == kind && p.sequence() == sequence;
      });
    });
  };
  for (int i = 0; i < 3; ++i) {
    const auto source = test::MapOdometry(i);
    const auto original = source.SerializeAsString();
    worker.AddOdometry(source, Clock::now());
    ASSERT_TRUE(wait("odom", i + 1));
    worker.AddRawMap(test::MapCloud(i), Clock::now());
    ASSERT_TRUE(wait("raw_map", i + 1));
    ASSERT_TRUE(wait("map_match", i + 1));
    EXPECT_EQ(source.SerializeAsString(), original);
  }
  const auto full = worker.Sample(200);
  ASSERT_TRUE(full.usable()) << full.reason();
  ASSERT_TRUE(full.full_pose_available());
  EXPECT_GT(full.correction_id(), 0);
  EXPECT_EQ(full.map_tasks_completed(), 3);
  EXPECT_EQ(full.processed_events(), 6);
  EXPECT_FALSE(full.trusted());
  EXPECT_FALSE(full.target_graph_available());
  EXPECT_NEAR(full.map_from_imu().x(), 0.35, 0.05);
  worker.AddOdometry(test::MapOdometry(3, 2), Clock::now());
  ASSERT_TRUE(wait("odom", 4));
  const auto retired = worker.Sample(200);
  EXPECT_FALSE(retired.usable());
  EXPECT_FALSE(retired.has_map_from_imu());
  EXPECT_EQ(retired.generation(), 2);
  worker.Finish();
  std::ifstream file(prefix + ".csv");
  const std::string journal((std::istreambuf_iterator<char>(file)), {});
  EXPECT_NE(journal.find("online-map"), std::string::npos);
  EXPECT_NE(journal.find("map_match,"), std::string::npos);
}

}  // namespace
}  // namespace apollo::localization::openloc_diagnostic
