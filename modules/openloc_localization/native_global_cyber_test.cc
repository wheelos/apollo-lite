// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <signal.h>
#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <thread>

#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"
#include "native_global_cyber.h"
#include "native_global_test_input.h"
#include "native_map_test_input.h"
#include "native_graph_test_input.h"

extern char** environ;

namespace apollo::localization::openloc_diagnostic {
namespace {
std::string executable;

class Child {
 public:
  explicit Child(std::string profile) {
    char* argv[] = {executable.data(), profile.data(), nullptr};
    posix_spawn_file_actions_t actions;
    if (posix_spawn_file_actions_init(&actions) != 0)
      throw std::runtime_error("cannot initialize child logging");
    int result = posix_spawn_file_actions_addopen(
        &actions, STDERR_FILENO, (profile + ".stderr").c_str(),
        O_WRONLY | O_CREAT | O_EXCL, 0600);
    if (!result)
      result = posix_spawn(&pid_, executable.c_str(), &actions, nullptr, argv, environ);
    posix_spawn_file_actions_destroy(&actions);
    if (result != 0) {
      pid_ = -1;
      throw std::runtime_error("cannot spawn global host: " + std::to_string(result));
    }
  }
  ~Child() {
    if (pid_ > 0) {
      kill(pid_, SIGTERM);
      waitpid(pid_, nullptr, 0);
    }
  }
  int Stop() {
    if (kill(pid_, SIGINT) != 0)
      throw std::runtime_error("cannot interrupt global host");
    for (int i = 0; i < 500; ++i) {
      int status;
      const auto result = waitpid(pid_, &status, WNOHANG);
      if (result == pid_) {
        pid_ = -1;
        return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
      }
      if (result < 0) throw std::runtime_error("cannot reap global host");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error("global host shutdown deadline exceeded");
  }
 private:
  pid_t pid_ = -1;
};

template <typename Wait>
bool ConfirmOdometryTransport(
    const std::shared_ptr<cyber::Writer<NativeOdometryDiagnostic>>& writer,
    bool recorded, Wait wait) {
  // Discovery is not delivery readiness. Distinct unavailable publications
  // probe the actual input/completion path; no sensor observation is retried.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  uint64_t sequence = 1000000;
  while (std::chrono::steady_clock::now() < deadline) {
    NativeOdometryDiagnostic probe;
    probe.set_session("transport-readiness-probe");
    probe.set_publication_sequence(++sequence);
    probe.set_publication_time(cyber::Time::Now().ToSecond());
    probe.set_diagnostic_only(true);
    probe.set_trusted(false);
    probe.set_usable(false);
    probe.set_recorded_input(recorded);
    if (!writer->Write(probe)) return false;
    if (wait(sequence)) return true;
  }
  return false;
}

TEST(NativeGlobalCyberTest, CrossProcessCorrectionsHeldStaleAndEpochRetirement) {
  const char* artifacts = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR");
  const auto directory = artifacts ? std::string(artifacts) + "/" :
                                     ::testing::TempDir();
  const auto prefix = directory + "global-cyber-" +
                      std::to_string(getpid());
  auto config = test::Config(prefix + ".csv");
  const auto suffix = std::to_string(getpid());
  config.set_odometry_topic(config.odometry_topic() + suffix);
  config.set_gnss_topic(config.gnss_topic() + suffix);
  config.set_output_topic(config.output_topic() + suffix);
  config.set_map_topic("/openloc/diagnostic/global-test/map" + suffix);
  std::string text;
  ASSERT_TRUE(google::protobuf::TextFormat::PrintToString(config, &text));
  {
    std::ofstream profile(prefix + ".pb.txt");
    profile << text;
    profile.close();
    ASSERT_TRUE(profile);
  }
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<NativeGlobalDiagnostic> received;
  std::vector<NativeGlobalProcessed> processed;
  std::shared_ptr<cyber::Node> node =
      cyber::CreateNode("global_cyber_test_" + suffix);
  ASSERT_TRUE(node);
  auto odometry = node->CreateWriter<NativeOdometryDiagnostic>(
      config.odometry_topic());
  auto gnss = node->CreateWriter<NativeGlobalGnssInput>(config.gnss_topic());
  auto map = node->CreateWriter<NativeGlobalMapInput>(config.map_topic());
  ASSERT_TRUE(odometry);
  ASSERT_TRUE(gnss);
  ASSERT_TRUE(map);
  auto reader = node->CreateReader<NativeGlobalDiagnostic>(
      config.output_topic(), [&](const auto& input) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          received.push_back(*input);
        }
        condition.notify_all();
      });
  auto completion = node->CreateReader<NativeGlobalProcessed>(
      config.output_topic() + "/processed", [&](const auto& input) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          processed.push_back(*input);
        }
        condition.notify_all();
      });
  ASSERT_TRUE(reader);
  ASSERT_TRUE(completion);
  Child child(prefix + ".pb.txt");
  for (int i = 0; i < 500 &&
       (!odometry->HasReader() || !gnss->HasReader() || !map->HasReader() ||
        !reader->HasWriter() || !completion->HasWriter()); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_TRUE(odometry->HasReader());
  ASSERT_TRUE(gnss->HasReader());
  ASSERT_TRUE(map->HasReader());
  ASSERT_TRUE(reader->HasWriter());
  ASSERT_TRUE(completion->HasWriter());
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5),
                                   [&] { return !received.empty(); }));
  }
  const auto wait_processed = [&](const char* kind, uint64_t sequence,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, timeout, [&] {
      return std::any_of(processed.begin(), processed.end(), [&](const auto& p) {
        return p.kind() == kind && p.sequence() == sequence;
      });
    });
  };
  ASSERT_TRUE(ConfirmOdometryTransport(odometry, true, [&](uint64_t sequence) {
    return wait_processed("odom", sequence, std::chrono::milliseconds(100));
  }));
  for (int i = 0; i < 12; ++i) {
    auto source = test::Odometry(i);
    source.set_publication_time(cyber::Time::Now().ToSecond());
    ASSERT_TRUE(odometry->Write(source));
    ASSERT_TRUE(wait_processed("odom", i + 1)) << "source index " << i;
    ASSERT_TRUE(gnss->Write(test::Gnss(source)));
    ASSERT_TRUE(wait_processed("gnss", i + 1));
  }
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::any_of(received.begin(), received.end(), [](const auto& p) {
        return p.usable() && p.odometry_sequence() == 12 && !p.new_measurement();
      });
    }));
    bool corrected = false;
    for (const auto& output : received) {
      EXPECT_FALSE(output.trusted());
      EXPECT_FALSE(output.full_pose_available());
      EXPECT_FALSE(output.target_graph_available());
      if (output.usable()) {
        EXPECT_GT(output.correction_id(), 0);
        EXPECT_EQ(output.last_gnss_measurement_ns() % 100000000, 0);
        EXPECT_NEAR(output.alignment_x(), 100, 1e-6);
        EXPECT_NEAR(output.alignment_y(), 20, 1e-6);
        corrected = true;
      } else {
        EXPECT_FALSE(output.has_x());
      }
    }
    EXPECT_TRUE(corrected);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::any_of(received.begin(), received.end(), [](const auto& p) {
        return p.reason() == "LOCAL_ODOMETRY_STALE" &&
               p.odometry_sequence() == 12 && !p.has_x();
      });
    }));
  }
  for (int i = 12; i < 15; ++i) {
    auto source = test::Odometry(i);
    source.set_publication_time(cyber::Time::Now().ToSecond());
    ASSERT_TRUE(odometry->Write(source));
    ASSERT_TRUE(wait_processed("odom", i + 1));
    ASSERT_TRUE(map->Write(test::Map(source)));
    ASSERT_TRUE(wait_processed("map", i + 1));
  }
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::any_of(received.begin(), received.end(), [](const auto& p) {
        return p.usable() && p.full_pose_available() && p.odometry_sequence() == 15;
      });
    }));
    const auto full = std::find_if(received.begin(), received.end(),
        [](const auto& p) { return p.full_pose_available(); });
    ASSERT_NE(full, received.end());
    EXPECT_FALSE(full->trusted());
    EXPECT_FALSE(full->target_graph_available());
    EXPECT_NEAR(full->map_from_imu().x(), 114, 1e-6);
    EXPECT_NEAR(full->map_from_imu().z(), 3, 1e-6);
    EXPECT_EQ(full->last_full_pose_measurement_ns(), 101400000000LL);
  }
  auto source = test::Odometry(15);
  source.set_publication_time(cyber::Time::Now().ToSecond());
  ASSERT_TRUE(odometry->Write(source));
  ASSERT_TRUE(wait_processed("odom", 16));
  ASSERT_TRUE(gnss->Write(test::Gnss(source)));
  ASSERT_TRUE(wait_processed("gnss", 16));
  auto reset = test::Odometry(16, 2);
  reset.set_publication_time(cyber::Time::Now().ToSecond());
  ASSERT_TRUE(odometry->Write(reset));
  ASSERT_TRUE(wait_processed("odom", 17));
  auto retired = test::Gnss(reset);
  retired.set_generation(1);
  ASSERT_TRUE(gnss->Write(retired));
  ASSERT_TRUE(wait_processed("gnss", 17));
  auto retired_map = test::Map(reset);
  retired_map.set_generation(1);
  ASSERT_TRUE(map->Write(retired_map));
  ASSERT_TRUE(wait_processed("map", 17));
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::any_of(received.begin(), received.end(), [](const auto& p) {
        return p.generation() == 2 && !p.usable() && !p.has_x() &&
               !p.has_map_from_imu();
      });
    }));
    const auto rejected = std::find_if(processed.begin(), processed.end(),
        [](const auto& p) { return p.kind() == "gnss" && p.sequence() == 17; });
    ASSERT_NE(rejected, processed.end());
    EXPECT_NE(rejected->status(), 0);
    const auto map_rejected = std::find_if(processed.begin(), processed.end(),
        [](const auto& p) { return p.kind() == "map" && p.sequence() == 17; });
    ASSERT_NE(map_rejected, processed.end());
    EXPECT_NE(map_rejected->status(), 0);
  }
  EXPECT_EQ(child.Stop(), 0);
}

TEST(NativeGlobalCyberTest, ActualDelayedNativeScansReachTheSoleTargetGraphAndGnss) {
  const char* artifacts = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR");
  const auto directory = artifacts ? std::string(artifacts) + "/" : ::testing::TempDir();
  const auto prefix = directory + "shared-graph-cyber-" + std::to_string(getpid());
  auto config = test::Config(prefix + ".csv");
  test::EnableSharedMap(&config, prefix + ".pcd");
  test::EnableRawGga(&config);
  config.mutable_raw_gga()->set_fixed_stability_ns(1000000);
  config.mutable_raw_gga()->set_gnss_map_construction(NativeGlobalGnssInput::ABSENT);
  auto* qualification = config.mutable_raw_gga()->mutable_graph_qualification();
  qualification->set_uncertainty_model_id("synthetic-independent-white-gga");
  qualification->set_independence_evidence_id("synthetic-receiver-independent-of-fixed-map-and-lio");
  qualification->set_quality_valid(true);
  qualification->set_uncertainty_qualified(false);
  qualification->set_independent_of_local(true);
  qualification->set_independent_of_graph_sources(true);
  const auto suffix = std::to_string(getpid());
  config.set_odometry_topic(config.odometry_topic() + "/joint-" + suffix);
  config.set_gnss_topic(config.gnss_topic() + "/joint-" + suffix);
  config.set_output_topic(config.output_topic() + "/joint-" + suffix);
  const auto frames = test::ActualDelayedNativeFrames();
  ASSERT_EQ(frames.size(), 17);
  std::string text;
  ASSERT_TRUE(google::protobuf::TextFormat::PrintToString(config, &text));
  {
    std::ofstream file(prefix + ".pb.txt");
    file << text;
    file.close();
    ASSERT_TRUE(file);
  }
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<NativeGlobalProcessed> processed;
  std::vector<NativeGlobalDiagnostic> outputs;
  auto node = cyber::CreateNode("native_shared_graph_test_" + suffix);
  ASSERT_TRUE(node);
  auto odometry = node->CreateWriter<NativeJointOdometryInput>(config.odometry_topic());
  auto gga = node->CreateWriter<drivers::gnss::RawData>(config.gnss_topic());
  ASSERT_TRUE(odometry);
  ASSERT_TRUE(gga);
  auto reader = node->CreateReader<NativeGlobalDiagnostic>(
      config.output_topic(), [&](const auto& input) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          outputs.push_back(*input);
        }
        condition.notify_all();
      });
  auto completion = node->CreateReader<NativeGlobalProcessed>(
      config.output_topic() + "/processed", [&](const auto& input) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          processed.push_back(*input);
        }
        condition.notify_all();
      });
  ASSERT_TRUE(reader);
  ASSERT_TRUE(completion);
  Child child(prefix + ".pb.txt");
  for (int i = 0; i < 500 &&
       (!odometry->HasReader() || !gga->HasReader() ||
        !reader->HasWriter() || !completion->HasWriter()); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_TRUE(odometry->HasReader());
  ASSERT_TRUE(gga->HasReader());
  ASSERT_TRUE(completion->HasWriter());
  const auto wait_processed = [&](const char* kind, uint64_t sequence) {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, std::chrono::seconds(60), [&] {
      return std::any_of(processed.begin(), processed.end(), [&](const auto& event) {
        return event.kind() == kind && event.sequence() == sequence;
      });
    });
  };
  // Probe the actual input/completion path without retrying a sensor frame.
  const auto ready_until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  bool ready = false;
  for (uint64_t sequence = 1000000;
       !ready && std::chrono::steady_clock::now() < ready_until; ++sequence) {
    NativeJointOdometryInput probe;
    auto* source = probe.mutable_odometry();
    source->set_publication_sequence(sequence);
    source->set_publication_time(cyber::Time::Now().ToSecond());
    source->set_session("native-graph-readiness");
    source->set_diagnostic_only(true);
    source->set_trusted(false);
    source->set_usable(false);
    source->set_recorded_input(true);
    probe.set_error_model_id(config.target_graph().motion_error_model_id());
    probe.set_model_available(false);
    probe.set_model_detail("transport_readiness_probe");
    probe.set_measurement_ns(100000000000LL);
    probe.set_receive_ns(100000000000LL);
    ASSERT_TRUE(odometry->Write(probe));
    std::unique_lock<std::mutex> lock(mutex);
    ready = condition.wait_for(lock, std::chrono::milliseconds(100), [&] {
      return std::any_of(processed.begin(), processed.end(), [&](const auto& event) {
        return event.kind() == "joint_odom" && event.sequence() == sequence;
      });
    });
  }
  ASSERT_TRUE(ready);
  NativePublication publication(0.2);
  for (const auto& frame : frames) {
    SCOPED_TRACE(frame.nominal.stamp.sequence);
    const auto now = NativePublication::Clock::now();
    // Recorded diagnostic playback of immutable, previously computed native
    // snapshots; this does not claim live acquisition/solve latency.
    publication.Update(frame.nominal, frame.integrity, now);
    auto source = publication.Sample(cyber::Time::Now().ToSecond(), now);
    source.set_recorded_input(true);
    const auto packet = EncodeNativeGraph(frame, source,
        config.target_graph().motion_error_model_id());
    ASSERT_TRUE(odometry->Write(packet));
    ASSERT_TRUE(wait_processed("joint_odom", source.publication_sequence()));
    if (frame.scan) {
      ASSERT_TRUE(wait_processed("map_match", frame.scan->reference.sequence));
    }
    if (frame.nominal.stamp.sequence == 18 || frame.nominal.stamp.sequence == 19) {
      const bool last = frame.nominal.stamp.sequence == 19;
      drivers::gnss::RawData raw;
      raw.mutable_header()->set_sequence_num(last ? 2 : 1);
      raw.mutable_header()->set_timestamp_sec(frame.nominal.stamp.receive_time);
      raw.set_data(test::Nmea(
          std::string("GNGGA,000140.") + (last ? "180000000" : "170000000") +
          ",4000.0000000,N,11700.0000000,E,4,12,0.8,30,M,0,M,0.5,0001"));
      ASSERT_TRUE(gga->Write(raw));
      ASSERT_TRUE(wait_processed("raw_gga", last ? 2 : 1));
    }
  }
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return !outputs.empty() && outputs.back().target_graph_available() &&
          outputs.back().odometry_sequence() == 19;
    }));
    const auto& output = outputs.back();
    EXPECT_TRUE(output.usable());
    EXPECT_TRUE(output.full_pose_available());
    EXPECT_EQ(output.selected_owner(), "fixed_lag");
    EXPECT_EQ(output.motion_error_model_id(), config.target_graph().motion_error_model_id());
    EXPECT_FALSE(output.graph_assessment_reason().empty());
    EXPECT_TRUE(output.diagnostic_only());
    EXPECT_FALSE(output.trusted());
    EXPECT_EQ(output.map_tasks_completed(), 4);
    EXPECT_EQ(output.last_full_pose_measurement_ns(), 100140000000LL);
    int updates = 0;
    for (const auto& event : processed) {
      if (event.kind() == "map_match" && event.status() == 0) ++updates;
      if (event.kind() == "raw_gga" && event.sequence() == 2) {
        EXPECT_EQ(event.status(), 0);
      }
    }
    EXPECT_EQ(updates, 3);
  }
  EXPECT_EQ(child.Stop(), 0);
  std::ifstream journal(config.journal());
  const std::string evidence((std::istreambuf_iterator<char>(journal)), {});
  EXPECT_NE(evidence.find("online-map"), std::string::npos);
  EXPECT_NE(evidence.find("raw-gga-horizontal"), std::string::npos);
  reader->Shutdown();
  completion->Shutdown();
}

TEST(NativeGlobalCyberTest, FinishStopsReadersRetainedByTheNode) {
  const auto journal = ::testing::TempDir() + "global-finish-" +
                       std::to_string(getpid()) + ".csv";
  auto config = test::Config(journal);
  test::EnableReset(&config);
  config.set_map_topic("/openloc/diagnostic/global-finish/map");
  std::shared_ptr<cyber::Node> node =
      cyber::CreateNode("global_finish_test_" + std::to_string(getpid()));
  ASSERT_TRUE(node);
  auto writer = node->CreateWriter<NativeOdometryDiagnostic>(
      config.odometry_topic());
  ASSERT_TRUE(writer);
  auto map = node->CreateWriter<NativeGlobalMapInput>(config.map_topic());
  ASSERT_TRUE(map);
  auto reset = node->CreateWriter<NativeGlobalResetRequest>(config.reset_topic());
  auto ack = node->CreateWriter<NativeGlobalResetAck>(config.reset_topic() + "/ack");
  ASSERT_TRUE(reset);
  ASSERT_TRUE(ack);
  NativeGlobalCyber global(node, config);
  ASSERT_TRUE(writer->HasReader());
  ASSERT_TRUE(map->HasReader());
  ASSERT_TRUE(reset->HasReader());
  ASSERT_TRUE(ack->HasReader());
  global.Finish();
  EXPECT_FALSE(writer->HasReader());
  EXPECT_FALSE(map->HasReader());
  EXPECT_FALSE(reset->HasReader());
  EXPECT_FALSE(ack->HasReader());
  EXPECT_NO_THROW(global.Check());
}

TEST(NativeGlobalCyberTest, RawPointCloudMatchesAndPublishesAcrossProcesses) {
  const char* artifacts = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR");
  const auto directory = artifacts ? std::string(artifacts) + "/" : ::testing::TempDir();
  const auto prefix = directory + "global-map-" + std::to_string(getpid());
  auto config = test::Config(prefix + ".csv");
  test::EnableOnlineMap(&config, prefix + ".pcd");
  const auto suffix = std::to_string(getpid()) + "_raw_map";
  config.set_odometry_topic(config.odometry_topic() + suffix);
  config.set_gnss_topic(config.gnss_topic() + suffix);
  config.set_output_topic(config.output_topic() + suffix);
  config.mutable_online_map()->set_cloud_topic(
      config.online_map().cloud_topic() + suffix);
  std::string text;
  ASSERT_TRUE(google::protobuf::TextFormat::PrintToString(config, &text));
  {
    std::ofstream file(prefix + ".pb.txt");
    file << text;
    file.close();
    ASSERT_TRUE(file);
  }
  auto node = cyber::CreateNode("global_map_" + suffix);
  ASSERT_TRUE(node);
  auto odometry = node->CreateWriter<NativeOdometryDiagnostic>(config.odometry_topic());
  auto cloud = node->CreateWriter<drivers::PointCloud>(config.online_map().cloud_topic());
  ASSERT_TRUE(odometry);
  ASSERT_TRUE(cloud);
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<NativeGlobalProcessed> processed;
  std::vector<NativeGlobalDiagnostic> outputs;
  auto reader = node->CreateReader<NativeGlobalDiagnostic>(
      config.output_topic(), [&](const auto& input) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          outputs.push_back(*input);
        }
        condition.notify_all();
      });
  auto completion = node->CreateReader<NativeGlobalProcessed>(
      config.output_topic() + "/processed", [&](const auto& input) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          processed.push_back(*input);
        }
        condition.notify_all();
      });
  ASSERT_TRUE(reader);
  ASSERT_TRUE(completion);
  Child child(prefix + ".pb.txt");
  for (int i = 0; i < 500 && (!odometry->HasReader() || !cloud->HasReader() ||
       !reader->HasWriter() || !completion->HasWriter()); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_TRUE(odometry->HasReader());
  ASSERT_TRUE(cloud->HasReader());
  const auto wait = [&](const char* kind, uint64_t sequence) {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::any_of(processed.begin(), processed.end(), [&](const auto& p) {
        return p.kind() == kind && p.sequence() == sequence;
      });
    });
  };
  ASSERT_TRUE(ConfirmOdometryTransport(odometry, true, [&](uint64_t sequence) {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, std::chrono::milliseconds(20), [&] {
      return std::any_of(processed.begin(), processed.end(), [&](const auto& p) {
        return p.session() == "transport-readiness-probe" && p.sequence() == sequence;
      });
    });
  }));
  for (int i = 0; i < 3; ++i) {
    auto source = test::MapOdometry(i);
    source.set_publication_time(cyber::Time::Now().ToSecond());
    ASSERT_TRUE(odometry->Write(source));
    ASSERT_TRUE(wait("odom", i + 1));
    ASSERT_TRUE(cloud->Write(test::MapCloud(i).cloud));
    ASSERT_TRUE(wait("raw_map", i + 1));
    ASSERT_TRUE(wait("map_match", i + 1));
  }
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::any_of(outputs.begin(), outputs.end(), [](const auto& p) {
        return p.usable() && p.full_pose_available() && p.map_tasks_completed() == 3;
      });
    }));
    const auto full = std::find_if(outputs.begin(), outputs.end(),
        [](const auto& p) { return p.full_pose_available(); });
    ASSERT_NE(full, outputs.end());
    EXPECT_GT(full->correction_id(), 0);
    EXPECT_NEAR(full->map_from_imu().x(), 0.35, 0.05);
    EXPECT_FALSE(full->trusted());
    EXPECT_FALSE(full->target_graph_available());
  }
  EXPECT_EQ(child.Stop(), 0);
}

void RecordedMapAcknowledgementFailure(bool skipped) {
  const auto suffix = skipped ? "gap-" : "retry-";
  const auto prefix = ::testing::TempDir() + "global-map-ack-" + suffix +
      std::to_string(getpid());
  auto config = test::Config(prefix + ".csv");
  test::EnableOnlineMap(&config, prefix + ".pcd");
  config.mutable_online_map()->set_recorded_ack_session("recorded-map-test");
  auto invalid = config;
  invalid.set_recorded_input(false);
  EXPECT_THROW(NativeGlobalWorker rejected(invalid), std::invalid_argument);
  invalid = config;
  invalid.mutable_online_map()->set_recorded_ack_session("");
  EXPECT_THROW(NativeGlobalWorker rejected(invalid), std::invalid_argument);
  std::shared_ptr<cyber::Node> node =
      cyber::CreateNode("recorded_map_ack_" + std::string(suffix) + std::to_string(getpid()));
  ASSERT_TRUE(node);
  auto writer = node->CreateWriter<drivers::PointCloud>(config.online_map().cloud_topic());
  ASSERT_TRUE(writer);
  std::mutex mutex;
  std::condition_variable condition;
  int acknowledgements = 0;
  auto ack = node->CreateReader<NativeRecordedIngressAck>(
      config.online_map().cloud_topic() + "/recorded_ack", [&](const auto& input) {
        if (input->session() != "recorded-map-test" || input->source_sequence() != 1)
          return;
        std::lock_guard<std::mutex> lock(mutex);
        ++acknowledgements;
        condition.notify_all();
      });
  ASSERT_TRUE(ack);
  NativeGlobalCyber global(node, config);
  for (int i = 0; i < 500 && (!writer->HasReader() || !ack->HasWriter()); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_TRUE(writer->HasReader());
  ASSERT_TRUE(ack->HasWriter());
  const auto cloud = test::MapCloud(0).cloud;
  for (int expected = 1; expected <= 2; ++expected) {
    ASSERT_TRUE(writer->Write(cloud));
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5),
                                   [&] { return acknowledgements >= expected; }));
  }
  auto conflict = cloud;
  if (skipped)
    conflict.mutable_header()->set_sequence_num(3);
  else
    conflict.mutable_point(0)->set_x(cloud.point(0).x() + 1);
  ASSERT_TRUE(writer->Write(conflict));
  bool rejected = false;
  for (int i = 0; i < 500 && !rejected; ++i) {
    try {
      global.Check();
    } catch (const std::invalid_argument& error) {
      EXPECT_STREQ(error.what(), skipped
          ? "recorded raw map source sequence skipped"
          : "conflicting recorded raw map retry");
      rejected = true;
    }
    if (!rejected) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(rejected);
  EXPECT_THROW(global.Finish(), std::invalid_argument);
  ack->Shutdown();
  std::ifstream journal(config.journal());
  std::string line;
  int originals = 0;
  while (std::getline(journal, line))
    originals += line.rfind("raw_map,1,", 0) == 0;
  EXPECT_EQ(originals, 1);
}

TEST(NativeGlobalCyberTest, RecordedMapRetriesAcknowledgeOnlyOneOriginalEnqueue) {
  RecordedMapAcknowledgementFailure(false);
}

TEST(NativeGlobalCyberTest, RecordedMapGapsAreNotRenumberedOrAcknowledged) {
  RecordedMapAcknowledgementFailure(true);
}

void RawGgaCrossProcess(bool recorded) {
  const char* artifacts = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR");
  const auto directory = artifacts ? std::string(artifacts) + "/" : ::testing::TempDir();
  const auto prefix = directory + "global-raw-" +
      std::to_string(getpid()) + (recorded ? "-recorded" : "-live");
  auto config = test::Config(prefix + ".csv");
  test::EnableRawGga(&config);
  config.set_recorded_input(recorded);
  const auto suffix = std::to_string(getpid()) + (recorded ? "_recorded" : "_live");
  config.set_odometry_topic(config.odometry_topic() + suffix);
  config.set_gnss_topic(config.gnss_topic() + suffix);
  config.set_output_topic(config.output_topic() + suffix);
  // Live ingress clocks advance in wall time; dwell itself is covered with
  // the normal two-second policy by adapter/worker and recorded-process tests.
  if (!recorded) config.mutable_raw_gga()->set_fixed_stability_ns(0);
  std::string text;
  ASSERT_TRUE(google::protobuf::TextFormat::PrintToString(config, &text));
  {
    std::ofstream profile(prefix + ".pb.txt");
    profile << text;
    profile.close();
    ASSERT_TRUE(profile);
  }
  std::shared_ptr<cyber::Node> node = cyber::CreateNode("global_raw_" + suffix);
  ASSERT_TRUE(node);
  auto odometry = node->CreateWriter<NativeOdometryDiagnostic>(config.odometry_topic());
  auto raw = node->CreateWriter<drivers::gnss::RawData>(config.gnss_topic());
  ASSERT_TRUE(odometry);
  ASSERT_TRUE(raw);
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<NativeGlobalProcessed> processed;
  std::vector<NativeGlobalDiagnostic> outputs;
  auto reader = node->CreateReader<NativeGlobalDiagnostic>(
      config.output_topic(), [&](const auto& input) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          outputs.push_back(*input);
        }
        condition.notify_all();
      });
  auto completion = node->CreateReader<NativeGlobalProcessed>(
      config.output_topic() + "/processed", [&](const auto& input) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          processed.push_back(*input);
        }
        condition.notify_all();
      });
  ASSERT_TRUE(reader);
  ASSERT_TRUE(completion);
  Child child(prefix + ".pb.txt");
  for (int i = 0; i < 500 &&
       (!odometry->HasReader() || !raw->HasReader() || !reader->HasWriter() ||
        !completion->HasWriter()); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_TRUE(odometry->HasReader());
  ASSERT_TRUE(raw->HasReader());
  ASSERT_TRUE(reader->HasWriter());
  ASSERT_TRUE(completion->HasWriter());
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5),
                                   [&] { return !outputs.empty(); }));
  }
  auto wait = [&](const char* kind, uint64_t sequence) {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::any_of(processed.begin(), processed.end(), [&](const auto& p) {
        return p.kind() == kind && p.sequence() == sequence;
      });
    });
  };
  ASSERT_TRUE(ConfirmOdometryTransport(odometry, recorded, [&](uint64_t sequence) {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, std::chrono::milliseconds(20), [&] {
      return std::any_of(processed.begin(), processed.end(), [&](const auto& p) {
        return p.session() == "transport-readiness-probe" && p.sequence() == sequence;
      });
    });
  }));
  for (int i = 0; i < 16; ++i) {
    auto source = test::GgaOdometry(i);
    auto sentence = test::Gga(i);
    if (!recorded) {
      const int64_t ns = cyber::Time::Now().ToNanosecond();
      const double seconds = ns / 1e9;
      source.set_measurement_time(seconds);
      source.set_receive_time(seconds);
      source.set_recorded_input(false);
      source.mutable_odom_from_imu()->mutable_header()->set_timestamp_sec(seconds);
      const std::time_t whole = ns / 1000000000LL;
      std::tm utc;
      ASSERT_NE(gmtime_r(&whole, &utc), nullptr);
      char time[32];
      std::snprintf(time, sizeof(time), "%02d%02d%02d.%09lld",
          utc.tm_hour, utc.tm_min, utc.tm_sec,
          static_cast<long long>(ns % 1000000000LL));
      auto body = sentence.substr(1, sentence.find('*') - 1);
      body.replace(6, body.find(',', 6) - 6, time);
      sentence = test::Nmea(body);
    }
    source.set_publication_time(cyber::Time::Now().ToSecond());
    ASSERT_TRUE(odometry->Write(source));
    ASSERT_TRUE(wait("odom", i + 1)) << "source index " << i;
    drivers::gnss::RawData packet;
    packet.set_data(sentence);
    if (recorded) {
      packet.mutable_header()->set_timestamp_sec(source.receive_time());
      packet.mutable_header()->set_sequence_num(i + 1);
    }
    ASSERT_TRUE(raw->Write(packet));
    ASSERT_TRUE(wait("raw_gga", i + 1));
  }
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::any_of(outputs.begin(), outputs.end(), [](const auto& p) {
        return p.usable() && p.odometry_sequence() == 16 && p.correction_id() > 0;
      });
    }));
    for (const auto& output : outputs) {
      EXPECT_FALSE(output.full_pose_available());
      EXPECT_FALSE(output.trusted());
      EXPECT_FALSE(output.target_graph_available());
      EXPECT_EQ(output.recorded_input(), recorded);
    }
  }
  EXPECT_EQ(child.Stop(), 0);
}

TEST(NativeGlobalCyberTest, RecordedRawGgaCommitsAcrossProcesses) {
  RawGgaCrossProcess(true);
}

TEST(NativeGlobalCyberTest, HeaderlessLiveRawGgaCommitsAcrossProcesses) {
  RawGgaCrossProcess(false);
}

TEST(NativeGlobalCyberTest, DistinctOutputStreamsKeepTheirOwnGlobalHost) {
  const char* artifacts = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR");
  const auto directory = artifacts ? std::string(artifacts) + "/" :
                                     ::testing::TempDir();
  const auto suffix = std::to_string(getpid()) + "_parallel";
  std::shared_ptr<cyber::Node> node = cyber::CreateNode("global_parallel_" + suffix);
  ASSERT_TRUE(node);
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<uint64_t> completed(2, 0);
  std::vector<std::shared_ptr<cyber::Writer<NativeOdometryDiagnostic>>> writers;
  std::vector<std::shared_ptr<cyber::Reader<NativeGlobalProcessed>>> readers;
  std::vector<std::unique_ptr<Child>> children;
  for (std::size_t index = 0; index < completed.size(); ++index) {
    const auto identity = suffix + "_" + std::to_string(index);
    const auto prefix = directory + "global-" + identity;
    auto config = test::Config(prefix + ".csv");
    config.set_odometry_topic(config.odometry_topic() + identity);
    config.set_gnss_topic(config.gnss_topic() + identity);
    config.set_output_topic(config.output_topic() + identity);
    std::string text;
    ASSERT_TRUE(google::protobuf::TextFormat::PrintToString(config, &text));
    std::ofstream profile(prefix + ".pb.txt");
    profile << text;
    profile.close();
    ASSERT_TRUE(profile);
    writers.push_back(
        node->CreateWriter<NativeOdometryDiagnostic>(config.odometry_topic()));
    readers.push_back(node->CreateReader<NativeGlobalProcessed>(
        config.output_topic() + "/processed", [&, index](const auto& input) {
          if (input->kind() != "odom") return;
          {
            std::lock_guard<std::mutex> lock(mutex);
            completed[index] = input->sequence();
          }
          condition.notify_all();
        }));
    ASSERT_TRUE(writers.back());
    ASSERT_TRUE(readers.back());
    children.push_back(std::make_unique<Child>(prefix + ".pb.txt"));
  }
  for (std::size_t index = 0; index < writers.size(); ++index) {
    for (int wait = 0; wait < 500 &&
         (!writers[index]->HasReader() || !readers[index]->HasWriter()); ++wait)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_TRUE(writers[index]->HasReader());
    ASSERT_TRUE(readers[index]->HasWriter());
    ASSERT_TRUE(ConfirmOdometryTransport(
        writers[index], true, [&](uint64_t sequence) {
          std::unique_lock<std::mutex> lock(mutex);
          return condition.wait_for(lock, std::chrono::milliseconds(50),
              [&] { return completed[index] == sequence; });
        }));
  }
  for (int sample = 0; sample < 12; ++sample) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      std::fill(completed.begin(), completed.end(), 0);
    }
    const auto input = test::Odometry(sample);
    for (const auto& writer : writers) ASSERT_TRUE(writer->Write(input));
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::all_of(completed.begin(), completed.end(), [&](uint64_t value) {
        return value == input.publication_sequence();
      });
    }));
  }
  for (const auto& child : children) EXPECT_EQ(child->Stop(), 0);
  for (const auto& reader : readers) reader->Shutdown();
}

TEST(NativeGlobalCyberTest, ExplicitResetWaitsForConsumerRetirementAcrossProcesses) {
  const auto suffix = std::to_string(getpid()) + "_reset";
  const char* artifacts = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR");
  const auto directory = artifacts ? std::string(artifacts) + "/" :
                                     ::testing::TempDir();
  const auto prefix = directory + "global-" + suffix;
  auto config = test::Config(prefix + ".csv");
  test::EnableReset(&config);
  config.set_odometry_topic(config.odometry_topic() + suffix);
  config.set_gnss_topic(config.gnss_topic() + suffix);
  config.set_output_topic(config.output_topic() + suffix);
  config.set_reset_topic(config.reset_topic() + suffix);
  std::string text;
  ASSERT_TRUE(google::protobuf::TextFormat::PrintToString(config, &text));
  {
    std::ofstream file(prefix + ".pb.txt");
    file << text;
    file.close();
    ASSERT_TRUE(file);
  }
  std::shared_ptr<cyber::Node> node = cyber::CreateNode("global_reset_" + suffix);
  ASSERT_TRUE(node);
  auto odom = node->CreateWriter<NativeOdometryDiagnostic>(config.odometry_topic());
  auto gnss = node->CreateWriter<NativeGlobalGnssInput>(config.gnss_topic());
  auto reset = node->CreateWriter<NativeGlobalResetRequest>(config.reset_topic());
  auto ack = node->CreateWriter<NativeGlobalResetAck>(config.reset_topic() + "/ack");
  ASSERT_TRUE(odom);
  ASSERT_TRUE(gnss);
  ASSERT_TRUE(reset);
  ASSERT_TRUE(ack);
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<NativeGlobalProcessed> processed;
  std::vector<NativeGlobalDiagnostic> outputs;
  auto reader = node->CreateReader<NativeGlobalDiagnostic>(
      config.output_topic(), [&](const auto& input) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          outputs.push_back(*input);
        }
        condition.notify_all();
      });
  auto completion = node->CreateReader<NativeGlobalProcessed>(
      config.output_topic() + "/processed", [&](const auto& input) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          processed.push_back(*input);
        }
        condition.notify_all();
      });
  ASSERT_TRUE(reader);
  ASSERT_TRUE(completion);
  Child child(prefix + ".pb.txt");
  for (int i = 0; i < 500 &&
       (!odom->HasReader() || !gnss->HasReader() || !reset->HasReader() ||
        !ack->HasReader() || !reader->HasWriter() || !completion->HasWriter()); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_TRUE(odom->HasReader());
  ASSERT_TRUE(gnss->HasReader());
  ASSERT_TRUE(reset->HasReader());
  ASSERT_TRUE(ack->HasReader());
  ASSERT_TRUE(reader->HasWriter());
  ASSERT_TRUE(completion->HasWriter());
  const auto wait_output = [&](const auto& predicate) {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return !outputs.empty() && predicate(outputs.back());
    });
  };
  ASSERT_TRUE(wait_output([](const auto&) { return true; }));
  std::size_t events = 0;
  const auto send = [&](const auto& writer, const auto& input) {
    if (!writer->Write(input)) return false;
    ++events;
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, std::chrono::seconds(5),
                              [&] { return processed.size() == events; });
  };
  for (int i = 0; i < 12; ++i) {
    auto source = test::Odometry(i);
    source.set_publication_time(cyber::Time::Now().ToSecond());
    ASSERT_TRUE(send(odom, source));
    ASSERT_TRUE(send(gnss, test::Alignment(test::Gnss(source), 1)));
  }
  ASSERT_TRUE(wait_output([](const auto& p) {
    return p.usable() && p.alignment_generation() == 1;
  }));
  std::optional<NativeGlobalDiagnostic> planner_cache;
  {
    std::lock_guard<std::mutex> lock(mutex);
    planner_cache = outputs.back();
  }
  ASSERT_TRUE(send(reset, test::Reset()));
  ASSERT_TRUE(wait_output([](const auto& p) {
    return p.reset_pending() && p.alignment_generation() == 2;
  }));
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_FALSE(outputs.back().has_x());
    EXPECT_FALSE(outputs.back().has_map_from_imu());
    EXPECT_EQ(outputs.back().pending_reset_consumers_size(), 2);
  }
  // The consumer acknowledges only after retiring its old cached pose.
  planner_cache.reset();
  ASSERT_TRUE(send(ack, test::ResetAck("planner")));
  ASSERT_TRUE(wait_output([](const auto& p) {
    return p.reset_pending() && p.pending_reset_consumers_size() == 1;
  }));
  auto source = test::Odometry(12);
  source.set_publication_time(cyber::Time::Now().ToSecond());
  ASSERT_TRUE(send(odom, source));
  ASSERT_TRUE(send(gnss, test::Alignment(test::Gnss(source), 2)));
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_NE(processed.back().status(), 0);
  }
  ASSERT_TRUE(send(ack, test::ResetAck("observer", 1, 1)));
  ASSERT_TRUE(wait_output([](const auto& p) { return p.reset_pending(); }));
  ASSERT_TRUE(send(ack, test::ResetAck("observer")));
  ASSERT_TRUE(wait_output([](const auto& p) {
    return p.alignment_generation() == 2 && !p.reset_pending() && !p.usable();
  }));
  for (int i = 13; i < 25; ++i) {
    source = test::Odometry(i);
    source.set_publication_time(cyber::Time::Now().ToSecond());
    ASSERT_TRUE(send(odom, source));
    ASSERT_TRUE(send(gnss, test::Alignment(test::Gnss(source), 1)));
    {
      std::lock_guard<std::mutex> lock(mutex);
      EXPECT_NE(processed.back().status(), 0);
    }
    ASSERT_TRUE(send(gnss, test::Alignment(test::Gnss(source), 2)));
  }
  ASSERT_TRUE(wait_output([&](const auto& p) {
    return p.usable() && p.alignment_generation() == 2 &&
        p.odometry_sequence() == 25 && p.processed_events() == events;
  }));
  {
    std::lock_guard<std::mutex> lock(mutex);
    const auto& output = outputs.back();
    EXPECT_EQ(output.generation(), 1);
    EXPECT_EQ(output.reset_sequence(), 1);
    EXPECT_GT(output.correction_id(), 0);
    EXPECT_NEAR(output.x(), 124, 1e-6);
    EXPECT_FALSE(output.trusted());
    EXPECT_FALSE(output.full_pose_available());
    EXPECT_FALSE(output.target_graph_available());
    bool reset_seen = false;
    for (const auto& p : outputs) {
      if (p.alignment_generation() == 2) reset_seen = true;
      if (reset_seen) {
        EXPECT_EQ(p.alignment_generation(), 2);
      }
      if (p.reset_pending()) {
        EXPECT_FALSE(p.usable());
        EXPECT_FALSE(p.has_x());
      }
    }
  }
  EXPECT_FALSE(planner_cache);
  EXPECT_EQ(child.Stop(), 0);
}

TEST(NativeGlobalCyberTest, StructuredNovatelPositionAndHeadingAcrossProcesses) {
  const char* artifacts = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR");
  const std::string directory = artifacts ? std::string(artifacts) + "/" : ::testing::TempDir();
  const std::string suffix = std::to_string(getpid()) + "-structured";
  const auto prefix = directory + suffix;
  auto config = test::Config(prefix + ".csv");
  test::EnableStructuredGnss(&config);
  config.set_odometry_topic(config.odometry_topic() + suffix);
  config.set_gnss_topic(config.gnss_topic() + suffix);
  config.set_output_topic(config.output_topic() + suffix);
  config.mutable_structured_gnss()->mutable_heading()->set_topic(
      config.structured_gnss().heading().topic() + suffix);
  std::string text;
  ASSERT_TRUE(google::protobuf::TextFormat::PrintToString(config, &text));
  {
    std::ofstream profile(prefix + ".pb.txt");
    profile << text;
    profile.close();
    ASSERT_TRUE(profile);
  }
  auto node = cyber::CreateNode("structured_novatel_test_" + std::to_string(getpid()));
  ASSERT_TRUE(node);
  auto odometry = node->CreateWriter<NativeOdometryDiagnostic>(config.odometry_topic());
  auto best = node->CreateWriter<drivers::gnss::GnssBestPose>(config.gnss_topic());
  auto heading = node->CreateWriter<drivers::gnss::Heading>(
      config.structured_gnss().heading().topic());
  ASSERT_TRUE(odometry);
  ASSERT_TRUE(best);
  ASSERT_TRUE(heading);
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<NativeGlobalProcessed> events;
  std::vector<NativeGlobalDiagnostic> outputs;
  auto completion = node->CreateReader<NativeGlobalProcessed>(
      config.output_topic() + "/processed", [&](const auto& value) {
        std::lock_guard<std::mutex> lock(mutex);
        events.push_back(*value);
        condition.notify_all();
      });
  auto output = node->CreateReader<NativeGlobalDiagnostic>(
      config.output_topic(), [&](const auto& value) {
        std::lock_guard<std::mutex> lock(mutex);
        outputs.push_back(*value);
        condition.notify_all();
      });
  ASSERT_TRUE(completion);
  ASSERT_TRUE(output);
  Child child(prefix + ".pb.txt");
  for (int i = 0; i < 500 && (!odometry->HasReader() || !best->HasReader() ||
      !heading->HasReader() || !completion->HasWriter() || !output->HasWriter()); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_TRUE(odometry->HasReader());
  ASSERT_TRUE(best->HasReader());
  ASSERT_TRUE(heading->HasReader());
  const auto wait = [&](const char* kind, uint64_t sequence,
                        std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, timeout, [&] {
      return std::any_of(events.begin(), events.end(), [&](const auto& event) {
        return event.kind() == kind && event.sequence() == sequence;
      });
    });
  };
  ASSERT_TRUE(ConfirmOdometryTransport(odometry, true, [&](uint64_t sequence) {
    return wait("odom", sequence, std::chrono::milliseconds(100));
  }));
  // Distinct incomplete samples confirm actual callback delivery for both
  // receiver channels before replaying any real test observation.
  bool best_ready = false, heading_ready = false;
  for (uint32_t i = 1000000; i < 1000050 && (!best_ready || !heading_ready); ++i) {
    if (!best_ready) {
      drivers::gnss::GnssBestPose probe;
      probe.mutable_header()->set_sequence_num(i);
      probe.mutable_header()->set_timestamp_sec(100);
      ASSERT_TRUE(best->Write(probe));
      best_ready = wait("best_pose", i, std::chrono::milliseconds(100));
    }
    if (!heading_ready) {
      drivers::gnss::Heading probe;
      probe.mutable_header()->set_sequence_num(i);
      probe.mutable_header()->set_timestamp_sec(100);
      ASSERT_TRUE(heading->Write(probe));
      heading_ready = wait("heading", i, std::chrono::milliseconds(100));
    }
  }
  ASSERT_TRUE(best_ready);
  ASSERT_TRUE(heading_ready);
  for (int i = 0; i < 5; ++i) {
    auto source = test::GgaOdometry(i);
    source.set_publication_time(cyber::Time::Now().ToSecond());
    ASSERT_TRUE(odometry->Write(source));
    ASSERT_TRUE(wait("odom", i + 1));
    ASSERT_TRUE(heading->Write(test::DualHeading(i)));
    ASSERT_TRUE(wait("heading", i + 1));
    ASSERT_TRUE(best->Write(test::BestPose(i)));
    ASSERT_TRUE(wait("best_pose", i + 1));
  }
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::any_of(outputs.begin(), outputs.end(), [](const auto& value) {
        return value.usable() && value.odometry_sequence() == 5;
      });
    }));
    for (const auto& value : outputs) {
      EXPECT_FALSE(value.trusted());
      EXPECT_FALSE(value.full_pose_available());
      EXPECT_FALSE(value.has_map_from_imu());
      if (value.usable()) {
        EXPECT_EQ(value.reason(), "DIAGNOSTIC_HORIZONTAL_ONLY");
        EXPECT_GT(value.correction_id(), 0);
      }
    }
  }
  EXPECT_EQ(child.Stop(), 0);
}

}  // namespace
}  // namespace apollo::localization::openloc_diagnostic

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc != 2) return 1;
  apollo::localization::openloc_diagnostic::executable = argv[1];
  if (!apollo::cyber::Init(argv[0])) return 1;
  const int result = RUN_ALL_TESTS();
  apollo::cyber::Clear();
  return result;
}
