// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "google/protobuf/text_format.h"
#include "google/protobuf/util/json_util.h"
#include "openloc/localization/local/native_types.h"

#include "native_lio_host_config.pb.h"
#include "native_map_test_input.h"
#include "native_graph_test_input.h"
#include "native_recorded_input.h"
#include "native_odometry_diagnostic.pb.h"
#include "wheelos_msgs/sensor_msgs/imu.pb.h"
#include "wheelos_msgs/sensor_msgs/gnss.pb.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

#include "cyber/cyber.h"

extern char** environ;

namespace apollo::localization::openloc_diagnostic {
namespace {
std::string executable;
std::string lio_executable;
std::string lio_profile;
std::string global_executable;
std::string replay_executable;

class Child {
 public:
  Child(const std::string& topic, const std::string& session) {
    Spawn({executable, topic, "1000", session});
  }
  explicit Child(std::vector<std::string> arguments) {
    Spawn(std::move(arguments));
  }
  void Spawn(std::vector<std::string> arguments) {
    std::vector<char*> argv;
    for (auto& argument : arguments) argv.push_back(argument.data());
    argv.push_back(nullptr);
    const int result = posix_spawn(&pid_, arguments.front().c_str(), nullptr,
                                   nullptr, argv.data(), environ);
    if (result != 0) {
      pid_ = -1;
      throw std::runtime_error("cannot spawn native IMU host: " +
                               std::to_string(result));
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
      throw std::runtime_error("cannot signal native IMU host");
    return Wait();
  }
  int Wait(const std::function<void()>& while_running = {},
           int timeout_ms = 5000) {
    for (int elapsed = 0; elapsed < timeout_ms; elapsed += 10) {
      int status = 0;
      const auto result = waitpid(pid_, &status, WNOHANG);
      if (result == pid_) {
        pid_ = -1;
        return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
      }
      if (result < 0) throw std::runtime_error("cannot reap native IMU host");
      if (while_running) while_running();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::ostringstream diagnostic;
    diagnostic << "Timed out child " << pid_ << " thread wait channels:\n";
    std::error_code error;
    for (const auto& thread : std::filesystem::directory_iterator(
             "/proc/" + std::to_string(pid_) + "/task", error)) {
      std::ifstream input(thread.path() / "wchan");
      std::string channel;
      if (std::getline(input, channel))
        diagnostic << thread.path().filename().string() << ':' << channel << '\n';
      else
        diagnostic << thread.path().filename().string() << ":unreadable\n";
    }
    if (error) diagnostic << "Cannot inspect child: " << error.message() << '\n';
    std::cerr << diagnostic.str();
    ::testing::Test::RecordProperty("timed_out_child_wait_channels", diagnostic.str());
    if (const auto* debugger = std::getenv("OPENLOC_TEST_TIMEOUT_DEBUGGER")) {
      const std::string process = std::to_string(pid_);
      std::vector<std::string> arguments{
          debugger, "--nx", "--batch", "-p", process,
          "-ex", "thread apply all bt", "-ex", "detach"};
      std::vector<char*> argv;
      for (auto& argument : arguments) argv.push_back(argument.data());
      argv.push_back(nullptr);
      pid_t debugger_pid;
      const int spawned = posix_spawn(&debugger_pid, debugger, nullptr, nullptr,
                                      argv.data(), environ);
      if (spawned) {
        std::cerr << "Cannot start timeout debugger: " << spawned << '\n';
      } else {
        bool finished = false;
        for (int i = 0; i < 200 && !finished; ++i) {
          int status;
          const auto result = waitpid(debugger_pid, &status, WNOHANG);
          if (result == debugger_pid) {
            finished = true;
          } else if (result < 0) {
            std::cerr << "Cannot reap timeout debugger\n";
            finished = true;
          } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
        }
        if (!finished) {
          kill(debugger_pid, SIGKILL);
          waitpid(debugger_pid, nullptr, 0);
          std::cerr << "Timeout debugger exceeded its diagnostic budget\n";
        }
      }
    }
    throw std::runtime_error("native test child did not stop within " +
                             std::to_string(timeout_ms) + " ms");
  }

 private:
  pid_t pid_ = -1;
};

void StrictLiveSequenceFailure(bool lidar, bool missing_header = false,
                               bool missing_axis = false, bool graph_export = false) {
  const std::string session = std::string("strict-") +
      (graph_export ? "joint-" : "") +
      (lidar ? "lidar-" : "imu-") +
      (missing_header ? "header-" : missing_axis ? "axis-" : "gap-") +
      std::to_string(getpid());
  NativeLioHostConfig config;
  {
    std::ifstream input(lio_profile);
    const std::string text((std::istreambuf_iterator<char>(input)), {});
    ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &config));
  }
  config.set_session(session);
  config.set_recorded_receive_time_from_header(false);
  config.set_require_live_source_sequences(true);
  config.set_imu_topic("/openloc/test/" + session + "/imu");
  config.set_lidar_topic("/openloc/test/" + session + "/lidar");
  if (graph_export) {
    auto* graph = config.mutable_graph();
    graph->set_output_topic("/openloc/diagnostic/joint/" + session);
    graph->set_motion_error_model_id("native-terminal-source-test");
    graph->set_imu_sample_period_s(0.01);
    graph->set_maximum_error_dimension(1024);
    graph->set_maximum_memory_rows(4096);
    graph->set_maximum_snapshots(64);
    graph->set_history_duration_s(0.1);
    graph->set_odometry_stride(1);
  }
  const std::string journal = ::testing::TempDir() + session + ".csv";
  config.set_journal(journal);
  const std::string profile = ::testing::TempDir() + session + ".pb.txt";
  std::string text;
  ASSERT_TRUE(google::protobuf::TextFormat::PrintToString(config, &text));
  {
    std::ofstream output(profile);
    output << text;
    ASSERT_TRUE(output);
  }
  auto node = cyber::CreateNode(session);
  ASSERT_TRUE(node);
  auto imu_writer = node->CreateWriter<drivers::gnss::Imu>(config.imu_topic());
  auto lidar_writer = node->CreateWriter<drivers::PointCloud>(config.lidar_topic());
  ASSERT_TRUE(imu_writer);
  ASSERT_TRUE(lidar_writer);
  struct Health {
    std::mutex mutex;
    std::condition_variable condition;
    bool heartbeat = false;
    std::optional<NativeOdometryDiagnostic> failure;
    std::optional<NativeJointOdometryInput> graph_failure;
  };
  const auto health = std::make_shared<Health>();
  auto output = node->CreateReader<NativeOdometryDiagnostic>(
      "/openloc/diagnostic/odometry", [health](const auto& message) {
        std::lock_guard<std::mutex> lock(health->mutex);
        health->heartbeat = true;
        if (message->reason() == "INVALID_INPUT" &&
            message->detail().rfind("Native host stopped: ", 0) == 0)
          health->failure = *message;
        health->condition.notify_all();
      });
  ASSERT_TRUE(output);
  std::shared_ptr<cyber::Reader<NativeJointOdometryInput>> graph_output;
  if (graph_export) {
    graph_output = node->CreateReader<NativeJointOdometryInput>(
        config.graph().output_topic(), [health](const auto& message) {
          std::lock_guard<std::mutex> lock(health->mutex);
          if (!message->model_available() &&
              message->model_detail().rfind("Native host stopped: ", 0) == 0)
            health->graph_failure = *message;
          health->condition.notify_all();
        });
    ASSERT_TRUE(graph_output);
  }
  Child child(std::vector<std::string>{lio_executable, profile});
  for (int i = 0; i < 500 && (!imu_writer->HasReader() || !lidar_writer->HasReader()); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_TRUE(imu_writer->HasReader());
  ASSERT_TRUE(lidar_writer->HasReader());
  if (graph_output) {
    for (int i = 0; i < 500 && !graph_output->HasWriter(); ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_TRUE(graph_output->HasWriter());
  }
  {
    std::unique_lock<std::mutex> lock(health->mutex);
    ASSERT_TRUE(health->condition.wait_for(lock, std::chrono::seconds(5),
                                         [&] { return health->heartbeat; }));
  }
  const auto submit = [&](uint32_t sequence, bool omit_header = false,
                          bool omit_axis = false) {
    const double now = cyber::Time::Now().ToSecond();
    if (lidar) {
      drivers::PointCloud message;
      if (!omit_header) message.mutable_header()->set_sequence_num(sequence);
      const uint64_t timestamp = static_cast<uint64_t>(now * 1e9);
      message.set_measurement_time(std::max(now, timestamp / 1e9));
      message.set_frame_id(config.lidar_frame());
      auto* point = message.add_point();
      point->set_x(1); point->set_y(2); point->set_z(3);
      if (omit_axis) point->clear_y();
      point->set_timestamp(timestamp);
      ASSERT_TRUE(lidar_writer->Write(message));
    } else {
      drivers::gnss::Imu message;
      if (!omit_header) message.mutable_header()->set_sequence_num(sequence);
      message.set_measurement_time(now);
      message.mutable_linear_acceleration()->set_x(0);
      message.mutable_linear_acceleration()->set_y(0);
      message.mutable_linear_acceleration()->set_z(9.80665);
      message.mutable_angular_velocity()->set_x(0);
      message.mutable_angular_velocity()->set_y(0);
      message.mutable_angular_velocity()->set_z(0);
      ASSERT_TRUE(imu_writer->Write(message));
    }
  };
  const auto read = [&] {
    std::ifstream input(journal + ".source.csv");
    return std::string(std::istreambuf_iterator<char>(input), {});
  };
  // HasReader is topology-only. Send distinct synthetic readiness samples,
  // never retry a real observation, until actual callback admission is seen.
  uint32_t sequence = 7;
  bool admitted = false;
  for (int i = 0; i < 100 && !admitted; ++i, ++sequence) {
    submit(sequence);
    for (int wait = 0; wait < 5 && !admitted; ++wait) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      admitted = read().find(",1,1,baseline_admitted,") != std::string::npos;
    }
  }
  ASSERT_TRUE(admitted);
  --sequence;
  const std::string expected_failure = missing_header
      ? "strict live source requires producer header sequence"
      : missing_axis ? "raw point lacks finite XYZ/time"
      : "live source sequence gap: previous=" + std::to_string(sequence) +
        " received=" + std::to_string(sequence + 3) + " missing=2";
  submit(sequence + (missing_axis ? 1 : 3), missing_header, missing_axis);
  NativeOdometryDiagnostic terminal;
  {
    std::unique_lock<std::mutex> lock(health->mutex);
    ASSERT_TRUE(health->condition.wait_for(lock, std::chrono::seconds(5),
        [&] { return health->failure.has_value() &&
            (!graph_export || health->graph_failure.has_value()); }));
    EXPECT_FALSE(health->failure->usable());
    EXPECT_FALSE(health->failure->has_odom_from_imu());
    EXPECT_FALSE(health->failure->has_velocity_in_odom());
    EXPECT_FALSE(health->failure->trusted());
    EXPECT_TRUE(health->failure->diagnostic_only());
    EXPECT_EQ(health->failure->detail(), "Native host stopped: " + expected_failure);
    terminal = *health->failure;
    if (graph_export) {
      ASSERT_TRUE(health->graph_failure);
      EXPECT_TRUE(health->graph_failure->IsInitialized());
      EXPECT_FALSE(health->graph_failure->model_available());
      EXPECT_FALSE(health->graph_failure->has_dimension());
      EXPECT_FALSE(health->graph_failure->odometry().has_odom_from_imu());
      EXPECT_EQ(health->graph_failure->model_detail(),
                "Native host stopped: " + expected_failure);
    }
  }
  EXPECT_NE(child.Wait(), 0);
  output->Shutdown();
  if (graph_output) graph_output->Shutdown();
  const auto ledger = read();
  EXPECT_NE(ledger.find(",1,1,fatal,"),
            std::string::npos) << ledger;
  EXPECT_NE(ledger.find(expected_failure), std::string::npos) << ledger;
  EXPECT_EQ(ledger.find("," + std::to_string(sequence + 3) + ",2,1,admitted,"),
            std::string::npos);
  if (const auto* artifacts = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR")) {
    std::string health_text;
    ASSERT_TRUE(google::protobuf::TextFormat::PrintToString(terminal, &health_text));
    std::ofstream health_output(std::filesystem::path(artifacts) /
                                (session + ".terminal.pb.txt"));
    health_output << health_text;
    ASSERT_TRUE(health_output);
    std::filesystem::copy_file(journal + ".source.csv",
        std::filesystem::path(artifacts) / (session + ".source.csv"));
    std::filesystem::copy_file(profile,
        std::filesystem::path(artifacts) / (session + ".pb.txt"));
  }
}

TEST(NativeImuHostTest, StrictLiveImuGapFailsBeforeRenumbering) {
  StrictLiveSequenceFailure(false);
}

TEST(NativeImuHostTest, SourceFailureWithdrawsTheSeparateTargetGraphStream) {
  StrictLiveSequenceFailure(false, false, false, true);
}

TEST(NativeImuHostTest, StrictLiveLidarGapFailsBeforeRenumbering) {
  StrictLiveSequenceFailure(true);
}

TEST(NativeImuHostTest, StrictLiveMissingHeaderFailsExplicitly) {
  StrictLiveSequenceFailure(false, true);
}

TEST(NativeImuHostTest, MissingRawPointAxisCannotBecomeAnImplicitZero) {
  StrictLiveSequenceFailure(true, false, true);
}

TEST(NativeImuHostTest, RawImuClockConversionAndDiagnosticTfCrossProcess) {
  const std::string session = "imu-host-" + std::to_string(getpid());
  const std::string topic = "/openloc/test/imu/" + std::to_string(getpid());
  std::shared_ptr<cyber::Node> node = cyber::CreateNode(session);
  ASSERT_TRUE(node);
  auto input = node->CreateWriter<drivers::gnss::Imu>(topic);
  ASSERT_TRUE(input);
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<NativeOdometryDiagnostic> odometry;
  std::vector<transform::TransformStamped> transforms;
  auto output = node->CreateReader<NativeOdometryDiagnostic>(
      "/openloc/diagnostic/odometry",
      [&](const std::shared_ptr<NativeOdometryDiagnostic>& message) {
        if (message->session() != session) return;
        {
          std::lock_guard<std::mutex> lock(mutex);
          odometry.push_back(*message);
        }
        condition.notify_all();
      });
  auto tf = node->CreateReader<transform::TransformStampeds>(
      "/openloc/diagnostic/tf",
      [&](const std::shared_ptr<transform::TransformStampeds>& message) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          for (const auto& item : message->transforms()) {
            if (item.header().frame_id().find("/" + session + "/") !=
                std::string::npos)
              transforms.push_back(item);
          }
        }
        condition.notify_all();
      });
  ASSERT_TRUE(output);
  ASSERT_TRUE(tf);
  Child child(topic, session);
  for (int attempt = 0; !input->HasReader() && attempt < 500; ++attempt)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_TRUE(input->HasReader());
  std::map<double, double> source_times;
  int submissions = 0;
  const auto submit = [&] {
    drivers::gnss::Imu imu;
    const double measurement = cyber::Time::Now().ToSecond() - 1000;
    imu.set_measurement_time(measurement);
    imu.mutable_linear_acceleration()->set_x(0);
    imu.mutable_linear_acceleration()->set_y(0);
    imu.mutable_linear_acceleration()->set_z(9.80665);
    imu.mutable_angular_velocity()->set_x(0);
    imu.mutable_angular_velocity()->set_y(0);
    imu.mutable_angular_velocity()->set_z(0);
    source_times.emplace(measurement + 1000, measurement);
    if (!input->Write(imu))
      throw std::runtime_error("raw IMU submission failed");
    ++submissions;
  };
  bool admitted = false;
  const auto ready_deadline = std::chrono::steady_clock::now() +
                              std::chrono::seconds(5);
  while (!admitted && std::chrono::steady_clock::now() < ready_deadline) {
    submit();
    std::unique_lock<std::mutex> lock(mutex);
    admitted = condition.wait_for(lock, std::chrono::milliseconds(10),
                                  [&] { return !odometry.empty(); });
  }
  ASSERT_TRUE(admitted);
  auto target = std::chrono::steady_clock::now();
  for (int index = 0; index < 560; ++index) {
    submit();
    target += std::chrono::milliseconds(5);
    std::this_thread::sleep_until(target);
  }
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(5), [&] {
      return !transforms.empty() &&
             std::any_of(
                 odometry.begin(), odometry.end(),
                 [](const auto& item) { return item.reason() == "IMU_STALE"; });
    }));
    bool usable = false;
    bool held = false;
    int new_count = 0;
    int held_count = 0;
    int stale_count = 0;
    int64_t maximum_interval_ns = 0;
    for (const auto& item : odometry) {
      EXPECT_FALSE(item.trusted());
      EXPECT_TRUE(item.diagnostic_only());
      maximum_interval_ns =
          std::max(maximum_interval_ns, item.publication_interval_ns());
      if (item.reason() == "IMU_STALE") ++stale_count;
      EXPECT_NE(source_times.find(item.measurement_time()), source_times.end());
      if (!item.usable()) {
        EXPECT_FALSE(item.has_odom_from_imu());
        continue;
      }
      usable = true;
      held |= !item.new_measurement();
      if (item.new_measurement())
        ++new_count;
      else
        ++held_count;
      EXPECT_DOUBLE_EQ(item.odom_from_imu().header().timestamp_sec(),
                       item.measurement_time());
      EXPECT_EQ(item.odom_from_imu().header().frame_id(),
                "openloc_diag/odom/" + session + "/1");
    }
    EXPECT_TRUE(usable);
    EXPECT_TRUE(held);
    RecordProperty("raw_imu_submissions", submissions);
    RecordProperty("new_measurements_received", new_count);
    RecordProperty("held_measurements_received", held_count);
    RecordProperty("stale_records_received", stale_count);
    RecordProperty("diagnostic_tf_received",
                   static_cast<int>(transforms.size()));
    RecordProperty("maximum_publication_interval_ns",
                   std::to_string(maximum_interval_ns));
    RecordProperty("fresh_100hz_or_deadline_qualified", "false");
    for (const auto& item : transforms) {
      EXPECT_NE(source_times.find(item.header().timestamp_sec()),
                source_times.end());
      EXPECT_EQ(item.child_frame_id(), "openloc_diag/imu/" + session + "/1");
    }
  }
  EXPECT_EQ(child.Stop(), 0);
}

TEST(NativeImuHostTest, IncompleteRawInputExitsWithFailure) {
  const std::string session = "imu-fault-" + std::to_string(getpid());
  const std::string topic =
      "/openloc/test/imu_fault/" + std::to_string(getpid());
  std::shared_ptr<cyber::Node> node = cyber::CreateNode(session);
  ASSERT_TRUE(node);
  auto input = node->CreateWriter<drivers::gnss::Imu>(topic);
  ASSERT_TRUE(input);
  Child child(topic, session);
  for (int attempt = 0; !input->HasReader() && attempt < 500; ++attempt)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_TRUE(input->HasReader());
  drivers::gnss::Imu incomplete;
  incomplete.set_measurement_time(cyber::Time::Now().ToSecond() - 1000);
  // HasReader is discovery, and Write only acknowledges transport submission.
  // Keep exercising the fault until the separate process actually consumes it.
  EXPECT_EQ(child.Wait([&] {
    if (!input->Write(incomplete))
      throw std::runtime_error("malformed input submission failed");
  }),
            1);
}

void RawPointCloudReachesLocalGeometry(bool recorded) {
  const std::string session =
      std::string(recorded ? "recorded-lio-host-" : "lio-host-") +
      std::to_string(getpid());
  NativeLioHostConfig config;
  {
    std::ifstream input(lio_profile);
    const std::string text((std::istreambuf_iterator<char>(input)), {});
    ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &config));
  }
  config.set_session(session);
  config.set_recorded_receive_time_from_header(recorded);
  config.set_lidar_commit_delay_ms(recorded ? 50 : 0);
  config.set_imu_topic("/openloc/test/lio_imu/" + session);
  config.set_lidar_topic("/openloc/test/lidar/" + session);
  const std::string journal = ::testing::TempDir() + session + ".csv";
  config.set_journal(journal);
  const std::string profile = ::testing::TempDir() + session + ".pb.txt";
  std::string text;
  ASSERT_TRUE(google::protobuf::TextFormat::PrintToString(config, &text));
  {
    std::ofstream output(profile);
    output << text;
  }
  std::shared_ptr<cyber::Node> node = cyber::CreateNode(session);
  ASSERT_TRUE(node);
  auto imu_writer = node->CreateWriter<drivers::gnss::Imu>(config.imu_topic());
  auto lidar_writer =
      node->CreateWriter<drivers::PointCloud>(config.lidar_topic());
  ASSERT_TRUE(imu_writer);
  ASSERT_TRUE(lidar_writer);
  std::mutex mutex;
  std::vector<NativeOdometryDiagnostic> received;
  std::condition_variable acknowledged;
  uint64_t accepted_imu = 0;
  uint64_t accepted_scan = 0;
  const auto accept = [&](bool scan) {
    return [&, scan](const std::shared_ptr<NativeRecordedIngressAck>& ack) {
      if (ack->session() != session) return;
      std::lock_guard<std::mutex> lock(mutex);
      auto& frontier = scan ? accepted_scan : accepted_imu;
      frontier = std::max(frontier, ack->source_sequence());
      acknowledged.notify_all();
    };
  };
  auto imu_ack = node->CreateReader<NativeRecordedIngressAck>(
      config.imu_topic() + "/recorded_ack", accept(false));
  auto scan_ack = node->CreateReader<NativeRecordedIngressAck>(
      config.lidar_topic() + "/recorded_ack", accept(true));
  ASSERT_TRUE(imu_ack);
  ASSERT_TRUE(scan_ack);
  const auto submit = [&](const auto& writer, const auto& message,
                          uint64_t sequence, bool scan) {
    for (int attempt = 0; attempt < (recorded ? 100 : 1); ++attempt) {
      if (!writer->Write(message))
        throw std::runtime_error("raw test submission rejected");
      if (!recorded) return;
      std::unique_lock<std::mutex> lock(mutex);
      if (acknowledged.wait_for(lock, std::chrono::milliseconds(10), [&] {
            return (scan ? accepted_scan : accepted_imu) >= sequence;
          }))
        return;
    }
    throw std::runtime_error("recorded raw test input not acknowledged");
  };
  auto output = node->CreateReader<NativeOdometryDiagnostic>(
      "/openloc/diagnostic/odometry",
      [&](const std::shared_ptr<NativeOdometryDiagnostic>& message) {
        if (message->session() != session) return;
        std::lock_guard<std::mutex> lock(mutex);
        received.push_back(*message);
      });
  ASSERT_TRUE(output);
  Child child(std::vector<std::string>{lio_executable, profile});
  for (int index = 0;
       index < 500 && (!imu_writer->HasReader() || !lidar_writer->HasReader());
       ++index)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_TRUE(imu_writer->HasReader());
  ASSERT_TRUE(lidar_writer->HasReader());
  auto target = std::chrono::steady_clock::now();
  int imu_submissions = 0;
  int scan_submissions = 0;
  std::string decisions;
  const double recorded_origin = cyber::Time::Now().ToSecond() - 100;
  for (int index = 0; index < 1600; ++index) {
    const double now = recorded ? recorded_origin + 0.005 * index
                                : cyber::Time::Now().ToSecond();
    drivers::gnss::Imu imu;
    imu.set_measurement_time(now);
    if (recorded) {
      imu.mutable_header()->set_timestamp_sec(now + 0.001);
      imu.mutable_header()->set_sequence_num(index + 1);
    }
    imu.mutable_linear_acceleration()->set_x(0);
    imu.mutable_linear_acceleration()->set_y(0);
    imu.mutable_linear_acceleration()->set_z(9.80665);
    imu.mutable_angular_velocity()->set_x(0);
    imu.mutable_angular_velocity()->set_y(0);
    imu.mutable_angular_velocity()->set_z(0);
    submit(imu_writer, imu, index + 1, false);
    if (recorded && index == 10) ASSERT_TRUE(imu_writer->Write(imu));
    ++imu_submissions;
    if (index % 20 == 0) {
      drivers::PointCloud cloud;
      if (recorded) {
        cloud.mutable_header()->set_timestamp_sec(now + 0.001);
        cloud.mutable_header()->set_sequence_num(scan_submissions + 1);
      }
      const uint64_t timestamp = static_cast<uint64_t>(now * 1e9);
      // The measurement end must contain the integer point timestamp
      // after floating-point conversion, rather than round it into
      // the future.
      cloud.set_measurement_time(std::max(now, timestamp / 1e9));
      cloud.set_frame_id(config.lidar_frame());
      for (int x = 0; x < 25; ++x)
        for (int y = 0; y < 25; ++y)
          for (const Eigen::Vector3d& point :
               {Eigen::Vector3d(0.2 * x, 0.2 * y, 4),
                Eigen::Vector3d(4, 0.2 * x, 0.2 * y),
                Eigen::Vector3d(0.2 * x, 4, 0.2 * y)}) {
            auto* raw = cloud.add_point();
            raw->set_x(point.x());
            raw->set_y(point.y());
            raw->set_z(point.z());
            raw->set_timestamp(timestamp);
          }
      // Explicit no-return sentinel is removed before bounded
      // sampling.
      auto* sentinel = cloud.add_point();
      sentinel->set_x(0);
      sentinel->set_y(0);
      sentinel->set_z(0);
      sentinel->set_timestamp(timestamp);
      submit(lidar_writer, cloud, scan_submissions + 1, true);
      ++scan_submissions;
    }
    target += std::chrono::milliseconds(5);
    std::this_thread::sleep_until(target);
    if (index >= 639 && index % 20 == 19) {
      std::ifstream journal_input(journal);
      decisions.assign(std::istreambuf_iterator<char>(journal_input), {});
      std::lock_guard<std::mutex> lock(mutex);
      std::string task_decisions = decisions;
      if (recorded) {
        std::ifstream tasks(journal + ".tasks.csv");
        task_decisions.assign(std::istreambuf_iterator<char>(tasks), {});
      }
      if (task_decisions.find("GEOMETRY_UNAVAILABLE") != std::string::npos &&
          task_decisions.find(",NONE,") != std::string::npos &&
          std::any_of(received.begin(), received.end(),
                      [](const auto& item) { return item.usable(); }))
        break;
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_EQ(child.Stop(), 0);
  if (recorded) {
    std::ifstream ingress(journal + ".ingress.csv");
    const std::string actual((std::istreambuf_iterator<char>(ingress)), {});
    EXPECT_EQ(actual, "imu_enqueued,scans_enqueued,recorded_receive_time\n" +
                          std::to_string(imu_submissions) + "," +
                          std::to_string(scan_submissions) + ",1\n");
  }
  std::ifstream input(journal);
  decisions.assign(std::istreambuf_iterator<char>(input), {});
  if (recorded) {
    std::ifstream tasks(journal + ".tasks.csv");
    decisions.assign(std::istreambuf_iterator<char>(tasks), {});
  }
  EXPECT_NE(decisions.find("GEOMETRY_UNAVAILABLE"), std::string::npos)
      << decisions;
  EXPECT_NE(decisions.find(",NONE,"), std::string::npos) << decisions;
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto& item : received)
      EXPECT_EQ(item.recorded_input(), recorded);
    EXPECT_TRUE(
        std::any_of(received.begin(), received.end(), [](const auto& item) {
          return item.usable() && !item.trusted() && item.diagnostic_only();
        }));
    ::testing::Test::RecordProperty("diagnostic_messages_received",
                                    static_cast<int>(received.size()));
    ::testing::Test::RecordProperty("full_lidar_global_host_complete", "false");
    if (recorded)
      for (const auto& item : received) {
        EXPECT_LT(item.receive_time(), cyber::Time::Now().ToSecond() - 90);
        EXPECT_NEAR(item.receive_time() - item.measurement_time(), 0.001, 1e-6);
      }
  }
  ::testing::Test::RecordProperty("raw_imu_submissions", imu_submissions);
  ::testing::Test::RecordProperty("raw_scan_submissions", scan_submissions);
}

TEST(NativeLioHostTest, RawPointCloudReachesLocalGeometryAcrossProcesses) {
  RawPointCloudReachesLocalGeometry(false);
}

TEST(NativeLioHostTest, RecordedHeaderTimeAndDelayedLioCrossProcesses) {
  RawPointCloudReachesLocalGeometry(true);
}

void RecordedProtocolFailure(bool conflict) {
  const std::string session =
      std::string(conflict ? "recorded-conflict-" : "recorded-missing-") +
      std::to_string(getpid());
  std::ifstream fixture(lio_profile);
  const std::string text((std::istreambuf_iterator<char>(fixture)), {});
  NativeLioHostConfig config;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &config));
  config.set_session(session);
  config.set_recorded_receive_time_from_header(true);
  config.set_imu_topic("/openloc/test/" + session + "/imu");
  config.set_lidar_topic("/openloc/test/" + session + "/lidar");
  config.set_journal(::testing::TempDir() + session + ".csv");
  std::string profile_text;
  ASSERT_TRUE(
      google::protobuf::TextFormat::PrintToString(config, &profile_text));
  const auto profile_path = ::testing::TempDir() + session + ".pb.txt";
  {
    std::ofstream profile(profile_path);
    profile << profile_text;
  }
  auto node = cyber::CreateNode(session);
  ASSERT_TRUE(node);
  auto writer = node->CreateWriter<drivers::gnss::Imu>(config.imu_topic());
  ASSERT_TRUE(writer);
  std::mutex mutex;
  std::condition_variable condition;
  bool accepted = false;
  auto ack = node->CreateReader<NativeRecordedIngressAck>(
      config.imu_topic() + "/recorded_ack",
      [&](const std::shared_ptr<NativeRecordedIngressAck>& message) {
        if (message->session() != session) return;
        std::lock_guard<std::mutex> lock(mutex);
        accepted = message->source_sequence() == 1;
        condition.notify_all();
      });
  ASSERT_TRUE(ack);
  Child child(std::vector<std::string>{lio_executable, profile_path});
  for (int index = 0;
       (!writer->HasReader() || (conflict && !ack->HasWriter())) && index < 500;
       ++index)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_TRUE(writer->HasReader());
  if (conflict) {
    ASSERT_TRUE(ack->HasWriter());
  }
  drivers::gnss::Imu sample;
  const double time = cyber::Time::Now().ToSecond() - 100;
  sample.set_measurement_time(time);
  sample.mutable_linear_acceleration()->set_x(0);
  sample.mutable_linear_acceleration()->set_y(0);
  sample.mutable_linear_acceleration()->set_z(9.80665);
  sample.mutable_angular_velocity()->set_x(0);
  sample.mutable_angular_velocity()->set_y(0);
  sample.mutable_angular_velocity()->set_z(0);
  if (conflict) {
    sample.mutable_header()->set_timestamp_sec(time + 0.001);
    sample.mutable_header()->set_sequence_num(1);
    for (int attempt = 0; attempt < 200; ++attempt) {
      ASSERT_TRUE(writer->Write(sample));
      std::unique_lock<std::mutex> lock(mutex);
      if (condition.wait_for(lock, std::chrono::milliseconds(10),
                             [&] { return accepted; }))
        break;
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      ASSERT_TRUE(accepted);
    }
    sample.mutable_linear_acceleration()->set_x(1);
  }
  EXPECT_EQ(child.Wait([&] {
    if (!writer->Write(sample))
      throw std::runtime_error("recorded fault submission rejected");
  }),
            1);
}

TEST(NativeLioHostTest, RecordedModeRejectsMissingSourceHeader) {
  RecordedProtocolFailure(false);
}

TEST(NativeLioHostTest, RecordedModeRejectsConflictingRetry) {
  RecordedProtocolFailure(true);
}

void ConnectedRawSources(bool target_graph) {
  const auto* artifacts = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR");
  const auto root = std::filesystem::path(
      artifacts ? artifacts : ::testing::TempDir()) /
      (std::string(target_graph ? "connected-target-" : "connected-") +
       std::to_string(getpid()));
  ASSERT_TRUE(std::filesystem::create_directory(root));
  NativeLioHostConfig local;
  {
    std::ifstream file(lio_profile);
    const std::string text((std::istreambuf_iterator<char>(file)), {});
    ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(text, &local));
  }
  const auto session = std::string(target_graph ? "connected-target-" : "connected-") +
                       std::to_string(getpid());
  local.set_session(session);
  local.set_recorded_receive_time_from_header(true);
  local.set_startup_task_timeout_ms(target_graph ? 0 : 500);
  local.set_lidar_commit_delay_ms(target_graph ? 0 : 50);
  local.set_imu_topic("/openloc/test/" + session + "/imu");
  local.set_lidar_topic("/openloc/test/" + session + "/lidar");
  local.set_journal((root / "local.csv").string());
  auto global = test::Config((root / "global.csv").string());
  if (target_graph) {
    test::EnableSharedMap(&global, (root / "map.pcd").string());
    test::EnableRawGga(&global);
    auto* raw = global.mutable_raw_gga();
    raw->set_fixed_stability_ns(1000000);
    raw->set_gnss_map_construction(NativeGlobalGnssInput::ABSENT);
    auto* qualification = raw->mutable_graph_qualification();
    qualification->set_uncertainty_model_id("synthetic-independent-white-gga");
    qualification->set_independence_evidence_id("synthetic-receiver-independent-of-fixed-map-and-lio");
    qualification->set_quality_valid(true);
    qualification->set_uncertainty_qualified(false);
    qualification->set_independent_of_local(true);
    qualification->set_independent_of_graph_sources(true);
    global.mutable_target_graph()->set_recorded_source_clock(true);
    auto* graph = local.mutable_graph();
    graph->set_output_topic("/openloc/diagnostic/" + session + "/joint");
    graph->set_motion_error_model_id(global.target_graph().motion_error_model_id());
    graph->set_imu_sample_period_s(0.01);
    graph->set_maximum_error_dimension(1024);
    graph->set_maximum_memory_rows(4096);
    graph->set_maximum_snapshots(64);
    graph->set_history_duration_s(0.05);
    graph->set_odometry_stride(4);
    graph->set_recorded_source_clock(true);
    for (auto& parameter : *local.mutable_parameter()) {
      if (parameter.name() == "max_scans") parameter.set_value(2);
      if (parameter.name() == "max_voxels") parameter.set_value(100);
      if (parameter.name() == "min_points_per_plane") parameter.set_value(3);
      if (parameter.name() == "min_correspondences") parameter.set_value(2);
    }
  } else {
    test::EnableOnlineMap(&global, (root / "map.pcd").string());
    test::EnableRawGga(&global);
  }
  global.set_odometry_topic("/openloc/diagnostic/odometry");
  if (target_graph) global.set_odometry_topic(local.graph().output_topic());
  global.set_gnss_topic("/openloc/test/" + session + "/gga");
  global.set_output_topic("/openloc/diagnostic/" + session + "/global");
  auto* map = global.mutable_online_map();
  map->set_cloud_topic("/openloc/test/" + session + "/map-cloud");
  if (!target_graph) map->set_recorded_ack_session(session);
  map->set_lidar_frame(local.lidar_frame());
  map->set_hypothesis_odometry_session(session);
  map->set_hypothesis_maximum_ns(104000000000LL);
  const auto profile = [&](const auto& message, const char* filename) {
    std::string text;
    if (!google::protobuf::TextFormat::PrintToString(message, &text))
      throw std::runtime_error("cannot serialize connected test profile");
    std::ofstream file(root / filename);
    file << text;
    file.close();
    if (!file) throw std::runtime_error("cannot write connected test profile");
  };
  profile(local, "local.pb.txt");
  profile(global, "global.pb.txt");
  const int imu_count = target_graph ? 224 : 800;
  const int scan_count = target_graph ? 4 : 40;
  const auto geometry = target_graph ? test::SharedMapGeometry() : test::MapGeometry();
  {
    std::ofstream imu(root / "imu.csv");
    imu << "timestamp_ns,receive_ns,ax,ay,az,gx,gy,gz\n";
    for (int i = 0; i < imu_count; ++i) {
      const int64_t time = 100000000000LL + i * (target_graph ? 10000000LL : 5000000LL);
      imu << time << ',' << time + 1000000 << ",0,0,9.80665,0,0,0\n";
    }
    imu.close();
    ASSERT_TRUE(imu);
    std::ofstream scans(root / "scans.csv");
    scans << "end_ns,receive_ns,points\n";
    for (int i = 0; i < scan_count; ++i) {
      const int64_t time = 100000000000LL +
          (target_graph ? 2030000000LL + i * 40000000LL : i * 100000000LL);
      const auto filename = "points-" + std::to_string(i) + ".csv";
      scans << time << ',' << time + 1000000 << ',' << filename << '\n';
      std::ofstream points(root / filename);
      points << std::setprecision(17) << "timestamp_ns,x,y,z\n";
      for (const auto& p : geometry)
        points << time << ',' << p.x() << ',' << p.y() << ',' << p.z() << '\n';
      points.close();
      ASSERT_TRUE(points);
    }
    scans.close();
    ASSERT_TRUE(scans);
    drivers::gnss::RawData packet;
    packet.mutable_header()->set_timestamp_sec(102.001);
    packet.mutable_header()->set_sequence_num(1);
    packet.set_data(test::Gga(2));
    std::string json;
    ASSERT_TRUE(google::protobuf::util::MessageToJsonString(packet, &json).ok());
    std::ofstream gga(root / "gga.jsonl");
    if (target_graph) {
      for (int i = 0; i < 2; ++i) {
        packet.mutable_header()->set_timestamp_sec(i ? 102.211 : 102.171);
        packet.mutable_header()->set_sequence_num(i + 1);
        packet.set_data(test::Nmea(
            std::string("GNGGA,000142.") + (i ? "190000000" : "150000000") +
            ",4000.0000000,N,11700.0000000,E,4,12,0.8,30,M,0,M,0.5,0001"));
        json.clear();
        ASSERT_TRUE(google::protobuf::util::MessageToJsonString(packet, &json).ok());
        gga << json << '\n';
      }
    } else {
      gga << json << '\n';
    }
    gga.close();
    ASSERT_TRUE(gga);
  }
  Child global_host(std::vector<std::string>{
      global_executable, (root / "global.pb.txt").string()});
  Child local_host(std::vector<std::string>{
      lio_executable, (root / "local.pb.txt").string()});
  std::vector<std::string> sender_arguments{
      replay_executable, root.string(), (root / "local.pb.txt").string(),
      (root / "subscriber").string(), "--prefetch-cloud",
      "--global-topic", global.output_topic()};
  if (target_graph) {
    sender_arguments.insert(sender_arguments.end(), {
        "--target-graph", "--raw-gga", (root / "gga.jsonl").string(),
        "--gga-topic", global.gnss_topic()});
  } else {
    sender_arguments.insert(sender_arguments.end(), {
        "--map-cloud-topic", map->cloud_topic(), "--map-admission-ack",
        "--raw-gga", (root / "gga.jsonl").string(), "--gga-topic", global.gnss_topic()});
  }
  Child sender(std::move(sender_arguments));
  ASSERT_EQ(sender.Wait({}, target_graph ? 120000 : 20000), 0);
  EXPECT_EQ(local_host.Stop(), 0);
  EXPECT_EQ(global_host.Stop(), 0);
  const auto summary = recorded::Read(
      root / "subscriber/summary.csv",
      std::string("imu_submitted,scans_submitted,messages_received,usable_received,"
      "last_source_sequence,tf_received,tail_received,transport_submissions,"
      "maximum_send_lateness_ns,global_messages_received,global_full_pose_received,"
      "global_last_odometry_sequence,global_usable_received,raw_gga_submitted,"
      "raw_map_submitted") +
          (target_graph ? ",target_graph_available_received" : ""), target_graph ? 16 : 15);
  ASSERT_EQ(summary.size(), 1);
  EXPECT_EQ(summary[0][4], std::to_string(imu_count));
  EXPECT_EQ(summary[0][6], "1");
  EXPECT_GT(std::stoul(summary[0][10]), 0);
  EXPECT_EQ(summary[0][11], std::to_string(imu_count));
  EXPECT_EQ(summary[0][13], target_graph ? "2" : "1");
  EXPECT_EQ(summary[0][14], target_graph ? "0" : "40");
  if (target_graph) {
    EXPECT_GT(std::stoul(summary[0][15]), 0);
  }
  const auto admitted = recorded::Read(
      root / "local.csv.ingress.csv",
      "imu_enqueued,scans_enqueued,recorded_receive_time", 3);
  ASSERT_EQ(admitted.size(), 1);
  EXPECT_EQ(admitted[0], (std::vector<std::string>{
      std::to_string(imu_count), std::to_string(scan_count), "1"}));
  std::ifstream journal(root / "global.csv");
  std::string line;
  int clouds = 0, packets = 0, commits = 0, map_commits = 0, gnss_commits = 0;
  while (std::getline(journal, line)) {
    clouds += line.rfind("raw_map,", 0) == 0;
    packets += line.rfind("raw_gga,", 0) == 0;
    if (line.rfind("owner,", 0) == 0) {
      std::istringstream cells(line);
      std::string field;
      for (int i = 0; i <= 6; ++i) std::getline(cells, field, ',');
      commits += field == "1";
      if (field == "1") {
        map_commits += line.find("\"online-map\"") != std::string::npos;
        gnss_commits += line.find("\"raw-gga-horizontal\"") != std::string::npos;
      }
    }
  }
  EXPECT_EQ(clouds, target_graph ? 0 : 40);
  EXPECT_EQ(packets, target_graph ? 2 : 1);
  EXPECT_GT(commits, 0);
  if (target_graph) {
    EXPECT_GT(map_commits, 0);
    EXPECT_EQ(gnss_commits, 1);
  }
}

TEST(NativeLioHostTest, ActualLocalAndGlobalProcessesConsumeRawSources) {
  ConnectedRawSources(false);
}

TEST(NativeLioHostTest, ActualRawSourcesReachTheSelectedTargetGraph) {
  ConnectedRawSources(true);
}

}  // namespace
}  // namespace apollo::localization::openloc_diagnostic

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  if (argc != 6) return 1;
  apollo::localization::openloc_diagnostic::executable = argv[1];
  apollo::localization::openloc_diagnostic::lio_executable = argv[2];
  apollo::localization::openloc_diagnostic::lio_profile = argv[3];
  apollo::localization::openloc_diagnostic::global_executable = argv[4];
  apollo::localization::openloc_diagnostic::replay_executable = argv[5];
  if (!apollo::cyber::Init(argv[0])) return 1;
  const int result = RUN_ALL_TESTS();
  apollo::cyber::Clear();
  return result;
}
