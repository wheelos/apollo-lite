// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <condition_variable>
#include <deque>
#include <exception>
#include <fstream>
#include <functional>
#include <future>
#include <mutex>
#include <optional>
#include <thread>
#include <variant>

#include "native_publication.h"
#include "native_graph_transport.h"
#include "openloc/localization/local/native_motion_bootstrap.h"

namespace apollo::localization::openloc_diagnostic {

// One owner thread for raw IMU/startup and commits. Optional delayed LIO uses
// isolated snapshots. Startup geometry can independently opt into this boundary.
class NativeLioWorker {
 public:
  using Clock = NativePublication::Clock;
  using Imu = ::openloc::localization::unified::ImuSample;
  using Scan = ::openloc::localization::unified::LidarScan;
  using Config = ::openloc::localization::unified::LocalConfig;
  using Callback =
      std::function<void(const NativePublication::State&,
                         const NativePublication::Result&, Clock::time_point)>;
  using StartupOperation = std::function<NativePublication::Result(
      ::openloc::localization::NativeMotionBootstrap::Task*)>;
  // True only when a usable model was submitted; false preserves the last
  // submitted motion endpoint. This is not a transport delivery ACK.
  using GraphCallback = std::function<bool(const NativeGraphFrame&, Clock::time_point)>;

  NativeLioWorker(const Config& config, double bootstrap_voxel,
                  const std::string& journal, Callback callback,
                  std::chrono::milliseconds lidar_commit_delay =
                      std::chrono::milliseconds(0),
                  std::chrono::milliseconds startup_timeout =
                      std::chrono::milliseconds(0),
                  StartupOperation startup_operation = {},
                  GraphCallback graph_callback = {}, uint32_t graph_stride = 1);
  ~NativeLioWorker();
  void AddImu(Imu sample, Clock::time_point arrival);
  void AddLidar(Scan scan, Clock::time_point arrival);
  void Check() const;
  void Finish();

 private:
  struct Event {
    std::variant<Imu, Scan> sample;
    Clock::time_point arrival;
    std::size_t bytes;
  };
  void Enqueue(Event event);
  void Run();
  void Process(const Imu& imu, Clock::time_point arrival);
  void DrainScans(const Imu& imu);
  void PublishGraph(Clock::time_point arrival);
  void AdvanceTask(const Imu& imu);
  void AdvanceStartup(const Imu& imu);
  void PollStartup();
  NativePublication::Result SubmitStartup(const Scan& scan);
  void RecordStartup(const char* status, double receive,
                     const NativePublication::Result& decision);
  NativePublication::Result SubmitTask(const Scan& scan);
  void FinishTask();
  void RecordTask(const char* status, double receive,
                  const NativePublication::Result& decision);
  void Stop();
  void Record(const Scan& scan, double reference, double receive,
              const NativePublication::Result& decision);

  const Config config_;
  ::openloc::localization::NativeOdometry owner_;
  ::openloc::localization::NativeMotionBootstrap bootstrap_;
  Callback callback_;
  GraphCallback graph_callback_;
  uint32_t graph_stride_ = 1;
  bool graph_frontier_required_ = false;
  std::optional<uint64_t> graph_scan_sequence_;
  std::optional<NativePublication::State> previous_graph_;
  std::ofstream journal_;
  const double commit_delay_;
  std::ofstream task_journal_;
  std::deque<Imu> catchup_;
  uint64_t task_sequence_ = 0;
  double task_reference_ = 0;
  double task_due_ = 0;
  bool task_expired_ = false;
  std::unique_ptr<::openloc::localization::NativeOdometry::LidarTask> task_;
  std::future<NativePublication::Result> task_future_;
  const double startup_timeout_;
  StartupOperation startup_operation_;
  std::ofstream startup_journal_;
  std::unique_ptr<::openloc::localization::NativeMotionBootstrap::Task> startup_;
  std::future<NativePublication::Result> startup_future_;
  std::optional<NativePublication::Result> startup_computed_;
  std::deque<Imu> startup_catchup_;
  uint64_t startup_sequence_ = 0;
  double startup_reference_ = 0;
  double startup_due_ = 0;
  Clock::time_point startup_started_;
  bool startup_expired_ = false;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Event> queue_;
  std::size_t queued_bytes_ = 0;
  std::deque<Scan> pending_;
  uint64_t last_scan_sequence_ = 0;
  double last_scan_time_ = 0;
  double last_scan_receive_ = 0;
  double initialized_time_ = 0;
  bool stopping_ = false;
  std::exception_ptr error_;
  std::thread thread_;
};

}  // namespace apollo::localization::openloc_diagnostic
