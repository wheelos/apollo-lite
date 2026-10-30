// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_lio_worker.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <stdexcept>
#include <utility>

namespace apollo::localization::openloc_diagnostic {
namespace u = ::openloc::localization::unified;
namespace {
constexpr std::size_t kMaximumEvents = 256;
constexpr std::size_t kMaximumBytes = 16 * 1024 * 1024;
}  // namespace

NativeLioWorker::NativeLioWorker(const Config& config, double bootstrap_voxel,
                                 const std::string& journal, Callback callback,
                                 std::chrono::milliseconds lidar_commit_delay,
                                 std::chrono::milliseconds startup_timeout,
                                 StartupOperation startup_operation,
                                 GraphCallback graph_callback, uint32_t graph_stride)
    : config_(config),
      owner_(config),
      bootstrap_(false, bootstrap_voxel),
      callback_(std::move(callback)),
      graph_callback_(std::move(graph_callback)),
      graph_stride_(graph_stride),
      commit_delay_(lidar_commit_delay.count() / 1000.0),
      startup_timeout_(startup_timeout.count() / 1000.0),
      startup_operation_(std::move(startup_operation)) {
  const auto valid = owner_.ValidateConfig();
  if (!valid.ok() || !config.enable_lidar ||
      (graph_callback_ && (!config.enable_native_joint_error ||
                          graph_stride == 0 || graph_stride > 100)) ||
      !config.enable_motion_initialization || !callback_ ||
      (commit_delay_ != 0 && (commit_delay_ < 0.005 || commit_delay_ > 0.1)) ||
      (startup_timeout_ != 0 && (startup_timeout_ < 0.001 || startup_timeout_ > 0.5)))
    throw std::invalid_argument("invalid native LIO worker configuration: " +
                                valid.message);
  if (std::filesystem::exists(journal))
    throw std::invalid_argument("LIO journal must not already exist");
  journal_.open(journal);
  if (!journal_) throw std::runtime_error("cannot open native LIO journal");
  journal_ << std::setprecision(17)
           << "sequence,raw_end_s,source_receive_s,reference_s,"
              "frontier_receive_s,reason,detail\n";
  if (commit_delay_ != 0) {
    if (std::filesystem::exists(journal + ".tasks.csv"))
      throw std::invalid_argument("LIO task journal must not already exist");
    task_journal_.open(journal + ".tasks.csv");
    if (!task_journal_)
      throw std::runtime_error("cannot open LIO task journal");
    task_journal_ << std::setprecision(17)
                  << "sequence,reference_s,due_receive_s,processing_receive_s,"
                     "status,reason,detail\n";
  }
  if (startup_timeout_ != 0) {
    if (std::filesystem::exists(journal + ".startup.csv"))
      throw std::invalid_argument("startup journal must not already exist");
    startup_journal_.open(journal + ".startup.csv");
    if (!startup_journal_) throw std::runtime_error("cannot open startup journal");
    startup_journal_ << std::setprecision(17)
                     << "sequence,reference_s,due_receive_s,processing_receive_s,"
                        "status,reason,detail\n";
  }
  thread_ = std::thread([this] { Run(); });
}

NativeLioWorker::~NativeLioWorker() { Stop(); }

void NativeLioWorker::Check() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (error_) std::rethrow_exception(error_);
}

void NativeLioWorker::Enqueue(Event event) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (error_) std::rethrow_exception(error_);
  if (stopping_) throw std::runtime_error("native LIO worker stopped");
  if (queue_.size() >= kMaximumEvents ||
      event.bytes > kMaximumBytes - queued_bytes_) {
    error_ = std::make_exception_ptr(
        std::runtime_error("native LIO input queue overflow"));
    condition_.notify_all();
    std::rethrow_exception(error_);
  }
  queued_bytes_ += event.bytes;
  queue_.push_back(std::move(event));
  condition_.notify_one();
}

void NativeLioWorker::AddImu(Imu sample, Clock::time_point arrival) {
  if (arrival > Clock::now())
    throw std::invalid_argument("IMU arrival cannot be in the future");
  Enqueue({std::move(sample), arrival, sizeof(Imu)});
}

void NativeLioWorker::AddLidar(Scan scan, Clock::time_point arrival) {
  if (arrival > Clock::now() || scan.points.empty() ||
      scan.points.size() > config_.lidar_max_points ||
      scan.frame_id != config_.lidar_frame ||
      scan.calibration_id != config_.lidar_calibration_id ||
      !scan.base_from_lidar.matrix().allFinite() ||
      !(scan.base_from_lidar.linear().transpose() *
        scan.base_from_lidar.linear())
           .isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
      std::abs(scan.base_from_lidar.linear().determinant() - 1) > 1e-6 ||
      !scan.base_from_lidar.matrix().row(3).isApprox(
          Eigen::RowVector4d(0, 0, 0, 1), 1e-9))
    throw std::invalid_argument("invalid bounded LiDAR input");
  double previous = 0;
  for (const auto& point : scan.points) {
    if (!point.position.allFinite() || !std::isfinite(point.time) ||
        point.time <= 0 || point.time < previous ||
        point.time > scan.stamp.time)
      throw std::invalid_argument("invalid ordered raw LiDAR point");
    previous = point.time;
  }
  const auto bytes =
      sizeof(Scan) + scan.points.size() * sizeof(u::TimedLidarPoint);
  Enqueue({std::move(scan), arrival, bytes});
}

void NativeLioWorker::Stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  condition_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void NativeLioWorker::Finish() {
  Stop();
  Check();
}

void NativeLioWorker::Run() {
  try {
    for (;;) {
      Event event;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        const auto pending = [this] { return stopping_ || error_ || !queue_.empty(); };
        if (startup_) condition_.wait_for(lock, std::chrono::milliseconds(5), pending);
        else condition_.wait(lock, pending);
        if (error_) {
          lock.unlock();
          FinishTask();
          return;
        }
        if (queue_.empty()) {
          if (stopping_) break;
          lock.unlock();
          PollStartup();
          continue;
        }
        event = std::move(queue_.front());
        queue_.pop_front();
        queued_bytes_ -= event.bytes;
      }
      if (const auto* imu = std::get_if<Imu>(&event.sample)) {
        Process(*imu, event.arrival);
      } else {
        auto scan = std::get<Scan>(std::move(event.sample));
        if (!std::isfinite(scan.stamp.time) || scan.stamp.time <= 0 ||
            !std::isfinite(scan.stamp.receive_time) ||
            scan.stamp.receive_time <= 0 ||
            scan.stamp.time - scan.stamp.receive_time >
                config_.lidar_future_tolerance ||
            scan.stamp.clock_id != config_.clock_id ||
            scan.stamp.sequence <= last_scan_sequence_ ||
            scan.stamp.time <= last_scan_time_ ||
            scan.stamp.receive_time < last_scan_receive_)
          throw std::runtime_error(
              "LiDAR source frontier invalid or regressed");
        last_scan_sequence_ = scan.stamp.sequence;
        last_scan_time_ = scan.stamp.time;
        last_scan_receive_ = scan.stamp.receive_time;
        if (pending_.size() >= 2)
          throw std::runtime_error("pending native LiDAR queue overflow");
        pending_.push_back(std::move(scan));
      }
    }
    FinishTask();
    for (const auto& scan : pending_)
      Record(scan, 0, 0,
             {u::Reason::HISTORY_UNAVAILABLE,
              "End of input before a causal IMU frontier"});
    journal_.flush();
    if (!journal_) throw std::runtime_error("incomplete native LIO journal");
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    error_ = std::current_exception();
  }
}

void NativeLioWorker::Process(const Imu& imu, Clock::time_point arrival) {
  const auto result = owner_.AddImu(imu);
  const auto integrity = owner_.EvaluateIntegrity(imu.stamp.receive_time);
  if (!result.ok() &&
      !(integrity.ok() &&
        (result.reason == u::Reason::WHEEL_STALE ||
         result.reason == u::Reason::CORRELATION_UNQUALIFIED ||
         result.reason == u::Reason::CORRECTION_BUDGET_EXCEEDED)))
    throw std::runtime_error(result.message);
  if (!owner_.state().valid) bootstrap_.AddImu(imu);
  AdvanceStartup(imu);
  AdvanceTask(imu);
  if (owner_.state().stamp.sequence) {
    auto publication = owner_.EvaluateIntegrity(imu.stamp.receive_time);
    const auto capability = owner_.Evaluate(imu.stamp.receive_time);
    if (publication.ok() &&
        capability.reason == u::Reason::CORRECTION_BUDGET_EXCEEDED)
      publication = capability;
    callback_(owner_.state(), publication, arrival);
  }
  if (owner_.state().valid && initialized_time_ == 0)
    initialized_time_ = owner_.state().stamp.time;
  DrainScans(imu);
  PublishGraph(arrival);
}

void NativeLioWorker::PublishGraph(Clock::time_point arrival) {
  if (!graph_callback_ || !owner_.state().valid ||
      (!graph_frontier_required_ && !graph_scan_sequence_ &&
       previous_graph_ && owner_.state().stamp.sequence % graph_stride_))
    return;
  NativeGraphFrame frame;
  frame.nominal = owner_.state();
  frame.integrity = owner_.EvaluateIntegrity(frame.nominal.stamp.receive_time);
  const auto capability = owner_.Evaluate(frame.nominal.stamp.receive_time);
  if (frame.integrity.ok() &&
      capability.reason == u::Reason::CORRECTION_BUDGET_EXCEEDED)
    frame.integrity = capability;
  if (graph_scan_sequence_) {
    frame.scan.emplace();
    frame.assessment = owner_.PrepareJointLidarScan(*graph_scan_sequence_, &*frame.scan);
    if (!frame.assessment.ok()) frame.scan.reset();
  }
  if (frame.assessment.ok())
    frame.assessment = owner_.JointErrorLookup(frame.nominal.stamp.time, &frame.joint);
  if (frame.assessment.ok() && previous_graph_) {
    frame.motion.emplace();
    frame.assessment = owner_.JointErrorMotionBetween(
        previous_graph_->stamp.time, frame.joint.stamp.time, &*frame.motion);
    frame.previous_dimension = previous_graph_->joint_error->covariance.rows();
  }
  const bool submitted = graph_callback_(frame, arrival);
  if (submitted && frame.assessment.ok()) previous_graph_ = frame.joint;
  graph_scan_sequence_.reset();
  graph_frontier_required_ = false;
}

void NativeLioWorker::DrainScans(const Imu& imu) {
  while (!pending_.empty() && pending_.front().stamp.time <= imu.stamp.time &&
         pending_.front().stamp.receive_time <= imu.stamp.receive_time) {
    auto source = std::move(pending_.front());
    pending_.pop_front();
    auto scan = source;
    scan.stamp.time = imu.stamp.time;
    scan.stamp.receive_time = imu.stamp.receive_time;
    scan.epoch = owner_.state().epoch;
    u::Result result;
    const bool bootstrapping = !owner_.state().valid;
    if (imu.stamp.receive_time - source.stamp.time >
        config_.lidar_max_scan_age) {
      result = {u::Reason::LIDAR_STALE, "Raw scan exceeded source age budget"};
    } else if (bootstrapping) {
      result = startup_timeout_ != 0 ? SubmitStartup(scan)
                                    : bootstrap_.Observe(scan, &owner_);
      if (result.ok()) initialized_time_ = imu.stamp.time;
    } else if (scan.points.front().time < initialized_time_) {
      result = {u::Reason::HISTORY_UNAVAILABLE,
                "Scan overlaps pre-initialization history"};
    } else if (startup_) {
      result = {u::Reason::LIDAR_STALE, "Retired startup computation is still outstanding"};
    } else if (commit_delay_ != 0) {
      result = SubmitTask(scan);
    } else {
      result = owner_.AddLidar(scan);
      if (graph_callback_ &&
          (result.ok() || result.reason == u::Reason::GEOMETRY_UNAVAILABLE))
        graph_scan_sequence_ = scan.stamp.sequence;
    }
    Record(source, imu.stamp.time, imu.stamp.receive_time, result);
    const bool expected =
        result.ok() || result.reason == u::Reason::GEOMETRY_UNAVAILABLE ||
        result.reason == u::Reason::HISTORY_UNAVAILABLE ||
        result.reason == u::Reason::LIDAR_STALE ||
        (bootstrapping && (result.reason == u::Reason::DEGENERATE ||
                           result.reason == u::Reason::MATCH_FAILED ||
                           result.reason == u::Reason::INNOVATION_REJECTED));
    if (!expected) throw std::runtime_error(result.message);
  }
}

NativePublication::Result NativeLioWorker::SubmitStartup(const Scan& scan) {
  if (startup_)
    return {u::Reason::LIDAR_STALE, "One relative startup snapshot is outstanding"};
  const auto prepared = bootstrap_.PrepareTask(scan, owner_, &startup_);
  if (!prepared.ok()) return prepared;
  startup_sequence_ = scan.stamp.sequence;
  startup_reference_ = scan.stamp.time;
  startup_due_ = scan.stamp.receive_time +
                 std::min(startup_timeout_, config_.lidar_max_scan_age);
  startup_started_ = Clock::now();
  startup_expired_ = false;
  startup_computed_.reset();
  startup_catchup_.clear();
  startup_future_ = std::async(std::launch::async, [this, task = startup_.get()] {
    return startup_operation_ ? startup_operation_(task) : task->Compute();
  });
  const u::Result queued{u::Reason::HISTORY_UNAVAILABLE,
                         "Relative startup snapshot queued"};
  RecordStartup("queued", scan.stamp.receive_time, queued);
  return queued;
}

void NativeLioWorker::AdvanceStartup(const Imu& imu) {
  PollStartup();
  if (!startup_) return;
  const bool ready = startup_computed_.has_value();
  if (!startup_expired_ &&
      (owner_.state().valid || imu.stamp.receive_time > startup_due_ ||
       std::chrono::duration<double>(Clock::now() - startup_started_).count() >
           startup_timeout_ || startup_catchup_.size() >= 128)) {
    startup_expired_ = true;
    startup_catchup_.clear();
    RecordStartup(owner_.state().valid ? "owner_already_initialized" : "deadline_or_buffer",
        imu.stamp.receive_time,
        {u::Reason::LIDAR_STALE, "Startup snapshot retired without changing owner"});
  }
  if (startup_expired_) {
    if (ready) {
      RecordStartup("discarded", imu.stamp.receive_time, *startup_computed_);
      startup_.reset();
      startup_computed_.reset();
    }
    return;
  }
  startup_catchup_.push_back(imu);
  if (!ready) return;
  const auto computed = *startup_computed_;
  for (const auto& sample : startup_catchup_) {
    const auto replayed = startup_->ReplayImu(sample);
    if (!replayed.ok() && replayed.reason != u::Reason::WHEEL_STALE &&
        replayed.reason != u::Reason::CORRELATION_UNQUALIFIED &&
        replayed.reason != u::Reason::CORRECTION_BUDGET_EXCEEDED)
      throw std::runtime_error(replayed.message);
  }
  const auto committed = bootstrap_.CommitTask(startup_.get(), &owner_);
  if (committed.reason != computed.reason || committed.message != computed.message)
    throw std::runtime_error("startup commit rejected: " + committed.message);
  if (owner_.state().valid) initialized_time_ = startup_reference_;
  RecordStartup(owner_.state().valid ? "initialized" : "relative_history_retained",
                imu.stamp.receive_time, committed);
  startup_.reset();
  startup_computed_.reset();
  startup_catchup_.clear();
}

void NativeLioWorker::PollStartup() {
  if (!startup_) return;
  if (!startup_computed_ &&
      startup_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    startup_computed_ = startup_future_.get();
    const auto& result = *startup_computed_;
    if (!result.ok() && result.reason != u::Reason::HISTORY_UNAVAILABLE &&
        result.reason != u::Reason::GEOMETRY_UNAVAILABLE &&
        result.reason != u::Reason::DEGENERATE &&
        result.reason != u::Reason::MATCH_FAILED &&
        result.reason != u::Reason::INNOVATION_REJECTED)
      throw std::runtime_error(result.message);
  }
  const double receive = owner_.state().stamp.receive_time;
  if (!startup_expired_ &&
      std::chrono::duration<double>(Clock::now() - startup_started_).count() >
          startup_timeout_) {
    startup_expired_ = true;
    startup_catchup_.clear();
    RecordStartup("deadline_or_buffer", receive,
                  {u::Reason::LIDAR_STALE, "Startup exceeded its steady-clock deadline"});
  }
  if (startup_expired_ && startup_computed_) {
    RecordStartup("discarded", receive, *startup_computed_);
    startup_.reset();
    startup_computed_.reset();
  }
}

void NativeLioWorker::RecordStartup(
    const char* status, double receive, const NativePublication::Result& decision) {
  startup_journal_ << startup_sequence_ << ',' << startup_reference_ << ','
                   << startup_due_ << ',' << receive << ',' << status << ','
                   << u::ReasonName(decision.reason) << ','
                   << std::quoted(decision.message) << '\n';
  startup_journal_.flush();
  if (!startup_journal_) throw std::runtime_error("incomplete startup task journal");
}

NativePublication::Result NativeLioWorker::SubmitTask(const Scan& scan) {
  if (task_) {
    const u::Result busy{u::Reason::LIDAR_STALE,
                         "One native LIO snapshot is already outstanding"};
    return busy;
  }
  const auto prepared = owner_.PrepareLidarTask(scan, &task_);
  if (!prepared.ok()) return prepared;
  task_sequence_ = scan.stamp.sequence;
  graph_frontier_required_ = true;
  task_reference_ = scan.stamp.time;
  task_due_ = scan.stamp.receive_time + commit_delay_;
  task_expired_ = false;
  catchup_.clear();
  task_future_ = std::async(std::launch::async,
                            [task = task_.get()] { return task->Compute(); });
  const u::Result pending{u::Reason::HISTORY_UNAVAILABLE,
                          "Native LIO snapshot queued for delayed commit"};
  RecordTask("queued", scan.stamp.receive_time, pending);
  return pending;
}

void NativeLioWorker::AdvanceTask(const Imu& imu) {
  if (!task_) return;
  const bool ready = task_future_.wait_for(std::chrono::seconds(0)) ==
                     std::future_status::ready;
  if (task_expired_) {
    if (ready) {
      const auto decision = task_future_.get();
      RecordTask("discarded_after_deadline", imu.stamp.receive_time, decision);
      task_.reset();
    }
    return;
  }
  if (catchup_.size() >= 128)
    throw std::runtime_error("native LIO IMU catch-up buffer overflow");
  catchup_.push_back(imu);
  if (imu.stamp.receive_time < task_due_) return;
  if (!ready) {
    task_expired_ = true;
    catchup_.clear();
    RecordTask("deadline_missed", imu.stamp.receive_time,
               {u::Reason::LIDAR_STALE,
                "Native LIO task missed its source receive-time deadline"});
    return;
  }
  const auto computed = task_future_.get();
  if (!computed.ok() && computed.reason != u::Reason::GEOMETRY_UNAVAILABLE) {
    RecordTask("compute_rejected", imu.stamp.receive_time, computed);
    task_.reset();
    catchup_.clear();
    if (computed.reason != u::Reason::HISTORY_UNAVAILABLE &&
        computed.reason != u::Reason::LIDAR_STALE &&
        computed.reason != u::Reason::INNOVATION_REJECTED &&
        computed.reason != u::Reason::DEGENERATE &&
        computed.reason != u::Reason::MATCH_FAILED)
      throw std::runtime_error(computed.message);
    return;
  }
  for (const auto& sample : catchup_) {
    const auto replayed = task_->ReplayImu(sample);
    if (!replayed.ok() && replayed.reason != u::Reason::WHEEL_STALE &&
        replayed.reason != u::Reason::CORRELATION_UNQUALIFIED &&
        replayed.reason != u::Reason::CORRECTION_BUDGET_EXCEEDED)
      throw std::runtime_error(replayed.message);
  }
  const auto committed = owner_.CommitLidarTask(task_.get());
  if (graph_callback_ &&
      (committed.ok() || committed.reason == u::Reason::GEOMETRY_UNAVAILABLE))
    graph_scan_sequence_ = task_sequence_;
  RecordTask(
      committed.ok() || committed.reason == u::Reason::GEOMETRY_UNAVAILABLE
          ? "committed"
          : "commit_rejected",
      imu.stamp.receive_time, committed);
  task_.reset();
  catchup_.clear();
  if (!committed.ok() && committed.reason != u::Reason::GEOMETRY_UNAVAILABLE)
    throw std::runtime_error(committed.message);
}

void NativeLioWorker::FinishTask() {
  if (startup_) {
    if (startup_computed_)
      RecordStartup("end_of_input_uncommitted", 0, *startup_computed_);
    else if (startup_future_.valid())
      RecordStartup("end_of_input_uncommitted", 0, startup_future_.get());
    startup_.reset();
    startup_computed_.reset();
    startup_catchup_.clear();
  }
  if (task_) {
    if (task_future_.valid()) {
      const auto result = task_future_.get();
      RecordTask(task_expired_ ? "discarded_after_deadline"
                               : "end_of_input_uncommitted",
                 0, result);
    }
    task_.reset();
    catchup_.clear();
  }
  if (task_journal_.is_open()) {
    task_journal_.flush();
    if (!task_journal_) throw std::runtime_error("incomplete LIO task journal");
  }
}

void NativeLioWorker::RecordTask(const char* status, double receive,
                                 const NativePublication::Result& decision) {
  task_journal_ << task_sequence_ << ',' << task_reference_ << ',' << task_due_
                << ',' << receive << ',' << status << ','
                << u::ReasonName(decision.reason) << ','
                << std::quoted(decision.message) << '\n';
  task_journal_.flush();
  if (!task_journal_) throw std::runtime_error("incomplete LIO task journal");
}

void NativeLioWorker::Record(const Scan& scan, double reference, double receive,
                             const NativePublication::Result& decision) {
  journal_ << scan.stamp.sequence << ',' << scan.stamp.time << ','
           << scan.stamp.receive_time << ',' << reference << ',' << receive
           << ',' << u::ReasonName(decision.reason) << ','
           << std::quoted(decision.message) << '\n';
  journal_.flush();
  if (!journal_) throw std::runtime_error("incomplete native LIO journal");
}

}  // namespace apollo::localization::openloc_diagnostic
