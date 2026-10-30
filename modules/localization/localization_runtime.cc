// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "modules/localization/localization_runtime.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace apollo::localization::openloc {
namespace {

double UnixNow() {
  return std::chrono::duration<double>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

std::optional<uint32_t> SourceSequence(const common::Header& header) {
  return header.has_sequence_num()
             ? std::optional<uint32_t>(header.sequence_num())
             : std::nullopt;
}

}  // namespace

LocalizationRuntime::LocalizationRuntime(const LocalizationConfig& config)
    : config_(config),
      local_config_(host::LocalConfiguration(config.local())),
      publication_(local_config_.max_sample_age) {
  ValidateConfiguration(config);
  global_ = std::make_unique<host::NativeGlobalWorker>(config.global());
  ingress_ = std::make_unique<host::NativeSourceIngress>(
      config.local().journal() + ".source.csv",
      config.local().require_live_source_sequences());
  local_ = std::make_unique<host::NativeLioWorker>(
      local_config_, config.local().bootstrap_voxel_m(),
      config.local().journal(), [](const auto&, const auto&, auto) {},
      std::chrono::milliseconds(config.local().lidar_commit_delay_ms()),
      std::chrono::milliseconds(config.local().startup_task_timeout_ms()),
      host::NativeLioWorker::StartupOperation{},
      [this](const auto& frame, auto arrival) {
        return AddGraph(frame, arrival);
      },
      config.local().graph().odometry_stride());
}

LocalizationRuntime::~LocalizationRuntime() {
  local_.reset();
  global_.reset();
}

void LocalizationRuntime::Check() const {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!failure_.empty()) throw std::runtime_error(failure_);
  }
  local_->Check();
  global_->Check();
}

void LocalizationRuntime::Invalidate(const std::string& reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (failure_.empty()) failure_ = reason.empty() ? "runtime failure" : reason;
}

void LocalizationRuntime::AddImu(const drivers::gnss::Imu& input,
                                 double receive) {
  const auto arrival = host::NativePublication::Clock::now();
  std::lock_guard<std::mutex> serial(input_mutex_);
  Check();
  auto sample = DecodeImu(input, config_, receive, imu_sequence_ + 1);
  ingress_->Admit(host::NativeSourceIngress::Stream::kImu,
                  SourceSequence(input.header()), receive,
                  [&] { local_->AddImu(std::move(sample), arrival); });
  ++imu_sequence_;
}

void LocalizationRuntime::AddCloud(const drivers::PointCloud& input,
                                   double receive) {
  const auto arrival = host::NativePublication::Clock::now();
  std::lock_guard<std::mutex> lidar_serial(lidar_mutex_);
  Check();
  auto scan = DecodeCloud(input, config_, local_config_.lidar_max_points,
                          receive, lidar_sequence_ + 1);
  std::lock_guard<std::mutex> serial(input_mutex_);
  Check();
  ingress_->Admit(host::NativeSourceIngress::Stream::kLidar,
                  SourceSequence(input.header()), receive,
                  [&] { local_->AddLidar(std::move(scan), arrival); });
  ++lidar_sequence_;
}

void LocalizationRuntime::AddBestPose(const drivers::gnss::GnssBestPose& input,
                                      double receive) {
  Check();
  global_->AddBestPose({input, host::NativeMapMatcher::Nanoseconds(receive)},
                       host::NativePublication::Clock::now());
}

void LocalizationRuntime::AddHeading(const drivers::gnss::Heading& input,
                                     double receive) {
  Check();
  global_->AddHeading({input, host::NativeMapMatcher::Nanoseconds(receive)},
                      host::NativePublication::Clock::now());
}

bool LocalizationRuntime::AddGraph(
    const host::NativeGraphFrame& frame,
    host::NativePublication::Clock::time_point arrival) {
  publication_.Update(frame.nominal, frame.integrity, arrival);
  auto source = publication_.Sample(UnixNow());
  source.set_recorded_input(false);
  const auto model_id = config_.local().graph().motion_error_model_id();
  auto packet =
      source.usable()
          ? host::EncodeNativeGraph(frame, source, model_id)
          : host::UnavailableNativeGraph(source, model_id, source.detail());
  const bool available = packet.model_available() && source.usable();
  if (available) {
    native::LocalState state;
    state.stamp = frame.nominal.stamp;
    state.epoch = frame.nominal.epoch;
    state.velocity = frame.nominal.velocity;
    state.angular_velocity = frame.nominal.angular_velocity;
    state.valid = frame.nominal.valid;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!failure_.empty()) return false;
    kinematics_.push_back(std::move(state));
    while (kinematics_.size() > 4096 ||
           frame.nominal.stamp.time - kinematics_.front().stamp.time > 30) {
      kinematics_.pop_front();
    }
  }
  global_->AddGraphOdometry(std::move(packet),
                            host::NativePublication::Clock::now());
  return available;
}

Publication LocalizationRuntime::Sample(double now, uint32_t sequence) {
  try {
    Check();
    const auto global = global_->Sample(now);
    std::optional<native::LocalState> state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto found = std::find_if(
          kinematics_.rbegin(), kinematics_.rend(), [&](const auto& candidate) {
            return candidate.stamp.sequence == global.odometry_sequence() &&
                   candidate.epoch.session == global.session() &&
                   candidate.epoch.generation == global.generation();
          });
      if (found != kinematics_.rend()) state = *found;
    }
    return MakePublication(config_, global, state ? &*state : nullptr, now,
                           sequence);
  } catch (const std::exception& error) {
    Invalidate(error.what());
    host::NativeGlobalDiagnostic unavailable;
    unavailable.set_reason(error.what());
    return MakePublication(config_, unavailable, nullptr, now, sequence);
  }
}

}  // namespace apollo::localization::openloc
