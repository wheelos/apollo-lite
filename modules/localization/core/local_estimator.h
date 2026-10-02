// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <cstddef>
#include <deque>

#include "modules/localization/core/continuous_output.h"
#include "modules/localization/core/local_lio.h"
#include "modules/localization/core/local_motion.h"

namespace apollo {
namespace localization {
namespace unified {

// The local owner deliberately has no absolute-pose or global-reset input.
class LocalEstimator {
 public:
  explicit LocalEstimator(const LocalConfig& config);
  Result ValidateConfig() const;
  Result AddImu(const ImuSample& sample);
  Result AddWheel(const WheelSample& sample);
  Result AddLidar(const LidarScan& scan);
  Result ResetStopped(double now);
  Result EvaluateIntegrity(double now) const;
  Result Evaluate(double now) const;
  // valid reports initialized, fresh, numerically sound propagation. Precision
  // and wheel-aided accuracy remain the separate Evaluate policy.
  const LocalState& state() const { return output_state_; }
  const LocalState& internal_state() const { return state_; }
  const SourceCounters& imu_health() const { return imu_health_; }
  const SourceCounters& wheel_health() const { return wheel_health_; }
  const SourceCounters& lidar_health() const { return lidar_health_; }
  // Null means no qualified JOINT_LOCAL_MARGINAL is available. In particular,
  // LiDAR-active continuous output remains publishable but motion-ineligible.
  const MotionIncrement* last_motion() const {
    return has_motion_ ? &last_motion_ : nullptr;
  }
  const Result& motion_status() const { return motion_status_; }
  // Exact committed measurement times only; no nearest-state substitution.
  Result Lookup(double time, LocalState* state) const;
  Result MotionBetween(double start_time, double end_time,
                       MotionIncrement* motion) const;
  const Eigen::MatrixXd& lidar_observed_pose_basis() const {
    return lio_.observed_pose_basis();
  }

 private:
  static constexpr std::size_t kMaxPendingWheelSamples = 64;

  Result Reject(Reason reason, const std::string& message,
                SourceCounters* counters);
  Result Initialize(const ImuSample& sample);
  Result Propagate(const ImuSample& start, const ImuSample& end,
                   LocalState* state, Matrix15d* error_transition) const;
  Result UpdateWheel(const WheelSample& sample, const ImuSample& imu,
                     LocalState* state, Matrix15d* error_transition);
  void ConsumeStationaryWheels(double time);
  bool IsStationary(const ImuSample& sample) const;
  void AccumulateStationary(const ImuSample& sample, bool reset);
  void RecordHistory(const Matrix15d& transition = Matrix15d::Identity(),
                     const std::vector<SourceSampleId>& sources = {},
                     bool motion_qualified = true);
  void RecordInternalHistory();
  Result LookupInternal(double time, LocalState* state) const;

  struct HistoryEntry {
    LocalState state;
    Matrix15d transition;
    std::vector<SourceSampleId> sources;
    bool motion_qualified = true;
  };

  LocalConfig config_;
  LocalState state_;
  LocalState output_state_;
  SourceCounters imu_health_;
  SourceCounters wheel_health_;
  SourceCounters lidar_health_;
  WheelSample last_received_wheel_;
  WheelSample stationary_wheel_;
  WheelSample last_trusted_wheel_;
  ImuSample previous_imu_;
  bool initialized_ = false;
  bool faulted_ = false;
  bool has_imu_ = false;
  bool has_received_wheel_ = false;
  bool has_stationary_wheel_ = false;
  bool has_trusted_wheel_ = false;
  uint64_t last_imu_sequence_ = 0;
  uint64_t last_wheel_sequence_ = 0;
  uint32_t stationary_count_ = 0;
  Eigen::Vector3d stationary_acceleration_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d stationary_rotation_ = Eigen::Vector3d::Zero();
  std::deque<WheelSample> pending_wheels_;
  std::deque<HistoryEntry> history_;
  std::deque<LocalState> internal_history_;
  LocalLio lio_;
  ContinuousOutput continuous_output_;
  bool lio_active_ = false;
  std::vector<SourceSampleId> pending_lidar_sources_;
  MotionIncrement last_motion_;
  bool has_motion_ = false;
  Result motion_status_{Reason::HISTORY_UNAVAILABLE,
                        "Two committed local states are required"};
};

}  // namespace unified
}  // namespace localization
}  // namespace apollo
