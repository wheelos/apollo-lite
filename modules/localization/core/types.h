// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace apollo {
namespace localization {
namespace unified {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix15d = Eigen::Matrix<double, 15, 15>;

enum class Reason {
  NONE,
  CONFIG_INVALID,
  WAITING_FOR_STATIONARY,
  WAITING_FOR_IMU,
  IMU_STALE,
  WHEEL_STALE,
  INVALID_INPUT,
  DUPLICATE,
  TIMESTAMP_REGRESSION,
  CLOCK_INVALID,
  IMU_GAP,
  COVARIANCE_EXCEEDED,
  INNOVATION_REJECTED,
  HISTORY_UNAVAILABLE,
  EPOCH_MISMATCH,
  RESET_WHILE_MOVING,
  GLOBAL_UNAVAILABLE,
  GLOBAL_STALE,
  MAP_MISMATCH,
  MAP_NOT_READY,
  MATCH_FAILED,
  DEGENERATE,
  RELOCALIZATION_AMBIGUOUS,
  RELOCALIZATION_VERIFYING,
  LIDAR_STALE,
  POINT_TIME_MISSING,
  GEOMETRY_UNAVAILABLE,
  CORRECTION_BUDGET_EXCEEDED,
  CORRELATION_UNQUALIFIED,
};

struct Result {
  Reason reason = Reason::NONE;
  std::string message;
  bool ok() const { return reason == Reason::NONE; }
};

struct Stamp {
  double time = 0.0;
  double receive_time = 0.0;
  uint64_t sequence = 0;
  std::string clock_id;
};

struct Epoch {
  std::string session;
  uint64_t generation = 0;
  bool operator==(const Epoch& other) const {
    return session == other.session && generation == other.generation;
  }
  bool operator!=(const Epoch& other) const { return !(*this == other); }
};

struct ImuSample {
  Stamp stamp;
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

struct WheelSample {
  Stamp stamp;
  double speed = 0.0;
  double variance = 0.0;
};

struct TimedLidarPoint {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double time = 0.0;
};

struct LidarScan {
  Stamp stamp;
  Epoch epoch;
  std::string frame_id;
  std::string calibration_id;
  Eigen::Isometry3d base_from_lidar = Eigen::Isometry3d::Identity();
  std::vector<TimedLidarPoint> points;
};

struct LocalState {
  Stamp stamp;
  Epoch epoch;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Matrix15d covariance = Matrix15d::Zero();
  // True only while covariance/cross-time error claims follow the estimator's
  // declared error model. This is not deployment or accuracy qualification.
  bool covariance_model_valid = true;
  bool valid = false;
};

struct LocalConfig {
  std::string session;
  std::string clock_id = "unix";
  double gravity = 9.80665;
  double max_sample_age = 0.2;
  double future_tolerance = 0.02;
  double max_imu_gap = 0.05;
  double wheel_timeout = 0.2;
  double stationary_speed = 0.02;
  double stationary_gyro = 0.02;
  double stationary_accel_tolerance = 0.3;
  uint32_t stationary_samples = 50;
  double gyro_noise = 0.01;
  double accel_noise = 0.1;
  double gyro_bias_noise = 0.0001;
  double accel_bias_noise = 0.001;
  double initial_velocity_std = 0.05;
  double initial_attitude_std = 0.02;
  double initial_gyro_bias_std = 0.01;
  double initial_accel_bias_std = 0.1;
  double nonholonomic_variance = 0.01;
  bool use_nonholonomic_constraint = false;
  double innovation_gate = 11.345;
  double max_position_std = 1.0;
  double max_velocity_std = 0.5;
  double max_attitude_std = 0.1;
  double history_duration = 3.0;
  uint32_t max_history_states = 4096;
  bool enable_lidar = false;
  std::string lidar_frame;
  std::string lidar_calibration_id;
  double lidar_point_noise_std = 0.0;
  double lidar_plane_noise_std = 0.0;
  double lidar_voxel_size = 0.0;
  uint32_t lidar_max_voxels = 0;
  uint32_t lidar_max_scans = 0;
  uint32_t lidar_min_points_per_plane = 0;
  uint32_t lidar_min_correspondences = 0;
  uint32_t lidar_max_points = 0;
  double lidar_max_correspondence_distance = 0.0;
  double lidar_observed_singular_value = 0.0;
  double lidar_innovation_gate = 0.0;
  double lidar_max_scan_age = 0.0;
  double lidar_future_tolerance = 0.0;
  double lidar_time_tolerance = 0.0;
  double lidar_max_translation_correction_rate = 0.0;
  double lidar_max_rotation_correction_rate = 0.0;
  double lidar_max_velocity_correction_acceleration = 0.0;
  double lidar_anchor_variance_scale = 0.0;
  double lidar_correction_variance_scale = 0.0;
  double lidar_max_translation_correction = 0.0;
  double lidar_max_rotation_correction = 0.0;
  double lidar_max_velocity_correction = 0.0;
  uint32_t lidar_max_update_iterations = 0;
  double lidar_process_noise_psd_tolerance = 0.0;
  double lidar_rotation_length = 0.0;
};

struct SourceCounters {
  uint64_t accepted = 0;
  uint64_t rejected = 0;
  uint64_t duplicates = 0;
  uint64_t regressions = 0;
  uint64_t evictions = 0;
  double last_measurement = 0.0;
  double last_receive = 0.0;
  uint64_t last_sequence = 0;
  Reason reason = Reason::NONE;
};

Eigen::Isometry3d Pose(const LocalState& state);
Matrix6d PoseCovariance(const LocalState& state);
Eigen::Matrix3d Skew(const Eigen::Vector3d& value);
Eigen::Quaterniond ExpRotation(const Eigen::Vector3d& value);
Eigen::Vector3d LogRotation(const Eigen::Quaterniond& value);
bool ValidCovariance(const Matrix6d& covariance);
const char* ReasonName(Reason reason);

}  // namespace unified
}  // namespace localization
}  // namespace apollo
