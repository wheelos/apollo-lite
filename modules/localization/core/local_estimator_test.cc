// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/core/local_estimator.h"

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "Eigen/Cholesky"
#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

LocalConfig TestConfig() {
  LocalConfig config;
  config.session = "local-test-session";
  config.clock_id = "test-clock";
  config.gravity = 10.0;
  config.stationary_samples = 3;
  config.max_sample_age = 0.1;
  config.future_tolerance = 0.001;
  config.max_imu_gap = 0.025;
  config.wheel_timeout = 0.05;
  config.stationary_speed = 0.01;
  config.stationary_gyro = 0.03;
  config.stationary_accel_tolerance = 0.1;
  config.gyro_noise = 0.001;
  config.accel_noise = 0.01;
  config.gyro_bias_noise = 0.00001;
  config.accel_bias_noise = 0.0001;
  config.initial_velocity_std = 0.1;
  config.initial_attitude_std = 0.01;
  config.initial_gyro_bias_std = 0.001;
  config.initial_accel_bias_std = 0.01;
  config.max_position_std = 10.0;
  config.max_velocity_std = 10.0;
  config.max_attitude_std = 1.0;
  config.innovation_gate = 100.0;
  return config;
}

void EnableTestLidar(LocalConfig* config) {
  config->enable_lidar = true;
  config->lidar_frame = "lidar";
  config->lidar_calibration_id = "test-lidar-calibration";
  config->lidar_point_noise_std = 0.01;
  config->lidar_plane_noise_std = 0.01;
  config->lidar_voxel_size = 1.0;
  config->lidar_max_voxels = 100;
  config->lidar_max_scans = 3;
  config->lidar_min_points_per_plane = 3;
  config->lidar_min_correspondences = 2;
  config->lidar_max_points = 1000;
  config->lidar_max_correspondence_distance = 0.3;
  config->lidar_observed_singular_value = 1.0;
  config->lidar_innovation_gate = 100.0;
  config->lidar_max_scan_age = 0.1;
  config->lidar_future_tolerance = 0.001;
  config->lidar_time_tolerance = 0.0;
  config->lidar_max_translation_correction_rate = 0.2;
  config->lidar_max_rotation_correction_rate = 0.1;
  config->lidar_max_velocity_correction_acceleration = 0.3;
  config->lidar_anchor_variance_scale = 1.0;
  config->lidar_correction_variance_scale = 2.0;
  config->lidar_max_translation_correction = 2.0;
  config->lidar_max_rotation_correction = 0.5;
  config->lidar_max_velocity_correction = 2.0;
  config->lidar_max_update_iterations = 2;
  config->lidar_process_noise_psd_tolerance = 1e-12;
  config->lidar_rotation_length = 1.0;
}

LidarScan CorridorScan(const LocalState& state, uint64_t sequence,
                       double lateral_offset = 0.0) {
  LidarScan scan;
  scan.stamp = state.stamp;
  scan.stamp.sequence = sequence;
  scan.epoch = state.epoch;
  scan.frame_id = "lidar";
  scan.calibration_id = "test-lidar-calibration";
  for (double x : {0.1, 0.3, 0.5}) {
    for (double z : {0.1, 0.3, 0.5}) {
      scan.points.push_back(
          {Eigen::Vector3d(x, 1.0 + lateral_offset, z), state.stamp.time});
      scan.points.push_back(
          {Eigen::Vector3d(x, -1.0 + lateral_offset, z), state.stamp.time});
    }
  }
  return scan;
}

Stamp MakeStamp(double time, uint64_t sequence) {
  Stamp stamp;
  stamp.time = time;
  stamp.receive_time = time;
  stamp.sequence = sequence;
  stamp.clock_id = "test-clock";
  return stamp;
}

ImuSample MakeImu(double time, uint64_t sequence) {
  ImuSample sample;
  sample.stamp = MakeStamp(time, sequence);
  sample.acceleration.z() = 10.0;
  return sample;
}

WheelSample MakeWheel(double time, uint64_t sequence, double speed = 0.0) {
  WheelSample sample;
  sample.stamp = MakeStamp(time, sequence);
  sample.speed = speed;
  sample.variance = 0.01;
  return sample;
}

void ExpectSameState(const LocalState& before, const LocalState& after) {
  EXPECT_EQ(before.stamp.time, after.stamp.time);
  EXPECT_EQ(before.stamp.receive_time, after.stamp.receive_time);
  EXPECT_EQ(before.stamp.sequence, after.stamp.sequence);
  EXPECT_EQ(before.stamp.clock_id, after.stamp.clock_id);
  EXPECT_TRUE(before.epoch == after.epoch);
  EXPECT_EQ((before.position - after.position).norm(), 0.0);
  EXPECT_EQ((before.velocity - after.velocity).norm(), 0.0);
  EXPECT_EQ((before.orientation.coeffs() - after.orientation.coeffs()).norm(),
            0.0);
  EXPECT_EQ((before.gyro_bias - after.gyro_bias).norm(), 0.0);
  EXPECT_EQ((before.accel_bias - after.accel_bias).norm(), 0.0);
  EXPECT_EQ((before.angular_velocity - after.angular_velocity).norm(), 0.0);
  EXPECT_EQ((before.covariance - after.covariance).norm(), 0.0);
  EXPECT_EQ(before.covariance_model_valid,
            after.covariance_model_valid);
  EXPECT_EQ(before.valid, after.valid);
}

void ExpectPositiveCovariance(const LocalState& state) {
  EXPECT_TRUE(state.covariance.allFinite());
  EXPECT_TRUE(state.covariance.isApprox(state.covariance.transpose(), 1e-12));
  Eigen::LLT<Matrix15d> factor(state.covariance);
  EXPECT_EQ(factor.info(), Eigen::Success);
  for (int i = 0; i < 15; ++i) {
    EXPECT_GT(state.covariance(i, i), 0.0);
  }
}

class LocalEstimatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_ = TestConfig();
    estimator_.reset(new LocalEstimator(config_));
  }

  void Initialize() {
    for (uint64_t sequence = 1; sequence <= config_.stationary_samples;
         ++sequence) {
      const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
      ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
      const Result result = estimator_->AddImu(MakeImu(time, sequence));
      ASSERT_TRUE(result.ok()) << result.message;
      if (sequence < config_.stationary_samples) {
        ASSERT_EQ(estimator_->Evaluate(time).reason,
                  Reason::WAITING_FOR_STATIONARY);
      }
    }
    ASSERT_TRUE(estimator_->state().valid);
  }

  LocalConfig config_;
  std::unique_ptr<LocalEstimator> estimator_;
};

TEST(LocalEstimatorConfigTest, RejectsMissingInvalidAndUnrepresentableConfig) {
  std::vector<LocalConfig> invalid;
  LocalConfig config = TestConfig();
  config.session.clear();
  invalid.push_back(config);
  config = TestConfig();
  config.clock_id.clear();
  invalid.push_back(config);
  config = TestConfig();
  config.stationary_samples = 0;
  invalid.push_back(config);
  config = TestConfig();
  config.gravity = 0.0;
  invalid.push_back(config);
  config = TestConfig();
  config.max_imu_gap = -1.0;
  invalid.push_back(config);
  config = TestConfig();
  config.future_tolerance = -1.0;
  invalid.push_back(config);
  config = TestConfig();
  config.gyro_noise = std::numeric_limits<double>::quiet_NaN();
  invalid.push_back(config);
  config = TestConfig();
  config.accel_noise = std::numeric_limits<double>::max();
  invalid.push_back(config);
  config = TestConfig();
  config.initial_attitude_std = 0.0;
  invalid.push_back(config);
  config = TestConfig();
  config.initial_velocity_std = config.max_velocity_std + 1.0;
  invalid.push_back(config);
  config = TestConfig();
  config.stationary_accel_tolerance = config.gravity;
  invalid.push_back(config);
  config = TestConfig();
  config.nonholonomic_variance = 0.0;
  invalid.push_back(config);
  config = TestConfig();
  config.wheel_timeout = std::numeric_limits<double>::infinity();
  invalid.push_back(config);
  config = TestConfig();
  config.history_duration = 0.0;
  invalid.push_back(config);
  config = TestConfig();
  config.max_history_states = 1;
  invalid.push_back(config);
  config = TestConfig();
  config.enable_lidar = true;
  invalid.push_back(config);
  config = TestConfig();
  EnableTestLidar(&config);
  config.lidar_time_tolerance = 1e-6;
  invalid.push_back(config);
  for (const auto& item : invalid) {
    LocalEstimator estimator(item);
    EXPECT_EQ(estimator.ValidateConfig().reason, Reason::CONFIG_INVALID);
    EXPECT_EQ(estimator.AddImu(MakeImu(1.0, 1)).reason, Reason::CONFIG_INVALID);
    EXPECT_EQ(estimator.AddWheel(MakeWheel(1.0, 1)).reason,
              Reason::CONFIG_INVALID);
    EXPECT_EQ(estimator.EvaluateIntegrity(1.0).reason,
              Reason::CONFIG_INVALID);
    EXPECT_EQ(estimator.Evaluate(1.0).reason, Reason::CONFIG_INVALID);
    EXPECT_EQ(estimator.ResetStopped(1.0).reason, Reason::CONFIG_INVALID);
    EXPECT_FALSE(estimator.state().valid);
  }
  EXPECT_TRUE(LocalEstimator(TestConfig()).ValidateConfig().ok());
}

TEST_F(LocalEstimatorTest, StationaryStartupNeedsWheelAndConsecutiveSamples) {
  EXPECT_EQ(estimator_->Evaluate(1.0).reason, Reason::WAITING_FOR_IMU);
  EXPECT_EQ(estimator_->EvaluateIntegrity(1.0).reason, Reason::WAITING_FOR_IMU);
  EXPECT_TRUE(estimator_->AddImu(MakeImu(1.0, 1)).ok());
  EXPECT_EQ(estimator_->Evaluate(1.0).reason, Reason::WAITING_FOR_STATIONARY);
  EXPECT_EQ(estimator_->EvaluateIntegrity(1.0).reason,
            Reason::WAITING_FOR_STATIONARY);
  EXPECT_EQ(estimator_->state().stamp.time, 1.0);
  EXPECT_FALSE(estimator_->state().valid);
  for (uint64_t sequence = 2; sequence <= 3; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    EXPECT_TRUE(estimator_->AddImu(MakeImu(time, sequence)).ok());
    EXPECT_EQ(estimator_->Evaluate(time).reason,
              Reason::WAITING_FOR_STATIONARY);
  }
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4, 0.1)).ok());
  EXPECT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
  EXPECT_EQ(estimator_->Evaluate(1.03).reason,
            Reason::WAITING_FOR_STATIONARY);
  for (uint64_t sequence = 5; sequence <= 7; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    const Result result = estimator_->AddImu(MakeImu(time, sequence));
    EXPECT_TRUE(result.ok());
    EXPECT_EQ(estimator_->Evaluate(time).reason,
              sequence < 7 ? Reason::WAITING_FOR_STATIONARY : Reason::NONE);
  }
  EXPECT_TRUE(estimator_->state().valid);
  EXPECT_EQ(estimator_->state().position.norm(), 0.0);
  EXPECT_EQ(estimator_->state().velocity.norm(), 0.0);
  EXPECT_EQ(estimator_->state().epoch.session, config_.session);
  EXPECT_EQ(estimator_->state().epoch.generation, 1U);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest, MotionContainsOnlyAppliedSamplesAndJointUncertainty) {
  Initialize();
  EXPECT_EQ(nullptr, estimator_->last_motion());
  const auto start = estimator_->state();
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.025, 4, 1000.0)).ok());
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.027, 5)).ok());
  EXPECT_EQ(Reason::INNOVATION_REJECTED,
            estimator_->AddImu(MakeImu(1.03, 4)).reason);
  ASSERT_TRUE(estimator_->state().valid);
  ASSERT_TRUE(estimator_->motion_status().ok())
      << estimator_->motion_status().message;
  const auto* motion = estimator_->last_motion();
  ASSERT_NE(nullptr, motion);
  EXPECT_EQ(start.stamp.time, motion->start.time);
  EXPECT_EQ(1.03, motion->end.time);
  EXPECT_TRUE(motion->start_covariance.isApprox(start.covariance));
  EXPECT_TRUE(motion->end_covariance.isApprox(estimator_->state().covariance));
  ASSERT_EQ(3U, motion->sources.size());
  EXPECT_EQ("imu", motion->sources[0].source);
  EXPECT_EQ(3U, motion->sources[0].sequence);
  EXPECT_EQ("imu", motion->sources[1].source);
  EXPECT_EQ(4U, motion->sources[1].sequence);
  EXPECT_EQ("wheel", motion->sources[2].source);
  EXPECT_EQ(5U, motion->sources[2].sequence);
  EXPECT_EQ(Reason::IMU_GAP,
            estimator_->AddImu(MakeImu(1.2, 5)).reason);
  EXPECT_EQ(nullptr, estimator_->last_motion());
}

TEST_F(LocalEstimatorTest,
       LocalLidarCorrectsInternalSubspaceWithoutSameTimeOdomJump) {
  EnableTestLidar(&config_);
  estimator_.reset(new LocalEstimator(config_));
  Initialize();
  EXPECT_EQ(estimator_->AddLidar(
                CorridorScan(estimator_->internal_state(), 1)).reason,
            Reason::GEOMETRY_UNAVAILABLE);
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4)).ok());
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
  const LocalState published_before = estimator_->state();
  ASSERT_TRUE(estimator_->AddLidar(
      CorridorScan(estimator_->internal_state(), 2, 0.1)).ok());
  EXPECT_FALSE(estimator_->internal_state().covariance_model_valid);
  EXPECT_FALSE(estimator_->state().covariance_model_valid);
  EXPECT_EQ(estimator_->Evaluate(estimator_->state().stamp.time).reason,
            Reason::CORRELATION_UNQUALIFIED);
  EXPECT_EQ(estimator_->last_motion(), nullptr);
  EXPECT_NE(estimator_->internal_state().position.y(),
            published_before.position.y());
  EXPECT_DOUBLE_EQ(estimator_->state().position.y(),
                   published_before.position.y());
  EXPECT_LT(estimator_->lidar_observed_pose_basis().rows(), 6);
  EXPECT_LT(estimator_->lidar_observed_pose_basis().col(0).norm(), 1e-10);

  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.04, 5)).ok());
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.04, 5)).reason,
            Reason::CORRELATION_UNQUALIFIED);
  ASSERT_TRUE(estimator_->state().valid);
  EXPECT_FALSE(estimator_->state().covariance_model_valid);
  EXPECT_LE(std::abs(estimator_->state().position.y() -
                     published_before.position.y()),
            config_.lidar_max_translation_correction_rate * 0.01 + 1e-12);
  EXPECT_GE(estimator_->state().covariance(1, 1),
            published_before.covariance(1, 1));
  EXPECT_EQ(estimator_->last_motion(), nullptr);
  EXPECT_EQ(estimator_->motion_status().reason,
            Reason::CORRELATION_UNQUALIFIED);
  MotionIncrement unqualified;
  EXPECT_EQ(estimator_->MotionBetween(
                published_before.stamp.time, estimator_->state().stamp.time,
                &unqualified).reason,
            Reason::CORRELATION_UNQUALIFIED);
}

TEST_F(LocalEstimatorTest, StaleLidarIsRejectedAndStoppedResetClearsGeometry) {
  EnableTestLidar(&config_);
  estimator_.reset(new LocalEstimator(config_));
  Initialize();
  LidarScan stale = CorridorScan(estimator_->internal_state(), 1);
  stale.stamp.receive_time =
      stale.stamp.time + config_.lidar_max_scan_age + 0.01;
  EXPECT_EQ(estimator_->AddLidar(stale).reason, Reason::LIDAR_STALE);
  EXPECT_EQ(estimator_->lidar_health().accepted, 0U);
  LidarScan delayed = CorridorScan(estimator_->internal_state(), 2);
  delayed.stamp.time -= 1e-9;
  for (auto& point : delayed.points) {
    point.time = delayed.stamp.time;
  }
  EXPECT_EQ(estimator_->AddLidar(delayed).reason, Reason::LIDAR_STALE);

  EXPECT_EQ(estimator_->AddLidar(
                CorridorScan(estimator_->internal_state(), 3)).reason,
            Reason::GEOMETRY_UNAVAILABLE);
  EXPECT_EQ(Reason::IMU_GAP,
            estimator_->AddImu(MakeImu(1.2, 4)).reason);
  for (uint64_t sequence = 5; sequence <= 7; ++sequence) {
    const double time = 1.21 + 0.01 * static_cast<double>(sequence - 5);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    EXPECT_EQ(estimator_->AddImu(MakeImu(time, sequence)).reason,
              Reason::IMU_GAP);
  }
  ASSERT_TRUE(estimator_->ResetStopped(1.24).ok());
  EXPECT_EQ(estimator_->state().epoch.generation, 2U);
  EXPECT_TRUE(estimator_->state().covariance_model_valid);
  for (uint64_t sequence = 8; sequence <= 10; ++sequence) {
    const double time = 1.25 + 0.01 * static_cast<double>(sequence - 8);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    ASSERT_TRUE(estimator_->AddImu(MakeImu(time, sequence)).ok());
  }
  ASSERT_TRUE(estimator_->state().valid);
  EXPECT_TRUE(estimator_->state().covariance_model_valid);
  EXPECT_EQ(estimator_->AddLidar(
                CorridorScan(estimator_->internal_state(), 4)).reason,
            Reason::GEOMETRY_UNAVAILABLE);
}

TEST_F(LocalEstimatorTest, BoundedHistoryRejectsInterpolationAndResetCrossing) {
  config_.max_history_states = 3;
  estimator_.reset(new LocalEstimator(config_));
  Initialize();
  for (uint64_t sequence = 4; sequence <= 7; ++sequence) {
    const double time = 1.02 + 0.01 * static_cast<double>(sequence - 3);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    ASSERT_TRUE(estimator_->AddImu(MakeImu(time, sequence)).ok());
  }
  LocalState state;
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE, estimator_->Lookup(1.02, &state).reason);
  const double start_time = 1.02 + 0.01 * 2;
  const double end_time = 1.02 + 0.01 * 4;
  ASSERT_TRUE(estimator_->Lookup(start_time, &state).ok());
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE,
            estimator_->Lookup(start_time + 0.005, &state).reason);
  MotionIncrement motion;
  ASSERT_TRUE(estimator_->MotionBetween(start_time, end_time, &motion).ok());
  EXPECT_EQ(5U, motion.start.sequence);
  EXPECT_EQ(7U, motion.end.sequence);
  EXPECT_EQ(5U, motion.sources.size());  // Three IMU samples, two wheel samples.
  const auto* last = estimator_->last_motion();
  ASSERT_NE(nullptr, last);
  MotionIncrement one_step;
  ASSERT_TRUE(estimator_->MotionBetween(last->start.time, last->end.time,
                                        &one_step).ok());
  EXPECT_TRUE(one_step.cross_covariance.isApprox(last->cross_covariance));
  EXPECT_TRUE(one_step.covariance.isApprox(last->covariance));
  // Gather fresh stopped evidence after an explicit integrity fault.
  EXPECT_EQ(Reason::IMU_GAP, estimator_->AddImu(MakeImu(1.2, 8)).reason);
  for (uint64_t sequence = 9; sequence <= 11; ++sequence) {
    const double time = 1.21 + 0.01 * static_cast<double>(sequence - 9);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    EXPECT_EQ(Reason::IMU_GAP,
              estimator_->AddImu(MakeImu(time, sequence)).reason);
  }
  ASSERT_TRUE(estimator_->ResetStopped(1.24).ok());
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE,
            estimator_->MotionBetween(start_time, end_time, &motion).reason);
}

TEST_F(LocalEstimatorTest, TiltAlignmentUsesConfiguredGravityAndGyroAverage) {
  const Eigen::Quaterniond tilt =
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d body_gravity =
      tilt.conjugate() * Eigen::Vector3d(0.0, 0.0, config_.gravity);
  const Eigen::Vector3d bias(0.01, -0.01, 0.005);
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    ImuSample imu = MakeImu(time, sequence);
    imu.acceleration = body_gravity;
    imu.angular_velocity = bias;
    const Result result = estimator_->AddImu(imu);
    EXPECT_TRUE(result.ok());
    EXPECT_EQ(estimator_->Evaluate(time).reason,
              sequence < 3 ? Reason::WAITING_FOR_STATIONARY : Reason::NONE);
  }
  EXPECT_LT(LogRotation(tilt.conjugate() * estimator_->state().orientation).norm(),
            1e-12);
  EXPECT_LT((estimator_->state().gyro_bias - bias).norm(), 1e-12);
  EXPECT_EQ(estimator_->state().accel_bias.norm(), 0.0);
  // No GNSS is involved: translation and yaw are gauge choices, not observed.
  EXPECT_EQ(estimator_->state().position.norm(), 0.0);
  EXPECT_NEAR(estimator_->state().orientation.toRotationMatrix()(1, 0), 0.0,
              1e-12);
  for (uint64_t sequence = 4; sequence <= 13; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    ImuSample imu = MakeImu(time, sequence);
    imu.acceleration = body_gravity;
    imu.angular_velocity = bias;
    ASSERT_TRUE(estimator_->AddImu(imu).ok());
  }
  EXPECT_LT(estimator_->state().velocity.norm(), 1e-12);
  EXPECT_LT(estimator_->state().position.norm(), 1e-12);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest, StartupGyroAndAccelerationMotionBreakStationaryRun) {
  for (uint64_t sequence = 1; sequence <= 9; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    ImuSample imu = MakeImu(time, sequence);
    if (sequence == 3) {
      imu.angular_velocity.z() = 0.5;
    }
    if (sequence == 6) {
      imu.acceleration.z() = 11.0;
    }
    const Result result = estimator_->AddImu(imu);
    EXPECT_TRUE(result.ok());
    EXPECT_EQ(estimator_->Evaluate(time).reason,
              sequence < 9 ? Reason::WAITING_FOR_STATIONARY : Reason::NONE);
    EXPECT_EQ(estimator_->state().valid, sequence == 9);
  }
  EXPECT_EQ(estimator_->state().gyro_bias.norm(), 0.0);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest, ExactDuplicateImuDoesNotPropagateOrAlterState) {
  Initialize();
  const LocalState before = estimator_->state();
  const uint64_t accepted = estimator_->imu_health().accepted;
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.02, 3)).reason, Reason::DUPLICATE);
  ExpectSameState(before, estimator_->state());
  EXPECT_EQ(estimator_->imu_health().accepted, accepted);
  EXPECT_EQ(estimator_->imu_health().rejected, 1U);
  EXPECT_EQ(estimator_->imu_health().duplicates, 1U);
  EXPECT_TRUE(estimator_->Evaluate(1.02).ok());
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
  EXPECT_EQ(estimator_->state().stamp.sequence, 4U);
  EXPECT_TRUE(estimator_->Evaluate(1.03).ok());
}

TEST_F(LocalEstimatorTest, InvalidImuDoesNotAdvanceWatermarkOrPose) {
  Initialize();
  const LocalState before = estimator_->state();
  std::vector<ImuSample> samples;
  std::vector<Reason> reasons;
  ImuSample sample = MakeImu(1.03, 4);
  sample.stamp.clock_id.clear();
  samples.push_back(sample);
  reasons.push_back(Reason::CLOCK_INVALID);
  sample = MakeImu(1.03, 4);
  sample.stamp.time = 0.0;
  samples.push_back(sample);
  reasons.push_back(Reason::INVALID_INPUT);
  sample = MakeImu(1.03, 4);
  sample.stamp.receive_time = 0.0;
  samples.push_back(sample);
  reasons.push_back(Reason::INVALID_INPUT);
  sample = MakeImu(1.03, 0);
  samples.push_back(sample);
  reasons.push_back(Reason::INVALID_INPUT);
  sample = MakeImu(1.03, 4);
  sample.stamp.receive_time = std::numeric_limits<double>::quiet_NaN();
  samples.push_back(sample);
  reasons.push_back(Reason::INVALID_INPUT);
  sample = MakeImu(1.03, 4);
  sample.stamp.time = std::numeric_limits<double>::infinity();
  samples.push_back(sample);
  reasons.push_back(Reason::INVALID_INPUT);
  sample = MakeImu(1.03, 4);
  sample.stamp.receive_time = 1.0;
  samples.push_back(sample);
  reasons.push_back(Reason::INVALID_INPUT);
  sample = MakeImu(1.03, 4);
  sample.stamp.receive_time = 1.2;
  samples.push_back(sample);
  reasons.push_back(Reason::IMU_STALE);
  sample = MakeImu(1.01, 4);
  samples.push_back(sample);
  reasons.push_back(Reason::TIMESTAMP_REGRESSION);
  sample = MakeImu(1.03, 2);
  samples.push_back(sample);
  reasons.push_back(Reason::TIMESTAMP_REGRESSION);
  sample = MakeImu(1.03, 4);
  sample.stamp.receive_time = 1.0195;
  sample.stamp.time = 1.02;
  samples.push_back(sample);
  reasons.push_back(Reason::TIMESTAMP_REGRESSION);
  sample = MakeImu(1.02, 4);
  samples.push_back(sample);
  reasons.push_back(Reason::DUPLICATE);
  sample = MakeImu(1.03, 4);
  sample.acceleration.x() = std::numeric_limits<double>::quiet_NaN();
  samples.push_back(sample);
  reasons.push_back(Reason::INVALID_INPUT);
  sample = MakeImu(1.03, 4);
  sample.angular_velocity.z() = std::numeric_limits<double>::infinity();
  samples.push_back(sample);
  reasons.push_back(Reason::INVALID_INPUT);
  for (size_t i = 0; i < samples.size(); ++i) {
    EXPECT_EQ(estimator_->AddImu(samples[i]).reason, reasons[i]) << i;
    ExpectSameState(before, estimator_->state());
    EXPECT_EQ(estimator_->imu_health().accepted, 3U);
    EXPECT_TRUE(estimator_->Evaluate(1.03).ok());
  }
  EXPECT_EQ(estimator_->imu_health().rejected, samples.size());
  EXPECT_EQ(estimator_->imu_health().regressions, 3U);
  EXPECT_EQ(estimator_->imu_health().last_measurement, 1.02);
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
  EXPECT_EQ(estimator_->state().stamp.time, 1.03);
}

TEST_F(LocalEstimatorTest, WheelRejectsDuplicateRegressionClockAndInvalidFields) {
  Initialize();
  const LocalState before = estimator_->state();
  EXPECT_EQ(estimator_->AddWheel(MakeWheel(1.02, 3)).reason, Reason::DUPLICATE);
  EXPECT_EQ(estimator_->AddWheel(MakeWheel(1.01, 4)).reason,
            Reason::TIMESTAMP_REGRESSION);
  EXPECT_EQ(estimator_->AddWheel(MakeWheel(1.03, 2)).reason,
            Reason::TIMESTAMP_REGRESSION);
  WheelSample wheel = MakeWheel(1.03, 4);
  wheel.stamp.clock_id = "another-clock";
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::CLOCK_INVALID);
  wheel = MakeWheel(1.03, 0);
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::INVALID_INPUT);
  wheel = MakeWheel(1.03, 4);
  wheel.stamp.receive_time = 0.0;
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::INVALID_INPUT);
  wheel = MakeWheel(1.03, 4);
  wheel.stamp.receive_time = 1.0;
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::INVALID_INPUT);
  wheel = MakeWheel(1.03, 4);
  wheel.stamp.receive_time = 1.2;
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::WHEEL_STALE);
  wheel = MakeWheel(1.03, 4);
  wheel.stamp.receive_time = 1.09;
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::WHEEL_STALE);
  wheel = MakeWheel(1.03, 4);
  wheel.speed = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::INVALID_INPUT);
  wheel = MakeWheel(1.03, 4);
  wheel.variance = -0.01;
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::INVALID_INPUT);
  wheel.variance = 0.0;
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::INVALID_INPUT);
  wheel.variance = std::numeric_limits<double>::infinity();
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::INVALID_INPUT);
  ExpectSameState(before, estimator_->state());
  EXPECT_EQ(estimator_->wheel_health().accepted, 3U);
  EXPECT_EQ(estimator_->wheel_health().duplicates, 1U);
  EXPECT_EQ(estimator_->wheel_health().regressions, 2U);
  EXPECT_TRUE(estimator_->Evaluate(1.03).ok());
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4)).ok());
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
}

TEST_F(LocalEstimatorTest, StraightAccelerationAndWheelVelocityRemainContinuous) {
  Initialize();
  // Midpoint from the last stationary sample to the first accelerating sample.
  for (uint64_t sequence = 4; sequence <= 103; ++sequence) {
    const double elapsed = 0.01 * static_cast<double>(sequence - 3);
    const double time = 1.02 + elapsed;
    ASSERT_TRUE(estimator_->AddWheel(
        MakeWheel(time, sequence, elapsed - 0.005)).ok());
    ImuSample imu = MakeImu(time, sequence);
    imu.acceleration.x() = 1.0;
    ASSERT_TRUE(estimator_->AddImu(imu).ok());
  }
  EXPECT_NEAR(estimator_->state().velocity.x(), 0.995, 1e-9);
  EXPECT_NEAR(estimator_->state().position.x(), 0.495025, 1e-9);
  EXPECT_NEAR(estimator_->state().position.y(), 0.0, 1e-12);
  EXPECT_NEAR(estimator_->state().position.z(), 0.0, 1e-12);
  EXPECT_NEAR(LogRotation(estimator_->state().orientation).norm(), 0.0, 1e-12);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest, WheelUpdateUsesInterpolatedMeasurementTime) {
  Initialize();
  LocalEstimator bracketed(config_);
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(bracketed.AddWheel(MakeWheel(time, sequence)).ok());
    ASSERT_TRUE(bracketed.AddImu(MakeImu(time, sequence)).ok());
  }

  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.025, 4, 0.5)).ok());
  ImuSample interval_end = MakeImu(1.03, 4);
  interval_end.acceleration.x() = 1.0;
  ASSERT_TRUE(estimator_->AddImu(interval_end).ok());

  ImuSample midpoint = MakeImu(1.025, 4);
  midpoint.acceleration.x() = 0.5;
  ASSERT_TRUE(bracketed.AddImu(midpoint).ok());
  ASSERT_TRUE(bracketed.AddWheel(MakeWheel(1.025, 4, 0.5)).ok());
  interval_end.stamp.sequence = 5;
  ASSERT_TRUE(bracketed.AddImu(interval_end).ok());

  EXPECT_NEAR((estimator_->state().position -
               bracketed.state().position).norm(),
              0.0, 1e-14);
  EXPECT_NEAR((estimator_->state().velocity -
               bracketed.state().velocity).norm(),
              0.0, 1e-14);
  EXPECT_NEAR((estimator_->state().orientation.coeffs() -
               bracketed.state().orientation.coeffs()).norm(),
              0.0, 1e-14);
  EXPECT_NEAR((estimator_->state().covariance -
               bracketed.state().covariance).norm(),
              0.0, 1e-13);
}

TEST_F(LocalEstimatorTest, MultipleQueuedWheelsFuseInMeasurementTimeOrder) {
  Initialize();
  LocalEstimator bracketed(config_);
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(bracketed.AddWheel(MakeWheel(time, sequence)).ok());
    ASSERT_TRUE(bracketed.AddImu(MakeImu(time, sequence)).ok());
  }

  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.024, 4, 0.2)).ok());
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.028, 5, 0.4)).ok());
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());

  ASSERT_TRUE(bracketed.AddImu(MakeImu(1.024, 4)).ok());
  ASSERT_TRUE(bracketed.AddWheel(MakeWheel(1.024, 4, 0.2)).ok());
  ASSERT_TRUE(bracketed.AddImu(MakeImu(1.028, 5)).ok());
  ASSERT_TRUE(bracketed.AddWheel(MakeWheel(1.028, 5, 0.4)).ok());
  ASSERT_TRUE(bracketed.AddImu(MakeImu(1.03, 6)).ok());

  EXPECT_NEAR((estimator_->state().position -
               bracketed.state().position).norm(),
              0.0, 1e-14);
  EXPECT_NEAR((estimator_->state().velocity -
               bracketed.state().velocity).norm(),
              0.0, 1e-14);
  EXPECT_NEAR((estimator_->state().covariance -
               bracketed.state().covariance).norm(),
              0.0, 1e-13);
  EXPECT_EQ(estimator_->wheel_health().accepted, 5U);
}

TEST_F(LocalEstimatorTest, GyroIntegratesLocalYawWithoutAbsoluteHeading) {
  Initialize();
  for (uint64_t sequence = 4; sequence <= 13; ++sequence) {
    const double time = 1.02 + 0.01 * static_cast<double>(sequence - 3);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    ImuSample imu = MakeImu(time, sequence);
    imu.angular_velocity.z() = 1.0;
    ASSERT_TRUE(estimator_->AddImu(imu).ok());
  }
  EXPECT_NEAR(LogRotation(estimator_->state().orientation).z(), 0.095, 1e-12);
  EXPECT_EQ(estimator_->state().position.norm(), 0.0);
  EXPECT_EQ(estimator_->state().velocity.norm(), 0.0);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest, WheelUsesConstrainedGainAndNeverSnapsPose) {
  Initialize();
  LocalEstimator inertial_only(config_);
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(inertial_only.AddWheel(MakeWheel(time, sequence)).ok());
    inertial_only.AddImu(MakeImu(time, sequence));
  }
  ImuSample imu = MakeImu(1.03, 4);
  imu.acceleration.x() = 1.0;
  imu.angular_velocity.z() = 0.2;
  ASSERT_TRUE(inertial_only.AddImu(imu).ok());
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4, 0.5)).ok());
  ASSERT_TRUE(estimator_->AddImu(imu).ok());
  EXPECT_EQ((estimator_->state().position -
             inertial_only.state().position).norm(), 0.0);
  EXPECT_EQ((estimator_->state().orientation.coeffs() -
             inertial_only.state().orientation.coeffs()).norm(), 0.0);
  EXPECT_GT(estimator_->state().velocity.x(), inertial_only.state().velocity.x());
  EXPECT_GT((estimator_->state().accel_bias -
             inertial_only.state().accel_bias).norm(), 0.0);
  EXPECT_NEAR(estimator_->state().covariance(0, 0),
              inertial_only.state().covariance(0, 0), 1e-15);
  EXPECT_NEAR(estimator_->state().covariance(6, 6),
              inertial_only.state().covariance(6, 6), 1e-15);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest, BufferedWheelIsFusedOnceAndNeverHeldAsRepeatedEvidence) {
  Initialize();
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.025, 4, 0.5)).ok());
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
  const double corrected_speed = estimator_->state().velocity.x();
  EXPECT_GT(corrected_speed, 0.2);
  EXPECT_LT(corrected_speed, 0.3);
  const LocalState corrected = estimator_->state();
  EXPECT_EQ(estimator_->AddWheel(MakeWheel(1.025, 4, 0.5)).reason,
            Reason::DUPLICATE);
  ExpectSameState(corrected, estimator_->state());
  EXPECT_TRUE(estimator_->AddImu(MakeImu(1.04, 5)).ok());
  // Propagation consumes the actual IMU once without reusing rejected wheel input.
  EXPECT_EQ(estimator_->state().stamp.sequence, 5U);
  EXPECT_LT(std::abs(estimator_->state().velocity.x() - corrected_speed), 0.001);
  EXPECT_EQ(estimator_->wheel_health().accepted, 4U);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest, WheelInnovationRejectedOnceWithoutPoseOrVelocitySnap) {
  Initialize();
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4, 100.0)).ok());
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.03, 4)).reason,
            Reason::INNOVATION_REJECTED);
  EXPECT_EQ(estimator_->state().velocity.norm(), 0.0);
  EXPECT_EQ(estimator_->state().position.norm(), 0.0);
  EXPECT_EQ(estimator_->wheel_health().rejected, 1U);
  EXPECT_TRUE(estimator_->state().valid);
  EXPECT_TRUE(estimator_->Evaluate(1.03).ok());
  EXPECT_TRUE(estimator_->AddImu(MakeImu(1.04, 5)).ok());
  EXPECT_EQ(estimator_->wheel_health().rejected, 1U);
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.05, 5)).ok());
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.05, 6)).ok());
  EXPECT_TRUE(estimator_->state().valid);
}

TEST_F(LocalEstimatorTest, WheelsBeforeCommittedFrontierAreNotReplayed) {
  Initialize();
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
  EXPECT_EQ(estimator_->AddWheel(MakeWheel(1.025, 4)).reason,
            Reason::HISTORY_UNAVAILABLE);
  EXPECT_EQ(estimator_->AddWheel(MakeWheel(1.06, 4)).reason,
            Reason::INVALID_INPUT);
  EXPECT_TRUE(estimator_->Evaluate(1.03).ok());
}

TEST_F(LocalEstimatorTest, PendingWheelOverflowIsExplicitWithoutOverwrite) {
  Initialize();
  for (uint64_t offset = 0; offset < 64; ++offset) {
    const double time = 1.0201 + 0.0003 * static_cast<double>(offset);
    ASSERT_TRUE(
        estimator_->AddWheel(MakeWheel(time, 4 + offset, 0.1)).ok());
  }
  const WheelSample overflow = MakeWheel(1.0393, 68, 0.2);
  EXPECT_EQ(estimator_->AddWheel(overflow).reason, Reason::INVALID_INPUT);
  EXPECT_EQ(estimator_->wheel_health().accepted, 67U);
  EXPECT_EQ(estimator_->wheel_health().rejected, 1U);
  EXPECT_EQ(estimator_->wheel_health().evictions, 1U);
}

TEST_F(LocalEstimatorTest, BufferedWheelCannotBeFusedOutsideReceiveAgeWindow) {
  Initialize();
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4, 0.5)).ok());
  ImuSample imu = MakeImu(1.04, 4);
  imu.stamp.receive_time = 1.09;
  EXPECT_EQ(estimator_->AddImu(imu).reason, Reason::WHEEL_STALE);
  EXPECT_EQ(estimator_->state().velocity.norm(), 0.0);
  EXPECT_EQ(estimator_->wheel_health().rejected, 1U);
  EXPECT_TRUE(estimator_->EvaluateIntegrity(1.09).ok());
  EXPECT_TRUE(estimator_->state().valid);
}

TEST_F(LocalEstimatorTest, WheelFreshnessDegradesPrecisionNotIntegrity) {
  Initialize();
  EXPECT_EQ(estimator_->Evaluate(1.075).reason, Reason::WHEEL_STALE);
  EXPECT_TRUE(estimator_->EvaluateIntegrity(1.075).ok());
  EXPECT_EQ(estimator_->Evaluate(1.13).reason, Reason::IMU_STALE);
  EXPECT_EQ(estimator_->EvaluateIntegrity(1.13).reason, Reason::IMU_STALE);
  EXPECT_EQ(estimator_->Evaluate(1.0).reason, Reason::INVALID_INPUT);
  EXPECT_EQ(estimator_->EvaluateIntegrity(1.0).reason, Reason::INVALID_INPUT);
  EXPECT_EQ(estimator_->Evaluate(std::numeric_limits<double>::quiet_NaN()).reason,
            Reason::INVALID_INPUT);
  EXPECT_EQ(
      estimator_
          ->EvaluateIntegrity(std::numeric_limits<double>::quiet_NaN())
          .reason,
      Reason::INVALID_INPUT);
  for (uint64_t sequence = 4; sequence <= 9; ++sequence) {
    const double time = 1.02 + 0.01 * static_cast<double>(sequence - 3);
    const Result result = estimator_->AddImu(MakeImu(time, sequence));
    if (sequence <= 7) {
      EXPECT_TRUE(result.ok()) << result.message;
    } else {
      EXPECT_EQ(result.reason, Reason::WHEEL_STALE);
      EXPECT_TRUE(estimator_->state().valid);
      EXPECT_TRUE(estimator_->EvaluateIntegrity(time).ok());
    }
  }
  EXPECT_EQ(estimator_->wheel_health().accepted, 3U);
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.09, 4)).ok());
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.09, 10)).ok());
  EXPECT_TRUE(estimator_->state().valid);
}

TEST_F(LocalEstimatorTest, NonholonomicConstraintsReduceTransverseVelocityOnly) {
  config_.use_nonholonomic_constraint = true;
  estimator_.reset(new LocalEstimator(config_));
  Initialize();
  LocalConfig free_config = config_;
  free_config.use_nonholonomic_constraint = false;
  LocalEstimator free_estimator(free_config);
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(free_estimator.AddWheel(MakeWheel(time, sequence)).ok());
    free_estimator.AddImu(MakeImu(time, sequence));
  }
  ImuSample imu = MakeImu(1.03, 4);
  imu.acceleration.y() = 2.0;
  imu.acceleration.z() = 11.0;
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4)).ok());
  ASSERT_TRUE(free_estimator.AddWheel(MakeWheel(1.03, 4)).ok());
  ASSERT_TRUE(estimator_->AddImu(imu).ok());
  ASSERT_TRUE(free_estimator.AddImu(imu).ok());
  EXPECT_LT(std::abs(estimator_->state().velocity.y()),
            std::abs(free_estimator.state().velocity.y()));
  EXPECT_LT(std::abs(estimator_->state().velocity.z()),
            std::abs(free_estimator.state().velocity.z()));
  EXPECT_EQ((estimator_->state().position -
             free_estimator.state().position).norm(), 0.0);
  EXPECT_EQ((estimator_->state().orientation.coeffs() -
             free_estimator.state().orientation.coeffs()).norm(), 0.0);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest, ImuGapLatchesFaultWithoutTeleportOrAutomaticRecovery) {
  Initialize();
  const LocalState before = estimator_->state();
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.1, 4)).reason, Reason::IMU_GAP);
  EXPECT_EQ(estimator_->state().stamp.time, before.stamp.time);
  EXPECT_EQ((estimator_->state().position - before.position).norm(), 0.0);
  EXPECT_EQ((estimator_->state().orientation.coeffs() -
             before.orientation.coeffs()).norm(), 0.0);
  EXPECT_EQ((estimator_->state().covariance - before.covariance).norm(), 0.0);
  EXPECT_FALSE(estimator_->state().valid);
  EXPECT_EQ(estimator_->imu_health().accepted, 4U);
  EXPECT_EQ(estimator_->imu_health().rejected, 0U);
  EXPECT_EQ(estimator_->ResetStopped(1.1).reason, Reason::RESET_WHILE_MOVING);
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.11, 4)).ok());
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.11, 5)).reason, Reason::IMU_GAP);
  EXPECT_EQ(estimator_->Evaluate(1.11).reason, Reason::IMU_GAP);
  EXPECT_EQ(estimator_->EvaluateIntegrity(1.11).reason, Reason::IMU_GAP);
  EXPECT_EQ(estimator_->state().stamp.time, before.stamp.time);
  EXPECT_EQ(estimator_->state().epoch.generation, 1U);
}

TEST_F(LocalEstimatorTest, StoppedResetChangesEpochAndRequiresNewAlignmentEvidence) {
  Initialize();
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.1, 4)).reason, Reason::IMU_GAP);
  for (uint64_t sequence = 5; sequence <= 7; ++sequence) {
    const double time = 1.11 + 0.01 * static_cast<double>(sequence - 5);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    EXPECT_EQ(estimator_->AddImu(MakeImu(time, sequence)).reason, Reason::IMU_GAP);
  }
  ASSERT_TRUE(estimator_->ResetStopped(1.14).ok());
  EXPECT_EQ(estimator_->state().epoch.session, config_.session);
  EXPECT_EQ(estimator_->state().epoch.generation, 2U);
  EXPECT_FALSE(estimator_->state().valid);
  EXPECT_NEAR(estimator_->state().stamp.time, 1.13, 1e-12);
  EXPECT_EQ(estimator_->state().position.norm(), 0.0);
  EXPECT_EQ(estimator_->Evaluate(1.14).reason, Reason::WAITING_FOR_IMU);
  EXPECT_EQ(estimator_->ResetStopped(1.14).reason, Reason::RESET_WHILE_MOVING);
  // Epoch reset clears evidence, not the source duplicate watermarks.
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.13, 7)).reason, Reason::DUPLICATE);
  for (uint64_t sequence = 8; sequence <= 10; ++sequence) {
    const double time = 1.14 + 0.01 * static_cast<double>(sequence - 8);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    EXPECT_TRUE(estimator_->AddImu(MakeImu(time, sequence)).ok());
    EXPECT_EQ(estimator_->Evaluate(time).reason,
              sequence < 10 ? Reason::WAITING_FOR_STATIONARY : Reason::NONE);
  }
  EXPECT_TRUE(estimator_->state().valid);
  EXPECT_EQ(estimator_->state().epoch.generation, 2U);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest, MovingOrStaleEvidenceCannotResetEpoch) {
  Initialize();
  const LocalState before = estimator_->state();
  EXPECT_EQ(estimator_->ResetStopped(1.2).reason, Reason::RESET_WHILE_MOVING);
  ExpectSameState(before, estimator_->state());
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4, 0.5)).ok());
  EXPECT_EQ(estimator_->ResetStopped(1.03).reason, Reason::RESET_WHILE_MOVING);
  ExpectSameState(before, estimator_->state());
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.04, 5)).ok());
  ImuSample imu = MakeImu(1.04, 5);
  imu.angular_velocity.z() = 0.5;
  ASSERT_TRUE(estimator_->AddImu(imu).ok());
  const LocalState moving = estimator_->state();
  EXPECT_EQ(estimator_->ResetStopped(1.04).reason, Reason::RESET_WHILE_MOVING);
  ExpectSameState(moving, estimator_->state());
  EXPECT_EQ(estimator_->ResetStopped(0.0).reason, Reason::INVALID_INPUT);
  ExpectSameState(moving, estimator_->state());
}

TEST_F(LocalEstimatorTest, RecoveryNeedsConsecutiveBoundedStationaryEvidence) {
  Initialize();
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.1, 4)).reason, Reason::IMU_GAP);
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.11, 4)).ok());
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.11, 5)).reason, Reason::IMU_GAP);
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.2, 5)).ok());
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.2, 6)).reason, Reason::IMU_GAP);
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.21, 6)).ok());
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.21, 7)).reason, Reason::IMU_GAP);
  // A second recovery gap discarded the first stopped sample.
  EXPECT_EQ(estimator_->ResetStopped(1.21).reason, Reason::RESET_WHILE_MOVING);
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.22, 7)).ok());
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.22, 8)).reason, Reason::IMU_GAP);
  ASSERT_TRUE(estimator_->ResetStopped(1.22).ok());
}

TEST_F(LocalEstimatorTest, PositionGaugeUncertaintyGrowsAndTripsPerAxisBudget) {
  config_.max_position_std = 0.0001;
  estimator_.reset(new LocalEstimator(config_));
  Initialize();
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4)).ok());
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.03, 4)).reason,
            Reason::COVARIANCE_EXCEEDED);
  EXPECT_EQ(estimator_->Evaluate(1.03).reason, Reason::COVARIANCE_EXCEEDED);
  EXPECT_TRUE(estimator_->EvaluateIntegrity(1.03).ok());
  EXPECT_TRUE(estimator_->state().valid);
  EXPECT_GT(std::sqrt(estimator_->state().covariance(0, 0)),
            config_.max_position_std);
  const Eigen::Vector3d first_position = estimator_->state().position;
  ImuSample next = MakeImu(1.04, 5);
  next.acceleration.x() = 1.0;
  EXPECT_EQ(estimator_->AddImu(next).reason, Reason::COVARIANCE_EXCEEDED);
  EXPECT_EQ(estimator_->state().stamp.time, 1.04);
  EXPECT_TRUE(estimator_->state().valid);
  EXPECT_TRUE(estimator_->EvaluateIntegrity(1.04).ok());
  EXPECT_NE((estimator_->state().position - first_position).norm(), 0.0);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest, VelocityAndAttitudeEachHaveIndependentSigmaBudgets) {
  for (bool attitude_budget : {false, true}) {
    config_ = TestConfig();
    if (attitude_budget) {
      config_.max_attitude_std = config_.initial_attitude_std;
    } else {
      config_.max_velocity_std = config_.initial_velocity_std;
    }
    estimator_.reset(new LocalEstimator(config_));
    Initialize();
    EXPECT_EQ(estimator_->AddImu(MakeImu(1.03, 4)).reason,
              Reason::COVARIANCE_EXCEEDED);
    EXPECT_TRUE(estimator_->EvaluateIntegrity(1.03).ok());
    EXPECT_TRUE(estimator_->state().valid);
    const int index = attitude_budget ? 6 : 3;
    const double limit = attitude_budget ? config_.max_attitude_std
                                        : config_.max_velocity_std;
    EXPECT_GT(std::sqrt(estimator_->state().covariance(index, index)), limit);
    ExpectPositiveCovariance(estimator_->state());
  }
}

TEST_F(LocalEstimatorTest, BiasRandomWalkAndFullCovarianceRemainPositive) {
  Initialize();
  const Matrix15d initial = estimator_->state().covariance;
  // No new wheel is fused, so the measured bias growth is only process noise.
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_GT(estimator_->state().covariance(9 + axis, 9 + axis),
              initial(9 + axis, 9 + axis));
    EXPECT_GT(estimator_->state().covariance(12 + axis, 12 + axis),
              initial(12 + axis, 12 + axis));
  }
  EXPECT_NE(estimator_->state().covariance(0, 3), 0.0);
  EXPECT_NE(estimator_->state().covariance(3, 12), 0.0);
  EXPECT_NE(estimator_->state().covariance(6, 9), 0.0);
  ExpectPositiveCovariance(estimator_->state());
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace apollo
