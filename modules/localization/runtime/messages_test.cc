// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/runtime/messages.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

TEST(LocalizationMessagesTest, LocalRoundtripPreservesFrameEpochAndCovariance) {
  LocalState state;
  state.epoch = {"unique-session", 3};
  state.stamp = {10.0, 10.0, 5, "unix"};
  state.covariance = Matrix15d::Identity() * 0.01;
  state.velocity.x() = 2.0;
  state.valid = true;
  LocalEstimatorConfig config;
  LocalOdometry message;
  EncodeLocal(state, config, 10.01, &message);
  ASSERT_TRUE(message.IsInitialized());
  LocalState decoded;
  GlobalEstimatorConfig global;
  ASSERT_TRUE(DecodeLocal(message, global, 10.01, &decoded).ok());
  EXPECT_EQ(state.epoch, decoded.epoch);
  EXPECT_EQ(5U, decoded.stamp.sequence);
  EXPECT_TRUE(decoded.covariance.isApprox(state.covariance));
  EXPECT_DOUBLE_EQ(2.0, decoded.velocity.x());
  message.set_frame_id("map");
  EXPECT_EQ(Reason::INVALID_INPUT, DecodeLocal(message, global, 10.01, &decoded).reason);
}

TEST(LocalizationMessagesTest, RejectsIncompleteAndExpiredLocalContracts) {
  LocalState state;
  GlobalEstimatorConfig config;
  LocalOdometry empty;
  EXPECT_EQ(Reason::INVALID_INPUT, DecodeLocal(empty, config, 10.0, &state).reason);
  state.epoch = {"s", 1};
  state.stamp = {10.0, 10.0, 1, "unix"};
  state.covariance = Matrix15d::Identity() * 0.01;
  state.valid = true;
  LocalOdometry message;
  EncodeLocal(state, LocalEstimatorConfig(), 10.0, &message);
  EXPECT_EQ(Reason::INVALID_INPUT, DecodeLocal(message, config, 10.3, &state).reason);
  message.set_state_covariance(0, -1.0);
  EXPECT_EQ(Reason::INVALID_INPUT, DecodeLocal(message, config, 10.0, &state).reason);
}

TEST(LocalizationMessagesTest, MotionRoundtripRetainsJointCovarianceAndLineage) {
  LocalState start;
  start.epoch = {"s", 1};
  start.stamp = {10.0, 10.0, 5, "unix"};
  start.covariance = Matrix15d::Identity();
  start.valid = true;
  LocalState end = start;
  end.stamp = {10.01, 10.01, 6, "unix"};
  end.covariance += Matrix15d::Identity() * 0.01;
  MotionIncrement expected;
  ASSERT_TRUE(ComputeMotionIncrement(
      start, end, start.covariance, {{"imu", 5}, {"imu", 6}},
      &expected).ok());
  LocalOdometry message;
  LocalEstimatorConfig config;
  EncodeLocal(end, config, 10.01, &message);
  EncodeMotion(expected, config, 10.01, message.mutable_motion());
  ASSERT_TRUE(message.IsInitialized());
  LocalState decoded;
  ASSERT_TRUE(DecodeLocal(message, GlobalEstimatorConfig(), 10.01,
                          &decoded).ok());
  MotionIncrement motion;
  ASSERT_TRUE(DecodeMotion(message, decoded, &motion).ok());
  EXPECT_TRUE(expected.covariance.isApprox(motion.covariance));
  EXPECT_TRUE(expected.cross_covariance.isApprox(motion.cross_covariance));
  EXPECT_EQ("s:1", motion.correlation_group);
  EXPECT_EQ(2U, motion.sources.size());
  message.set_covariance_model_valid(false);
  EXPECT_EQ(Reason::INVALID_INPUT, DecodeMotion(message, decoded, &motion).reason);
  message.set_covariance_model_valid(true);
  decoded.covariance_model_valid = false;
  EXPECT_EQ(Reason::INVALID_INPUT, DecodeMotion(message, decoded, &motion).reason);
  decoded.covariance_model_valid = true;
  message.mutable_motion()->set_relative_covariance(0, 999.0);
  EXPECT_EQ(Reason::INVALID_INPUT, DecodeMotion(message, decoded, &motion).reason);
  EncodeMotion(expected, config, 10.01, message.mutable_motion());
  message.mutable_motion()->set_correlation_group("independent");
  EXPECT_EQ(Reason::INVALID_INPUT, DecodeMotion(message, decoded, &motion).reason);
  EncodeMotion(expected, config, 10.01, message.mutable_motion());
  message.mutable_motion()->set_cross_covariance(0, 2.0);
  EXPECT_EQ(Reason::INVALID_INPUT, DecodeMotion(message, decoded, &motion).reason);
}

TEST(LocalizationMessagesTest, HealthNeverInventsGlobalOrLaneCapabilities) {
  LocalState state;
  state.epoch = {"s", 1};
  state.covariance = Matrix15d::Identity() * 0.01;
  HealthReporter reporter;
  auto assessment = reporter.Evaluate(
      "global", &state, true, false, false, false,
      Reason::GLOBAL_UNAVAILABLE, IDLE, 10.0, true);
  EXPECT_EQ(DEGRADED, assessment.availability());
  EXPECT_NE(0U, assessment.capabilities() & LOCAL_POSE_VALID);
  EXPECT_EQ(0U, assessment.capabilities() & GLOBAL_POSE_VALID);
  EXPECT_EQ(0U, assessment.capabilities() & LANE_LEVEL_VALID);
  EXPECT_EQ(0U, assessment.capabilities() & RELOCALIZATION_AVAILABLE);
  LocalizationHealthEvent event;
  ASSERT_TRUE(reporter.TakeEvent(&event));
  EXPECT_EQ(DEGRADED, event.current());
  assessment = reporter.Evaluate("global", &state, false, true, true, true,
                                Reason::IMU_STALE, IDLE, 10.3, true);
  EXPECT_EQ(INVALID, assessment.availability());
  EXPECT_EQ(0U, assessment.capabilities());
  EXPECT_TRUE(reporter.TakeEvent(&event));
  EXPECT_EQ(INVALID, event.current());
}

TEST(LocalizationMessagesTest, QuaternionAndMissingFieldsCannotBecomeIdentity) {
  LocalizationEstimate message;
  message.set_measurement_time(10.0);
  Eigen::Isometry3d pose;
  EXPECT_EQ(Reason::INVALID_INPUT, DecodePose(message, &pose).reason);
  EncodePose(Eigen::Isometry3d::Identity(), Matrix6d::Identity(), 10.0, 10.0,
             "map", &message);
  message.mutable_pose()->mutable_orientation()->set_qw(0.0);
  EXPECT_EQ(Reason::INVALID_INPUT, DecodePose(message, &pose).reason);
}

TEST(LocalizationMessagesTest, DegradedPrecisionDoesNotEraseIntegrityState) {
  LocalState state;
  state.epoch = {"s", 1};
  state.stamp = {10.0, 10.0, 1, "unix"};
  state.covariance = Matrix15d::Identity() * 10.0;
  state.valid = true;
  LocalOdometry message;
  EncodeLocal(state, LocalEstimatorConfig(), 10.0, &message);
  message.set_precision_valid(false);
  message.set_degradation_reason("COVARIANCE_EXCEEDED");
  LocalState decoded;
  EXPECT_TRUE(DecodeLocal(message, GlobalEstimatorConfig(), 10.0, &decoded).ok());
  EXPECT_TRUE(decoded.valid);
  EXPECT_FALSE(message.precision_valid());
  HealthReporter reporter;
  const auto assessment = reporter.Evaluate(
      "local", &state, false, false, false, false,
      Reason::COVARIANCE_EXCEEDED, IDLE, 10.0, false, true);
  EXPECT_EQ(DEGRADED, assessment.availability());
  EXPECT_TRUE(assessment.propagation_valid());
  EXPECT_EQ(0U, assessment.capabilities() & LOCAL_POSE_VALID);
  LocalizationHealthEvent event;
  ASSERT_TRUE(reporter.TakeEvent(&event));
  EXPECT_EQ(DEGRADED, event.current());
}

TEST(LocalizationMessagesTest, PositionObservationRequiresNoQuaternion) {
  GlobalEstimatorConfig config;
  config.mutable_map()->set_frame_id("map");
  GlobalObservationMessage message;
  message.mutable_epoch()->set_producer_session("s");
  message.mutable_epoch()->set_generation(1);
  message.set_source("position");
  message.set_sequence(1);
  message.set_clock_id("unix");
  message.set_frame_id("map");
  message.set_child_frame_id("base_link");
  message.set_map_id("site");
  message.set_map_version("v1");
  message.set_calibration_id("cal");
  message.set_quality_valid(true);
  message.set_kind(GlobalObservationMessage::POSITION);
  message.set_measurement_time(10.0);
  for (auto* point : {message.mutable_position(), message.mutable_point_in_base()}) {
    point->set_x(0.0);
    point->set_y(0.0);
    point->set_z(0.0);
  }

  message.set_observed_dimension(3);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      message.add_observed_covariance(row == col ? 0.01 : 0.0);
    }
  }
  GlobalObservation observation;
  ASSERT_TRUE(DecodeObservation(message, config, 10.0, &observation).ok());
  EXPECT_EQ(GlobalObservation::Kind::POSITION, observation.kind);
  EXPECT_FALSE(message.has_localization());
  GlobalObservationMessage output;
  EncodeObservation(observation, config, 10.0, &output);
  ASSERT_TRUE(output.IsInitialized());
  EXPECT_FALSE(output.has_localization());
  EXPECT_EQ(GlobalObservationMessage::POSITION, output.kind());
  EXPECT_EQ(9, output.observed_covariance_size());
  EXPECT_TRUE(DecodeObservation(output, config, 10.0, &observation).ok());
  message.clear_point_in_base();
  EXPECT_EQ(Reason::INVALID_INPUT,
            DecodeObservation(message, config, 10.0, &observation).reason);
}

TEST(LocalizationMessagesTest, UnmodeledCovarianceDoesNotBecomeTrustedPrecision) {
  LocalState state;
  state.epoch = {"s", 1};
  state.stamp = {10.0, 10.0, 1, "unix"};
  state.covariance = Matrix15d::Identity() * 0.001;
  state.valid = true;
  state.covariance_model_valid = false;
  LocalOdometry message;
  EncodeLocal(state, LocalEstimatorConfig(), 10.0, &message);
  EXPECT_TRUE(message.valid());
  EXPECT_FALSE(message.covariance_model_valid());
  EXPECT_FALSE(message.localization().has_uncertainty());
  LocalState decoded;
  ASSERT_TRUE(DecodeLocal(message, GlobalEstimatorConfig(), 10.0, &decoded).ok());
  EXPECT_TRUE(decoded.valid);
  EXPECT_FALSE(decoded.covariance_model_valid);
  message.clear_covariance_model_valid();
  ASSERT_TRUE(DecodeLocal(message, GlobalEstimatorConfig(), 10.0, &decoded).ok());
  EXPECT_FALSE(decoded.covariance_model_valid);

  HealthReporter reporter;
  const auto assessment = reporter.Evaluate(
      "local", &state, true, true, true, false, Reason::NONE,
      IDLE, 10.0, false, true);
  EXPECT_EQ(DEGRADED, assessment.availability());
  EXPECT_TRUE(assessment.propagation_valid());
  EXPECT_TRUE(assessment.output_continuous());
  EXPECT_FALSE(assessment.covariance_valid());
  EXPECT_EQ("INVALID_INPUT", assessment.primary_reason());
  EXPECT_FALSE(assessment.has_position_std());
  EXPECT_FALSE(assessment.has_attitude_std());
  EXPECT_EQ(0U, assessment.capabilities() & LOCAL_POSE_VALID);
  EXPECT_EQ(0U, assessment.capabilities() & GLOBAL_POSE_VALID);
}

TEST(LocalizationMessagesTest, ProjectedObservationPublishesOnlyObservedCovariance) {
  GlobalEstimatorConfig config;
  config.mutable_map()->set_frame_id("map");
  GlobalObservation observation;
  observation.kind = GlobalObservation::Kind::PROJECTED_POSE;
  observation.stamp = {10.0, 10.0, 1, "unix"};
  observation.epoch = {"s", 1};
  observation.source = "corridor";
  observation.map_id = "site";
  observation.map_version = "v1";
  observation.calibration_id = "cal";
  observation.quality_valid = true;
  observation.projection = Eigen::Matrix<double, 2, 6>::Zero();
  observation.projection(0, 1) = 1.0;
  observation.projection(1, 5) = 2.0;
  observation.projected_covariance = Eigen::Matrix2d::Identity() * 0.01;
  GlobalObservationMessage message;
  EncodeObservation(observation, config, 10.0, &message);
  ASSERT_TRUE(message.IsInitialized());
  EXPECT_EQ(0, message.covariance_size());
  EXPECT_FALSE(message.localization().has_uncertainty());
  EXPECT_EQ(12, message.projection_size());
  EXPECT_EQ(4, message.observed_covariance_size());
  GlobalObservation decoded;
  ASSERT_TRUE(DecodeObservation(message, config, 10.0, &decoded).ok());
  EXPECT_TRUE(decoded.projection.isApprox(observation.projection));
  EXPECT_TRUE(decoded.projected_covariance.isApprox(observation.projected_covariance));
  message.set_valid_until(9.0);
  EXPECT_EQ(Reason::INVALID_INPUT,
            DecodeObservation(message, config, 10.0, &decoded).reason);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace apollo
