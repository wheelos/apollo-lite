// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/core/local_estimator.h"
#include "modules/localization/core/global_alignment.h"
#include "modules/localization/runtime/messages.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

TEST(LocalizationPipelineTest, FirstEpochAndDegradedUpdatesRemainDecodable) {
  LocalConfig policy;
  policy.session = "degraded-pipeline";
  policy.stationary_samples = 2;
  policy.max_position_std = 10.0;
  policy.max_attitude_std = 1.0;
  policy.wheel_timeout = 0.015;
  LocalEstimator estimator(policy);
  for (uint64_t sequence = 1; sequence <= 2; ++sequence) {
    const double time = 10.0 + 0.01 * static_cast<double>(sequence - 1);
    WheelSample wheel;
    wheel.stamp = {time, time, sequence, "unix"};
    wheel.variance = 0.01;
    ASSERT_TRUE(estimator.AddWheel(wheel).ok());
    ImuSample imu;
    imu.stamp = wheel.stamp;
    imu.acceleration.z() = policy.gravity;
    ASSERT_TRUE(estimator.AddImu(imu).ok());
  }
  ASSERT_EQ(1U, estimator.state().epoch.generation);
  ImuSample imu;
  imu.stamp = {10.03, 10.03, 3, "unix"};
  imu.acceleration.z() = policy.gravity;
  ASSERT_EQ(Reason::WHEEL_STALE, estimator.AddImu(imu).reason);
  ASSERT_TRUE(estimator.EvaluateIntegrity(10.03).ok());
  ASSERT_EQ(imu.stamp.sequence, estimator.state().stamp.sequence);
  LocalOdometry message;
  LocalEstimatorConfig config;
  EncodeLocal(estimator.state(), config, 10.03, &message);
  message.set_precision_valid(false);
  message.set_degradation_reason(ReasonName(estimator.Evaluate(10.03).reason));
  ASSERT_NE(nullptr, estimator.last_motion());
  EncodeMotion(*estimator.last_motion(), config, 10.03, message.mutable_motion());
  LocalState decoded;
  ASSERT_TRUE(
      DecodeLocal(message, GlobalEstimatorConfig(), 10.03, &decoded).ok());
  EXPECT_TRUE(decoded.valid);
  EXPECT_EQ(3U, decoded.stamp.sequence);
  EXPECT_EQ("WHEEL_STALE", message.degradation_reason());
  MotionIncrement motion;
  ASSERT_TRUE(DecodeMotion(message, decoded, &motion).ok());
  EXPECT_EQ(2U, motion.start.sequence);
  EXPECT_EQ(3U, motion.end.sequence);
}

TEST(LocalizationPipelineTest, GlobalRecoveryNeverWritesLocalState) {
  LocalConfig policy;
  policy.session = "pipeline";
  policy.stationary_samples = 5;
  policy.max_position_std = 10.0;
  policy.max_attitude_std = 1.0;
  LocalEstimator estimator(policy);
  ASSERT_TRUE(estimator.ValidateConfig().ok());
  GlobalEstimatorConfig global_config;
  auto* manifest = global_config.mutable_map();
  manifest->set_map_id("site");
  manifest->set_version("v1");
  manifest->set_frame_id("map");
  manifest->set_calibration_id("calibrated");
  manifest->set_pcd_path("");
  global_config.set_max_position_std(10.0);
  global_config.set_max_attitude_std(1.0);
  GlobalAlignment alignment(GlobalPolicy(global_config));
  uint64_t sequence = 0;
  uint64_t global_sequence = 0;
  uint32_t accepted_global = 0;
  Epoch epoch;
  Eigen::Vector3d previous_position = Eigen::Vector3d::Zero();
  for (int sample = 0; sample < 150; ++sample) {
    const double time = 10.0 + sample * 0.01;
    ++sequence;
    WheelSample wheel;
    wheel.stamp = {time, time, sequence, "unix"};
    wheel.variance = 0.01;
    ASSERT_TRUE(estimator.AddWheel(wheel).ok());
    ImuSample imu;
    imu.stamp = wheel.stamp;
    imu.acceleration.z() = policy.gravity;
    const auto propagated = estimator.AddImu(imu);
    ASSERT_TRUE(propagated.ok() ||
                propagated.reason == Reason::WAITING_FOR_STATIONARY);
    if (!estimator.Evaluate(time).ok()) {
      continue;
    }
    const LocalState before = estimator.state();
    if (epoch.generation == 0) {
      epoch = before.epoch;
    }
    ASSERT_EQ(epoch, before.epoch);
    ASSERT_LT((before.position - previous_position).norm(), 0.001);
    previous_position = before.position;
    LocalEstimatorConfig local_config;
    LocalOdometry envelope;
    EncodeLocal(before, local_config, time, &envelope);
    LocalState decoded;
    ASSERT_TRUE(DecodeLocal(envelope, global_config, time, &decoded).ok());
    ASSERT_TRUE(alignment.AddLocal(decoded).ok());
    if (sample % 10 == 0) {
      GlobalObservation observation;
      observation.stamp = before.stamp;
      observation.stamp.sequence = ++global_sequence;
      observation.epoch = before.epoch;
      observation.source = "synthetic_map";
      observation.map_id = "site";
      observation.map_version = "v1";
      observation.calibration_id = "calibrated";
      observation.pose = Pose(before);
      observation.pose.translation().x() += sample < 80 ? 100.0 : 105.0;
      observation.covariance = Matrix6d::Identity() * 0.01;
      observation.quality_valid = true;
      observation.recovery = sample >= 80 && sample <= 100;
      const auto observed = alignment.Observe(observation);
      if (observed.ok()) {
        ++accepted_global;
      } else {
        ASSERT_EQ(Reason::RELOCALIZATION_VERIFYING, observed.reason);
      }
      EXPECT_TRUE(estimator.state().position.isApprox(before.position));
      EXPECT_TRUE(estimator.state().orientation.coeffs().isApprox(
          before.orientation.coeffs()));
      EXPECT_TRUE(estimator.state().covariance.isApprox(before.covariance));
    }
  }
  ASSERT_GT(accepted_global, 3U);
  EXPECT_NEAR(105.0, alignment.state().map_to_odom.translation().x(), 1e-6);
  EXPECT_LT(estimator.state().position.norm(), 0.001);
  const auto last = estimator.state();
  HealthReporter reporter;
  const double stale_time = last.stamp.time + policy.max_sample_age + 0.01;
  EXPECT_FALSE(estimator.Evaluate(stale_time).ok());
  const auto assessment = reporter.Evaluate(
      "global", &last, false, alignment.state().valid, false, false,
      Reason::IMU_STALE, IDLE, stale_time, true);
  EXPECT_EQ(INVALID, assessment.availability());
  EXPECT_EQ(0U, assessment.capabilities());
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace apollo
