// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "modules/localization/core/continuous_output.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

LocalState State(double time, double x) {
  LocalState state;
  state.stamp = {time, time, static_cast<uint64_t>(time * 10.0), "clock"};
  state.epoch = {"session", 1};
  state.position.x() = x;
  state.covariance.setIdentity();
  state.valid = true;
  return state;
}

TEST(ContinuousOutputTest, CorrectionStartsWithoutSameTimePoseJump) {
  LocalConfig config;
  config.lidar_max_translation_correction_rate = 0.2;
  config.lidar_max_rotation_correction_rate = 0.1;
  config.lidar_max_velocity_correction_acceleration = 0.3;
  config.lidar_correction_variance_scale = 2.0;
  config.lidar_max_translation_correction = 0.5;
  config.lidar_max_rotation_correction = 1.0;
  config.lidar_max_velocity_correction = 3.0;
  config.lidar_process_noise_psd_tolerance = 1e-12;
  ContinuousOutput output(config);
  LocalState published = State(1.0, 0.0);
  published.velocity.x() = 1.0;
  output.Reset(published);

  LocalState corrected = State(1.0, 1.0);
  corrected.velocity.x() = 1.0;
  corrected.covariance_model_valid = false;
  EXPECT_DOUBLE_EQ(output.state().position.x(), 0.0);
  EXPECT_DOUBLE_EQ(output.state().stamp.time, corrected.stamp.time);

  LocalState propagated = State(1.1, 1.1);
  propagated.velocity.x() = 1.0;
  Matrix15d transition = Matrix15d::Identity();
  ASSERT_TRUE(output.Advance(corrected, propagated, transition, 0.1).ok());
  EXPECT_FALSE(output.state().covariance_model_valid);
  EXPECT_NEAR(output.state().position.x(), 0.1015, 1e-12);
  EXPECT_NEAR(output.state().position.x() - published.position.x(), 0.1015,
              1e-12);
  EXPECT_NEAR(output.state().velocity.x(), 1.015, 1e-12);
  EXPECT_GT(output.state().covariance(0, 0), published.covariance(0, 0));
  EXPECT_TRUE(output.correction_budget_exceeded());

  const LocalState next_internal = State(1.2, 1.2);
  LocalState next = next_internal;
  next.velocity.x() = 1.0;
  ASSERT_TRUE(output.Advance(propagated, next, transition, 0.1).ok());
  EXPECT_NEAR(output.state().position.x(), 0.206, 1e-12);
  EXPECT_NEAR(output.state().velocity.x(), 1.045, 1e-12);
}

TEST(ContinuousOutputTest, RotationVelocityAndCovarianceUseExplicitBudgets) {
  LocalConfig config;
  config.lidar_max_translation_correction_rate = 0.1;
  config.lidar_max_rotation_correction_rate = 0.2;
  config.lidar_max_velocity_correction_acceleration = 0.4;
  config.lidar_correction_variance_scale = 3.0;
  config.lidar_max_translation_correction = 2.0;
  config.lidar_max_rotation_correction = 1.0;
  config.lidar_max_velocity_correction = 3.0;
  config.lidar_process_noise_psd_tolerance = 1e-12;
  ContinuousOutput output(config);
  LocalState published = State(2.0, 0.0);
  published.covariance *= 0.5;
  output.Reset(published);

  LocalState corrected = published;
  corrected.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  corrected.velocity =
      corrected.orientation * Eigen::Vector3d(2.0, 0.0, 0.0);
  LocalState propagated = corrected;
  propagated.stamp = {2.2, 2.2, 22, "clock"};
  propagated.position =
      corrected.orientation * Eigen::Vector3d(0.4, 0.0, 0.0);
  Matrix15d transition = Matrix15d::Identity();
  ASSERT_TRUE(output.Advance(corrected, propagated, transition, 0.2).ok());
  EXPECT_NEAR(LogRotation(output.state().orientation).norm(), 0.04, 1e-12);
  EXPECT_TRUE(output.state().velocity.isApprox(
      (output.state().position - published.position) / 0.2, 1e-12));
  EXPECT_GT(output.state().covariance(3, 3),
            published.covariance(3, 3));
  EXPECT_GT(output.state().covariance(8, 8), published.covariance(8, 8));
}

TEST(ContinuousOutputTest, RejectsInconsistentProcessCovariance) {
  LocalConfig config;
  config.lidar_max_translation_correction_rate = 0.1;
  config.lidar_max_rotation_correction_rate = 0.1;
  config.lidar_max_velocity_correction_acceleration = 0.1;
  config.lidar_correction_variance_scale = 1.0;
  config.lidar_max_translation_correction = 1.0;
  config.lidar_max_rotation_correction = 1.0;
  config.lidar_max_velocity_correction = 1.0;
  config.lidar_process_noise_psd_tolerance = 1e-12;
  ContinuousOutput output(config);
  const LocalState start = State(3.0, 0.0);
  output.Reset(start);
  LocalState end = State(3.1, 0.0);
  end.covariance *= 0.5;
  EXPECT_EQ(output.Advance(start, end, Matrix15d::Identity(), 0.1).reason,
            Reason::INVALID_INPUT);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace apollo
