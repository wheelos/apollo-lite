// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/core/observed_subspace.h"

#include "Eigen/Cholesky"
#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

TEST(ObservedSubspaceTest, BasisRotationAndScalingPreserveInformationAndRows) {
  Eigen::MatrixXd projection = Eigen::MatrixXd::Zero(2, 6);
  projection(0, 1) = 1.0;
  projection(0, 5) = 2.0;
  projection(1, 2) = 1.0;
  Eigen::Matrix2d noise;
  noise << 0.2, 0.03, 0.03, 0.1;
  Eigen::MatrixXd canonical;
  Eigen::MatrixXd covariance;
  ASSERT_TRUE(CanonicalizeObservedSubspace(
      projection, noise, 3.0, &canonical, &covariance).ok());
  const Matrix6d information =
      projection.transpose() * noise.llt().solve(projection);
  EXPECT_TRUE(information.isApprox(
      canonical.transpose() * covariance.llt().solve(canonical), 1e-10));

  Eigen::Matrix2d change;
  change << -2.0, 3.0, 4.0, 1.0;
  Eigen::MatrixXd other;
  Eigen::MatrixXd other_noise;
  ASSERT_TRUE(CanonicalizeObservedSubspace(
      change * projection, change * noise * change.transpose(), 3.0,
      &other, &other_noise).ok());
  EXPECT_TRUE(canonical.isApprox(other, 1e-10));
  EXPECT_TRUE(covariance.isApprox(other_noise, 1e-10));
  EXPECT_TRUE(canonical.col(0).isZero(1e-12));
}

TEST(ObservedSubspaceTest, InPlaceIsSafeAndNullspaceIsNotFilled) {
  Eigen::MatrixXd projection = Eigen::MatrixXd::Zero(1, 6);
  projection(0, 1) = -2.0;
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Constant(1, 1, 0.4);
  ASSERT_TRUE(CanonicalizeObservedSubspace(
      projection, covariance, 2.0, &projection, &covariance).ok());
  EXPECT_NEAR(1.0, projection(0, 1), 1e-12);
  EXPECT_NEAR(0.1, covariance(0, 0), 1e-12);
  EXPECT_EQ(1, projection.rows());
  EXPECT_DOUBLE_EQ(0.0, projection(0, 0));
}

TEST(ObservedSubspaceTest, InvalidRankNoiseAndMetricAreRejected) {
  Eigen::MatrixXd projection = Eigen::MatrixXd::Zero(2, 6);
  projection(0, 0) = projection(1, 0) = 1.0;
  Eigen::MatrixXd covariance = Eigen::Matrix2d::Identity();
  Eigen::MatrixXd output;
  Eigen::MatrixXd noise;
  EXPECT_EQ(Reason::DEGENERATE, CanonicalizeObservedSubspace(
      projection, covariance, 1.0, &output, &noise).reason);
  projection(1, 1) = 1.0;
  covariance(0, 0) = -1.0;
  EXPECT_EQ(Reason::INVALID_INPUT, CanonicalizeObservedSubspace(
      projection, covariance, 1.0, &output, &noise).reason);
  covariance.setIdentity();
  EXPECT_EQ(Reason::INVALID_INPUT, CanonicalizeObservedSubspace(
      projection, covariance, 0.0, &output, &noise).reason);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace apollo
