// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/core/local_motion.h"

#include <limits>

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

LocalState Endpoint(double time, uint64_t sequence) {
  LocalState state;
  state.valid = true;
  state.epoch = {"motion", 1};
  state.stamp = {time, time, sequence, "unix"};
  state.covariance = Matrix15d::Identity();
  return state;
}

std::vector<SourceSampleId> Sources() {
  return {{"imu", 1}, {"imu", 2}};
}

TEST(LocalMotionTest, SharedPoseUncertaintyCancelsInsteadOfAddingMarginals) {
  const auto start = Endpoint(1.0, 1);
  auto end = Endpoint(1.01, 2);
  end.covariance += Matrix15d::Identity() * 0.01;
  MotionIncrement motion;
  ASSERT_TRUE(ComputeMotionIncrement(start, end, start.covariance, Sources(),
                                    &motion).ok());
  EXPECT_TRUE(motion.covariance.isApprox(Matrix6d::Identity() * 0.01, 1e-10));
  EXPECT_EQ("motion:1", motion.correlation_group);
  EXPECT_TRUE(motion.cross_covariance.isApprox(start.covariance));
  EXPECT_LT(motion.covariance.trace(), PoseCovariance(end).trace());
}

TEST(LocalMotionTest, CovarianceAgreesWithPerturbedRelativeTransform) {
  auto start = Endpoint(1.0, 1);
  auto end = Endpoint(1.01, 2);
  start.position = Eigen::Vector3d(2.0, -3.0, 4.0);
  start.orientation = ExpRotation(Eigen::Vector3d(0.1, -0.2, 0.3));
  end.position = Eigen::Vector3d(3.0, -1.0, 5.0);
  end.orientation = ExpRotation(Eigen::Vector3d(-0.2, 0.4, 0.1));
  Matrix15d transition = Matrix15d::Identity();
  transition.block<3, 3>(0, 3) = 0.01 * Eigen::Matrix3d::Identity();
  const Matrix15d cross = start.covariance * transition.transpose();
  end.covariance = transition * start.covariance * transition.transpose() +
                   Matrix15d::Identity() * 0.01;
  MotionIncrement motion;
  ASSERT_TRUE(
      ComputeMotionIncrement(start, end, cross, Sources(), &motion).ok());
  Eigen::Matrix<double, 30, 30> joint;
  joint.topLeftCorner<15, 15>() = start.covariance;
  joint.topRightCorner<15, 15>() = cross;
  joint.bottomLeftCorner<15, 15>() = cross.transpose();
  joint.bottomRightCorner<15, 15>() = end.covariance;
  Eigen::Matrix<double, 6, 30> jacobian =
      Eigen::Matrix<double, 6, 30>::Zero();
  const double epsilon = 1e-6;
  for (int column = 0; column < 30; ++column) {
    Eigen::Matrix<double, 6, 1> residual[2];
    for (int side = 0; side < 2; ++side) {
      auto a = start;
      auto b = end;
      auto& perturbed = column < 15 ? a : b;
      const int offset = column % 15;
      const double step = side == 0 ? epsilon : -epsilon;
      if (offset < 3) {
        perturbed.position(offset) += step;
      } else if (offset >= 6 && offset < 9) {
        Eigen::Vector3d angle = Eigen::Vector3d::Zero();
        angle(offset - 6) = step;
        perturbed.orientation = perturbed.orientation * ExpRotation(angle);
      }
      const Eigen::Isometry3d delta = Pose(a).inverse() * Pose(b);
      residual[side].head<3>() =
          delta.translation() - motion.delta.translation();
      residual[side].tail<3>() = LogRotation(
          Eigen::Quaterniond(motion.delta.linear().transpose() * delta.linear()));
    }
    jacobian.col(column) = (residual[0] - residual[1]) / (2.0 * epsilon);
  }
  const Matrix6d numerical = jacobian * joint * jacobian.transpose();
  EXPECT_TRUE(motion.covariance.isApprox(numerical, 1e-8));
}

TEST(LocalMotionTest, RejectsImpossibleCrossCovarianceAndPreservesOutput) {
  const auto start = Endpoint(1.0, 1);
  const auto end = Endpoint(1.01, 2);
  MotionIncrement output;
  output.correlation_group = "unchanged";
  EXPECT_EQ(Reason::INVALID_INPUT,
            ComputeMotionIncrement(start, end, 2.0 * start.covariance,
                                   Sources(), &output).reason);
  EXPECT_EQ("unchanged", output.correlation_group);
}

TEST(LocalMotionTest, PositiveCovarianceWithoutErrorModelCannotProduceMotion) {
  auto start = Endpoint(1.0, 1);
  auto end = Endpoint(1.01, 2);
  end.covariance += Matrix15d::Identity() * 0.01;
  MotionIncrement output;
  output.correlation_group = "unchanged";
  end.covariance_model_valid = false;
  EXPECT_EQ(Reason::INVALID_INPUT,
            ComputeMotionIncrement(start, end, start.covariance,
                                   Sources(), &output).reason);
  end.covariance_model_valid = true;
  start.covariance_model_valid = false;
  EXPECT_EQ(Reason::INVALID_INPUT,
            ComputeMotionIncrement(start, end, start.covariance,
                                   Sources(), &output).reason);
  EXPECT_EQ("unchanged", output.correlation_group);
}

TEST(LocalMotionTest, RejectsResetClockRegressionAndMissingProvenance) {
  const auto start = Endpoint(1.0, 1);
  auto end = Endpoint(1.01, 2);
  MotionIncrement output;
  end.epoch.generation = 2;
  EXPECT_EQ(Reason::EPOCH_MISMATCH,
            ComputeMotionIncrement(start, end, Matrix15d::Zero(), Sources(),
                                   &output).reason);
  end = Endpoint(1.01, 2);
  end.stamp.clock_id = "gps";
  EXPECT_EQ(Reason::CLOCK_INVALID,
            ComputeMotionIncrement(start, end, Matrix15d::Zero(), Sources(),
                                   &output).reason);
  end = Endpoint(1.0, 2);
  EXPECT_EQ(Reason::TIMESTAMP_REGRESSION,
            ComputeMotionIncrement(start, end, Matrix15d::Zero(), Sources(),
                                   &output).reason);
  end = Endpoint(1.01, 2);
  EXPECT_EQ(Reason::INVALID_INPUT,
            ComputeMotionIncrement(start, end, Matrix15d::Zero(), {},
                                   &output).reason);
  auto sources = Sources();
  sources.push_back(sources.front());
  EXPECT_EQ(Reason::INVALID_INPUT,
            ComputeMotionIncrement(start, end, Matrix15d::Zero(), sources,
                                   &output).reason);
  end.position.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(Reason::INVALID_INPUT,
            ComputeMotionIncrement(start, end, Matrix15d::Zero(), Sources(),
                                   &output).reason);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace apollo
