// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/core/global_alignment.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

GlobalConfig Policy() {
  GlobalConfig config;
  config.map_id = "campus";
  config.map_version = "v1";
  config.calibration_id = "calibration";
  return config;
}

LocalState Local(double time, uint64_t sequence) {
  LocalState local;
  local.stamp = {time, time, sequence, "unix"};
  local.epoch = {"producer", 1};
  local.position.x() = time - 10.0;
  local.covariance = Matrix15d::Identity() * 0.0001;
  local.valid = true;
  return local;
}

GlobalObservation Observation(const LocalState& local, double offset = 100.0) {
  GlobalObservation observation;
  observation.stamp = local.stamp;
  observation.epoch = local.epoch;
  observation.source = "map";
  observation.map_id = "campus";
  observation.map_version = "v1";
  observation.calibration_id = "calibration";
  observation.pose = Pose(local);
  observation.pose.translation().x() += offset;
  observation.covariance = Matrix6d::Identity() * 0.001;
  observation.quality_valid = true;
  return observation;
}

void Initialize(GlobalAlignment* alignment) {
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    auto local = Local(10.0 + sequence * 0.01, sequence);
    ASSERT_TRUE(alignment->AddLocal(local).ok());
    const auto result = alignment->Observe(Observation(local));
    if (sequence < 3) {
      EXPECT_EQ(Reason::RELOCALIZATION_VERIFYING, result.reason);
    } else {
      EXPECT_TRUE(result.ok()) << result.message;
    }
  }
}

TEST(GlobalAlignmentTest, RequiresVerifiedGlobalEvidenceWithoutMutatingLocal) {
  GlobalAlignment alignment(Policy());
  ASSERT_TRUE(alignment.ValidateConfig().ok());
  Initialize(&alignment);
  ASSERT_TRUE(alignment.state().valid);
  EXPECT_NEAR(100.0, alignment.state().map_to_odom.translation().x(), 1e-9);
  EXPECT_TRUE(alignment.Evaluate(10.03).ok());
  EXPECT_NEAR(0.03, alignment.latest_local()->position.x(), 1e-9);
  EXPECT_FALSE(alignment.state().georeferenced);
}

TEST(GlobalAlignmentTest, DelayedObservationUsesHistoricalPose) {
  GlobalAlignment alignment(Policy());
  auto first = Local(10.0, 1);
  auto second = Local(10.04, 2);
  ASSERT_TRUE(alignment.AddLocal(first).ok());
  ASSERT_TRUE(alignment.AddLocal(second).ok());
  LocalState interpolated;
  ASSERT_TRUE(alignment.Lookup(10.02, &interpolated).ok());
  EXPECT_NEAR(0.02, interpolated.position.x(), 1e-9);
  auto observation = Observation(interpolated);
  observation.stamp.receive_time = 10.04;
  EXPECT_EQ(Reason::RELOCALIZATION_VERIFYING,
            alignment.Observe(observation).reason);
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE, alignment.Lookup(9.99, &interpolated).reason);
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE, alignment.Lookup(10.05, &interpolated).reason);
}

TEST(GlobalAlignmentTest, UnmodeledLocalCovarianceRevokesAlignmentAndHistory) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  auto local = Local(10.04, 4);
  local.covariance_model_valid = false;
  EXPECT_EQ(Reason::INVALID_INPUT, alignment.AddLocal(local).reason);
  EXPECT_FALSE(alignment.state().valid);
  ASSERT_NE(nullptr, alignment.latest_local());
  EXPECT_TRUE(alignment.latest_local()->valid);
  EXPECT_FALSE(alignment.latest_local()->covariance_model_valid);
  LocalState output;
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE, alignment.Lookup(10.03, &output).reason);
  EXPECT_EQ(Reason::INVALID_INPUT, alignment.Lookup(10.04, &output).reason);
  EXPECT_EQ(Reason::INVALID_INPUT,
            alignment.Observe(Observation(local)).reason);
  EXPECT_FALSE(alignment.state().valid);
  EXPECT_EQ(Reason::TIMESTAMP_REGRESSION,
            alignment.AddLocal(Local(10.03, 3)).reason);
}

TEST(GlobalAlignmentTest, RejectsClockMapCovarianceAndEpochMismatch) {
  GlobalAlignment alignment(Policy());
  auto local = Local(10.0, 1);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  auto observation = Observation(local);
  observation.map_version = "v2";
  EXPECT_EQ(Reason::MAP_MISMATCH, alignment.Observe(observation).reason);
  observation = Observation(local);
  observation.epoch.generation = 2;
  EXPECT_EQ(Reason::EPOCH_MISMATCH, alignment.Observe(observation).reason);
  observation = Observation(local);
  observation.stamp.clock_id = "gps";
  EXPECT_EQ(Reason::CLOCK_INVALID, alignment.Observe(observation).reason);
  observation = Observation(local);
  observation.covariance.setZero();
  EXPECT_EQ(Reason::INVALID_INPUT, alignment.Observe(observation).reason);
  EXPECT_FALSE(alignment.state().valid);
}

TEST(GlobalAlignmentTest, RejectsDuplicateAndLargeInnovation) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  auto local = Local(10.04, 4);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  EXPECT_EQ(Reason::INNOVATION_REJECTED,
            alignment.Observe(Observation(local, 200.0)).reason);
  EXPECT_EQ(Reason::DUPLICATE, alignment.Observe(Observation(local)).reason);
  EXPECT_NEAR(100.0, alignment.state().map_to_odom.translation().x(), 1e-9);
  EXPECT_NEAR(0.04, alignment.latest_local()->position.x(), 1e-9);
}

TEST(GlobalAlignmentTest, GlobalLossDoesNotInvalidateLocalAndNeedsReverification) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  auto local = Local(12.1, 4);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  EXPECT_EQ(Reason::GLOBAL_STALE, alignment.Evaluate(12.1).reason);
  EXPECT_TRUE(alignment.latest_local()->valid);
  EXPECT_EQ(Reason::RELOCALIZATION_VERIFYING,
            alignment.Observe(Observation(local)).reason);
  EXPECT_FALSE(alignment.state().valid);
}

TEST(GlobalAlignmentTest, EpochChangeInvalidatesAlignmentAndRetiredSessions) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  auto reset = Local(10.04, 4);
  reset.epoch.generation = 2;
  ASSERT_TRUE(alignment.AddLocal(reset).ok());
  EXPECT_FALSE(alignment.state().valid);
  auto old = Local(10.05, 5);
  EXPECT_EQ(Reason::EPOCH_MISMATCH, alignment.AddLocal(old).reason);
  reset = Local(10.06, 1);
  reset.epoch = {"new-producer", 1};
  ASSERT_TRUE(alignment.AddLocal(reset).ok());
  old = Local(10.07, 6);
  EXPECT_EQ(Reason::EPOCH_MISMATCH, alignment.AddLocal(old).reason);
}

TEST(GlobalAlignmentTest, AmbiguityCannotBecomeSingleGlobalPose) {
  GlobalAlignment alignment(Policy());
  auto local = Local(10.0, 1);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  auto observation = Observation(local);
  observation.ambiguous = true;
  EXPECT_EQ(Reason::RELOCALIZATION_AMBIGUOUS,
            alignment.Observe(observation).reason);
  EXPECT_FALSE(alignment.state().valid);
}

TEST(GlobalAlignmentTest, CorrelationBoundsDoNotClaimIndependentPrecision) {
  auto local = Local(10.0, 1);
  const auto covariance = TransformCovarianceBound(
      Pose(local), local, Matrix6d::Identity() * 0.01);
  EXPECT_GE(covariance(0, 0), 2.0 * (0.01 + 0.0001));
  EXPECT_TRUE(ValidCovariance(covariance));
}

TEST(GlobalAlignmentTest, UnanchoredPositionIsEvidenceNotFullAlignment) {
  GlobalAlignment alignment(Policy());
  const auto local = Local(10.0, 1);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  auto observation = Observation(local);
  observation.kind = GlobalObservation::Kind::POSITION;
  observation.covariance.bottomRightCorner<3, 3>().setZero();
  ASSERT_TRUE(alignment.Observe(observation).ok());
  EXPECT_FALSE(alignment.state().valid);
  EXPECT_FALSE(alignment.Evaluate(10.0).ok());
  ASSERT_EQ(1, alignment.evidence().size());
  EXPECT_DOUBLE_EQ(local.position.x(), alignment.latest_local()->position.x());
}

TEST(GlobalAlignmentTest, CorridorUpdateDoesNotInventLongitudinalInformation) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  const auto before = alignment.state();
  const auto local = Local(10.04, 4);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  auto observation = Observation(local);
  observation.kind = GlobalObservation::Kind::PROJECTED_POSE;
  observation.pose.translation().y() = 0.001;
  observation.projection = Eigen::Matrix<double, 1, 6>::Zero();
  observation.projection(0, 1) = 1.0;
  observation.projected_covariance = Eigen::Matrix<double, 1, 1>::Constant(0.001);
  ASSERT_TRUE(alignment.Observe(observation).ok());
  EXPECT_DOUBLE_EQ(before.map_to_odom.translation().x(),
                   alignment.state().map_to_odom.translation().x());
  EXPECT_GE(alignment.state().covariance(0, 0), before.covariance(0, 0));
  EXPECT_DOUBLE_EQ(before.last_full_observation,
                   alignment.state().last_full_observation);
  EXPECT_DOUBLE_EQ(local.position.y(), alignment.latest_local()->position.y());
}

TEST(GlobalAlignmentTest, PartialUpdatesCannotRenewFullPoseLease) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  auto local = Local(12.04, 4);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  auto observation = Observation(local);
  observation.kind = GlobalObservation::Kind::POSITION;
  ASSERT_TRUE(alignment.Observe(observation).ok());
  EXPECT_EQ(Reason::GLOBAL_STALE, alignment.Evaluate(12.04).reason);
  EXPECT_TRUE(alignment.latest_local()->valid);
}

TEST(GlobalAlignmentTest, AlignmentUncertaintyGrowsWithoutNewGlobalEvidence) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  const auto before = alignment.state();
  const auto predicted = alignment.Predict(10.13);
  EXPECT_GT(predicted.covariance(0, 0), before.covariance(0, 0));
  EXPECT_GT(predicted.covariance(5, 5), before.covariance(5, 5));
  EXPECT_TRUE(predicted.map_to_odom.matrix().isApprox(before.map_to_odom.matrix()));
  EXPECT_DOUBLE_EQ(before.last_full_observation, predicted.last_full_observation);
}

TEST(GlobalAlignmentTest, RankDeficientRowsCannotCompleteRecovery) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  auto local = Local(10.04, 4);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  auto observation = Observation(local);
  observation.kind = GlobalObservation::Kind::PROJECTED_POSE;
  observation.projection = Eigen::Matrix<double, 2, 6>::Zero();
  observation.projection(0, 1) = 1.0;
  observation.projection(1, 1) = 1.0;
  observation.projected_covariance = Eigen::Matrix2d::Identity();
  EXPECT_EQ(Reason::INVALID_INPUT, alignment.Observe(observation).reason);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace apollo
