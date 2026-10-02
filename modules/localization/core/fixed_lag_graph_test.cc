// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/core/fixed_lag_graph.h"
#include "modules/localization/core/global_alignment.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

LocalState Local(uint64_t index) {
  LocalState state;
  state.epoch = {"graph-local", 1};
  state.stamp = {10.0 + 0.1 * static_cast<double>(index),
                 10.0 + 0.1 * static_cast<double>(index), index + 1, "unix"};
  state.covariance = Matrix15d::Identity() * (0.1 + 0.001 * index);
  state.valid = true;
  return state;
}

GlobalObservation Observation(const LocalState& local, uint64_t sequence) {
  GlobalObservation observation;
  observation.stamp = local.stamp;
  observation.stamp.sequence = sequence;
  observation.epoch = local.epoch;
  observation.source = "independent-survey";
  observation.map_id = "site";
  observation.map_version = "v1";
  observation.calibration_id = "calibrated";
  observation.pose.translation().x() = 100.0;
  observation.covariance = Matrix6d::Identity() * 0.01;
  observation.quality_valid = true;
  observation.independent_of_local = true;
  return observation;
}

FixedLagConfig Policy() {
  FixedLagConfig config;
  config.lag = 0.35;
  // Numerical tests do not qualify a production execution deadline.
  config.solve_budget_ms = 100000.0;
  return config;
}

MotionIncrement Motion(const LocalState& start, const LocalState& end) {
  MotionIncrement motion;
  const auto result = ComputeMotionIncrement(
      start, end, start.covariance,
      {{"imu", start.stamp.sequence}, {"imu", end.stamp.sequence}}, &motion);
  EXPECT_TRUE(result.ok()) << result.message;
  return motion;
}

TEST(FixedLagGraphTest, PositionOrCorrelatedEvidenceCannotCreateAnAnchor) {
  FixedLagGraph graph(Policy());
  auto observation = Observation(Local(0), 1);
  observation.kind = GlobalObservation::Kind::POSITION;
  EXPECT_EQ(Reason::GLOBAL_UNAVAILABLE,
            graph.Initialize(Local(0), observation).reason);
  EXPECT_FALSE(graph.initialized());
  observation.kind = GlobalObservation::Kind::POSE;
  observation.independent_of_local = false;
  EXPECT_EQ(Reason::INVALID_INPUT,
            graph.Initialize(Local(0), observation).reason);
  EXPECT_FALSE(graph.state().valid);
}

TEST(FixedLagGraphTest, UnmodeledLocalCovarianceCannotAnchorOrAdvanceGraph) {
  FixedLagGraph graph(Policy());
  auto local = Local(0);
  local.covariance_model_valid = false;
  EXPECT_EQ(Reason::GLOBAL_UNAVAILABLE,
            graph.Initialize(local, Observation(local, 1)).reason);
  EXPECT_FALSE(graph.initialized());
  ASSERT_TRUE(graph.Initialize(Local(0), Observation(Local(0), 1)).ok());
  const auto previous = graph.state();
  const auto motion = Motion(Local(0), Local(1));
  local = Local(1);
  local.covariance_model_valid = false;
  EXPECT_EQ(Reason::INVALID_INPUT, graph.AddLocal(local, motion).reason);
  EXPECT_DOUBLE_EQ(previous.covariance_time, graph.state().covariance_time);
  EXPECT_TRUE(previous.map_to_odom.matrix().isApprox(
      graph.state().map_to_odom.matrix()));
}

TEST(FixedLagGraphTest, DelayedPositionUsesItsOwnStateWithoutRefreshingFullLease) {
  FixedLagGraph graph(Policy());
  ASSERT_TRUE(graph.Initialize(Local(0), Observation(Local(0), 1)).ok());
  for (uint64_t index = 1; index <= 3; ++index) {
    const auto motion = Motion(Local(index - 1), Local(index));
    const auto result = graph.AddLocal(Local(index), motion);
    ASSERT_TRUE(result.ok()) << result.message;
  }
  const double full_time = graph.state().last_full_observation;
  auto latest = Observation(Local(3), 3);
  latest.kind = GlobalObservation::Kind::POSITION;
  ASSERT_TRUE(graph.Observe(latest).ok());
  auto delayed = Observation(Local(1), 2);
  delayed.kind = GlobalObservation::Kind::POSITION;
  delayed.pose.translation().y() = 0.05;
  const auto observed = graph.Observe(delayed);
  ASSERT_TRUE(observed.ok()) << observed.message;
  EXPECT_GT(graph.state().map_to_odom.translation().y(), 0.0);
  EXPECT_DOUBLE_EQ(full_time, graph.state().last_full_observation);
  EXPECT_DOUBLE_EQ(Local(3).stamp.time, graph.state().last_observation);
  EXPECT_DOUBLE_EQ(Local(3).stamp.time, graph.state().covariance_time);
  EXPECT_EQ(Reason::DUPLICATE, graph.Observe(delayed).reason);
  delayed.stamp.sequence = 4;
  delayed.stamp.time += 0.01;
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE, graph.Observe(delayed).reason);
}

TEST(FixedLagGraphTest, MarginalizationRetainsAnchorAndRejectsOldTimes) {
  FixedLagGraph graph(Policy());
  ASSERT_TRUE(graph.Initialize(Local(0), Observation(Local(0), 1)).ok());
  for (uint64_t index = 1; index <= 8; ++index) {
    const auto motion = Motion(Local(index - 1), Local(index));
    const auto added = graph.AddLocal(Local(index), motion);
    ASSERT_TRUE(added.ok()) << added.message;
  }
  EXPECT_NEAR(100.0, graph.state().map_to_odom.translation().x(), 1e-8);
  EXPECT_TRUE(graph.state().valid);
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE,
            graph.Observe(Observation(Local(0), 2)).reason);
  auto fresh = Observation(Local(8), 3);
  ASSERT_TRUE(graph.Observe(fresh).ok());
  EXPECT_DOUBLE_EQ(Local(8).stamp.time, graph.state().last_full_observation);
}

TEST(FixedLagGraphTest, RejectedUpdatesLeaveTheCommittedSnapshotUntouched) {
  FixedLagGraph graph(Policy());
  const LocalState local = Local(0);
  ASSERT_TRUE(graph.Initialize(local, Observation(local, 1)).ok());
  const auto before = graph.state();
  auto outlier = Observation(local, 2);
  outlier.source = "other-survey";
  outlier.pose.translation().x() += 1000.0;
  EXPECT_EQ(Reason::INNOVATION_REJECTED, graph.Observe(outlier).reason);
  auto wrong_epoch = Local(1);
  wrong_epoch.epoch.generation = 2;
  EXPECT_EQ(Reason::EPOCH_MISMATCH,
            graph.AddLocal(wrong_epoch, Motion(local, Local(1))).reason);
  EXPECT_TRUE(before.map_to_odom.matrix().isApprox(
      graph.state().map_to_odom.matrix()));
  EXPECT_TRUE(before.covariance.isApprox(graph.state().covariance));
  EXPECT_EQ(before.correction_id, graph.state().correction_id);
  EXPECT_EQ(before.last_observation, graph.state().last_observation);
  EXPECT_TRUE(local.position.isZero());
  EXPECT_TRUE(local.covariance.isApprox(Local(0).covariance));
}

TEST(FixedLagGraphTest, ProjectedEvidenceDoesNotClaimTheNullDirection) {
  FixedLagGraph graph(Policy());
  ASSERT_TRUE(graph.Initialize(Local(0), Observation(Local(0), 1)).ok());
  const auto before = graph.state();
  auto observation = Observation(Local(0), 1);
  observation.source = "independent-wall";
  observation.kind = GlobalObservation::Kind::PROJECTED_POSE;
  observation.pose.translation().y() = 0.05;
  observation.projection = Eigen::Matrix<double, 1, 6>::Zero();
  observation.projection(0, 1) = 1.0;
  observation.projected_covariance = Eigen::MatrixXd::Constant(1, 1, 0.01);
  ASSERT_TRUE(graph.Observe(observation).ok());
  EXPECT_NEAR(before.map_to_odom.translation().x(),
              graph.state().map_to_odom.translation().x(), 1e-8);
  EXPECT_NEAR(before.covariance(0, 0), graph.state().covariance(0, 0), 1e-8);
  EXPECT_GT(graph.state().map_to_odom.translation().y(), 0.0);
  EXPECT_DOUBLE_EQ(before.last_full_observation,
                   graph.state().last_full_observation);
}

TEST(FixedLagGraphTest, OwnerRequiresExplicitMotionQualification) {
  GlobalConfig config;
  config.map_id = "site";
  config.map_version = "v1";
  config.calibration_id = "calibrated";
  config.enable_fixed_lag_graph = true;
  EXPECT_EQ(Reason::CONFIG_INVALID, GlobalAlignment(config).ValidateConfig().reason);
  config.motion_error_model_qualified = true;
  EXPECT_TRUE(GlobalAlignment(config).ValidateConfig().ok());
}

TEST(FixedLagGraphTest, GlobalOwnerRecoversAfterLostMotionWithoutResettingOdom) {
  GlobalConfig config;
  config.map_id = "site";
  config.map_version = "v1";
  config.calibration_id = "calibrated";
  config.enable_fixed_lag_graph = true;
  config.motion_error_model_qualified = true;
  config.graph_solve_budget_ms = 100000.0;
  config.verification_frames = 2;
  GlobalAlignment owner(config);
  ASSERT_TRUE(owner.ValidateConfig().ok());
  ASSERT_TRUE(owner.AddLocal(Local(0)).ok());
  EXPECT_EQ(Reason::RELOCALIZATION_VERIFYING,
            owner.Observe(Observation(Local(0), 1)).reason);
  auto motion = Motion(Local(0), Local(1));
  ASSERT_TRUE(owner.AddLocal(Local(1), &motion).ok());
  ASSERT_TRUE(owner.Observe(Observation(Local(1), 2)).ok());
  ASSERT_TRUE(owner.state().valid);
  // Miss Local(2) on the transport. Do not request a local reset or freeze
  // the global history forever at Local(1).
  motion = Motion(Local(2), Local(3));
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE, owner.AddLocal(Local(3), &motion).reason);
  EXPECT_FALSE(owner.state().valid);
  ASSERT_NE(nullptr, owner.latest_local());
  EXPECT_DOUBLE_EQ(Local(3).stamp.time, owner.latest_local()->stamp.time);
  EXPECT_EQ(Local(0).epoch, owner.latest_local()->epoch);
  EXPECT_EQ(Reason::RELOCALIZATION_VERIFYING,
            owner.Observe(Observation(Local(3), 3)).reason);
  motion = Motion(Local(3), Local(4));
  ASSERT_TRUE(owner.AddLocal(Local(4), &motion).ok());
  ASSERT_TRUE(owner.Observe(Observation(Local(4), 4)).ok());
  EXPECT_TRUE(owner.state().valid);
  EXPECT_EQ(Local(0).epoch, owner.state().epoch);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace apollo
