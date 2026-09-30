// Copyright 2026 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/world_model/local_map/temporal_lane_map.h"

#include <cmath>
#include <limits>

#include "gtest/gtest.h"

namespace apollo {
namespace world_model {
namespace {
TemporalLanePolicy Policy() {
  return {0.5, 2.0, 0.2, 0.5, 1.0, 0.2, 0.3, 2.0, 3.0, 6.0, 0.1, 0.3};
}
LaneObservation Observation(double time, uint64_t sequence, double vehicle_x,
                            double noise = 0.0) {
  LaneObservation o;
  o.session = "session";
  o.generation = 1;
  o.sequence = sequence;
  o.measurement_time = time;
  o.position_error = 0.02;
  o.forward_confirmed = true;
  for (double x = vehicle_x; x <= vehicle_x + 10.0; x += 0.5) {
    o.left.push_back({x - vehicle_x, 2.5 + noise});
    o.right.push_back({x - vehicle_x, -2.5 + noise});
  }
  return o;
}

TEST(TemporalLaneMapTest, TwoFramesCompensateMotionFuseOverlapAndExtend) {
  TemporalLaneMap map(Policy());
  ASSERT_TRUE(map.BeginEpoch("session", 1, {1.0, 0, 0, 0, 0.01}, true).ok());
  ASSERT_TRUE(map.Observe(Observation(1.0, 1, 0.0, 0.05), 1.0).ok());
  TemporalLaneSnapshot first, second;
  ASSERT_TRUE(map.Snapshot(1.0, &first).ok());
  ASSERT_TRUE(map.AddPose("session", 1, {1.1, 1.0, 0, 0, 0.01}).ok());
  ASSERT_TRUE(map.Observe(Observation(1.1, 2, 1.0, -0.05), 1.1).ok());
  ASSERT_TRUE(map.Snapshot(1.1, &second).ok());
  EXPECT_EQ(first.lane_id, second.lane_id);
  EXPECT_GT(second.version, first.version);
  EXPECT_NEAR(11.0, second.samples.back().left.x, 1e-9);
  EXPECT_NEAR(0.0, second.samples.front().left.x, 1e-9);
  EXPECT_NEAR(2.45, second.samples[4].left.y, 1e-9);
  EXPECT_NEAR(-2.48, second.samples[4].right.y, 1e-9);
  EXPECT_NEAR(0.03, second.samples[4].position_error, 1e-9);
  EXPECT_DOUBLE_EQ(1.0, second.samples.front().last_observed);
  EXPECT_DOUBLE_EQ(1.1, second.samples[4].last_observed);
}

TEST(TemporalLaneMapTest, DelayedMeasurementUsesInterpolatedPoseNotLatest) {
  TemporalLaneMap map(Policy());
  ASSERT_TRUE(map.BeginEpoch("session", 1, {1.0, 0, 0, 0, 0}, true).ok());
  ASSERT_TRUE(map.AddPose("session", 1, {1.2, 2.0, 0, 0, 0}).ok());
  ASSERT_TRUE(map.Observe(Observation(1.1, 1, 1.0), 1.2).ok());
  TemporalLaneSnapshot snapshot;
  ASSERT_TRUE(map.Snapshot(1.2, &snapshot).ok());
  EXPECT_NEAR(1.0, snapshot.samples.front().left.x, 1e-8);
  EXPECT_DOUBLE_EQ(1.1, snapshot.measurement_time);
  EXPECT_FALSE(map.Observe(Observation(1.3, 2, 3.0), 1.3).ok());
}

TEST(TemporalLaneMapTest, RotatedPoseAndBoundaryOrdering) {
  TemporalLaneMap map(Policy());
  const double yaw = 0.5;
  ASSERT_TRUE(map.BeginEpoch("session", 1, {1, 5, 3, yaw, 0}, true).ok());
  ASSERT_TRUE(map.Observe(Observation(1, 1, 0), 1).ok());
  TemporalLaneSnapshot snapshot;
  ASSERT_TRUE(map.Snapshot(1, &snapshot).ok());
  EXPECT_NEAR(5.0 - 2.5 * std::sin(yaw), snapshot.samples.front().left.x, 1e-8);
  EXPECT_NEAR(3.0 + 2.5 * std::cos(yaw), snapshot.samples.front().left.y, 1e-8);
}

TEST(TemporalLaneMapTest, MissingFramesExpireWithoutRefreshingMeasurement) {
  TemporalLaneMap map(Policy());
  ASSERT_TRUE(map.BeginEpoch("session", 1, {1, 0, 0, 0, 0}, true).ok());
  ASSERT_TRUE(map.Observe(Observation(1, 1, 0), 1).ok());
  TemporalLaneSnapshot snapshot;
  ASSERT_TRUE(map.Snapshot(1.2, &snapshot).ok());
  EXPECT_DOUBLE_EQ(1.0, snapshot.measurement_time);
  EXPECT_DOUBLE_EQ(1.4, snapshot.valid_until);
  EXPECT_FALSE(map.Snapshot(1.6, &snapshot).ok());
  EXPECT_TRUE(snapshot.samples.empty());
}

TEST(TemporalLaneMapTest, AssociationConflictInvalidatesInsteadOfFalseSplice) {
  TemporalLaneMap map(Policy());
  ASSERT_TRUE(map.BeginEpoch("session", 1, {1, 0, 0, 0, 0}, true).ok());
  ASSERT_TRUE(map.Observe(Observation(1, 1, 0), 1).ok());
  ASSERT_TRUE(map.AddPose("session", 1, {1.1, 0, 0, 0, 0}).ok());
  EXPECT_FALSE(map.Observe(Observation(1.1, 2, 0, 1.0), 1.1).ok());
  TemporalLaneSnapshot snapshot;
  EXPECT_FALSE(map.Snapshot(1.1, &snapshot).ok());
  ASSERT_TRUE(map.AddPose("session", 1, {1.2, 0, 0, 0, 0}).ok());
  ASSERT_TRUE(map.Observe(Observation(1.2, 3, 0), 1.2).ok());
  ASSERT_TRUE(map.Snapshot(1.2, &snapshot).ok());
  EXPECT_EQ(2U, snapshot.lane_id);
}

TEST(TemporalLaneMapTest, InvalidOrDisconnectedObservationsRejected) {
  for (int fault = 0; fault < 7; ++fault) {
    SCOPED_TRACE(fault);
    TemporalLaneMap map(Policy());
    ASSERT_TRUE(map.BeginEpoch("session", 1, {1, 0, 0, 0, 0}, true).ok());
    ASSERT_TRUE(map.Observe(Observation(1, 1, 0), 1).ok());
    ASSERT_TRUE(map.AddPose("session", 1, {1.1, 0, 0, 0, 0}).ok());
    auto o = Observation(1.1, 2, 0);
    if (fault == 0) o.sequence = 1;
    if (fault == 1) o.generation = 2;
    if (fault == 2) o.forward_confirmed = false;
    if (fault == 3) o.right.clear();
    if (fault == 4) o.left[2].x = std::numeric_limits<double>::quiet_NaN();
    if (fault == 5) o.left.swap(o.right);
    if (fault == 6) {
      for (auto& p : o.left) p.x += 20.0;
      for (auto& p : o.right) p.x += 20.0;
    }
    EXPECT_FALSE(map.Observe(o, 1.1).ok());
  }
}

TEST(TemporalLaneMapTest, ResetRejectsOldDataAndAllowsFreshCoordinates) {
  TemporalLaneMap map(Policy());
  ASSERT_TRUE(map.BeginEpoch("session", 1, {1, 0, 0, 0, 0}, true).ok());
  const auto old = Observation(1, 1, 0);
  ASSERT_TRUE(map.Observe(old, 1).ok());
  ASSERT_TRUE(map.BeginEpoch("restart", 2, {2, 0, 0, 0, 0}, true).ok());
  EXPECT_FALSE(map.Observe(old, 2).ok());
  auto fresh = Observation(2, 1, 0, 0.2);
  fresh.session = "restart";
  fresh.generation = 2;
  ASSERT_TRUE(map.Observe(fresh, 2).ok());
  TemporalLaneSnapshot snapshot;
  ASSERT_TRUE(map.Snapshot(2, &snapshot).ok());
  EXPECT_EQ("restart", snapshot.session);
  EXPECT_NEAR(2.7, snapshot.samples.front().left.y, 1e-8);
  EXPECT_FALSE(map.BeginEpoch("restart", 3, {3, 0, 0, 0, 0}, false).ok());
  EXPECT_FALSE(map.Snapshot(3, &snapshot).ok());
}

TEST(TemporalLaneMapTest, TerminalCoverageDoesNotJumpAtGridBoundaryOrExpiry) {
  TemporalLaneMap map(Policy());
  ASSERT_TRUE(map.BeginEpoch("session", 1, {1, 0, 0, 0, 0}, true).ok());
  ASSERT_TRUE(map.Observe(Observation(1, 1, 0), 1).ok());
  for (int i = 1; i <= 30; ++i) {
    const double now = 1.0 + i * 0.1;
    ASSERT_TRUE(map.AddPose("session", 1, {now, 0, 0, 0, 0}).ok());
    auto observation = Observation(now, i + 1, 0);
    // A sub-millimeter shift crosses floor(u / grid_spacing). After the old
    // end cell expires, coverage must not shrink by a whole half-meter cell.
    for (auto& p : observation.left) p.x -= 0.0001;
    for (auto& p : observation.right) p.x -= 0.0001;
    ASSERT_TRUE(map.Observe(observation, now).ok());
    TemporalLaneSnapshot snapshot;
    ASSERT_TRUE(map.Snapshot(now, &snapshot).ok());
    EXPECT_NEAR(10.0, snapshot.samples.back().left.x, 0.001);
    for (size_t j = 1; j < snapshot.samples.size(); ++j) {
      EXPECT_GE(snapshot.samples[j].left.x - snapshot.samples[j - 1].left.x,
                0.05);
    }
  }
}

TEST(TemporalLaneMapTest, EpochCannotRegressClockAndRejectedResetDisarmsMap) {
  TemporalLaneMap map(Policy());
  ASSERT_TRUE(map.BeginEpoch("session", 1, {1, 0, 0, 0, 0}, true).ok());
  ASSERT_TRUE(map.Observe(Observation(1, 1, 0), 1).ok());
  ASSERT_TRUE(map.AddPose("session", 1, {1.1, 0, 0, 0, 0}).ok());
  EXPECT_FALSE(map.BeginEpoch("restart", 2, {1, 0, 0, 0, 0}, true).ok());
  TemporalLaneSnapshot snapshot;
  EXPECT_FALSE(map.Snapshot(1.1, &snapshot).ok());
  ASSERT_TRUE(map.BeginEpoch("restart", 2, {1.2, 0, 0, 0, 0}, true).ok());
  auto fresh = Observation(1.2, 1, 0);
  fresh.session = "restart";
  fresh.generation = 2;
  ASSERT_TRUE(map.Observe(fresh, 1.2).ok());
  EXPECT_TRUE(map.Snapshot(1.2, &snapshot).ok());
}

TEST(TemporalLaneMapTest, NarrowingTakesEffectBeforeTemporalSmoothing) {
  TemporalLaneMap map(Policy());
  ASSERT_TRUE(map.BeginEpoch("session", 1, {1, 0, 0, 0, 0}, true).ok());
  ASSERT_TRUE(map.Observe(Observation(1, 1, 0), 1).ok());
  ASSERT_TRUE(map.AddPose("session", 1, {1.1, 0, 0, 0, 0}).ok());
  auto narrow = Observation(1.1, 2, 0);
  for (auto& p : narrow.left) p.y = 2.4;
  for (auto& p : narrow.right) p.y = -2.4;
  ASSERT_TRUE(map.Observe(narrow, 1.1).ok());
  TemporalLaneSnapshot snapshot;
  ASSERT_TRUE(map.Snapshot(1.1, &snapshot).ok());
  for (const auto& sample : snapshot.samples) {
    EXPECT_LE(sample.left.y, 2.4);
    EXPECT_GE(sample.right.y, -2.4);
  }
}
}  // namespace
}  // namespace world_model
}  // namespace apollo
