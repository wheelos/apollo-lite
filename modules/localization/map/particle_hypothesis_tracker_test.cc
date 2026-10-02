// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/map/particle_hypothesis_tracker.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

HypothesisTrackPolicy Policy() {
  HypothesisTrackPolicy policy;
  policy.maximum_tracks = 8;
  policy.minimum_consistent_frames = 3;
  policy.minimum_consistency_span = 1.0;
  policy.maximum_unobserved_gap = 0.6;
  policy.anchor_translation_gate = 0.15;
  policy.anchor_rotation_gate = 0.1;
  policy.association_translation_gate = 0.15;
  policy.association_rotation_gate = 0.1;
  return policy;
}

LocalState Local(double time, uint64_t sequence, double x) {
  LocalState local;
  local.epoch = {"session", 1};
  local.stamp = {time, time, sequence, "clock"};
  local.position.x() = x;
  local.valid = true;
  return local;
}

Eigen::Isometry3d MapOdom(double x) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().x() = x;
  return pose;
}

ParticleSnapshot Snapshot(const LocalState& local, uint64_t sequence,
                         const std::vector<double>& anchors) {
  ParticleSnapshot snapshot;
  snapshot.identity = {"map", "v1", "map", "calibrated"};
  snapshot.epoch = local.epoch;
  snapshot.stamp = local.stamp;
  snapshot.source = "lidar";
  snapshot.scan_sequence = sequence;
  for (double anchor : anchors) {
    ParticleMode mode;
    mode.support_region_id = anchor < 10.0 ? "west" : "east";
    mode.pose = MapOdom(anchor) * Pose(local);
    mode.search_mass = 1.0 / static_cast<double>(anchors.size());
    snapshot.modes.push_back(mode);
  }
  return snapshot;
}

TEST(ParticleHypothesisTrackerTest, TracksMultipleModesWithoutHandover) {
  ParticleHypothesisTracker tracker(Policy());
  HypothesisTrackingSnapshot output;
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    const double index = static_cast<double>(sequence - 1);
    const double time = 10.0 + 0.5 * index;
    const auto local = Local(time, sequence, 0.1 * index);
    ASSERT_TRUE(tracker.Update(Snapshot(local, sequence, {5.0, 15.0}),
                               local, &output).ok());
    ASSERT_EQ(2U, output.tracks.size());
    EXPECT_TRUE(output.diagnostic_only);
    EXPECT_FALSE(output.handover_eligible);
    for (const auto& track : output.tracks) {
      EXPECT_TRUE(track.diagnostic_only);
      EXPECT_EQ(sequence == 3, track.consistency_span_satisfied);
      EXPECT_NEAR(track.support_region_id == "west" ? 5.0 : 15.0,
                  track.map_to_odom_anchor.translation().x(), 1e-9);
    }
  }
}

TEST(ParticleHypothesisTrackerTest, FixedAnchorRejectsSlowAlignmentDrift) {
  ParticleHypothesisTracker tracker(Policy());
  HypothesisTrackingSnapshot output;
  auto local = Local(10.0, 1, 0.0);
  ASSERT_TRUE(tracker.Update(Snapshot(local, 1, {5.0}), local, &output).ok());
  local = Local(10.5, 2, 0.1);
  ASSERT_TRUE(tracker.Update(Snapshot(local, 2, {5.1}), local, &output).ok());
  ASSERT_EQ(1U, output.tracks.size());
  EXPECT_EQ(2U, output.tracks.front().consistent_frames);
  local = Local(11.0, 3, 0.2);
  ASSERT_TRUE(tracker.Update(Snapshot(local, 3, {5.2}), local, &output).ok());
  ASSERT_EQ(2U, output.tracks.size());
  EXPECT_TRUE(output.tracks.front().observed_this_update);
  EXPECT_EQ(1U, output.tracks.front().consistent_frames);
  EXPECT_EQ(2U, output.tracks.front().id);
  EXPECT_FALSE(output.tracks.back().observed_this_update);
  EXPECT_EQ(1U, output.tracks.back().id);
}

TEST(ParticleHypothesisTrackerTest, RejectsRegressingEvidenceAtomically) {
  ParticleHypothesisTracker tracker(Policy());
  HypothesisTrackingSnapshot output;
  auto local = Local(10.0, 1, 0.0);
  ASSERT_TRUE(tracker.Update(Snapshot(local, 1, {5.0}), local, &output).ok());
  const auto previous = output;
  auto regressing = Local(9.0, 2, 0.0);
  EXPECT_EQ(Reason::TIMESTAMP_REGRESSION,
            tracker.Update(Snapshot(regressing, 2, {5.0}), regressing,
                           &output).reason);
  EXPECT_EQ(previous.stamp.time, output.stamp.time);
  EXPECT_EQ(previous.tracks.front().id, output.tracks.front().id);
  auto next = Local(10.5, 2, 0.1);
  ASSERT_TRUE(tracker.Update(Snapshot(next, 2, {5.1}), next, &output).ok());
  EXPECT_EQ(2U, output.tracks.front().consistent_frames);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace apollo
