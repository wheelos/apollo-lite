// Copyright 2026 WheelOS. All Rights Reserved.

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "modules/lidar_fusion/core/runtime/fusion_test_support.h"

namespace apollo {
namespace lidar_fusion {
using testing_support::Config;
using testing_support::FusionTest;

TEST_F(FusionTest, ConfidentForegroundRecallRetainsMixedUnknownRisk) {
  for (double x : {5.0, 5.1, 5.2}) {
    for (double y : {1.3, 1.4}) {
      Add(x, y, 0.8, 1);
    }
  }
  for (double x : {5.25, 5.3, 5.35}) {
    Add(x, 1.35, 0.8, 9);
  }
  Sync();
  Run(nullptr, &seg);
  ASSERT_EQ(output.object_size(), 0);
  ASSERT_EQ(output.unknown_size(), 1);
  const auto baseline = output;
  config.mutable_candidates()->set_recall_confident_foreground(true);
  Run(nullptr, &seg);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).source(), SEG_RECALL);
  EXPECT_EQ(output.object(0).point_id_size(), 6);
  EXPECT_NE(
      std::find(output.object(0).reason().begin(),
                output.object(0).reason().end(), FOREGROUND_SUBSET_RECALL),
      output.object(0).reason().end());
  ASSERT_EQ(output.unknown_size(), 1);
  ASSERT_EQ(output.unknown(0).point_id_size(),
            baseline.unknown(0).point_id_size());
  for (int i = 0; i < output.unknown(0).point_id_size(); ++i) {
    EXPECT_EQ(output.unknown(0).point_id(i), baseline.unknown(0).point_id(i));
  }
  for (int i = 0; i < output.point_evidence_size(); ++i) {
    EXPECT_EQ(output.point_evidence(i).SerializeAsString(),
              baseline.point_evidence(i).SerializeAsString());
    EXPECT_EQ(output.point_decision(i).has_object_id(), i >= 6 && i < 12);
  }
  for (const auto& cell : baseline.space_map().cell()) {
    if (!cell.occupancy_obs() && !cell.occupancy_unknown()) {
      continue;
    }
    const auto found =
        std::find_if(output.space_map().cell().begin(),
                     output.space_map().cell().end(), [&](const auto& current) {
                       return current.x_index() == cell.x_index() &&
                              current.y_index() == cell.y_index();
                     });
    ASSERT_NE(found, output.space_map().cell().end());
    EXPECT_EQ(found->occupancy_obs(), cell.occupancy_obs());
    EXPECT_EQ(found->occupancy_unknown(), cell.occupancy_unknown());
  }
  config.mutable_quality()->set_semantic_identity_unverified(true);
  Run(nullptr, &seg);
  EXPECT_EQ(output.object_size(), 0);
  EXPECT_EQ(output.unknown_size(), 1);
  config.mutable_quality()->set_semantic_identity_unverified(false);
  config.set_unknown_min_points(10);
  config.set_corridor_half_width(0.1);
  Run(nullptr, &seg);
  EXPECT_EQ(output.object_size(), 0);
  EXPECT_EQ(output.unknown_size(), 0);
}

TEST_F(FusionTest, ForegroundRecallCannotConnectThroughRemovedBackground) {
  for (double x : {5.05, 5.15, 5.25, 5.85, 5.95, 6.05}) {
    Add(x, 1.3, 0.8, 1);
  }
  Add(5.45, 1.3, 0.8, 9);
  Add(5.65, 1.3, 0.8, 9);
  Sync();
  config.mutable_candidates()->set_recall_confident_foreground(true);
  Run(nullptr, &seg);
  EXPECT_EQ(output.object_size(), 0);
  ASSERT_EQ(output.unknown_size(), 1);
  EXPECT_EQ(output.unknown(0).point_id_size(), 8);
}

TEST_F(FusionTest, LocalRelationsDoNotInheritWholeSceneReferences) {
  config.set_resolution(0.05);
  config.set_association_margin(0);
  config.set_core_min_points(1);
  config.set_unknown_min_points(1);
  config.mutable_quality()->set_point_supported_unknown(true);
  config.mutable_candidates()->set_adjacent_distance(0);
  for (double x : {1.1, 1.3, 1.5, 1.7, 1.9, 2.1}) {
    Add(x, 1.4, 0.8, 0);
  }
  Sync();
  Box();
  det.mutable_detection(0)->set_center_x(1.7);
  det.mutable_detection(0)->set_size_x(0.1);
  Run(&det, &seg);
  const auto baseline = output;
  ASSERT_EQ(output.unknown_size(), 2);
  for (const auto& unknown : output.unknown()) {
    ASSERT_EQ(unknown.related_object_id_size(), 1);
  }
  config.mutable_candidates()->set_local_object_relations(true);
  Run(&det, &seg);
  ASSERT_EQ(output.unknown_size(), 2);
  for (const auto& unknown : output.unknown()) {
    EXPECT_EQ(unknown.related_object_id_size(), 0);
    EXPECT_EQ(unknown.relation_size(), 0);
  }
  EXPECT_EQ(output.space_map().SerializeAsString(),
            baseline.space_map().SerializeAsString());
  EXPECT_EQ(output.object(0).SerializeAsString(),
            baseline.object(0).SerializeAsString());
  for (int i = 0; i < output.point_evidence_size(); ++i) {
    EXPECT_EQ(output.point_evidence(i).SerializeAsString(),
              baseline.point_evidence(i).SerializeAsString());
  }
}

TEST_F(FusionTest, SemanticRecallDoesNotTreatBackgroundConnectivityAsIdentity) {
  Obstacle();
  Box();
  for (double x : {3.0, 3.1, 3.2}) {
    for (double y : {1.3, 1.4}) {
      Add(x, y, 0.8, 1);
    }
  }
  Add(2.4, 1.4, 0.8, 15);
  Add(2.6, 1.4, 0.8, 15);
  Add(2.8, 1.4, 0.8, 15);
  Sync();
  config.set_connectivity_size(0.3);
  config.mutable_quality()->set_compatible_margin(true);
  config.mutable_candidates()->set_separate_semantic_instances(true);
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 2);
  EXPECT_EQ(output.object(1).source(), SEG_RECALL);
  EXPECT_EQ(output.object(1).point_id_size(), 6);
}

TEST_F(FusionTest, ShadowPartitionsAreReadOnlyAndRequireQualifiedSemantics) {
  Obstacle();
  for (double x : {2.2, 2.3, 2.4, 2.5}) {
    for (double y : {1.3, 1.4, 1.5}) {
      Add(x, y, 0.8, 15);
    }
  }
  Sync();
  for (bool qualified : {false, true}) {
    config.mutable_quality()->set_semantic_identity_unverified(!qualified);
    for (int mode = NORMAL; mode <= GEO_ONLY; ++mode) {
      const auto* detection =
          mode == NO_DET || mode == GEO_ONLY ? nullptr : &det;
      const auto* semantic =
          mode == NO_SEG || mode == GEO_ONLY ? nullptr : &seg;
      config.mutable_candidates()->set_shadow_semantic_partitions(false);
      Run(detection, semantic);
      ASSERT_GT(output.unknown_size(), 0);
      ASSERT_EQ(output.shadow_partition_size(), 0);
      auto control = output;
      control.mutable_health()->clear_latency_ms();
      config.mutable_candidates()->set_shadow_semantic_partitions(true);
      Run(detection, semantic);
      EXPECT_EQ(output.shadow_partition_size() > 0,
                qualified && semantic != nullptr);
      output.clear_shadow_partition();
      output.mutable_health()->clear_latency_ms();
      EXPECT_EQ(output.SerializeAsString(), control.SerializeAsString());
    }
  }
}

TEST_F(FusionTest, SharedVoxelGraphRecomputesConnectivityAfterOwnershipMask) {
  config.set_association_margin(0);
  config.set_core_min_points(1);
  config.set_unknown_min_points(1);
  for (double x : {1.1, 1.3, 1.5, 1.7, 1.9, 2.1}) {
    Add(x, 1.4, 0.8, 0);
  }
  Sync();
  Box();
  auto* box = det.mutable_detection(0);
  box->set_center_x(1.7);
  box->set_size_x(0.1);
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  ASSERT_EQ(output.object(0).point_id_size(), 1);
  ASSERT_EQ(output.unknown_size(), 2);
  EXPECT_EQ(output.unknown(0).point_id_size(), 3);
  EXPECT_EQ(output.unknown(1).point_id_size(), 2);
  Add(1.78, 1.4, 0.8, 0);
  Sync();
  Run(&det, &seg);
  ASSERT_EQ(output.unknown_size(), 1);
  EXPECT_EQ(output.unknown(0).point_id_size(), 6);
}

TEST_F(FusionTest, VoxelAdjacencyRetainsDiagonalAndReverseConnections) {
  cloud.clear_point();
  seg.clear_point_label();
  config.set_min_x(-4);
  config.set_unknown_min_points(1);
  // The lexicographically first voxel reaches the second through the last.
  Add(-0.1, -0.1, 0.9, 0);
  Add(-0.3, 0.1, 0.7, 0);
  Add(-0.3, -0.3, 0.7, 0);
  Add(1.1, 2.1, 0.9, 0);
  Sync();
  Run(nullptr, nullptr);
  ASSERT_EQ(output.unknown_size(), 2);
  ASSERT_EQ(output.unknown(0).point_id_size(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(output.unknown(0).point_id(i), i);
  }
  ASSERT_EQ(output.unknown(1).point_id_size(), 1);
  EXPECT_EQ(output.unknown(1).point_id(0), 3U);
}

}  // namespace lidar_fusion
}  // namespace apollo
