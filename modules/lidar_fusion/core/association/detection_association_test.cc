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

TEST_F(FusionTest, CompatibleMarginAndTrustedViewDoNotRequireCoreAgreement) {
  Obstacle();
  Box();
  // Connected vegetation lies just outside the original detector box.
  Add(2.5, 1.4, 0.8, 15);
  Add(2.3, 1.4, 0.8, 15);
  Sync();
  config.set_connectivity_size(0.3);
  config.mutable_quality()->set_compatible_margin(true);
  config.mutable_quality()->set_trusted_point_view(true);
  config.mutable_spatial()->set_point_views(true);
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_FALSE(output.point_decision(12).has_object_id());
  EXPECT_TRUE(output.point_decision(13).has_object_id());
  EXPECT_EQ(output.object(0).point_id_size(), 7);
  EXPECT_EQ(output.point_view(TRUSTED_OBJECT_POINTS).point_id_size(), 6);
  Run(&det, nullptr);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).classification_use(), CLASSIFICATION_CANDIDATE);
  EXPECT_EQ(output.point_view(TRUSTED_OBJECT_POINTS).point_id_size(), 0);
}

TEST_F(FusionTest, ConflictRetainsClassFlagRawScoreAndObservedOccupancy) {
  Obstacle(15);
  Box();
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_TRUE(output.object(0).class_conflict());
  EXPECT_EQ(output.hypothesis(0).consistency(), CONFLICT);
  EXPECT_TRUE(output.hypothesis(0).existence_supported());
  EXPECT_FALSE(output.hypothesis(0).identity_supported());
  EXPECT_FLOAT_EQ(output.hypothesis(0).raw_detection().score(), 0.8F);
  ASSERT_EQ(output.unknown_size(), 1);
  ASSERT_EQ(output.unknown(0).related_object_id_size(), 1);
  EXPECT_EQ(output.unknown(0).related_object_id(0), output.object(0).id());
  EXPECT_TRUE(std::any_of(output.space_map().cell().begin(),
                          output.space_map().cell().end(),
                          [](const auto& c) { return c.occupancy_obs(); }));
}

TEST_F(FusionTest, CrossThingEvidencePreservesLegacyClassAndConsumerPolicy) {
  auto* mapping = config.add_semantic_mapping();
  mapping->set_raw_id(2);
  mapping->set_group(THING);
  mapping->set_unified_class(PEDESTRIAN);
  Obstacle(2);
  Box();
  Run(&det, &seg);
  const auto& hypothesis = output.hypothesis(0);
  const auto& semantic = hypothesis.association().semantic_evidence();
  ASSERT_TRUE(semantic.cross_thing_disagreement());
  EXPECT_EQ(hypothesis.consistency(), INSUFFICIENT);
  EXPECT_TRUE(hypothesis.existence_supported());
  EXPECT_FALSE(hypothesis.identity_supported());
  ASSERT_EQ(semantic.thing_support_size(), 1);
  EXPECT_EQ(semantic.thing_support(0).unified_class(), PEDESTRIAN);
  EXPECT_EQ(semantic.thing_support(0).point_id_size(), 6);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).unified_class(), CAR);
  EXPECT_FALSE(output.object(0).class_conflict());
  EXPECT_EQ(output.object(0).classification_use(), CLASSIFICATION_CANDIDATE);
  EXPECT_EQ(hypothesis.raw_detection().SerializeAsString(),
            det.detection(0).SerializeAsString());
}

TEST_F(FusionTest, DisagreementIgnoresMarginAndCannotAcceptWeakDetection) {
  auto* mapping = config.add_semantic_mapping();
  mapping->set_raw_id(2);
  mapping->set_group(THING);
  mapping->set_unified_class(PEDESTRIAN);
  Obstacle(2);
  Add(2.35, 1.4, 0.8, 2);
  Add(2.5, 1.4, 0.8, 1);
  Sync();
  Box(0.2F);
  Run(&det, &seg);
  const auto& hypothesis = output.hypothesis(0);
  const auto& association = hypothesis.association();
  const auto& semantic = association.semantic_evidence();
  ASSERT_EQ(association.n_core(), 7U);
  ASSERT_EQ(association.n_margin(), 1U);
  EXPECT_TRUE(semantic.cross_thing_disagreement());
  ASSERT_EQ(semantic.thing_support_size(), 1);
  EXPECT_EQ(semantic.thing_support(0).point_id_size(), 7);
  const auto& ids = semantic.thing_support(0).point_id();
  EXPECT_EQ(std::find(ids.begin(), ids.end(), 13U), ids.end());
  EXPECT_EQ(hypothesis.consistency(), INSUFFICIENT);
  EXPECT_EQ(hypothesis.decision(), BELOW_INSTANCE_THRESHOLD);
  EXPECT_FALSE(hypothesis.has_object_id());
  EXPECT_TRUE(std::none_of(
      output.object().begin(), output.object().end(),
      [](const ObjectCandidate& object) { return object.source() == DET; }));
}

TEST_F(FusionTest, DisconnectedTireIsNotSwallowedByExpandedCarBox) {
  Obstacle();
  Box();
  config.set_association_margin(1);
  Add(2.9, 1.4, 0.4, 0);
  Sync();
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).point_id_size(), 6);
  EXPECT_EQ(output.hypothesis(0).association().n_unconnected_margin(), 1);
  ASSERT_EQ(output.unknown_size(), 1);
  EXPECT_EQ(output.unknown(0).point_id_size(), 1);
  EXPECT_TRUE(output.unknown(0).in_corridor());
  EXPECT_EQ(output.unknown(0).related_object_id_size(), 0);
}

TEST_F(FusionTest, DuplicateInstancesMergeWithoutChangingRawHypotheses) {
  Obstacle();
  Box(0.8F);
  Box(0.9F);
  det.mutable_detection(1)->set_center_y(1.45);
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  ASSERT_EQ(output.hypothesis_size(), 2);
  EXPECT_EQ(output.object(0).point_id_size(), 6);
  EXPECT_FLOAT_EQ(output.object(0).raw_score(), 0.9F);
  EXPECT_FLOAT_EQ(output.hypothesis(0).raw_detection().score(), 0.8F);
  EXPECT_FLOAT_EQ(output.hypothesis(1).raw_detection().score(), 0.9F);
  EXPECT_TRUE(output.point_decision(6).competition());
  EXPECT_EQ(output.point_decision(6).object_id(), output.object(0).id());
}

TEST_F(FusionTest, NearbyInstancesAreNotMergedJustBecauseTheirBoxesOverlap) {
  Obstacle();
  for (double y : {1.3, 1.4}) {
    for (double x : {2.6, 2.7, 2.8}) {
      Add(x, y, 0.8, 1);
    }
  }
  Sync();
  Box();
  Box();
  det.mutable_detection(0)->set_size_x(1.0);
  det.mutable_detection(1)->set_center_x(2.7);
  det.mutable_detection(1)->set_size_x(1.0);
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 2);
  EXPECT_EQ(output.object(0).point_id_size(), 6);
  EXPECT_EQ(output.object(1).point_id_size(), 6);
  EXPECT_NE(output.point_decision(6).object_id(),
            output.point_decision(12).object_id());
}

TEST_F(FusionTest, SemanticEvidenceSeparatesMissingWeakAndUnqualifiedInput) {
  Obstacle();
  Box();
  Run(&det, nullptr);
  auto semantic = output.hypothesis(0).association().semantic_evidence();
  EXPECT_FALSE(semantic.source_available());
  EXPECT_FALSE(semantic.identity_qualified());
  EXPECT_EQ(semantic.n_unavailable(), 6U);
  EXPECT_EQ(semantic.thing_support_size(), 0);

  config.mutable_quality()->set_semantic_identity_unverified(true);
  Run(&det, &seg);
  semantic = output.hypothesis(0).association().semantic_evidence();
  EXPECT_TRUE(semantic.source_available());
  EXPECT_FALSE(semantic.identity_qualified());
  EXPECT_EQ(semantic.n_unavailable(), 0U);
  ASSERT_EQ(semantic.thing_support_size(), 1);
  EXPECT_EQ(semantic.thing_support(0).point_id_size(), 6);
  EXPECT_FALSE(semantic.cross_thing_disagreement());

  config.mutable_quality()->set_semantic_identity_unverified(false);
  for (int i = 6; i < seg.point_label_size(); ++i) {
    seg.mutable_point_label(i)->set_confidence(0.49F);
  }
  Run(&det, &seg);
  semantic = output.hypothesis(0).association().semantic_evidence();
  EXPECT_TRUE(semantic.source_available());
  EXPECT_TRUE(semantic.identity_qualified());
  EXPECT_EQ(semantic.n_low_confidence(), 6U);
  EXPECT_EQ(semantic.thing_support_size(), 0);
  EXPECT_EQ(semantic.group_support_size(), 0);
}

TEST_F(FusionTest, SparseHighScoreBoxesSurviveWithoutSemanticSupport) {
  Box(0.8F);
  Run(&det, nullptr);
  EXPECT_EQ(output.health().mode(), NO_SEG);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).consistency(), INSUFFICIENT);
  EXPECT_EQ(output.object(0).point_id_size(), 0);
  bool inferred_only = false;
  for (const auto& cell : output.space_map().cell()) {
    if (cell.occupancy_det() && !cell.occupancy_obs()) {
      inferred_only = true;
      EXPECT_EQ(cell.traversability(), OCCUPIED);
    }
  }
  EXPECT_TRUE(inferred_only);
}

TEST_F(FusionTest, SparseOrMixedAlternativesRemainExplicitButNotStrong) {
  auto* mapping = config.add_semantic_mapping();
  mapping->set_raw_id(2);
  mapping->set_group(THING);
  mapping->set_unified_class(PEDESTRIAN);
  Obstacle(2);
  Box();
  for (int i = 6; i < 9; ++i) {
    seg.mutable_point_label(i)->set_semantic_label(1);
  }
  Run(&det, &seg);
  auto semantic = output.hypothesis(0).association().semantic_evidence();
  ASSERT_EQ(semantic.thing_support_size(), 2);
  EXPECT_FALSE(semantic.cross_thing_disagreement());
  EXPECT_EQ(semantic.thing_support(0).point_id_size(), 3);
  EXPECT_EQ(semantic.thing_support(1).point_id_size(), 3);

  for (int i = 0; i < 3; ++i) {
    cloud.mutable_point()->RemoveLast();
    seg.mutable_point_label()->RemoveLast();
  }
  for (int i = 6; i < seg.point_label_size(); ++i) {
    seg.mutable_point_label(i)->set_semantic_label(2);
  }
  Sync();
  Run(&det, &seg);
  semantic = output.hypothesis(0).association().semantic_evidence();
  EXPECT_FALSE(output.hypothesis(0).existence_supported());
  EXPECT_FALSE(semantic.cross_thing_disagreement());
  ASSERT_EQ(semantic.thing_support_size(), 1);
  EXPECT_EQ(semantic.thing_support(0).point_id_size(), 3);
}

TEST_F(FusionTest, UnverifiedSemanticIdentityCannotVetoOrPromoteDetections) {
  Obstacle(15);
  Box();
  Box(0.15F);
  config.mutable_quality()->set_separate_weak_detections(true);
  config.mutable_quality()->set_semantic_identity_unverified(true);
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).consistency(), INSUFFICIENT);
  EXPECT_EQ(output.object(0).classification_use(), CLASSIFICATION_CANDIDATE);
  EXPECT_EQ(output.hypothesis(1).decision(), BELOW_INSTANCE_THRESHOLD);
  for (auto& label : *seg.mutable_point_label()) {
    label.set_semantic_label(1);
  }
  Run(nullptr, &seg);
  EXPECT_EQ(output.object_size(), 0);
  EXPECT_GT(output.unknown_size(), 0);
}

TEST_F(FusionTest, WeakClassPolicyRetainsRawHypothesisAndBothOccupancyLayers) {
  Obstacle(15);
  Box(0.15F);
  Run(&det, &seg);
  const auto original = output;
  config.mutable_quality()->set_separate_weak_detections(true);
  Run(&det, &seg);
  ASSERT_EQ(original.object_size(), 1);
  EXPECT_EQ(output.object_size(), 0);
  ASSERT_EQ(output.hypothesis_size(), 1);
  EXPECT_EQ(output.hypothesis(0).decision(), RETAINED_WEAK);
  EXPECT_EQ(output.hypothesis(0).consistency(), CONFLICT);
  EXPECT_EQ(output.hypothesis(0).raw_detection().SerializeAsString(),
            det.detection(0).SerializeAsString());
  EXPECT_FALSE(output.hypothesis(0).has_object_id());
  ASSERT_GT(output.unknown_size(), 0);
  for (const auto& cell : original.space_map().cell()) {
    const auto found =
        std::find_if(output.space_map().cell().begin(),
                     output.space_map().cell().end(), [&](const auto& current) {
                       return current.x_index() == cell.x_index() &&
                              current.y_index() == cell.y_index();
                     });
    ASSERT_NE(found, output.space_map().cell().end());
    EXPECT_EQ(found->occupancy_obs(), cell.occupancy_obs());
    EXPECT_EQ(found->occupancy_det(), cell.occupancy_det());
  }
  det.mutable_detection(0)->set_score(0.8F);
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).classification_use(), IDENTITY_UNCERTAIN);
}

}  // namespace lidar_fusion
}  // namespace apollo
