// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_fusion/core/candidates.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace lidar_fusion {
namespace {

uint32_t AddPoint(double x, double y, double z, FusionOutput* out) {
  const uint32_t id = out->point_evidence_size();
  auto* point = out->add_point_evidence();
  point->set_point_id(id);
  point->set_finite(true);
  point->mutable_position()->set_x(x);
  point->mutable_position()->set_y(y);
  point->mutable_position()->set_z(z);
  return id;
}

ObjectCandidate* AddBox(uint32_t id, double x, FusionOutput* out) {
  auto* object = out->add_object();
  object->set_id(id);
  object->mutable_box()->mutable_center()->set_x(x);
  auto* size = object->mutable_box()->mutable_size();
  size->set_x(2);
  size->set_y(2);
  size->set_z(2);
  return object;
}

void Refine(const FusionConfig& config, FusionOutput* frame,
            CandidateWork* work = nullptr, const PointIndex* index = nullptr) {
  CandidateResult candidates;
  candidates.objects.Swap(frame->mutable_object());
  candidates.unknowns.Swap(frame->mutable_unknown());
  RefineCandidates(config, frame->point_evidence(), &candidates, work, index);
  candidates.objects.Swap(frame->mutable_object());
  candidates.unknowns.Swap(frame->mutable_unknown());
}

TEST(CandidateTest, SeparatesTouchingClassesAndConnectsFarSparseReturns) {
  FusionConfig c;
  c.set_min_x(0);
  c.set_max_x(60);
  c.set_connectivity_size(0.2);
  c.mutable_candidates()->set_separate_semantic_instances(true);
  c.mutable_candidates()->set_far_connectivity_size(0.6);
  FusionOutput out;
  auto add = [&](double x, Group group, UnifiedClass cls) {
    auto* p = out.add_point_evidence();
    p->set_finite(true);
    p->set_group(group);
    p->set_unified_class(cls);
    p->mutable_position()->set_x(x);
    p->mutable_position()->set_y(1);
    p->mutable_position()->set_z(1);
    out.add_point_decision()->set_explanation(UNEXPLAINED);
  };
  add(1, THING, CAR);
  add(1.1, THING, PEDESTRIAN);
  add(1.2, STATIC_STRUCT, CLASS_UNKNOWN);
  add(40, THING, CAR);
  add(40.5, THING, CAR);
  add(2, THING, CAR);
  add(2.5, THING, CAR);
  const auto clusters =
      CandidateComponents(c, out.point_evidence(), out.point_decision());
  EXPECT_NE(clusters[0], clusters[1]);
  EXPECT_NE(clusters[1], clusters[2]);
  EXPECT_EQ(clusters[3], clusters[4]);
  EXPECT_NE(clusters[5], clusters[6]);
}

TEST(CandidateTest, ShadowPartitionsSeparateSeedsWithoutCrossingBackground) {
  FusionConfig c;
  c.set_connectivity_size(0.2);
  c.mutable_candidates()->set_shadow_semantic_partitions(true);
  c.mutable_candidates()->set_far_connectivity_size(0.6);
  c.mutable_candidates()->set_far_distance(1);
  FusionOutput out;
  auto* unknown = out.add_unknown();
  unknown->set_id(71);
  auto add = [&](double x, Group group, UnifiedClass cls, float confidence) {
    const auto id = AddPoint(x, 1, 1, &out);
    auto* p = out.mutable_point_evidence(id);
    p->set_group(group);
    p->set_unified_class(cls);
    p->set_semantic_probability(confidence);
    unknown->add_point_id(id);
  };
  add(1, THING, CAR, 0.95F);
  add(1.1, THING, CAR, 0.95F);
  add(1.26, STATIC_STRUCT, CLASS_UNKNOWN, 0.95F);
  add(1.45, THING, CAR, 0.95F);
  add(1.35, STATIC_STRUCT, CLASS_UNKNOWN, 0.95F);
  add(1.5, THING, PEDESTRIAN, 0.95F);
  add(1.6, VEGETATION, CLASS_UNKNOWN, 0.95F);
  add(1.2, THING, CAR, 0.4F);
  add(1.7, GENERIC_OBS, CLASS_UNKNOWN, 0.95F);
  const auto original = out.SerializeAsString();
  PartitionUnknowns(c, out.point_evidence(), out.unknown(),
                    out.mutable_shadow_partition());
  ASSERT_EQ(out.shadow_partition_size(), 6);
  std::set<uint32_t> all;
  std::vector<int> cars;
  for (const auto& partition : out.shadow_partition()) {
    EXPECT_EQ(partition.parent_unknown_id(), 71U);
    for (uint32_t id : partition.point_id()) {
      EXPECT_TRUE(all.insert(id).second);
    }
    if (partition.kind() == PARTITION_THING_SUPPORT &&
        partition.unified_class() == CAR) {
      cars.push_back(partition.point_id_size());
    }
    if (partition.kind() == PARTITION_UNEXPLAINED) {
      EXPECT_EQ(partition.point_id_size(), 2);
    }
  }
  std::sort(cars.begin(), cars.end());
  EXPECT_EQ(cars, (std::vector<int>{1, 2}));
  EXPECT_EQ(all, (std::set<uint32_t>{0, 1, 2, 3, 4, 5, 6, 7, 8}));
  out.clear_shadow_partition();
  EXPECT_EQ(out.SerializeAsString(), original);
}

TEST(CandidateTest, ShadowResidualRetainsGeometricallyRejectedSupport) {
  FusionConfig c;
  c.mutable_candidates()->set_shadow_semantic_partitions(true);
  FusionOutput out;
  auto* unknown = out.add_unknown();
  unknown->set_id(1);
  for (int i = 0; i < 4; ++i) {
    const auto id = AddPoint(1 + i * 0.1, 1, 1, &out);
    auto* p = out.mutable_point_evidence(id);
    p->set_group(THING);
    p->set_unified_class(CAR);
    p->set_semantic_probability(0.95F);
    unknown->add_point_id(id);
  }
  out.mutable_point_evidence(1)->set_finite(false);
  out.mutable_point_evidence(2)->set_ground_valid(true);
  out.mutable_point_evidence(2)->set_height_above_ground(0);
  out.mutable_point_evidence(3)->mutable_position()->set_x(c.max_x());
  PartitionUnknowns(c, out.point_evidence(), out.unknown(),
                    out.mutable_shadow_partition());
  ASSERT_EQ(out.shadow_partition_size(), 2);
  EXPECT_EQ(out.shadow_partition(0).kind(), PARTITION_THING_SUPPORT);
  EXPECT_EQ(out.shadow_partition(0).point_id_size(), 1);
  EXPECT_EQ(out.shadow_partition(1).kind(), PARTITION_UNEXPLAINED);
  EXPECT_EQ(out.shadow_partition(1).point_id_size(), 3);
  EXPECT_EQ(out.shadow_partition(1).unified_class(), CLASS_UNKNOWN);
  c.mutable_candidates()->set_shadow_semantic_partitions(false);
  PartitionUnknowns(c, out.point_evidence(), out.unknown(),
                    out.mutable_shadow_partition());
  EXPECT_EQ(out.shadow_partition_size(), 0);
}

TEST(CandidateTest, DetectionExtentUsesOnlyConfidentAssociatedClassSupport) {
  FusionConfig c;
  c.set_core_min_points(2);
  c.set_semantic_min_confidence(0.8);
  c.set_association_margin(0.3);
  FusionOutput out;
  auto* object = AddBox(1, 0, &out);
  object->set_source(DET);
  object->set_group(THING);
  object->set_unified_class(CAR);
  object->set_consistency(SUPPORT);
  object->set_classification_use(CLASSIFICATION_CANDIDATE);
  auto add = [&](double x, Group group, UnifiedClass cls, double probability,
                 bool associated) {
    const auto id = AddPoint(x, 0, 0, &out);
    auto* p = out.mutable_point_evidence(id);
    p->set_group(group);
    p->set_unified_class(cls);
    p->set_semantic_probability(probability);
    if (associated) {
      object->add_point_id(id);
    }
  };
  add(-0.5, THING, CAR, 0.9, true);
  add(0.5, THING, CAR, 0.9, true);
  add(1.2, THING, CAR, 0.9, true);
  add(-1.25, THING, PEDESTRIAN, 0.9, true);
  add(-1.29, SURFACE, CAR, 0.9, true);
  add(-1.28, THING, CAR, 0.7, true);
  add(-1.31, THING, CAR, 0.9, true);
  add(1.49, THING, CAR, 0.9, true);
  add(-1.2, THING, CAR, 0.9, false);
  const auto original_box = object->box().SerializeAsString();
  Refine(c, &out);
  EXPECT_EQ(object->box().SerializeAsString(), original_box);
  c.mutable_candidates()->set_extend_detection_support(true);
  Refine(c, &out);
  EXPECT_DOUBLE_EQ(object->box().center().x(), 0);
  EXPECT_NEAR(object->box().size().x(), 2.4, 1e-12);
  EXPECT_DOUBLE_EQ(object->box().size().y(), 2);
  EXPECT_DOUBLE_EQ(object->box().size().z(), 2);
  ASSERT_EQ(object->reason_size(), 1);
  EXPECT_EQ(object->reason(0), ASSOCIATED_SUPPORT_EXTENT);
  const auto refined = object->SerializeAsString();
  Refine(c, &out);
  EXPECT_EQ(object->SerializeAsString(), refined);
}

TEST(CandidateTest, DetectionExtentPreservesYawAndRequiresSupportedIdentity) {
  FusionConfig c;
  c.set_core_min_points(2);
  c.mutable_candidates()->set_extend_detection_support(true);
  for (bool supported : {false, true}) {
    FusionOutput out;
    auto* object = AddBox(1, 0, &out);
    object->set_source(DET);
    object->set_group(THING);
    object->set_unified_class(CAR);
    object->set_classification_use(CLASSIFICATION_CANDIDATE);
    object->set_consistency(supported ? SUPPORT : INSUFFICIENT);
    const double yaw = std::acos(-1) / 4;
    object->mutable_box()->set_qz(std::sin(yaw / 2));
    object->mutable_box()->set_qw(std::cos(yaw / 2));
    for (double x : {-0.5, 0.5, 1.2}) {
      const auto id = AddPoint(std::cos(yaw) * x, std::sin(yaw) * x, 0, &out);
      auto* point = out.mutable_point_evidence(id);
      point->set_group(THING);
      point->set_unified_class(CAR);
      point->set_semantic_probability(0.95);
      object->add_point_id(id);
    }
    const auto original = object->SerializeAsString();
    c.mutable_quality()->set_semantic_identity_unverified(true);
    Refine(c, &out);
    EXPECT_EQ(object->SerializeAsString(), original);
    c.mutable_quality()->set_semantic_identity_unverified(false);
    Refine(c, &out);
    EXPECT_DOUBLE_EQ(object->box().qz(), std::sin(yaw / 2));
    EXPECT_DOUBLE_EQ(object->box().qw(), std::cos(yaw / 2));
    if (supported) {
      EXPECT_DOUBLE_EQ(object->box().center().x(), 0);
      EXPECT_DOUBLE_EQ(object->box().center().y(), 0);
      EXPECT_NEAR(object->box().size().x(), 2.4, 1e-12);
    } else {
      EXPECT_EQ(object->SerializeAsString(), original);
    }
  }
}

TEST(CandidateTest, DetectionExtentRejectsUnassignedBackgroundShell) {
  FusionConfig c;
  c.set_min_x(-5);
  c.set_max_x(5);
  c.set_min_y(-5);
  c.set_max_y(5);
  c.set_core_min_points(2);
  c.set_semantic_min_confidence(0.8);
  c.set_conflict_ratio(0.7);
  c.mutable_candidates()->set_extend_detection_support(true);
  for (Group background : {STATIC_STRUCT, VEGETATION}) {
    for (double yaw : {0., std::acos(-1) / 4}) {
      FusionOutput out;
      auto* object = AddBox(1, 0, &out);
      object->set_source(DET);
      object->set_group(THING);
      object->set_unified_class(CAR);
      object->set_consistency(SUPPORT);
      object->set_classification_use(CLASSIFICATION_CANDIDATE);
      object->mutable_box()->set_qz(std::sin(yaw / 2));
      object->mutable_box()->set_qw(std::cos(yaw / 2));
      auto add = [&](double x, double y, Group group, bool associated) {
        const auto id =
            AddPoint(std::cos(yaw) * x - std::sin(yaw) * y,
                     std::sin(yaw) * x + std::cos(yaw) * y, 0, &out);
        auto* point = out.mutable_point_evidence(id);
        point->set_group(group);
        point->set_unified_class(group == THING ? CAR : CLASS_UNKNOWN);
        point->set_semantic_probability(0.95);
        if (associated) {
          object->add_point_id(id);
        }
      };
      for (double x : {-0.5, 0.5, 1.2}) {
        add(x, 0, THING, true);
      }
      for (double y : {-0.2, 0., 0.2}) {
        add(1.1, y, background, false);
      }
      const auto original = object->SerializeAsString();
      PointIndex index;
      for (const auto& point : out.point_evidence()) {
        index[{static_cast<int>(
                   std::floor(point.position().x() / c.resolution())),
               static_cast<int>(
                   std::floor(point.position().y() / c.resolution()))}]
            .push_back(point.point_id());
      }
      Refine(c, &out);
      EXPECT_EQ(object->SerializeAsString(), original);
      Refine(c, &out, nullptr, &index);
      EXPECT_EQ(object->SerializeAsString(), original);
    }
  }
}

TEST(CandidateTest, DetectionExtentIgnoresExistingOrNonVotingBackground) {
  FusionConfig c;
  c.set_core_min_points(2);
  c.set_semantic_min_confidence(0.8);
  c.mutable_candidates()->set_extend_detection_support(true);
  FusionOutput out;
  auto* object = AddBox(1, 0, &out);
  object->set_source(DET);
  object->set_group(THING);
  object->set_unified_class(CAR);
  object->set_consistency(SUPPORT);
  object->set_classification_use(CLASSIFICATION_CANDIDATE);
  for (double x : {-0.5, 0.5, 1.2}) {
    const auto id = AddPoint(x, 0, 0, &out);
    auto* point = out.mutable_point_evidence(id);
    point->set_group(THING);
    point->set_unified_class(CAR);
    point->set_semantic_probability(0.95);
    object->add_point_id(id);
  }
  for (double x : {0.5, 1.1, 1.3}) {
    const auto id = AddPoint(x, 0, 0, &out);
    auto* point = out.mutable_point_evidence(id);
    point->set_group(STATIC_STRUCT);
    point->set_semantic_probability(x == 1.1 ? 0.7 : 0.95);
  }
  const auto ground = AddPoint(1.1, 0, 0, &out);
  out.mutable_point_evidence(ground)->set_group(STATIC_STRUCT);
  out.mutable_point_evidence(ground)->set_semantic_probability(0.95);
  out.mutable_point_evidence(ground)->set_is_ground_geo(true);
  Refine(c, &out);
  EXPECT_NEAR(object->box().size().x(), 2.4, 1e-12);
  EXPECT_EQ(object->reason(0), ASSOCIATED_SUPPORT_EXTENT);
}

TEST(CandidateTest, ForegroundRecallRejectsAlternativeConnectivity) {
  FusionConfig c;
  c.mutable_candidates()->set_recall_confident_foreground(true);
  EXPECT_TRUE(ValidateCandidates(c));
  c.mutable_candidates()->set_separate_semantic_instances(true);
  EXPECT_FALSE(ValidateCandidates(c));
  c.mutable_candidates()->set_separate_semantic_instances(false);
  c.mutable_candidates()->set_far_connectivity_size(c.connectivity_size());
  EXPECT_FALSE(ValidateCandidates(c));
}

TEST(CandidateTest, FitsRotatedRecallAndConvexUnknownWithoutChangingEvidence) {
  FusionConfig c;
  c.mutable_candidates()->set_oriented_recall(true);
  c.mutable_candidates()->set_convex_unknown(true);
  FusionOutput out;
  auto* object = out.add_object();
  object->set_source(SEG_RECALL);
  object->set_id(1);
  object->mutable_box()->mutable_size()->set_z(1);
  auto* unknown = out.add_unknown();
  unknown->set_z_min(0);
  unknown->set_z_max(1);
  const double yaw = std::acos(-1) / 4;
  for (double x : {-2., 2.}) {
    for (double y : {-0.5, 0.5}) {
      auto* p = out.add_point_evidence();
      const auto id = out.point_evidence_size() - 1;
      p->set_point_id(id);
      p->mutable_position()->set_x(10 + std::cos(yaw) * x - std::sin(yaw) * y);
      p->mutable_position()->set_y(std::sin(yaw) * x + std::cos(yaw) * y);
      object->add_point_id(id);
      unknown->add_point_id(id);
    }
  }
  const auto evidence = out.point_evidence(0).SerializeAsString();
  Refine(c, &out);
  EXPECT_NEAR(object->box().size().x(), 4, 1e-8);
  EXPECT_NEAR(object->box().size().y(), 1, 1e-8);
  EXPECT_NEAR(object->box().qz(), std::sin(yaw / 2), 1e-8);
  EXPECT_FALSE(object->orientation_reliable());
  EXPECT_GE(unknown->polygon_size(), 4);
  ASSERT_EQ(unknown->relation_size(), 1);
  EXPECT_EQ(unknown->relation(0).kind(), RELATION_DUPLICATE);
  EXPECT_EQ(out.point_evidence(0).SerializeAsString(), evidence);
}

TEST(CandidateTest, UsesActualPointsAndExactThreeDimensionalAdjacencyBoundary) {
  FusionConfig c;
  c.mutable_candidates()->set_adjacent_distance(0.625);
  FusionOutput out;
  AddBox(4, 0, &out);
  for (double offset : {0., 1e-8, 100.}) {
    auto* unknown = out.add_unknown();
    unknown->add_point_id(AddPoint(1.5 + offset, 1.375, 0, &out));
    unknown->add_polygon()->set_x(-1000);
    unknown->set_z_min(-1000);
    unknown->set_z_max(-999);
  }
  auto* vertical = out.add_unknown();
  vertical->add_point_id(AddPoint(0, 0, 1.625, &out));
  auto* far_vertical = out.add_unknown();
  far_vertical->add_point_id(AddPoint(0, 0, 1.625 + 1e-8, &out));
  far_vertical->add_polygon()->set_x(0);
  const auto evidence = out.point_evidence(0).SerializeAsString();
  Refine(c, &out);
  ASSERT_EQ(out.unknown(0).relation_size(), 1);
  EXPECT_EQ(out.unknown(0).relation(0).kind(), RELATION_ADJACENT);
  EXPECT_EQ(out.unknown(0).related_object_id(0), 4);
  EXPECT_EQ(out.unknown(1).relation_size(), 0);
  EXPECT_EQ(out.unknown(2).relation_size(), 0);
  ASSERT_EQ(out.unknown(3).relation_size(), 1);
  EXPECT_EQ(out.unknown(3).relation(0).kind(), RELATION_ADJACENT);
  EXPECT_EQ(out.unknown(4).relation_size(), 0);
  EXPECT_EQ(out.point_evidence(0).SerializeAsString(), evidence);
  EXPECT_EQ(out.unknown(0).polygon(0).x(), -1000);
}

TEST(CandidateTest, PreservesTheImmediatelyOutsideAdjacencyDecision) {
  FusionConfig c;
  c.mutable_candidates()->set_adjacent_distance(0.5);
  FusionOutput out;
  AddBox(1, 0, &out);
  for (double x : {1.5, std::nextafter(1.5, 2.)}) {
    out.add_unknown()->add_point_id(AddPoint(x, 0, 0, &out));
  }
  Refine(c, &out);
  ASSERT_EQ(out.unknown(0).relation_size(), 1);
  EXPECT_EQ(out.unknown(0).relation(0).kind(), RELATION_ADJACENT);
  EXPECT_EQ(out.unknown(1).relation_size(), 0);
}

TEST(CandidateTest, PreservesSupportInsideYawAndFullThreeDimensionalRotations) {
  FusionConfig c;
  c.mutable_candidates()->set_adjacent_distance(0);
  for (bool full_rotation : {false, true}) {
    FusionOutput out;
    auto* object = AddBox(3, 0, &out);
    auto* box = object->mutable_box();
    box->mutable_size()->set_x(8);
    box->mutable_size()->set_z(0.5);
    if (full_rotation) {
      box->set_qw(0.5);
      box->set_qx(0.5);
      box->set_qy(0.5);
      box->set_qz(0.5);
      out.add_unknown()->add_point_id(AddPoint(0, 3.5, 0, &out));
    } else {
      box->set_qw(std::cos(std::acos(-1) / 8));
      box->set_qz(std::sin(std::acos(-1) / 8));
      out.add_unknown()->add_point_id(AddPoint(2.5, 2.5, 0, &out));
    }
    Refine(c, &out);
    ASSERT_EQ(out.unknown(0).relation_size(), 1);
    EXPECT_EQ(out.unknown(0).relation(0).kind(), RELATION_SUPPLEMENT);
  }
}

TEST(CandidateTest, RetainsReferencedKindsOrderingAndUnrelatedFields) {
  FusionConfig c;
  FusionOutput out;
  const auto point = AddPoint(0, 0, 0, &out);
  AddBox(9, 100, &out)->set_class_conflict(true);
  AddBox(7, 100, &out)->add_point_id(point);
  AddBox(8, 0, &out);
  AddBox(6, 100, &out);
  auto* unknown = out.add_unknown();
  unknown->add_point_id(point);
  unknown->add_reason(AMBIGUOUS_OBJECT_RELATION);
  unknown->set_track_id(123);
  for (uint32_t id : {9, 7, 6, 99, 7}) {
    unknown->add_related_object_id(id);
  }
  unknown->add_relation()->set_object_id(1000);
  FusionOutput expected = out;
  auto* result = expected.mutable_unknown(0);
  result->clear_relation();
  result->clear_related_object_id();
  for (uint32_t id : {6, 7, 8, 9, 99}) {
    result->add_related_object_id(id);
  }
  auto relation = [&](uint32_t id, RelationKind kind) {
    auto* r = result->add_relation();
    r->set_object_id(id);
    r->set_kind(kind);
  };
  relation(9, RELATION_AMBIGUOUS);
  relation(7, RELATION_DUPLICATE);
  relation(8, RELATION_SUPPLEMENT);
  relation(6, RELATION_ADJACENT);
  Refine(c, &out);
  EXPECT_EQ(out.SerializeAsString(), expected.SerializeAsString());
}

TEST(CandidateTest, ReferencesCreatedByEarlierDuplicateIdsBypassCulling) {
  FusionConfig c;
  FusionOutput out;
  AddBox(42, 0, &out);
  AddBox(42, 100, &out)->set_class_conflict(true);
  out.add_unknown()->add_point_id(AddPoint(0, 0, 0, &out));
  Refine(c, &out);
  ASSERT_EQ(out.unknown(0).relation_size(), 2);
  EXPECT_EQ(out.unknown(0).relation(0).kind(), RELATION_SUPPLEMENT);
  EXPECT_EQ(out.unknown(0).relation(1).kind(), RELATION_AMBIGUOUS);
  ASSERT_EQ(out.unknown(0).related_object_id_size(), 1);
  EXPECT_EQ(out.unknown(0).related_object_id(0), 42);
}

TEST(CandidateTest, RetainsEmptyDegenerateAndNonUnitQuaternionBehavior) {
  FusionConfig c;
  FusionOutput out;
  auto* object = AddBox(5, 0, &out);
  object->mutable_box()->mutable_size()->Clear();
  out.add_unknown();
  out.add_unknown()->add_related_object_id(5);
  out.add_unknown()->add_point_id(AddPoint(0, 0, 0, &out));
  out.add_unknown()->add_point_id(AddPoint(0.5, 0, 0, &out));
  Refine(c, &out);
  EXPECT_EQ(out.unknown(0).relation_size(), 0);
  ASSERT_EQ(out.unknown(1).relation_size(), 1);
  EXPECT_EQ(out.unknown(1).relation(0).kind(), RELATION_ADJACENT);
  ASSERT_EQ(out.unknown(2).relation_size(), 1);
  EXPECT_EQ(out.unknown(2).relation(0).kind(), RELATION_SUPPLEMENT);
  ASSERT_EQ(out.unknown(3).relation_size(), 1);
  EXPECT_EQ(out.unknown(3).relation(0).kind(), RELATION_ADJACENT);

  out.Clear();
  object = AddBox(6, 0, &out);
  object->mutable_box()->set_qw(0);
  object->mutable_box()->set_qx(std::sqrt(0.5));
  out.add_unknown()->add_point_id(AddPoint(0, 100, 0, &out));
  Refine(c, &out);
  ASSERT_EQ(out.unknown(0).relation_size(), 1);
  EXPECT_EQ(out.unknown(0).relation(0).kind(), RELATION_SUPPLEMENT);
}

TEST(CandidateTest, ExtremeFiniteCoordinatesRetainExactQuaternionArithmetic) {
  FusionConfig c;
  FusionOutput out;
  auto* object = AddBox(1, 0, &out);
  object->mutable_box()->set_qw(0);
  object->mutable_box()->set_qx(1);
  const double large = std::numeric_limits<double>::max();
  out.add_unknown()->add_point_id(AddPoint(large, large, large, &out));
  Refine(c, &out);
  ASSERT_EQ(out.unknown(0).relation_size(), 1);
  EXPECT_EQ(out.unknown(0).relation(0).kind(), RELATION_SUPPLEMENT);
}

TEST(CandidateTest, RelationExistenceQueriesStopWithoutScanningDistantSupport) {
  FusionConfig c;
  FusionOutput out;
  AddBox(1, 0, &out);
  AddBox(2, 100, &out)->set_class_conflict(true);
  auto* unknown = out.add_unknown();
  unknown->add_related_object_id(2);
  unknown->add_point_id(AddPoint(0, 0, 0, &out));
  for (int i = 0; i < 1000; ++i) {
    unknown->add_point_id(AddPoint(50 + i, 50, 5, &out));
  }
  CandidateWork work;
  const auto evidence = out.point_evidence(0).SerializeAsString();
  Refine(c, &out, &work);
  ASSERT_EQ(unknown->relation_size(), 2);
  EXPECT_EQ(unknown->relation(0).kind(), RELATION_SUPPLEMENT);
  EXPECT_EQ(unknown->relation(1).kind(), RELATION_AMBIGUOUS);
  EXPECT_EQ(work.pairs, 2);
  EXPECT_EQ(work.point_transforms, 1);
  EXPECT_EQ(out.point_evidence(0).SerializeAsString(), evidence);
}

TEST(CandidateTest, PointBlocksCullRemoteSupportButRetainLateBoundaryPoint) {
  FusionConfig c;
  c.mutable_candidates()->set_adjacent_distance(0.5);
  FusionOutput out;
  AddBox(1, 0, &out);
  AddBox(2, 0, &out);
  auto* unknown = out.add_unknown();
  unknown->add_related_object_id(2);
  for (int i = 0; i < 1024; ++i) {
    unknown->add_point_id(AddPoint(100 + i, 50, 5, &out));
  }
  unknown->add_point_id(AddPoint(1.5, 0, 0, &out));
  CandidateWork work;
  Refine(c, &out, &work);
  ASSERT_EQ(unknown->relation_size(), 2);
  EXPECT_EQ(unknown->relation(0).kind(), RELATION_ADJACENT);
  EXPECT_EQ(unknown->relation(1).kind(), RELATION_ADJACENT);
  EXPECT_EQ(work.point_transforms, 1);
}

}  // namespace
}  // namespace lidar_fusion
}  // namespace apollo
