// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_fusion/core/candidates.h"

#include <cmath>
#include <limits>

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
            CandidateWork* work = nullptr) {
  CandidateResult candidates;
  candidates.objects.Swap(frame->mutable_object());
  candidates.unknowns.Swap(frame->mutable_unknown());
  RefineCandidates(config, frame->point_evidence(), &candidates, work);
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
