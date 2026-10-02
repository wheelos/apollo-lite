// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/core/lane_relative.h"

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

LanePolicy Policy() {
  LanePolicy policy;
  policy.front = 2.0;
  policy.rear = 1.0;
  policy.half_width = 0.5;
  policy.clearance = 0.1;
  policy.boundary_std = 0.02;
  policy.heading_std = 0.01;
  return policy;
}

LaneGeometry Corridor(double start, double end) {
  LaneGeometry geometry;
  geometry.left = {{start, 1.75, 0.0}, {end, 1.75, 0.0}};
  geometry.right = {{start, -1.75, 0.0}, {end, -1.75, 0.0}};
  return geometry;
}

TEST(LaneRelativeTest, FullCoverageSeparatesRelationAndContainment) {
  LaneRelation relation;
  ASSERT_TRUE(EvaluateLane(Corridor(-5.0, 10.0), Policy(), 0.0, 0.0,
                          &relation).ok());
  EXPECT_DOUBLE_EQ(0.0, relation.lateral);
  EXPECT_DOUBLE_EQ(0.0, relation.heading);
  EXPECT_DOUBLE_EQ(3.5, relation.width);
  EXPECT_TRUE(relation.contained);
}

TEST(LaneRelativeTest, ForwardEvidenceCannotClaimRearContainment) {
  auto policy = Policy();
  policy.reference_x = 5.0;
  LaneRelation relation;
  ASSERT_TRUE(EvaluateLane(Corridor(2.0, 10.0), policy, 0.01, 0.01,
                          &relation).ok());
  EXPECT_FALSE(relation.contained);
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE, relation.containment_reason);
  EXPECT_DOUBLE_EQ(0.0, relation.lateral);
  EXPECT_GT(relation.lateral_std, policy.boundary_std);
}

TEST(LaneRelativeTest, CannotExtrapolateUnseenReferenceOrAcceptCrossing) {
  LaneRelation relation;
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE,
            EvaluateLane(Corridor(2.0, 10.0), Policy(), 0.0, 0.0, &relation).reason);
  auto geometry = Corridor(-5.0, 10.0);
  geometry.right = geometry.left;
  EXPECT_EQ(Reason::INVALID_INPUT,
            EvaluateLane(geometry, Policy(), 0.0, 0.0, &relation).reason);
}

TEST(LaneRelativeTest, UncertaintyCanRevokeContainmentWithoutHidingRelation) {
  LaneRelation relation;
  ASSERT_TRUE(EvaluateLane(Corridor(-10.0, 20.0), Policy(), 0.6, 0.01,
                          &relation).ok());
  EXPECT_FALSE(relation.contained);
  EXPECT_EQ(Reason::COVARIANCE_EXCEEDED, relation.containment_reason);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace apollo
