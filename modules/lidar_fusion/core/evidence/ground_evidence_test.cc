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

TEST_F(FusionTest, GroundHistoryCannotEraseCurrentMeasuredObstacle) {
  Obstacle(9);
  Run(nullptr, &seg);
  const auto baseline = output.space_map().SerializeAsString();
  GroundPrior prior;
  for (const auto& p : output.point_evidence()) {
    auto ground = p.position();
    ground.set_z(0.8);
    prior.push_back(ground);
  }
  Run(nullptr, &seg, &prior);
  EXPECT_EQ(output.space_map().SerializeAsString(), baseline);
  ASSERT_TRUE(output.point_evidence(6).has_ground_prior_z());
  EXPECT_FALSE(output.point_evidence(6).ground_prior_consistent());
  EXPECT_FALSE(output.point_evidence(6).is_ground_geo());
}

TEST_F(FusionTest, IndependentGroundPriorDoesNotChangeObjectPriorRecovery) {
  Obstacle(0);
  Box();
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  const ObjectPrior objects{output.object(0)};
  const auto prediction = objects.front().SerializeAsString();
  GroundPrior ground;
  for (const auto& point : output.point_evidence()) {
    auto sample = point.position();
    sample.set_z(0);
    ground.push_back(sample);
  }
  Run(nullptr, &seg, nullptr, &objects);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).source(), TRACK_PRIOR);
  EXPECT_FALSE(output.point_evidence(6).has_ground_prior_z());
  const auto recovered = output.object(0).SerializeAsString();
  const auto space = output.space_map().SerializeAsString();

  Run(nullptr, &seg, &ground, &objects);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).SerializeAsString(), recovered);
  EXPECT_EQ(output.space_map().SerializeAsString(), space);
  EXPECT_TRUE(output.point_evidence(6).has_ground_prior_z());
  EXPECT_EQ(objects.front().SerializeAsString(), prediction);
}

TEST_F(FusionTest, NearGroundThingKeepsEvidenceWithoutInventingOccupancy) {
  config.mutable_spatial()->set_point_views(true);
  const int index = cloud.point_size();
  Add(2, 1.4, 0.125, 1);
  Sync();
  Run(nullptr, &seg);
  const auto& point = output.point_evidence(index);
  ASSERT_TRUE(point.ground_valid());
  EXPECT_FALSE(point.is_ground_geo());
  EXPECT_NEAR(point.height_above_ground(), 0.125, 1e-6);
  EXPECT_EQ(point.unified_class(), CAR);
  EXPECT_EQ(output.point_decision(index).explanation(), UNEXPLAINED);
  EXPECT_FALSE(output.point_decision(index).has_object_id());
  EXPECT_EQ(output.object_size(), 0);
  EXPECT_EQ(output.unknown_size(), 0);
  const uint32_t x =
      std::floor((point.position().x() - config.min_x()) / config.resolution());
  const uint32_t y =
      std::floor((point.position().y() - config.min_y()) / config.resolution());
  const auto& cells = output.space_map().cell();
  const auto found =
      std::find_if(cells.begin(), cells.end(), [&](const SpaceCell& cell) {
        return cell.x_index() == x && cell.y_index() == y;
      });
  ASSERT_NE(found, cells.end());
  EXPECT_TRUE(found->endpoint_observed());
  EXPECT_FALSE(found->occupancy_obs());
  EXPECT_FALSE(found->occupancy_det());
  EXPECT_FALSE(found->occupancy_unknown());
  EXPECT_EQ(found->traversability(), UNKNOWN);
  const auto& risk = output.point_view(CORRIDOR_RISK_POINTS).point_id();
  EXPECT_NE(std::find(risk.begin(), risk.end(), index), risk.end());
}

TEST_F(FusionTest, PiecewiseGroundFitsKnownSlopeAndRefusesDegenerateSeeds) {
  for (int i = 0; i < cloud.point_size(); ++i) {
    auto* p = cloud.mutable_point(i);
    p->set_z(0.1 * p->x());
  }
  Add(2, 1.4, 1, 0);
  Sync();
  Run(nullptr, nullptr);
  ASSERT_TRUE(output.point_evidence(6).ground_valid());
  EXPECT_NEAR(output.point_evidence(6).z_ground(), 0.2, 1e-6);
  EXPECT_NEAR(output.point_evidence(6).height_above_ground(), 0.8, 1e-6);
  for (int i = 0; i < 6; ++i) {
    cloud.mutable_point(i)->set_y(0.2);
  }
  Run(nullptr, nullptr);
  EXPECT_FALSE(output.point_evidence(6).ground_valid());
}

TEST_F(FusionTest, RaisedSemanticRoadIsNotGroundOrFree) {
  Add(2, 1.4, 0.8, 9);
  Sync();
  Run(nullptr, &seg);
  const auto& p = output.point_evidence(6);
  ASSERT_TRUE(p.ground_valid());
  EXPECT_NEAR(p.height_above_ground(), 0.8, 1e-6);
  EXPECT_FALSE(p.is_ground_geo());
  EXPECT_EQ(output.point_decision(6).explanation(), UNEXPLAINED);
  ASSERT_EQ(output.unknown_size(), 1);
  EXPECT_EQ(output.unknown(0).reason(0), SEM_GEO_CONFLICT);
}

TEST_F(FusionTest, RobustGroundRejectsCompetingSurfacesAndDegenerateSupport) {
  config.mutable_quality()->set_robust_ground(true);
  for (double y : {0.2, 0.8}) {
    for (double x : {1.0, 1.6, 2.2}) {
      Add(x, y, 0.25, 9);
    }
  }
  Sync();
  Run(nullptr, nullptr);
  EXPECT_FALSE(output.point_evidence(0).ground_valid());
  cloud.clear_point();
  seg.clear_point_label();
  for (int i = 0; i < 10; ++i) {
    Add(1 + i * 0.1, 0.5, i < 8 ? 0 : 0.25, 9);
  }
  Sync();
  Run(nullptr, nullptr);
  EXPECT_FALSE(output.point_evidence(0).ground_valid());
  config.mutable_quality()->set_ground_inlier_ratio(0.5);
  SingleFrameFusion fusion;
  std::string error;
  EXPECT_FALSE(fusion.Init(config, &error));
}

TEST_F(FusionTest, RobustGroundRetainsDominantSurfaceAndRaisedReturns) {
  Add(1.6, 0.5, 0.25, 9);
  Sync();
  Run(nullptr, nullptr);
  EXPECT_FALSE(output.point_evidence(0).ground_valid());
  config.mutable_quality()->set_robust_ground(true);
  Run(nullptr, nullptr);
  ASSERT_TRUE(output.point_evidence(0).ground_valid());
  EXPECT_NEAR(output.point_evidence(0).z_ground(), 0, 1e-8);
  EXPECT_EQ(output.point_evidence(0).ground_seed_count(), 6U);
  EXPECT_FALSE(output.point_evidence(6).is_ground_geo());
  EXPECT_EQ(output.point_decision(6).explanation(), UNEXPLAINED);
  EXPECT_TRUE(std::any_of(
      output.space_map().cell().begin(), output.space_map().cell().end(),
      [](const auto& cell) { return cell.occupancy_obs(); }));
  Run(nullptr, &seg);
  EXPECT_NEAR(output.point_evidence(0).z_ground(), 0, 1e-8);
  EXPECT_FALSE(output.point_evidence(6).is_ground_geo());
}

TEST_F(FusionTest, RobustSemanticRefinementCannotSelectRaisedMinoritySurface) {
  config.mutable_quality()->set_robust_ground(true);
  for (int i = 0; i < seg.point_label_size(); ++i) {
    seg.mutable_point_label(i)->set_semantic_label(0);
  }
  for (double y : {0.4, 0.6, 1.0}) {
    for (double x : {1.1, 1.4, 1.7, 2.0}) {
      Add(x, y, 0, 0);
    }
  }
  for (double y : {0.3, 0.7}) {
    for (double x : {1.2, 1.8, 2.4}) {
      Add(x, y, 0.25, 9);
    }
  }
  Sync();
  Run(nullptr, &seg);
  ASSERT_TRUE(output.point_evidence(0).ground_valid());
  EXPECT_NEAR(output.point_evidence(0).z_ground(), 0, 1e-8);
  EXPECT_FALSE(output.point_evidence(18).is_ground_geo());
  EXPECT_EQ(output.point_decision(18).explanation(), UNEXPLAINED);
}

TEST_F(FusionTest, UnsupportedGroundIsConservativeAndNeverAssumedZero) {
  cloud.clear_point();
  seg.clear_point_label();
  Add(2, 1, 0.8, 9);
  Sync();
  Run(nullptr, &seg);
  EXPECT_FALSE(output.point_evidence(0).ground_valid());
  EXPECT_FALSE(output.point_evidence(0).has_z_ground());
  ASSERT_EQ(output.unknown_size(), 1);
  int observed = 0;
  for (const auto& cell : output.space_map().cell()) {
    observed += cell.occupancy_obs();
    EXPECT_TRUE(cell.occupancy_unknown());
    EXPECT_FALSE(cell.occupancy_det());
    EXPECT_EQ(cell.traversability(), OCCUPIED);
  }
  EXPECT_EQ(observed, 1);
}

}  // namespace lidar_fusion
}  // namespace apollo
