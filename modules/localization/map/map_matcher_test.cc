// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/map/map_matcher.h"

#include <limits>

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

MatchConfig Config() {
  MatchConfig config;
  config.minimum_points = 20;
  config.voxel_size = 0.1;
  config.position_variance_floor = 0.01;
  config.rotation_variance_floor = 0.001;
  return config;
}

TEST(MapMatcherTest, MissingMapAndEmptyHypothesesAreExplicitErrors) {
  MapMatcher matcher(Config());
  Cloud::Ptr cloud(new Cloud);
  MatchResult result;
  EXPECT_EQ(Reason::MAP_NOT_READY,
            matcher.Match(cloud, Eigen::Isometry3d::Identity(), &result).reason);
  EXPECT_EQ(Reason::INVALID_INPUT, matcher.Recover(cloud, {}, &result).reason);
  EXPECT_EQ(Reason::MAP_NOT_READY, matcher.SetMap(cloud).reason);
}

TEST(MapMatcherTest, RejectsNonfiniteMapsAndUncalibratedCovarianceModel) {
  Cloud::Ptr cloud(new Cloud);
  for (int index = 0; index < 30; ++index) {
    cloud->push_back(pcl::PointXYZ(index, 0.0f, 0.0f));
  }
  (*cloud)[0].x = std::numeric_limits<float>::quiet_NaN();
  MapMatcher matcher(Config());
  EXPECT_EQ(Reason::INVALID_INPUT, matcher.SetMap(cloud).reason);
  auto config = Config();
  config.position_variance_floor = 0.0;
  MapMatcher no_model(config);
  EXPECT_EQ(Reason::CONFIG_INVALID, no_model.SetMap(cloud).reason);
}

TEST(MapMatcherTest, PlanarGeometryCannotProduceTrustedSixDofPose) {
  Cloud::Ptr plane(new Cloud);
  for (int x = 0; x < 30; ++x) {
    for (int y = 0; y < 30; ++y) {
      plane->push_back(pcl::PointXYZ(x * 0.15f, y * 0.15f, 0.0f));
    }
  }
  MapMatcher matcher(Config());
  ASSERT_TRUE(matcher.SetMap(plane).ok());
  MatchResult result;
  const auto matched = matcher.Match(plane, Eigen::Isometry3d::Identity(), &result);
  if (matched.ok()) {
    EXPECT_FALSE(result.full_pose);
    EXPECT_LT(result.projection.rows(), 6);
  }
}

TEST(MapMatcherTest, CorridorInformationRetainsNormalButNotLongitudinal) {
  Matrix6d information = Matrix6d::Zero();
  information(1, 1) = 100.0;
  information(2, 2) = 100.0;
  information(3, 3) = 20.0;
  information(4, 4) = 20.0;
  information(5, 5) = 40.0;
  MatchResult result;
  ASSERT_TRUE(ProjectMapInformation(information, 0.01, Config(), &result).ok());
  EXPECT_FALSE(result.full_pose);
  ASSERT_EQ(5, result.projection.rows());
  EXPECT_TRUE(result.projection.col(0).isZero());
  EXPECT_GT(result.projection.col(1).norm(), 0.0);
  EXPECT_TRUE(result.covariance.isZero());
}

TEST(MapMatcherTest, ZeroOrIndefiniteInformationDoesNotBecomeCertain) {
  MatchResult result;
  EXPECT_EQ(Reason::DEGENERATE,
            ProjectMapInformation(Matrix6d::Zero(), 0.01, Config(), &result).reason);
  Matrix6d information = Matrix6d::Identity();
  information(0, 0) = -1.0;
  EXPECT_EQ(Reason::INVALID_INPUT,
            ProjectMapInformation(information, 0.01, Config(), &result).reason);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace apollo
