// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/map/likelihood_field.h"

#include <limits>

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

FieldConfig Config() {
  FieldConfig config;
  config.identity = {"test-map", "v1", "map", "measured-calibration"};
  config.minimum = Eigen::Vector3d(-1.0, -1.0, -1.0);
  config.maximum = Eigen::Vector3d(1.0, 1.0, 1.0);
  config.resolution = 0.1;
  config.maximum_distance = 0.5;
  config.maximum_cells = 8000;
  config.maximum_map_points = 10;
  config.support_regions = {
      {"search", Eigen::Vector3d(-0.8, -0.8, -0.8),
       Eigen::Vector3d(0.8, 0.8, 0.8)}};
  config.observed_regions = {
      {"mapped", config.minimum, config.maximum}};
  return config;
}

TEST(LikelihoodFieldTest, DistanceHasCellQuantizationAndExplicitDomain) {
  pcl::PointCloud<pcl::PointXYZ> map;
  map.push_back(pcl::PointXYZ(0.0f, 0.0f, 0.0f));
  std::shared_ptr<const LikelihoodField> field;
  ASSERT_TRUE(LikelihoodField::Build(map, Config(), &field).ok());
  double distance = -1.0;
  ASSERT_TRUE(field->Lookup(Eigen::Vector3d::Zero(), &distance));
  EXPECT_NEAR(field->quantization_radius(), distance, 1e-6);
  ASSERT_TRUE(field->Lookup(Eigen::Vector3d(0.95, 0.95, 0.95), &distance));
  EXPECT_DOUBLE_EQ(0.5, distance);
  distance = -1.0;
  EXPECT_FALSE(field->Lookup(Eigen::Vector3d(1.0, 0.0, 0.0), &distance));
  EXPECT_DOUBLE_EQ(-1.0, distance);
  EXPECT_FALSE(field->Lookup(Eigen::Vector3d(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0), &distance));
  EXPECT_FALSE(field->Lookup(Eigen::Vector3d::Zero(), nullptr));
  EXPECT_EQ(FieldCellStatus::OUTSIDE,
            field->Query(Eigen::Vector3d(1.0, 0.0, 0.0), &distance));
}

TEST(LikelihoodFieldTest, UnknownSpaceIsDistinctFromMappedCells) {
  pcl::PointCloud<pcl::PointXYZ> map;
  map.push_back(pcl::PointXYZ(0.0f, 0.0f, 0.0f));
  auto config = Config();
  config.observed_regions = {
      {"left", Eigen::Vector3d(-0.8, -0.8, -0.8),
       Eigen::Vector3d(-0.12, 0.8, 0.8)}};
  std::shared_ptr<const LikelihoodField> field;
  ASSERT_TRUE(LikelihoodField::Build(map, config, &field).ok());
  double distance = -1.0;
  EXPECT_EQ(FieldCellStatus::OBSERVED,
            field->Query(Eigen::Vector3d(-0.25, 0.0, 0.0), &distance));
  EXPECT_EQ(FieldCellStatus::UNKNOWN,
            field->Query(Eigen::Vector3d(-0.11, 0.0, 0.0), &distance));
  EXPECT_EQ(FieldCellStatus::UNKNOWN,
            field->Query(Eigen::Vector3d(0.5, 0.0, 0.0), &distance));
  EXPECT_EQ(FieldCellStatus::OUTSIDE,
            field->Query(Eigen::Vector3d(2.0, 0.0, 0.0), &distance));
}

TEST(LikelihoodFieldTest, RejectsCapacityAndInvalidGeometryWithoutReplacingMap) {
  pcl::PointCloud<pcl::PointXYZ> map;
  map.push_back(pcl::PointXYZ(0.0f, 0.0f, 0.0f));
  std::shared_ptr<const LikelihoodField> field;
  ASSERT_TRUE(LikelihoodField::Build(map, Config(), &field).ok());
  const auto previous = field;
  auto config = Config();
  config.maximum_cells = 7999;
  EXPECT_EQ(Reason::CONFIG_INVALID,
            LikelihoodField::Build(map, config, &field).reason);
  EXPECT_EQ(previous, field);
  config = Config();
  config.identity.calibration.clear();
  EXPECT_EQ(Reason::CONFIG_INVALID,
            LikelihoodField::Build(map, config, &field).reason);
  map.front().x = std::numeric_limits<float>::infinity();
  EXPECT_EQ(Reason::INVALID_INPUT,
            LikelihoodField::Build(map, Config(), &field).reason);
  map.clear();
  EXPECT_EQ(Reason::INVALID_INPUT,
            LikelihoodField::Build(map, Config(), &field).reason);
  EXPECT_EQ(previous, field);
  config = Config();
  config.support_regions.push_back(config.support_regions.front());
  config.support_regions.back().id = "overlap";
  EXPECT_EQ(Reason::CONFIG_INVALID,
            LikelihoodField::Build(map, config, &field).reason);
}

TEST(LikelihoodFieldTest, CallerMutationCannotChangeBuiltGeometry) {
  pcl::PointCloud<pcl::PointXYZ> map;
  map.push_back(pcl::PointXYZ(0.0f, 0.0f, 0.0f));
  auto config = Config();
  std::shared_ptr<const LikelihoodField> field;
  ASSERT_TRUE(LikelihoodField::Build(map, config, &field).ok());
  map.front().x = 0.9f;
  config.identity.version = "v2";
  double distance;
  ASSERT_TRUE(field->Lookup(Eigen::Vector3d::Zero(), &distance));
  EXPECT_NEAR(field->quantization_radius(), distance, 1e-6);
  EXPECT_EQ("v1", field->config().identity.version);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace apollo
