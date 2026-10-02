// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/map/particle_relocalizer.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

std::shared_ptr<const LikelihoodField> Field() {
  pcl::PointCloud<pcl::PointXYZ> map;
  for (float x : {-2.0f, 2.0f}) {
    for (float y : {-1.0f, 0.0f, 1.0f}) {
      map.push_back(pcl::PointXYZ(x, y, 0.0f));
    }
  }
  FieldConfig config;
  config.identity = {"symmetric-map", "v1", "map", "calibrated"};
  config.minimum = Eigen::Vector3d(-4.0, -2.0, -1.0);
  config.maximum = Eigen::Vector3d(4.0, 2.0, 1.0);
  config.resolution = 0.1;
  config.maximum_distance = 1.0;
  config.maximum_cells = 100000;
  config.maximum_map_points = 100;
  config.support_regions = {
      {"west", Eigen::Vector3d(-2.4, -1.5, -0.5),
       Eigen::Vector3d(-1.6, 1.5, 0.5)},
      {"east", Eigen::Vector3d(1.6, -1.5, -0.5),
       Eigen::Vector3d(2.4, 1.5, 0.5)}};
  config.observed_regions = {
      {"observed-map", config.minimum, config.maximum}};
  std::shared_ptr<const LikelihoodField> field;
  const auto result = LikelihoodField::Build(map, config, &field);
  EXPECT_TRUE(result.ok()) << result.message;
  return field;
}

ParticlePolicy Policy() {
  ParticlePolicy policy;
  policy.particles = 512;
  policy.maximum_scan_points = 100;
  policy.maximum_groups = 16;
  policy.minimum_groups = 2;
  policy.maximum_modes = 8;
  policy.maximum_support_regions = 8;
  policy.group_resolution = 0.2;
  policy.distance_sigma = 0.2;
  policy.inlier_distance = 0.4;
  policy.minimum_observed_fraction = 1.0;
  policy.minimum_inlier_fraction = 1.0;
  policy.outlier_floor = 0.05;
  policy.score_temperature = 8.0;
  policy.resample_ess_fraction = 0.8;
  policy.mode_position_radius = 0.6;
  policy.mode_angle_radius = 0.2;
  policy.maximum_interval = 0.2;
  policy.maximum_motion_translation = 0.5;
  policy.maximum_motion_rotation = 0.2;
  policy.maximum_query_age = 0.2;
  policy.future_tolerance = 0.01;
  // Test policy is not an execution-time qualification.
  policy.solve_budget_ms = 100000.0;
  return policy;
}

LocalState Local(double time = 10.0, uint64_t sequence = 1) {
  LocalState local;
  local.epoch = {"local-session", 1};
  local.stamp = {time, time, sequence, "unix"};
  local.valid = true;
  local.covariance = Matrix15d::Identity() * 0.01;
  return local;
}

ParticleDomain Domain() {
  ParticleDomain domain;
  domain.minimum.x() = -3.0;
  domain.maximum.x() = 3.0;
  domain.minimum.y() = -1.5;
  domain.maximum.y() = 1.5;
  domain.minimum.z() = -0.5;
  domain.maximum.z() = 0.5;
  return domain;
}

ParticleQuery Query(const std::shared_ptr<const LikelihoodField>& field) {
  ParticleQuery query;
  query.identity = field->config().identity;
  query.local = Local();
  query.source = "lidar";
  query.scan_sequence = 1;
  query.measurement_time_compensated = true;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->push_back(pcl::PointXYZ(0.0f, 0.0f, 0.0f));
  cloud->push_back(pcl::PointXYZ(0.0f, 1.0f, 0.0f));
  query.cloud = cloud;
  return query;
}

TEST(ParticleRelocalizerTest, SymmetricPlacesRemainSeparateDiagnosticModes) {
  const auto field = Field();
  ASSERT_NE(nullptr, field);
  ParticleRelocalizer search(Policy(), field);
  ASSERT_TRUE(search.Initialize(Domain(), Local(), 17).ok());
  auto query = Query(field);
  query.local.covariance_model_valid = false;
  const auto original = query.local;
  ParticleSnapshot result;
  ASSERT_TRUE(search.Update(query, &result).ok());
  ASSERT_GE(result.modes.size(), 2U);
  EXPECT_TRUE(result.diagnostic_only);
  EXPECT_FALSE(result.search_coverage_complete);
  EXPECT_FALSE(result.local_covariance_model_valid);
  EXPECT_FALSE(result.modes_truncated);
  EXPECT_TRUE(result.resampled);
  EXPECT_GT(result.effective_samples, 1.0);
  EXPECT_LT(result.effective_samples, Policy().particles);
  double mass = 0.0;
  bool left = false;
  bool right = false;
  for (const auto& mode : result.modes) {
    mass += mode.search_mass;
    EXPECT_GT(std::abs(mode.pose.translation().x()), 1.5);
    EXPECT_NEAR(2.0, std::abs(mode.pose.translation().x()), 0.3);
    left = left || mode.pose.translation().x() < 0.0;
    right = right || mode.pose.translation().x() > 0.0;
  }
  EXPECT_TRUE(left && right);
  EXPECT_EQ(2U, result.declared_support_regions);
  EXPECT_EQ(2U, result.weighted_support_regions);
  EXPECT_NEAR(1.0, mass, 1e-12);
  EXPECT_TRUE(query.local.position.isApprox(original.position));
  EXPECT_TRUE(query.local.covariance.isApprox(original.covariance));
}

TEST(ParticleRelocalizerTest, SeedAndSpatialGroupingAreReproducible) {
  const auto field = Field();
  ASSERT_NE(nullptr, field);
  ParticleRelocalizer first(Policy(), field);
  ParticleRelocalizer second(Policy(), field);
  ASSERT_TRUE(first.Initialize(Domain(), Local(), 17).ok());
  ASSERT_TRUE(second.Initialize(Domain(), Local(), 17).ok());
  auto query = Query(field);
  ParticleSnapshot a;
  ASSERT_TRUE(first.Update(query, &a).ok());
  pcl::PointCloud<pcl::PointXYZ>::Ptr repeated(
      new pcl::PointCloud<pcl::PointXYZ>(*query.cloud));
  for (int i = 0; i < 10; ++i) {
    repeated->push_back(query.cloud->front());
  }
  std::reverse(repeated->begin(), repeated->end());
  query.cloud = repeated;
  ParticleSnapshot b;
  ASSERT_TRUE(second.Update(query, &b).ok());
  ASSERT_EQ(a.modes.size(), b.modes.size());
  EXPECT_EQ(2U, b.query_groups);
  EXPECT_DOUBLE_EQ(a.effective_samples, b.effective_samples);
  for (size_t i = 0; i < a.modes.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.modes[i].search_mass, b.modes[i].search_mass);
    EXPECT_TRUE(a.modes[i].pose.matrix().isApprox(b.modes[i].pose.matrix()));
  }
}

TEST(ParticleRelocalizerTest, RejectsDuplicateTimeEpochMapAndUncompensatedInput) {
  const auto field = Field();
  ASSERT_NE(nullptr, field);
  ParticleRelocalizer search(Policy(), field);
  ASSERT_TRUE(search.Initialize(Domain(), Local(), 17).ok());
  auto query = Query(field);
  ParticleSnapshot output;
  ASSERT_TRUE(search.Update(query, &output).ok());
  EXPECT_EQ(Reason::DUPLICATE, search.Update(query, &output).reason);
  query.scan_sequence = 2;
  EXPECT_EQ(Reason::TIMESTAMP_REGRESSION, search.Update(query, &output).reason);
  query.local = Local(10.1, 2);
  query.local.epoch.generation = 2;
  EXPECT_EQ(Reason::EPOCH_MISMATCH, search.Update(query, &output).reason);
  query.local = Local(10.1, 2);
  query.identity.version = "v2";
  EXPECT_EQ(Reason::MAP_MISMATCH, search.Update(query, &output).reason);
  query.identity = field->config().identity;
  query.measurement_time_compensated = false;
  EXPECT_EQ(Reason::INVALID_INPUT, search.Update(query, &output).reason);
  query.measurement_time_compensated = true;
  query.local.stamp.clock_id = "gps";
  EXPECT_EQ(Reason::CLOCK_INVALID, search.Update(query, &output).reason);
  EXPECT_EQ(1U, output.scan_sequence);
  query.local = Local(10.1, 2);
  ASSERT_TRUE(search.Update(query, &output).ok());
  EXPECT_EQ(2U, output.scan_sequence);
  search.Reset();
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE, search.Update(query, &output).reason);
}

TEST(ParticleRelocalizerTest, NoSupportDoesNotBecomeUniformSuccessOrCommitState) {
  const auto field = Field();
  ASSERT_NE(nullptr, field);
  ParticleRelocalizer search(Policy(), field);
  ParticleRelocalizer reference(Policy(), field);
  ASSERT_TRUE(search.Initialize(Domain(), Local(), 17).ok());
  ASSERT_TRUE(reference.Initialize(Domain(), Local(), 17).ok());
  auto query = Query(field);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outside(new pcl::PointCloud<pcl::PointXYZ>);
  outside->push_back(pcl::PointXYZ(100.0f, 0.0f, 0.0f));
  outside->push_back(pcl::PointXYZ(100.0f, 1.0f, 0.0f));
  query.cloud = outside;
  ParticleSnapshot result;
  result.scan_sequence = 999;
  EXPECT_EQ(Reason::MATCH_FAILED, search.Update(query, &result).reason);
  EXPECT_EQ(999U, result.scan_sequence);
  ParticleSnapshot expected;
  query = Query(field);
  ASSERT_TRUE(search.Update(query, &result).ok());
  ASSERT_TRUE(reference.Update(query, &expected).ok());
  EXPECT_DOUBLE_EQ(expected.effective_samples, result.effective_samples);
  ASSERT_EQ(expected.modes.size(), result.modes.size());
  EXPECT_TRUE(expected.modes.front().pose.matrix().isApprox(
      result.modes.front().pose.matrix()));
}

TEST(ParticleRelocalizerTest, TruncationReportsMassInsteadOfClaimingUniqueness) {
  const auto field = Field();
  ASSERT_NE(nullptr, field);
  auto policy = Policy();
  policy.maximum_modes = 1;
  ParticleRelocalizer search(policy, field);
  ASSERT_TRUE(search.Initialize(Domain(), Local(), 17).ok());
  ParticleSnapshot result;
  ASSERT_TRUE(search.Update(Query(field), &result).ok());
  ASSERT_EQ(1U, result.modes.size());
  EXPECT_TRUE(result.modes_truncated);
  EXPECT_GT(result.unreported_mass, 0.0);
  EXPECT_NEAR(1.0, result.unreported_mass + result.modes[0].search_mass, 1e-12);
  EXPECT_FALSE(result.search_coverage_complete);
}

TEST(ParticleRelocalizerTest, MotionPredictionMovesCandidateNotLocalReference) {
  const auto field = Field();
  ASSERT_NE(nullptr, field);
  ParticleRelocalizer search(Policy(), field);
  ParticleDomain domain;
  domain.minimum = Eigen::Vector3d(-2.05, -1.0, -0.4);
  domain.maximum = Eigen::Vector3d(-1.95, 1.0, 0.4);
  ASSERT_TRUE(search.Initialize(domain, Local(), 17).ok());
  auto query = Query(field);
  ParticleSnapshot result;
  ASSERT_TRUE(search.Update(query, &result).ok());
  query.local = Local(10.1, 2);
  query.local.position.x() = 0.1;
  query.scan_sequence = 2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr shifted(new pcl::PointCloud<pcl::PointXYZ>);
  shifted->push_back(pcl::PointXYZ(-0.1f, 0.0f, 0.0f));
  shifted->push_back(pcl::PointXYZ(-0.1f, 1.0f, 0.0f));
  query.cloud = shifted;
  ASSERT_TRUE(search.Update(query, &result).ok());
  EXPECT_TRUE(std::any_of(result.modes.begin(), result.modes.end(),
      [](const ParticleMode& mode) {
        return std::abs(mode.pose.translation().x() + 1.9) < 0.06;
      }));
  EXPECT_DOUBLE_EQ(0.1, query.local.position.x());
}

TEST(ParticleRelocalizerTest, InvalidPoliciesGeometryAndFreshnessFailExplicitly) {
  const auto field = Field();
  ASSERT_NE(nullptr, field);
  auto policy = Policy();
  policy.distance_sigma = 0.001;
  ParticleRelocalizer invalid(policy, field);
  EXPECT_EQ(Reason::CONFIG_INVALID,
            invalid.Initialize(Domain(), Local(), 1).reason);
  policy = Policy();
  policy.particles = 4097;
  ParticleRelocalizer oversized(policy, field);
  EXPECT_EQ(Reason::CONFIG_INVALID,
            oversized.Initialize(Domain(), Local(), 1).reason);
  ParticleRelocalizer search(Policy(), field);
  auto domain = Domain();
  domain.tilt.coeffs().setZero();
  EXPECT_EQ(Reason::INVALID_INPUT, search.Initialize(domain, Local(), 1).reason);
  ASSERT_TRUE(search.Initialize(Domain(), Local(), 17).ok());
  auto query = Query(field);
  query.local.stamp.receive_time = 11.0;
  ParticleSnapshot output;
  EXPECT_EQ(Reason::CLOCK_INVALID, search.Update(query, &output).reason);
  query = Query(field);
  pcl::PointCloud<pcl::PointXYZ>::Ptr bad(
      new pcl::PointCloud<pcl::PointXYZ>(*query.cloud));
  bad->front().z = std::numeric_limits<float>::quiet_NaN();
  query.cloud = bad;
  EXPECT_EQ(Reason::INVALID_INPUT, search.Update(query, &output).reason);
  query = Query(field);
  query.local = Local(10.5, 2);
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE, search.Update(query, &output).reason);
  query.local = Local(10.1, 2);
  query.local.position.x() = 1.0;
  EXPECT_EQ(Reason::INVALID_INPUT, search.Update(query, &output).reason);
}

TEST(ParticleRelocalizerTest, YawBranchCutDoesNotChangeThePhysicalHypothesis) {
  const auto field = Field();
  ASSERT_NE(nullptr, field);
  ParticleRelocalizer a(Policy(), field);
  ParticleRelocalizer b(Policy(), field);
  ParticleDomain domain;
  domain.minimum = Eigen::Vector3d(-2.05, -1.0, -0.4);
  domain.maximum = Eigen::Vector3d(-1.95, 1.0, 0.4);
  domain.yaw_minimum = domain.yaw_maximum = 3.14159265358979323846;
  ASSERT_TRUE(a.Initialize(domain, Local(), 17).ok());
  domain.yaw_minimum = domain.yaw_maximum = -3.14159265358979323846;
  ASSERT_TRUE(b.Initialize(domain, Local(), 17).ok());
  ParticleSnapshot first;
  ParticleSnapshot second;
  ASSERT_TRUE(a.Update(Query(field), &first).ok());
  ASSERT_TRUE(b.Update(Query(field), &second).ok());
  ASSERT_EQ(first.modes.size(), second.modes.size());
  for (size_t i = 0; i < first.modes.size(); ++i) {
    EXPECT_TRUE(first.modes[i].pose.matrix().isApprox(
        second.modes[i].pose.matrix()));
  }
}

TEST(ParticleRelocalizerTest, UnknownMapCellsAreNotScoredAsFreeSpace) {
  pcl::PointCloud<pcl::PointXYZ> map;
  map.push_back(pcl::PointXYZ(-2.0f, 0.0f, 0.0f));
  map.push_back(pcl::PointXYZ(-2.0f, 1.0f, 0.0f));
  map.push_back(pcl::PointXYZ(2.0f, 0.0f, 0.0f));
  map.push_back(pcl::PointXYZ(2.0f, 1.0f, 0.0f));
  auto config = Field()->config();
  config.observed_regions = {
      {"west-observed", Eigen::Vector3d(-4.0, -2.0, -1.0),
       Eigen::Vector3d(0.0, 2.0, 1.0)}};
  std::shared_ptr<const LikelihoodField> field;
  ASSERT_TRUE(LikelihoodField::Build(map, config, &field).ok());
  auto policy = Policy();
  policy.minimum_observed_fraction = 0.5;
  ParticleRelocalizer search(policy, field);
  auto domain = Domain();
  domain.minimum.x() = -2.3;
  domain.maximum.x() = -1.7;
  ASSERT_TRUE(search.Initialize(domain, Local(), 5).ok());
  auto query = Query(field);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->push_back(pcl::PointXYZ(0.0f, 0.0f, 0.0f));
  cloud->push_back(pcl::PointXYZ(0.0f, 1.0f, 0.0f));
  cloud->push_back(pcl::PointXYZ(3.0f, 0.0f, 0.0f));
  cloud->push_back(pcl::PointXYZ(3.0f, 1.0f, 0.0f));
  query.cloud = cloud;
  ParticleSnapshot result;
  ASSERT_TRUE(search.Update(query, &result).ok());
  ASSERT_FALSE(result.modes.empty());
  EXPECT_GT(result.modes.front().unknown_fraction, 0.0);
  EXPECT_LT(result.modes.front().observed_fraction, 1.0);
  EXPECT_EQ(1U, result.declared_support_regions);
}

TEST(ParticleRelocalizerTest, InsufficientSupportRegionBudgetFailsExplicitly) {
  const auto field = Field();
  ASSERT_NE(nullptr, field);
  auto policy = Policy();
  policy.maximum_support_regions = 1;
  ParticleRelocalizer search(policy, field);
  EXPECT_EQ(Reason::MAP_NOT_READY,
            search.Initialize(Domain(), Local(), 5).reason);
}

TEST(ParticleRelocalizerTest, StratifiesSupportAndReportsVolumePrior) {
  pcl::PointCloud<pcl::PointXYZ> map;
  map.push_back(pcl::PointXYZ(-2.0f, 0.0f, 0.0f));
  map.push_back(pcl::PointXYZ(-2.0f, 1.0f, 0.0f));
  map.push_back(pcl::PointXYZ(2.0f, 0.0f, 0.0f));
  map.push_back(pcl::PointXYZ(2.0f, 1.0f, 0.0f));
  auto config = Field()->config();
  config.support_regions[1].minimum.x() = 1.2;
  config.support_regions[1].maximum.x() = 2.8;
  std::shared_ptr<const LikelihoodField> field;
  ASSERT_TRUE(LikelihoodField::Build(map, config, &field).ok());
  ParticleRelocalizer search(Policy(), field);
  ASSERT_TRUE(search.Initialize(Domain(), Local(), 23).ok());
  ParticleSnapshot result;
  ASSERT_TRUE(search.Update(Query(field), &result).ok());
  ASSERT_EQ(2U, result.initial_support_allocation.size());
  EXPECT_EQ("west", result.initial_support_allocation[0].region_id);
  EXPECT_EQ("east", result.initial_support_allocation[1].region_id);
  const auto& west = result.initial_support_allocation[0];
  const auto& east = result.initial_support_allocation[1];
  EXPECT_GT(west.initial_particles, 0U);
  EXPECT_GT(east.initial_particles, 0U);
  EXPECT_EQ(Policy().particles,
            west.initial_particles + east.initial_particles);
  EXPECT_NEAR(1.0 / 3.0, west.prior_search_mass, 1e-12);
  EXPECT_NEAR(2.0 / 3.0, east.prior_search_mass, 1e-12);
  EXPECT_NEAR(west.prior_search_mass,
              static_cast<double>(west.initial_particles) /
                  static_cast<double>(Policy().particles),
              0.01);
  EXPECT_NEAR(east.prior_search_mass,
              static_cast<double>(east.initial_particles) /
                  static_cast<double>(Policy().particles),
              0.01);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace apollo
