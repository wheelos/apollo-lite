// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "modules/localization/core/local_lio.h"

#include <cmath>
#include <map>

#include "gtest/gtest.h"

namespace apollo {
namespace localization {
namespace unified {
namespace {

LocalState State(double time, const Epoch& epoch,
                 const Eigen::Vector3d& position,
                 const Eigen::Quaterniond& orientation) {
  LocalState state;
  state.stamp = {time, time, static_cast<uint64_t>(time * 100.0), "clock"};
  state.epoch = epoch;
  state.position = position;
  state.orientation = orientation;
  state.covariance.setIdentity();
  state.covariance *= 0.01;
  state.valid = true;
  return state;
}

LocalConfig LioConfig() {
  LocalConfig config;
  config.enable_lidar = true;
  config.lidar_point_noise_std = 0.01;
  config.lidar_plane_noise_std = 0.01;
  config.lidar_voxel_size = 1.0;
  config.lidar_max_voxels = 100;
  config.lidar_max_scans = 3;
  config.lidar_min_points_per_plane = 3;
  config.lidar_min_correspondences = 2;
  config.lidar_max_points = 1000;
  config.lidar_max_correspondence_distance = 0.2;
  config.lidar_observed_singular_value = 1.0;
  config.lidar_innovation_gate = 100.0;
  config.lidar_anchor_variance_scale = 1.0;
  config.lidar_correction_variance_scale = 1.0;
  config.lidar_max_update_iterations = 2;
  config.lidar_rotation_length = 1.0;
  return config;
}

TEST(LocalLioDeskewTest, UsesPointRotationLeverArmAndMeasurementTime) {
  const Epoch epoch{"session", 4};
  const LocalState start =
      State(1.0, epoch, Eigen::Vector3d::Zero(),
            Eigen::Quaterniond::Identity());
  const LocalState end =
      State(2.0, epoch, Eigen::Vector3d::Zero(),
            Eigen::Quaterniond(Eigen::AngleAxisd(
                0.5 * std::acos(-1.0), Eigen::Vector3d::UnitZ())));
  const std::map<double, LocalState> history{{1.0, start}, {2.0, end}};
  LidarScan scan;
  scan.stamp = end.stamp;
  scan.epoch = epoch;
  scan.frame_id = "lidar";
  scan.calibration_id = "calibration";
  scan.base_from_lidar.translation().x() = 1.0;
  scan.points.push_back({Eigen::Vector3d(1.0, 0.0, 0.0), 1.0});
  DeskewedScan deskewed;
  const Result result = DeskewLidarScan(
      scan,
      [&history](double time, LocalState* state) {
        const auto found = history.find(time);
        if (found == history.end()) {
          return Result{Reason::HISTORY_UNAVAILABLE, "missing"};
        }
        *state = found->second;
        return Result{};
      },
      &deskewed);
  ASSERT_TRUE(result.ok()) << result.message;
  ASSERT_EQ(deskewed.points_in_end_base.size(), 1U);
  EXPECT_NEAR(deskewed.points_in_end_base[0].x(), 0.0, 1e-12);
  EXPECT_NEAR(deskewed.points_in_end_base[0].y(), -2.0, 1e-12);
}

TEST(LocalLioDeskewTest, RejectsMissingTimeAndEpochChange) {
  const Epoch epoch{"session", 1};
  const LocalState state = State(
      1.0, epoch, Eigen::Vector3d::Zero(),
      Eigen::Quaterniond::Identity());
  LidarScan scan;
  scan.stamp = state.stamp;
  scan.epoch = epoch;
  scan.frame_id = "lidar";
  scan.calibration_id = "calibration";
  scan.points.push_back({Eigen::Vector3d::Zero(), 0.0});
  DeskewedScan deskewed;
  auto lookup = [&state](double, LocalState* output) {
    *output = state;
    return Result{};
  };
  EXPECT_EQ(DeskewLidarScan(scan, lookup, &deskewed).reason,
            Reason::POINT_TIME_MISSING);
  scan.points[0].time = 1.0;
  auto changed_epoch = [state](double, LocalState* output) mutable {
    *output = state;
    output->epoch.generation = 2;
    return Result{};
  };
  EXPECT_EQ(DeskewLidarScan(scan, changed_epoch, &deskewed).reason,
            Reason::EPOCH_MISMATCH);
}

TEST(LocalLioGeometryTest, CorridorDoesNotCreateLongitudinalInformation) {
  const Epoch epoch{"session", 1};
  LocalState state = State(
      3.0, epoch, Eigen::Vector3d::Zero(),
      Eigen::Quaterniond::Identity());
  LocalConfig config = LioConfig();
  LocalLio lio(config);
  LidarScan scan;
  scan.stamp = state.stamp;
  scan.epoch = epoch;
  scan.frame_id = "lidar";
  scan.calibration_id = "calibration";
  for (double x : {0.1, 0.3, 0.5}) {
    for (double z : {0.1, 0.3, 0.5}) {
      scan.points.push_back({Eigen::Vector3d(x, 1.0, z), 3.0});
      scan.points.push_back({Eigen::Vector3d(x, -1.0, z), 3.0});
    }
  }
  auto lookup = [&state](double, LocalState* output) {
    *output = state;
    return Result{};
  };
  EXPECT_EQ(lio.Observe(scan, lookup, &state).reason,
            Reason::GEOMETRY_UNAVAILABLE);
  scan.stamp.sequence += 1;
  ASSERT_TRUE(lio.Observe(scan, lookup, &state).ok());
  ASSERT_GT(lio.observed_pose_basis().rows(), 0);
  EXPECT_LT(lio.observed_pose_basis().col(0).norm(), 1e-10);
  EXPECT_LT(lio.observed_pose_basis().rows(), 6);
  EXPECT_LE(lio.scan_count(), config.lidar_max_scans);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace apollo
