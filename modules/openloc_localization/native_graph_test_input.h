// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <fstream>
#include <iomanip>
#include <stdexcept>

#include "native_global_test_input.h"
#include "openloc/localization/local/native_odometry.h"

namespace apollo::localization::openloc_diagnostic::test {
namespace u = ::openloc::localization::unified;

inline std::vector<Eigen::Vector3d> SharedMapGeometry() {
  std::vector<Eigen::Vector3d> points;
  for (int a = 0; a < 4; ++a)
    for (int b = 0; b < 4; ++b) {
      const double x = 0.2 * a + 0.03713, y = 0.2 * b + 0.04317;
      points.emplace_back(x, y, 0.45119);
      points.emplace_back(2.73713, x, y + 1.6);
      points.emplace_back(x, 2.34317, y + 1.6);
    }
  return points;
}

inline void EnableSharedMap(NativeGlobalHostConfig* config, const std::string& path) {
  EnableTargetGraph(config);
  const auto points = SharedMapGeometry();
  std::ofstream file(path);
  file << "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
       << "WIDTH " << points.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
       << "POINTS " << points.size() << "\nDATA ascii\n" << std::setprecision(17);
  for (const auto& p : points) file << p.x() << ' ' << p.y() << ' ' << p.z() << '\n';
  file.close();
  if (!file) throw std::runtime_error("cannot write shared native fixture map");
  config->set_maximum_odometry_age_ms(1000);
  auto* online = config->mutable_online_map();
  online->set_cloud_topic("/openloc/diagnostic/shared-map-unused-raw");
  online->set_lidar_frame("lidar");
  online->set_map_pcd(path);
  online->set_map_sha256(NativeMapMatcher::Sha256(path));
  online->set_preparation_id("actual-native-shared-map");
  online->set_hypothesis_minimum_ns(100000000000LL);
  online->set_hypothesis_maximum_ns(101000000000LL);
  online->set_window_radius_m(10);
  online->set_window_cell_m(2);
  online->set_task_deadline_ms(500);
  online->set_hypothesis_odometry_session("actual-native-cyber-graph");
  online->set_hypothesis_odometry_generation(1);
  online->set_gnss_map_construction(NativeGlobalGnssInput::ABSENT);
  NativeGlobalPose identity;
  identity.set_x(0); identity.set_y(0); identity.set_z(0);
  identity.set_qx(0); identity.set_qy(0); identity.set_qz(0); identity.set_qw(1);
  *online->mutable_imu_from_lidar() = identity;
  *online->mutable_initial_map_from_odom() = identity;
}

inline std::vector<NativeGraphFrame> ActualDelayedNativeFrames() {
  u::LocalConfig config;
  config.session = "actual-native-cyber-graph";
  config.allow_imu_only_initialization = true;
  config.stationary_samples = 3;
  config.stationary_duration = 0.015;
  config.enable_native_joint_error = true;
  config.native_imu_sample_period = 0.01;
  config.native_joint_max_dimension = 1024;
  config.history_duration = 0.035;
  config.enable_lidar = true;
  config.lidar_frame = "lidar";
  config.lidar_calibration_id = "test-lidar";
  config.lidar_point_noise_std = config.lidar_plane_noise_std = 0.01;
  config.lidar_voxel_size = 1;
  config.lidar_max_voxels = 100;
  config.lidar_max_scans = 2;
  config.lidar_min_points_per_plane = 3;
  config.lidar_min_correspondences = 2;
  config.lidar_max_points = 1000;
  config.lidar_max_correspondence_distance = 0.3;
  config.lidar_observed_singular_value = 1;
  config.lidar_innovation_gate = 100;
  config.lidar_max_scan_age = 0.1;
  config.lidar_future_tolerance = 0.001;
  config.lidar_max_translation_correction_rate = 0.2;
  config.lidar_max_rotation_correction_rate = 0.1;
  config.lidar_max_velocity_correction_acceleration = 0.3;
  config.lidar_anchor_variance_scale = 1;
  config.lidar_correction_variance_scale = 2;
  config.lidar_max_translation_correction = 2;
  config.lidar_max_rotation_correction = 0.5;
  config.lidar_max_velocity_correction = 2;
  config.lidar_max_update_iterations = 2;
  config.lidar_process_noise_psd_tolerance = 1e-12;
  config.lidar_rotation_length = 1;
  ::openloc::localization::NativeOdometry owner(config);
  std::unique_ptr<::openloc::localization::NativeOdometry::LidarTask> delayed;
  std::vector<NativeGraphFrame> frames;
  const auto require = [](const u::Result& result) {
    if (!result.ok()) throw std::runtime_error(result.message);
  };
  for (uint64_t index = 1; index <= 19; ++index) {
    u::ImuSample imu;
    imu.stamp = {100 + 0.01 * (index - 1), 100 + 0.01 * (index - 1), index, "unix"};
    imu.acceleration.z() = config.gravity;
    owner.AddImu(imu);
    if (delayed) {
      const auto replayed = delayed->ReplayImu(imu);
      if (!replayed.ok() && replayed.reason != u::Reason::WHEEL_STALE &&
          replayed.reason != u::Reason::CORRELATION_UNQUALIFIED &&
          replayed.reason != u::Reason::CORRECTION_BUDGET_EXCEEDED)
        require(replayed);
    }
    if (index < 3) continue;
    require(owner.EvaluateIntegrity(imu.stamp.receive_time));
    NativeGraphFrame frame;
    frame.nominal = owner.state();
    if (index == 3 || index == 7 || index == 11 || index == 15) {
      u::LidarScan scan;
      scan.stamp = owner.internal_state().stamp;
      scan.stamp.sequence = (index + 1) / 4;
      scan.epoch = owner.state().epoch;
      scan.frame_id = config.lidar_frame;
      scan.calibration_id = config.lidar_calibration_id;
      for (const auto& point : SharedMapGeometry())
        scan.points.push_back({point.cast<float>().cast<double>(),
            scan.stamp.time - (index == 3 ? 0 : 0.005)});
      require(owner.PrepareLidarTask(scan, &delayed));
      const auto computed = delayed->Compute();
      if (computed.reason != u::Reason::GEOMETRY_UNAVAILABLE) require(computed);
    }
    if (index == 5 || index == 9 || index == 13 || index == 17) {
      const auto committed = owner.CommitLidarTask(delayed.get());
      if (committed.reason != u::Reason::GEOMETRY_UNAVAILABLE) require(committed);
      delayed.reset();
      frame.scan.emplace();
      require(owner.PrepareJointLidarScan((index - 1) / 4, &*frame.scan));
    }
    require(owner.JointErrorLookup(imu.stamp.time, &frame.joint));
    if (!frames.empty()) {
      frame.motion.emplace();
      require(owner.JointErrorMotionBetween(
          frames.back().joint.stamp.time, imu.stamp.time, &*frame.motion));
      frame.previous_dimension = frames.back().joint.joint_error->covariance.rows();
    }
    if (!owner.state().position.isApprox(frame.nominal.position, 0) ||
        !owner.state().orientation.isApprox(frame.nominal.orientation, 0) ||
        !owner.state().covariance.isApprox(frame.nominal.covariance, 0))
      throw std::runtime_error("graph source preparation changed published ODOM");
    frames.push_back(std::move(frame));
  }
  return frames;
}

}  // namespace apollo::localization::openloc_diagnostic::test
