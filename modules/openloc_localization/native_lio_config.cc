// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "native_lio_config.h"

#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

namespace apollo::localization::openloc_diagnostic {
namespace u = ::openloc::localization::unified;

u::LocalConfig LocalConfiguration(const NativeLioHostConfig& profile) {
  if (!profile.IsInitialized() || profile.imu_topic().empty() ||
      profile.lidar_topic().empty() ||
      profile.imu_topic() == profile.lidar_topic() ||
      profile.session().empty() || profile.journal().empty() ||
      profile.startup_task_timeout_ms() > 500 ||
      (profile.require_live_source_sequences() &&
       profile.recorded_receive_time_from_header()) ||
      (profile.lidar_commit_delay_ms() != 0 &&
       (profile.lidar_commit_delay_ms() < 5 ||
        profile.lidar_commit_delay_ms() > 100)) ||
      !std::isfinite(profile.imu_measurement_to_unix_offset()) ||
      !std::isfinite(profile.lidar_measurement_to_unix_offset()) ||
      !std::isfinite(profile.bootstrap_voxel_m()) ||
      profile.bootstrap_voxel_m() < 0.1 || profile.bootstrap_voxel_m() > 0.5)
    throw std::invalid_argument("incomplete or invalid native host profile");
  u::LocalConfig config;
  config.session = profile.session();
  config.allow_imu_only_initialization =
      profile.allow_imu_only_initialization();
  config.enable_motion_initialization = true;
  config.enable_lidar = true;
  config.lidar_frame = profile.lidar_frame();
  config.lidar_calibration_id = profile.calibration_id();
  if (profile.has_graph()) {
    const auto& graph = profile.graph();
    if (!graph.IsInitialized() ||
        (graph.recorded_source_clock() && !profile.recorded_receive_time_from_header()) ||
        graph.output_topic().rfind("/openloc/diagnostic/", 0) != 0 ||
        graph.output_topic() == profile.imu_topic() ||
        graph.output_topic() == profile.lidar_topic() ||
        graph.motion_error_model_id().empty() ||
        graph.motion_error_model_id().size() > 128 ||
        !std::isfinite(graph.imu_sample_period_s()) ||
        graph.imu_sample_period_s() <= 0 ||
        graph.maximum_error_dimension() < 15 ||
        graph.maximum_error_dimension() > 2048 ||
        graph.maximum_memory_rows() == 0 || graph.maximum_memory_rows() > 65536 ||
        graph.maximum_snapshots() < 2 || graph.maximum_snapshots() > 4096 ||
        !std::isfinite(graph.history_duration_s()) ||
        graph.history_duration_s() <= 0 || graph.history_duration_s() > 30 ||
        graph.odometry_stride() == 0 || graph.odometry_stride() > 100)
      throw std::invalid_argument("invalid explicit native graph export configuration");
    if (graph.odometry_stride() * graph.imu_sample_period_s() >
        graph.history_duration_s())
      throw std::invalid_argument("graph export stride exceeds native retained history");
    config.enable_native_joint_error = true;
    config.native_imu_sample_period = graph.imu_sample_period_s();
    config.native_joint_max_dimension = graph.maximum_error_dimension();
    config.native_joint_max_memory_rows = graph.maximum_memory_rows();
    config.native_joint_max_snapshots = graph.maximum_snapshots();
    config.history_duration = graph.history_duration_s();
  }
  std::map<std::string, double*> doubles{
      {"gyro_noise", &config.gyro_noise},
      {"accel_noise", &config.accel_noise},
      {"gyro_bias_noise", &config.gyro_bias_noise},
      {"accel_bias_noise", &config.accel_bias_noise},
      {"point_noise_std", &config.lidar_point_noise_std},
      {"plane_noise_std", &config.lidar_plane_noise_std},
      {"voxel_size", &config.lidar_voxel_size},
      {"max_correspondence_distance",
       &config.lidar_max_correspondence_distance},
      {"observed_singular_value", &config.lidar_observed_singular_value},
      {"innovation_gate", &config.lidar_innovation_gate},
      {"max_scan_age", &config.lidar_max_scan_age},
      {"translation_correction_rate",
       &config.lidar_max_translation_correction_rate},
      {"rotation_correction_rate", &config.lidar_max_rotation_correction_rate},
      {"velocity_correction_acceleration",
       &config.lidar_max_velocity_correction_acceleration},
      {"anchor_variance_scale", &config.lidar_anchor_variance_scale},
      {"correction_variance_scale", &config.lidar_correction_variance_scale},
      {"max_translation_correction", &config.lidar_max_translation_correction},
      {"max_rotation_correction", &config.lidar_max_rotation_correction},
      {"max_velocity_correction", &config.lidar_max_velocity_correction},
      {"process_noise_psd_tolerance",
       &config.lidar_process_noise_psd_tolerance},
      {"rotation_length", &config.lidar_rotation_length}};
  std::map<std::string, uint32_t*> integers{
      {"max_voxels", &config.lidar_max_voxels},
      {"max_scans", &config.lidar_max_scans},
      {"min_points_per_plane", &config.lidar_min_points_per_plane},
      {"min_correspondences", &config.lidar_min_correspondences},
      {"max_points", &config.lidar_max_points},
      {"max_update_iterations", &config.lidar_max_update_iterations}};
  for (const auto& parameter : profile.parameter()) {
    if (!std::isfinite(parameter.value()))
      throw std::invalid_argument("nonfinite native profile value");
    const auto real = doubles.find(parameter.name());
    const auto integer = integers.find(parameter.name());
    if (real != doubles.end()) {
      *real->second = parameter.value();
      doubles.erase(real);
    } else if (integer != integers.end()) {
      if (parameter.value() < 0 ||
          parameter.value() > std::numeric_limits<uint32_t>::max() ||
          std::floor(parameter.value()) != parameter.value())
        throw std::invalid_argument("invalid integer native profile value");
      *integer->second = static_cast<uint32_t>(parameter.value());
      integers.erase(integer);
    } else {
      throw std::invalid_argument("unknown or duplicate profile key: " +
                                  parameter.name());
    }
  }
  if (!doubles.empty() || !integers.empty())
    throw std::invalid_argument("incomplete native numeric profile");
  LidarExtrinsic(profile);
  return config;
}

Eigen::Isometry3d LidarExtrinsic(const NativeLioHostConfig& profile) {
  const auto& translation = profile.imu_from_lidar_translation();
  const auto& rotation = profile.imu_from_lidar_rotation();
  if (!translation.has_x() || !translation.has_y() || !translation.has_z() ||
      !rotation.has_qx() || !rotation.has_qy() || !rotation.has_qz() ||
      !rotation.has_qw())
    throw std::invalid_argument("incomplete LiDAR extrinsic");
  const Eigen::Quaterniond q(rotation.qw(), rotation.qx(), rotation.qy(),
                             rotation.qz());
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() =
      Eigen::Vector3d(translation.x(), translation.y(), translation.z());
  if (!transform.translation().allFinite() || !q.coeffs().allFinite() ||
      std::abs(q.norm() - 1) > 1e-9)
    throw std::invalid_argument("invalid calibrated LiDAR extrinsic");
  transform.linear() = q.toRotationMatrix();
  return transform;
}

}  // namespace apollo::localization::openloc_diagnostic
