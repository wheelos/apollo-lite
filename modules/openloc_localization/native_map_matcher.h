// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <deque>
#include <memory>
#include <optional>
#include <string>

#include "native_global_diagnostic.pb.h"
#include "Eigen/Geometry"
#include "openloc/localization/local/native_types.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo::localization::openloc_diagnostic {

struct NativeMapPose {
  int64_t time_ns = 0;
  Eigen::Isometry3d odom_from_imu = Eigen::Isometry3d::Identity();
};

struct NativeMapJob {
  drivers::PointCloud cloud;
  int64_t reference_ns = 0;
  int64_t receive_ns = 0;
  std::deque<NativeMapPose> history;
  std::string session;
  uint64_t generation = 0;
  std::string alignment_session;
  uint64_t alignment_generation = 0;
  std::optional<Eigen::Isometry3d> map_from_odom;
  std::optional<::openloc::localization::unified::JointLidarScan> joint_scan;
};

struct NativeMapMatch {
  uint64_t sequence = 0;
  int64_t measurement_ns = 0;
  int64_t receive_ns = 0;
  std::string reason;
  std::string search_detail;
  std::optional<NativeGlobalMapInput> observation;
};

// Serialized by one off-owner task. Consumes copied history only; never owns
// local/global estimates. Map I/O and clock/geometry adaptation are host work.
class NativeMapMatcher {
 public:
  explicit NativeMapMatcher(const NativeGlobalHostConfig& config);
  ~NativeMapMatcher();
  NativeMapMatch Match(const NativeMapJob& job);
  static std::string Sha256(const std::string& path);
  static Eigen::Isometry3d Pose(const NativeGlobalPose& pose);
  static int64_t Nanoseconds(double seconds);
  static int64_t Reference(const drivers::PointCloud& cloud);
  static std::optional<Eigen::Isometry3d> Lookup(
      const std::deque<NativeMapPose>& history, int64_t time);

 private:
  NativeMapMatch MatchShared(const NativeMapJob& job);
  NativeGlobalHostConfig config_;
  Eigen::Isometry3d imu_from_lidar_;
  Eigen::Isometry3d initial_map_from_odom_;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace apollo::localization::openloc_diagnostic
