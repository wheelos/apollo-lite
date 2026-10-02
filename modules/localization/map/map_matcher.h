// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <string>
#include <vector>

#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "modules/localization/core/types.h"

namespace apollo {
namespace localization {
namespace unified {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct MatchConfig {
  double voxel_size = 0.5;
  double resolution = 1.0;
  double max_fitness = 0.5;
  double correspondence_distance = 1.0;
  double minimum_overlap = 0.5;
  uint32_t minimum_points = 100;
  double max_translation = 2.0;
  double max_rotation = 0.3;
  double minimum_information = 0.0001;
  double maximum_condition = 1000000.0;
  double position_variance_floor = 0.0;
  double rotation_variance_floor = 0.0;
  double ambiguity_margin = 0.2;
  uint32_t maximum_seeds = 16;
  double rotation_length = 1.0;
};

struct MatchResult {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Matrix6d covariance = Matrix6d::Zero();
  Eigen::MatrixXd projection;
  Eigen::MatrixXd projected_covariance;
  bool full_pose = false;
  double score = 0.0;
  double overlap = 0.0;
  bool ambiguous = false;
};

Result ProjectMapInformation(const Matrix6d& information, double residual_variance,
                             const MatchConfig& config, MatchResult* result);

// A fixed, versioned global map is observation evidence, never the ODOM map.
class MapMatcher {
 public:
  explicit MapMatcher(const MatchConfig& config);
  Result Load(const std::string& pcd_path);
  Result SetMap(const Cloud::ConstPtr& cloud);
  Result Match(const Cloud::ConstPtr& base_cloud,
               const Eigen::Isometry3d& initial, MatchResult* result) const;
  Result Recover(const Cloud::ConstPtr& base_cloud,
                 const std::vector<Eigen::Isometry3d>& seeds,
                 MatchResult* result) const;
  bool ready() const { return ready_; }

 private:
  Result ValidateConfig() const;
  Result Evidence(const Cloud& cloud, MatchResult* result) const;
  MatchConfig config_;
  Cloud::Ptr map_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_;
  bool ready_ = false;
};

}  // namespace unified
}  // namespace localization
}  // namespace apollo
