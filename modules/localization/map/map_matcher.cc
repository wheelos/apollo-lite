// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/map/map_matcher.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "Eigen/Eigenvalues"
#include "Eigen/LU"
#include "modules/localization/core/observed_subspace.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"
#include "pcl/registration/ndt.h"

namespace apollo {
namespace localization {
namespace unified {

MapMatcher::MapMatcher(const MatchConfig& config)
    : config_(config), map_(new Cloud),
      normals_(new pcl::PointCloud<pcl::Normal>) {}

Result MapMatcher::ValidateConfig() const {
  for (double value : {config_.voxel_size, config_.resolution,
                       config_.max_fitness, config_.correspondence_distance,
                       config_.minimum_overlap, config_.max_translation,
                       config_.max_rotation, config_.minimum_information,
                       config_.maximum_condition, config_.position_variance_floor,
                       config_.rotation_variance_floor, config_.ambiguity_margin,
                       config_.rotation_length}) {
    if (!std::isfinite(value) || value <= 0.0) {
      return {Reason::CONFIG_INVALID, "map matcher budgets must be positive"};
    }
  }
  if (config_.minimum_points < 10 || config_.maximum_seeds < 2 ||
      config_.minimum_overlap > 1.0 || config_.maximum_condition < 1.0) {
    return {Reason::CONFIG_INVALID, "invalid matcher counts/overlap"};
  }
  return {};
}

Result MapMatcher::Load(const std::string& pcd_path) {
  Cloud::Ptr cloud(new Cloud);
  if (pcd_path.empty() || pcl::io::loadPCDFile(pcd_path, *cloud) != 0) {
    ready_ = false;
    return {Reason::MAP_NOT_READY, "cannot load global PCD: " + pcd_path};
  }
  return SetMap(cloud);
}

Result MapMatcher::SetMap(const Cloud::ConstPtr& cloud) {
  ready_ = false;
  const auto valid = ValidateConfig();
  if (!valid.ok()) {
    return valid;
  }
  if (!cloud || cloud->size() < config_.minimum_points) {
    return {Reason::MAP_NOT_READY, "global map has insufficient points"};
  }
  for (const auto& point : *cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      return {Reason::INVALID_INPUT, "nonfinite point in global map"};
    }
  }
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(config_.voxel_size, config_.voxel_size, config_.voxel_size);
  voxel.filter(*map_);
  if (map_->size() < config_.minimum_points) {
    return {Reason::MAP_NOT_READY, "filtered global map is too sparse"};
  }
  tree_.setInputCloud(map_);
  normals_->resize(map_->size());
  for (size_t index = 0; index < map_->size(); ++index) {
    auto& normal = (*normals_)[index];
    normal.normal_x = normal.normal_y = normal.normal_z =
        std::numeric_limits<float>::quiet_NaN();
    std::vector<int> neighbors(10);
    std::vector<float> distances(10);
    if (tree_.nearestKSearch((*map_)[index], 10, neighbors, distances) != 10) {
      continue;
    }
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (int neighbor : neighbors) {
      const auto& point = (*map_)[neighbor];
      center += Eigen::Vector3d(point.x, point.y, point.z);
    }
    center /= neighbors.size();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int neighbor : neighbors) {
      const auto& point = (*map_)[neighbor];
      const Eigen::Vector3d delta =
          Eigen::Vector3d(point.x, point.y, point.z) - center;
      covariance += delta * delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
    if (eigen.info() != Eigen::Success ||
        eigen.eigenvalues()(1) <= 1e-9 ||
        eigen.eigenvalues()(0) > eigen.eigenvalues()(1) * 0.2) {
      continue;
    }
    const Eigen::Vector3d n = eigen.eigenvectors().col(0);
    normal.normal_x = n.x();
    normal.normal_y = n.y();
    normal.normal_z = n.z();
  }
  ready_ = true;
  return {};
}

Result MapMatcher::Evidence(const Cloud& cloud, MatchResult* result) const {
  Matrix6d information = Matrix6d::Zero();
  uint32_t accepted = 0;
  double residual_sum = 0.0;
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  const double distance_squared =
      config_.correspondence_distance * config_.correspondence_distance;
  for (const auto& point : cloud) {
    if (tree_.nearestKSearch(point, 1, indices, distances) != 1 ||
        distances[0] > distance_squared) {
      continue;
    }
    const auto& normal = (*normals_)[indices[0]];
    Eigen::Vector3d n(normal.normal_x, normal.normal_y, normal.normal_z);
    if (!n.allFinite() || n.norm() < 0.5) {
      continue;
    }
    n.normalize();
    const auto& target = (*map_)[indices[0]];
    const Eigen::Vector3d x(point.x, point.y, point.z);
    const Eigen::Vector3d center(target.x, target.y, target.z);
    // Rotation error is left-multiplicative around base origin in map axes.
    Eigen::Matrix<double, 1, 6> jacobian;
    jacobian.head<3>() = n.transpose();
    jacobian.tail<3>() =
        -n.transpose() * Skew(x - result->pose.translation());
    information += jacobian.transpose() * jacobian;
    const double residual = n.dot(x - center);
    residual_sum += residual * residual;
    ++accepted;
  }
  result->overlap = static_cast<double>(accepted) / cloud.size();
  if (accepted < config_.minimum_points ||
      result->overlap < config_.minimum_overlap) {
    return {Reason::MATCH_FAILED, "insufficient map overlap"};
  }
  return ProjectMapInformation(information, residual_sum / accepted, config_, result);
}

Result ProjectMapInformation(const Matrix6d& information, double residual_variance,
                             const MatchConfig& config, MatchResult* result) {
  if (result == nullptr || !information.allFinite() ||
      !information.isApprox(information.transpose(), 1e-9) ||
      !std::isfinite(residual_variance) || residual_variance < 0.0 ||
      !std::isfinite(config.rotation_length) || config.rotation_length <= 0.0 ||
      !std::isfinite(config.minimum_information) || config.minimum_information <= 0.0 ||
      !std::isfinite(config.maximum_condition) || config.maximum_condition < 1.0 ||
      !std::isfinite(config.position_variance_floor) || config.position_variance_floor <= 0.0 ||
      !std::isfinite(config.rotation_variance_floor) || config.rotation_variance_floor <= 0.0) {
    return {Reason::INVALID_INPUT, "invalid geometric information or qualified floors"};
  }
  Matrix6d scale = Matrix6d::Identity();
  scale.bottomRightCorner<3, 3>() *= config.rotation_length;
  const Matrix6d inverse_scale = scale.inverse();
  Eigen::SelfAdjointEigenSolver<Matrix6d> eigen(
      inverse_scale * information * inverse_scale);
  if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite() ||
      eigen.eigenvalues().minCoeff() < -1e-9) {
    return {Reason::INVALID_INPUT, "scaled geometric information decomposition failed"};
  }
  const double threshold = std::max(
      config.minimum_information,
      eigen.eigenvalues().maxCoeff() / config.maximum_condition);
  const int rank = (eigen.eigenvalues().array() >= threshold).count();
  if (rank == 0) {
    return {Reason::DEGENERATE, "no observable registration direction"};
  }
  result->projection.resize(rank, 6);
  result->projected_covariance = Eigen::MatrixXd::Zero(rank, rank);
  const double variance =
      std::max(residual_variance, config.position_variance_floor);
  int row = 0;
  for (int index = 0; index < 6; ++index) {
    if (eigen.eigenvalues()(index) < threshold) {
      continue;
    }
    auto direction = eigen.eigenvectors().col(index).eval();
    Eigen::Index dominant;
    direction.cwiseAbs().maxCoeff(&dominant);
    if (direction(dominant) < 0.0) {
      direction = -direction;
    }
    result->projection.row(row) = direction.transpose() * scale;
    result->projected_covariance(row, row) = variance / eigen.eigenvalues()(index);
    ++row;
  }
  Matrix6d floor = Matrix6d::Zero();
  floor.topLeftCorner<3, 3>().diagonal().setConstant(config.position_variance_floor);
  floor.bottomRightCorner<3, 3>().diagonal().setConstant(config.rotation_variance_floor);
  result->projected_covariance +=
      result->projection * floor * result->projection.transpose();
  const Result canonical = CanonicalizeObservedSubspace(
      result->projection, result->projected_covariance, config.rotation_length,
      &result->projection, &result->projected_covariance);
  if (!canonical.ok()) {
    return canonical;
  }
  result->full_pose = rank == 6;
  result->covariance.setZero();
  if (result->full_pose) {
    const Matrix6d inverse_projection = result->projection.inverse();
    result->covariance = inverse_projection * result->projected_covariance *
                         inverse_projection.transpose();
    if (!ValidCovariance(result->covariance)) {
      return {Reason::INVALID_INPUT, "registration covariance invalid"};
    }
  }
  return {};
}

Result MapMatcher::Match(const Cloud::ConstPtr& base_cloud,
                         const Eigen::Isometry3d& initial,
                         MatchResult* result) const {
  if (!ready_) {
    return {Reason::MAP_NOT_READY, "map not loaded"};
  }
  if (result == nullptr || !base_cloud ||
      base_cloud->size() < config_.minimum_points ||
      !initial.matrix().allFinite()) {
    return {Reason::INVALID_INPUT, "invalid scan/initial pose"};
  }
  for (const auto& point : *base_cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      return {Reason::INVALID_INPUT, "nonfinite scan point"};
    }
  }
  Cloud::Ptr filtered(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(base_cloud);
  voxel.setLeafSize(config_.voxel_size, config_.voxel_size, config_.voxel_size);
  voxel.filter(*filtered);
  if (filtered->size() < config_.minimum_points) {
    return {Reason::MATCH_FAILED, "filtered scan is too sparse"};
  }
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setResolution(config_.resolution);
  ndt.setStepSize(0.1);
  ndt.setTransformationEpsilon(0.001);
  ndt.setMaximumIterations(40);
  ndt.setInputSource(filtered);
  ndt.setInputTarget(map_);
  Cloud aligned;
  ndt.align(aligned, initial.matrix().cast<float>());
  const double score = ndt.getFitnessScore(
      config_.correspondence_distance * config_.correspondence_distance);
  if (!ndt.hasConverged() || !std::isfinite(score) || score > config_.max_fitness) {
    return {Reason::MATCH_FAILED, "NDT did not produce an accepted match"};
  }
  const Eigen::Matrix4d transform =
      ndt.getFinalTransformation().cast<double>();
  if (!transform.allFinite() ||
      !(transform.topLeftCorner<3, 3>().transpose() *
        transform.topLeftCorner<3, 3>())
           .isApprox(Eigen::Matrix3d::Identity(), 1e-4) ||
      std::abs(transform.topLeftCorner<3, 3>().determinant() - 1.0) > 1e-4) {
    return {Reason::INVALID_INPUT, "NDT returned invalid rigid transform"};
  }
  result->pose = Eigen::Isometry3d(transform);
  result->pose.linear() =
      Eigen::Quaterniond(result->pose.linear()).normalized().toRotationMatrix();
  result->score = score;
  result->ambiguous = false;
  if ((result->pose.translation() - initial.translation()).norm() >
          config_.max_translation ||
      LogRotation(Eigen::Quaterniond(
          result->pose.linear() * initial.linear().transpose())).norm() >
          config_.max_rotation) {
    return {Reason::MATCH_FAILED, "match outside bounded convergence basin"};
  }
  return Evidence(aligned, result);
}

Result MapMatcher::Recover(const Cloud::ConstPtr& base_cloud,
                           const std::vector<Eigen::Isometry3d>& seeds,
                           MatchResult* result) const {
  if (result == nullptr || seeds.empty() ||
      seeds.size() > config_.maximum_seeds) {
    return {Reason::INVALID_INPUT, "missing or excessive recovery hypotheses"};
  }
  std::vector<MatchResult> hypotheses;
  for (const auto& seed : seeds) {
    MatchResult hypothesis;
    if (!Match(base_cloud, seed, &hypothesis).ok() || !hypothesis.full_pose) {
      continue;
    }
    auto same = std::find_if(
        hypotheses.begin(), hypotheses.end(), [&hypothesis](const MatchResult& other) {
          return (other.pose.translation() - hypothesis.pose.translation()).norm() < 0.5 &&
                 LogRotation(Eigen::Quaterniond(
                     other.pose.linear() * hypothesis.pose.linear().transpose())).norm() < 0.1;
        });
    if (same == hypotheses.end()) {
      hypotheses.push_back(hypothesis);
    } else if (hypothesis.score < same->score) {
      *same = hypothesis;
    }
  }
  if (hypotheses.empty()) {
    return {Reason::MATCH_FAILED, "all recovery hypotheses rejected"};
  }
  std::sort(hypotheses.begin(), hypotheses.end(),
            [](const MatchResult& a, const MatchResult& b) {
              return a.score < b.score;
            });
  *result = hypotheses.front();
  if (hypotheses.size() > 1 &&
      (hypotheses[1].score - result->score) /
              std::max(result->score, 1e-6) < config_.ambiguity_margin) {
    result->ambiguous = true;
    return {Reason::RELOCALIZATION_AMBIGUOUS, "distinct map matches have similar evidence"};
  }
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
