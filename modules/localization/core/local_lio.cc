// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "modules/localization/core/local_lio.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

#include "Eigen/Cholesky"
#include "Eigen/Eigenvalues"
#include "Eigen/SVD"
#include "modules/localization/core/observed_subspace.h"

namespace apollo {
namespace localization {
namespace unified {

Result DeskewLidarScan(const LidarScan& scan, const StateLookup& lookup,
                       DeskewedScan* deskewed) {
  if (deskewed == nullptr || scan.points.empty() ||
      scan.epoch.session.empty() || scan.epoch.generation == 0 ||
      scan.frame_id.empty() || scan.calibration_id.empty() ||
      !scan.base_from_lidar.matrix().allFinite()) {
    return {Reason::INVALID_INPUT, "Invalid raw LiDAR scan"};
  }
  LocalState end;
  Result result = lookup(scan.stamp.time, &end);
  if (!result.ok()) {
    return result;
  }
  if (end.epoch != scan.epoch) {
    return {Reason::EPOCH_MISMATCH, "LiDAR scan crosses local epoch"};
  }
  DeskewedScan next;
  next.stamp = scan.stamp;
  next.epoch = scan.epoch;
  next.points_in_end_base.reserve(scan.points.size());
  const Eigen::Isometry3d end_from_local = Pose(end).inverse();
  for (const auto& point : scan.points) {
    if (!point.position.allFinite() || !std::isfinite(point.time) ||
        point.time <= 0.0) {
      return {Reason::POINT_TIME_MISSING,
              "Raw LiDAR point lacks a valid measurement timestamp"};
    }
    if (point.time > scan.stamp.time) {
      return {Reason::INVALID_INPUT,
              "LiDAR point timestamp exceeds scan end"};
    }
    LocalState at_point;
    result = lookup(point.time, &at_point);
    if (!result.ok()) {
      return result;
    }
    if (at_point.epoch != scan.epoch) {
      return {Reason::EPOCH_MISMATCH,
              "LiDAR deskew history changed epoch"};
    }
    next.points_in_end_base.push_back(
        end_from_local * Pose(at_point) * scan.base_from_lidar *
        point.position);
  }
  *deskewed = std::move(next);
  return {};
}

std::size_t LocalLio::VoxelHash::operator()(const VoxelKey& key) const {
  std::size_t seed = std::hash<int>()(key.x);
  seed ^= std::hash<int>()(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<int>()(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}

LocalLio::VoxelKey LocalLio::Key(const Eigen::Vector3d& point) const {
  return {static_cast<int>(std::floor(point.x() / config_.lidar_voxel_size)),
          static_cast<int>(std::floor(point.y() / config_.lidar_voxel_size)),
          static_cast<int>(std::floor(point.z() / config_.lidar_voxel_size))};
}

void LocalLio::Reset(const Epoch& epoch) {
  epoch_ = epoch;
  scans_.clear();
  planes_.clear();
  observed_pose_basis_.resize(0, 6);
}

std::vector<uint64_t> LocalLio::geometry_source_sequences() const {
  std::vector<uint64_t> sequences;
  sequences.reserve(scans_.size());
  for (const auto& scan : scans_) {
    sequences.push_back(scan.sequence);
  }
  return sequences;
}

void LocalLio::Rebuild() {
  struct Accumulator {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Matrix3d second = Eigen::Matrix3d::Zero();
    double anchor_variance = 0.0;
    uint32_t count = 0;
  };
  std::unordered_map<VoxelKey, Accumulator, VoxelHash> accumulators;
  for (const auto& scan : scans_) {
    for (const auto& point : scan.points) {
      auto& accumulator = accumulators[Key(point)];
      accumulator.sum += point;
      accumulator.second += point * point.transpose();
      accumulator.anchor_variance =
          std::max(accumulator.anchor_variance, scan.anchor_variance);
      ++accumulator.count;
    }
  }
  planes_.clear();
  for (const auto& item : accumulators) {
    if (item.second.count < config_.lidar_min_points_per_plane) {
      continue;
    }
    const auto& accumulator = item.second;
    const Eigen::Vector3d mean = accumulator.sum / accumulator.count;
    Eigen::Matrix3d covariance =
        accumulator.second / accumulator.count - mean * mean.transpose();
    covariance =
        (0.5 * (covariance + covariance.transpose())).eval();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success ||
        solver.eigenvalues()(1) <= config_.lidar_point_noise_std *
                                   config_.lidar_point_noise_std) {
      continue;
    }
    Plane plane;
    plane.point = mean;
    plane.normal = solver.eigenvectors().col(0);
    plane.variance =
        std::max(solver.eigenvalues()(0),
                 config_.lidar_plane_noise_std *
                     config_.lidar_plane_noise_std) +
        config_.lidar_anchor_variance_scale * accumulator.anchor_variance;
    planes_.emplace(item.first, plane);
    if (planes_.size() >= config_.lidar_max_voxels) {
      break;
    }
  }
}

Result LocalLio::BuildUpdate(const DeskewedScan& scan,
                             const LocalState& state,
                             LioUpdate* update) const {
  std::vector<Eigen::Matrix<double, 1, 15>> rows;
  std::vector<double> innovations;
  std::vector<double> variances;
  std::unordered_set<VoxelKey, VoxelHash> used;
  const Eigen::Matrix3d rotation = state.orientation.toRotationMatrix();
  for (const auto& point_in_base : scan.points_in_end_base) {
    const Eigen::Vector3d point = Pose(state) * point_in_base;
    const VoxelKey key = Key(point);
    const auto found = planes_.find(key);
    if (found == planes_.end() || !used.emplace(key).second) {
      continue;
    }
    const Plane& plane = found->second;
    const double residual = plane.normal.dot(point - plane.point);
    if (std::abs(residual) > config_.lidar_max_correspondence_distance) {
      continue;
    }
    Eigen::Matrix<double, 1, 15> row =
        Eigen::Matrix<double, 1, 15>::Zero();
    row.block<1, 3>(0, 0) = plane.normal.transpose();
    row.block<1, 3>(0, 6) =
        -plane.normal.transpose() * rotation * Skew(point_in_base);
    const double variance =
        plane.variance +
        config_.lidar_point_noise_std * config_.lidar_point_noise_std;
    const double innovation_variance =
        (row * state.covariance * row.transpose())(0, 0) + variance;
    if (!std::isfinite(innovation_variance) || innovation_variance <= 0.0 ||
        residual * residual / innovation_variance >
            config_.lidar_innovation_gate) {
      continue;
    }
    rows.push_back(row);
    innovations.push_back(-residual);
    variances.push_back(variance);
  }
  if (rows.size() < config_.lidar_min_correspondences) {
    return {Reason::GEOMETRY_UNAVAILABLE,
            "Insufficient trusted local plane correspondences"};
  }

  Eigen::MatrixXd whitened_pose(rows.size(), 6);
  Eigen::MatrixXd whitened_full(rows.size(), 15);
  Eigen::VectorXd whitened_innovation(rows.size());
  for (std::size_t i = 0; i < rows.size(); ++i) {
    whitened_pose.row(i).head<3>() =
        rows[i].block<1, 3>(0, 0) / std::sqrt(variances[i]);
    whitened_pose.row(i).tail<3>() =
        rows[i].block<1, 3>(0, 6) /
        (config_.lidar_rotation_length * std::sqrt(variances[i]));
    whitened_full.row(i) = rows[i] / std::sqrt(variances[i]);
    whitened_innovation(i) =
        innovations[i] / std::sqrt(variances[i]);
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      whitened_pose, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (svd.info() != Eigen::Success) {
    return {Reason::DEGENERATE, "Local geometry SVD failed"};
  }
  uint32_t rank = 0;
  for (int i = 0; i < svd.singularValues().size(); ++i) {
    rank += svd.singularValues()(i) >=
            config_.lidar_observed_singular_value;
  }
  if (rank == 0) {
    return {Reason::DEGENERATE,
            "Local geometry has no trusted observed direction"};
  }
  Matrix6d metric = Matrix6d::Identity();
  metric.bottomRightCorner<3, 3>() *= config_.lidar_rotation_length;
  const Eigen::MatrixXd raw_basis =
      svd.matrixV().leftCols(rank).transpose() * metric;
  Eigen::VectorXd observed_standard_deviation =
      svd.singularValues().head(rank).cwiseInverse();
  Eigen::MatrixXd observed_covariance =
      observed_standard_deviation
          .cwiseProduct(observed_standard_deviation)
          .asDiagonal();
  Eigen::MatrixXd basis;
  Eigen::MatrixXd canonical_covariance;
  const Result canonical = CanonicalizeObservedSubspace(
      raw_basis, observed_covariance, config_.lidar_rotation_length,
      &basis, &canonical_covariance);
  if (!canonical.ok()) {
    return canonical;
  }
  const Eigen::MatrixXd observed_rows =
      svd.matrixU().leftCols(rank).transpose();
  update->observation = observed_rows * whitened_full;
  update->innovation = observed_rows * whitened_innovation;
  update->noise = Eigen::MatrixXd::Identity(rank, rank);
  update->observed_pose_basis = basis;
  update->correspondences = rows.size();
  return {};
}

Result LocalLio::ApplyUpdate(const LioUpdate& update,
                             LocalState* state) const {
  const Eigen::MatrixXd innovation_covariance =
      update.observation * state->covariance *
          update.observation.transpose() +
      update.noise;
  Eigen::LDLT<Eigen::MatrixXd> factor(innovation_covariance);
  if (factor.info() != Eigen::Success) {
    return {Reason::INVALID_INPUT,
            "LiDAR innovation covariance is invalid"};
  }
  const Eigen::MatrixXd gain =
      factor.solve(update.observation * state->covariance).transpose();
  const Eigen::Matrix<double, 15, 1> correction =
      gain * update.innovation;
  LocalState next = *state;
  next.position += correction.segment<3>(0);
  next.velocity += correction.segment<3>(3);
  next.orientation =
      (next.orientation * ExpRotation(correction.segment<3>(6))).normalized();
  next.gyro_bias += correction.segment<3>(9);
  next.accel_bias += correction.segment<3>(12);
  // The local planes reuse earlier local states and raw points. Their exact
  // cross-correlation is not available here, so they may steer the nominal
  // state but cannot reduce the filter covariance as independent anchors.
  next.covariance = state->covariance;
  next.covariance.diagonal() +=
      config_.lidar_correction_variance_scale *
      correction.cwiseProduct(correction);
  next.covariance =
      (0.5 * (next.covariance + next.covariance.transpose())).eval();
  if (!next.position.allFinite() || !next.velocity.allFinite() ||
      !next.orientation.coeffs().allFinite() ||
      Eigen::LLT<Matrix15d>(next.covariance).info() != Eigen::Success) {
    return {Reason::INVALID_INPUT, "LiDAR update is not finite SPD"};
  }
  *state = next;
  return {};
}

void LocalLio::Insert(const DeskewedScan& scan, const LocalState& state) {
  StoredScan stored;
  stored.epoch = scan.epoch;
  stored.sequence = scan.stamp.sequence;
  stored.anchor_variance =
      state.covariance.block<3, 3>(0, 0).trace() +
      config_.lidar_voxel_size * config_.lidar_voxel_size *
          state.covariance.block<3, 3>(6, 6).trace();
  stored.points.reserve(scan.points_in_end_base.size());
  for (const auto& point : scan.points_in_end_base) {
    stored.points.push_back(Pose(state) * point);
  }
  scans_.push_back(std::move(stored));
  while (scans_.size() > config_.lidar_max_scans) {
    scans_.pop_front();
  }
  Rebuild();
}

Result LocalLio::Observe(const LidarScan& scan, const StateLookup& lookup,
                         LocalState* state) {
  if (state == nullptr || !state->valid || scan.epoch != state->epoch) {
    return {Reason::EPOCH_MISMATCH,
            "LiDAR update requires the current local epoch"};
  }
  if (epoch_ != state->epoch) {
    Reset(state->epoch);
  }
  DeskewedScan deskewed;
  Result result = DeskewLidarScan(scan, lookup, &deskewed);
  if (!result.ok()) {
    return result;
  }
  if (planes_.empty()) {
    Insert(deskewed, *state);
    return {Reason::GEOMETRY_UNAVAILABLE,
            "First local scan only initializes geometry"};
  }
  LocalState candidate = *state;
  LioUpdate update;
  for (uint32_t iteration = 0;
       iteration < config_.lidar_max_update_iterations; ++iteration) {
    result = BuildUpdate(deskewed, candidate, &update);
    if (!result.ok()) {
      return result;
    }
    result = ApplyUpdate(update, &candidate);
    if (!result.ok()) {
      return result;
    }
  }
  *state = candidate;
  observed_pose_basis_ = update.observed_pose_basis;
  Insert(deskewed, *state);
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
