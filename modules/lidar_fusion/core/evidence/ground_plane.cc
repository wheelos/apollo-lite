// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/lidar_fusion/core/evidence/ground_plane.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "Eigen/QR"

namespace apollo {
namespace lidar_fusion {
namespace fusion_detail {
namespace {
bool SolveGround(const Eigen::MatrixXd& design, const Eigen::VectorXd& heights,
                 const std::vector<int>& rows, const FusionConfig& config,
                 GroundPlane* plane) {
  if (rows.size() < config.ground_min_points()) {
    return false;
  }
  Eigen::MatrixXd subset(rows.size(), 3);
  Eigen::VectorXd z(rows.size());
  for (std::size_t i = 0; i < rows.size(); ++i) {
    subset.row(i) = design.row(rows[i]);
    z[i] = heights[rows[i]];
  }
  const auto qr = subset.colPivHouseholderQr();
  if (qr.rank() != 3) {
    return false;
  }
  plane->coefficients = qr.solve(z);
  plane->residual = (subset * plane->coefficients - z).cwiseAbs().maxCoeff();
  plane->count = rows.size();
  return plane->coefficients.allFinite() &&
         plane->coefficients.head<2>().norm() <= config.ground_max_slope() &&
         plane->residual <= config.ground_max_residual();
}

bool RobustGround(const Eigen::MatrixXd& design, const Eigen::VectorXd& heights,
                  const FusionConfig& config, GroundPlane* plane) {
  std::vector<double> sorted(heights.data(), heights.data() + heights.size());
  auto median = sorted.begin() + sorted.size() / 2;
  std::nth_element(sorted.begin(), median, sorted.end());
  const std::array<Eigen::Vector3d, 2> starts{plane->coefficients,
                                              Eigen::Vector3d(0, 0, *median)};
  GroundPlane best = *plane;
  best.count = 0;
  for (const auto& start : starts) {
    GroundPlane candidate = *plane;
    candidate.coefficients = start;
    for (int iteration = 0; iteration < 3; ++iteration) {
      std::vector<int> rows;
      const Eigen::VectorXd residual =
          (design * candidate.coefficients - heights).cwiseAbs();
      for (int i = 0; i < residual.size(); ++i) {
        if (residual[i] <= config.ground_max_residual()) {
          rows.push_back(i);
        }
      }
      if (rows.size() < config.quality().ground_inlier_ratio() *
                            static_cast<double>(heights.size()) ||
          !SolveGround(design, heights, rows, config, &candidate)) {
        break;
      }
      if (candidate.count > best.count ||
          (candidate.count == best.count &&
           candidate.residual < best.residual)) {
        best = candidate;
      }
    }
  }
  if (best.count == 0) {
    return false;
  }
  *plane = best;
  return true;
}
}  // namespace

GroundPlanes FitGround(const std::vector<Eigen::Vector3d>& positions,
                       const SpatialIndex& seed_tiles,
                       const FusionConfig& config) {
  GroundPlanes planes;
  planes.reserve(seed_tiles.size());
  for (const auto& tile : seed_tiles) {
    if (tile.second.size() < config.ground_min_points()) {
      continue;
    }
    GroundPlane plane;
    plane.origin_x = (tile.first.first + 0.5) * config.ground_tile_size();
    plane.origin_y = (tile.first.second + 0.5) * config.ground_tile_size();
    Eigen::MatrixXd design(tile.second.size(), 3);
    Eigen::VectorXd heights(tile.second.size());
    for (std::size_t row = 0; row < tile.second.size(); ++row) {
      const auto& p = positions[tile.second[row]];
      design.row(row) << p.x() - plane.origin_x, p.y() - plane.origin_y, 1;
      heights[row] = p.z();
    }
    const auto qr = design.colPivHouseholderQr();
    if (qr.rank() != 3) {
      continue;
    }
    plane.coefficients = qr.solve(heights);
    plane.residual =
        (design * plane.coefficients - heights).cwiseAbs().maxCoeff();
    plane.count = tile.second.size();
    const bool accepted =
        plane.coefficients.allFinite() &&
        plane.coefficients.head<2>().norm() <= config.ground_max_slope() &&
        plane.residual <= config.ground_max_residual();
    if (!accepted) {
      if (!config.quality().robust_ground() ||
          !RobustGround(design, heights, config, &plane)) {
        continue;
      }
    }
    planes.emplace(tile.first, plane);
  }
  return planes;
}

}  // namespace fusion_detail
}  // namespace lidar_fusion
}  // namespace apollo
