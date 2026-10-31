// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/lidar_fusion/core/terrain/plane_fit.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "Eigen/Core"
#include "Eigen/QR"

namespace apollo {
namespace lidar_fusion {
namespace terrain {
namespace detail {

constexpr double kRankTolerance = 1e-12;
constexpr double kHullTolerance = 1e-12;

bool FitPlane(const std::vector<GridPoint>& xy, const std::vector<double>& z,
              Plane* plane, std::size_t min_samples) {
  if (xy.size() < min_samples || xy.size() != z.size()) {
    return false;
  }
  double center_x = 0.0;
  double center_y = 0.0;
  for (const auto& point : xy) {
    center_x += point.x;
    center_y += point.y;
  }
  center_x /= static_cast<double>(xy.size());
  center_y /= static_cast<double>(xy.size());
  Eigen::MatrixXd design(xy.size(), 3);
  Eigen::VectorXd values(xy.size());
  for (std::size_t i = 0; i < xy.size(); ++i) {
    design(static_cast<Eigen::Index>(i), 0) = xy[i].x - center_x;
    design(static_cast<Eigen::Index>(i), 1) = xy[i].y - center_y;
    design(static_cast<Eigen::Index>(i), 2) = 1.0;
    values(static_cast<Eigen::Index>(i)) = z[i];
  }
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(design);
  qr.setThreshold(kRankTolerance);
  if (qr.rank() != 3) {
    return false;
  }
  const Eigen::Vector3d coefficients = qr.solve(values);
  if (!coefficients.allFinite()) {
    return false;
  }
  const Eigen::VectorXd residuals = design * coefficients - values;
  const double variance =
      residuals.squaredNorm() / static_cast<double>(xy.size() - 1);
  if (!std::isfinite(variance) || variance < 0.0) {
    return false;
  }
  const double max_abs_residual = residuals.cwiseAbs().maxCoeff();
  *plane = Plane{
      coefficients[0],  coefficients[1], coefficients[2], std::sqrt(variance),
      max_abs_residual, center_x,        center_y};
  return true;
}

bool ContainsOriginStrict(const std::vector<GridPoint>& points) {
  std::vector<GridPoint> ordered = points;
  std::sort(ordered.begin(), ordered.end(),
            [](const GridPoint& lhs, const GridPoint& rhs) {
              return lhs.x == rhs.x ? lhs.y < rhs.y : lhs.x < rhs.x;
            });
  ordered.erase(std::unique(ordered.begin(), ordered.end(),
                            [](const GridPoint& lhs, const GridPoint& rhs) {
                              return lhs.x == rhs.x && lhs.y == rhs.y;
                            }),
                ordered.end());
  if (ordered.size() < 3) {
    return false;
  }
  const auto cross = [](const GridPoint& origin, const GridPoint& first,
                        const GridPoint& second) {
    return (first.x - origin.x) * (second.y - origin.y) -
           (first.y - origin.y) * (second.x - origin.x);
  };
  std::vector<GridPoint> lower;
  for (const auto& point : ordered) {
    while (lower.size() >= 2 &&
           cross(lower[lower.size() - 2], lower.back(), point) <= 0.0) {
      lower.pop_back();
    }
    lower.push_back(point);
  }
  std::vector<GridPoint> upper;
  for (auto it = ordered.rbegin(); it != ordered.rend(); ++it) {
    while (upper.size() >= 2 &&
           cross(upper[upper.size() - 2], upper.back(), *it) <= 0.0) {
      upper.pop_back();
    }
    upper.push_back(*it);
  }
  lower.pop_back();
  upper.pop_back();
  lower.insert(lower.end(), upper.begin(), upper.end());
  if (lower.size() < 3) {
    return false;
  }
  for (std::size_t i = 0; i < lower.size(); ++i) {
    const auto& first = lower[i];
    const auto& second = lower[(i + 1) % lower.size()];
    if (first.x * second.y - first.y * second.x <= kHullTolerance) {
      return false;
    }
  }
  return true;
}

}  // namespace detail
}  // namespace terrain
}  // namespace lidar_fusion
}  // namespace apollo
