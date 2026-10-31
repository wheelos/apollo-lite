// Copyright 2026 WheelOS. All Rights Reserved.

#pragma once

#include <cstddef>
#include <vector>

namespace apollo {
namespace lidar_fusion {
namespace terrain {
namespace detail {

struct GridPoint {
  double x;
  double y;
};

struct Plane {
  double a;
  double b;
  double c;
  double residual_std;
  double max_abs_residual;
  double center_x;
  double center_y;
};

bool FitPlane(const std::vector<GridPoint>& xy, const std::vector<double>& z,
              Plane* plane, std::size_t min_samples = 4);
bool ContainsOriginStrict(const std::vector<GridPoint>& points);

}  // namespace detail
}  // namespace terrain
}  // namespace lidar_fusion
}  // namespace apollo
