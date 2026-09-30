// Copyright 2026 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdint>
#include <optional>
#include <tuple>
#include <vector>

namespace apollo {
namespace local_planning {

struct Point2 {
  double x = 0.0;
  double y = 0.0;
  bool operator==(const Point2& p) const { return x == p.x && y == p.y; }
};

// One forward tube around a non-self-intersecting polyline, not a road graph.
struct LocalCorridor {
  uint64_t lane_id = 0;
  bool confirmed_forward = false;
  double half_width = 0.0;
  double position_error_bound = 0.0;
  std::vector<Point2> centerline;
  std::vector<Point2> left_boundary;
  std::vector<Point2> right_boundary;
  std::optional<double> speed_limit;
  std::optional<double> stop_station;
  bool operator==(const LocalCorridor& c) const {
    return std::tie(lane_id, confirmed_forward, half_width,
                    position_error_bound, centerline, left_boundary,
                    right_boundary, speed_limit, stop_station) ==
           std::tie(c.lane_id, c.confirmed_forward, c.half_width,
                    c.position_error_bound, c.centerline, c.left_boundary,
                    c.right_boundary, c.speed_limit, c.stop_station);
  }
};

struct PredictedDisc {
  uint64_t id = 0;
  Point2 position;
  Point2 velocity;
  double radius = 0.0;
  bool operator==(const PredictedDisc& d) const {
    return std::tie(id, position, velocity, radius) ==
           std::tie(d.id, d.position, d.velocity, d.radius);
  }
};

// Constant-velocity envelope at prediction.stamp.measurement_time in ODOM.
// Coverage declarations are producer claims, not a proof of spatial safety.
struct LocalOccupancy {
  bool corridor_fully_observed = false;
  bool drivable_region_fully_observed = false;
  double horizon = 0.0;
  std::vector<PredictedDisc> obstacles;
  bool operator==(const LocalOccupancy& o) const {
    return std::tie(corridor_fully_observed, drivable_region_fully_observed,
                    horizon, obstacles) ==
           std::tie(o.corridor_fully_observed, o.drivable_region_fully_observed,
                    o.horizon, o.obstacles);
  }
};

}  // namespace local_planning
}  // namespace apollo
