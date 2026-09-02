// Copyright 2026 WheelOS. All Rights Reserved.
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

#include "modules/open_space_planning/trajectory/speed_optimizer/piecewise_jerk_speed_osqp.h"

#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace open_space_planning {
namespace {

TEST(PiecewiseJerkSpeedOsqpTest, OptimizesSmoothSpeedProfile) {
  PiecewiseJerkSpeedOsqp speed_optimizer;

  const double s_init = 0.0;
  const double v_init = 0.0;
  const double a_init = 0.0;
  const double v_target = 2.0;
  const double max_path_s = 10.0;
  const std::vector<STBoundary> st_boundaries;

  std::vector<SpeedProfilePoint> profile;
  const bool success = speed_optimizer.Solve(
      s_init, v_init, a_init, v_target, max_path_s, st_boundaries, &profile);

  EXPECT_TRUE(success);
  EXPECT_FALSE(profile.empty());
  EXPECT_NEAR(profile.front().s, 0.0, 1e-3);
  EXPECT_NEAR(profile.front().v, 0.0, 1e-3);
  EXPECT_GT(profile.back().s, 0.0);
}

TEST(PiecewiseJerkSpeedOsqpTest, YieldsBehindDynamicObstacle) {
  PiecewiseJerkSpeedOsqp speed_optimizer;

  const double s_init = 0.0;
  const double v_init = 1.0;
  const double a_init = 0.0;
  const double v_target = 2.0;
  const double max_path_s = 10.0;

  // Obstacle blocking s between 3.0m and 5.0m from t = 1.0s to 3.0s
  STBoundary boundary;
  boundary.obstacle_id = "obs_1";
  for (int step = 0; step <= 20; ++step) {
    const double t = 1.0 + step * 0.1;
    boundary.points.push_back({t, 3.0, 5.0});
  }
  std::vector<STBoundary> st_boundaries = {boundary};

  std::vector<SpeedProfilePoint> profile;
  const bool success = speed_optimizer.Solve(
      s_init, v_init, a_init, v_target, max_path_s, st_boundaries, &profile);

  EXPECT_TRUE(success);
  EXPECT_FALSE(profile.empty());

  for (const auto& pt : profile) {
    if (pt.t >= 1.0 && pt.t <= 3.0) {
      EXPECT_LE(pt.s, 3.0 + 1e-2);  // Must not breach lower boundary
    }
  }
}

}  // namespace
}  // namespace open_space_planning
}  // namespace apollo
