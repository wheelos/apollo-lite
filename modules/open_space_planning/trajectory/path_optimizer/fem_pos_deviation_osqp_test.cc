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

#include "modules/open_space_planning/trajectory/path_optimizer/fem_pos_deviation_osqp.h"

#include <vector>

#include "gtest/gtest.h"

namespace apollo {
namespace open_space_planning {
namespace {

TEST(FemPosDeviationOsqpTest, SmoothsCoarseWaypointsWithinCorridor) {
  FemPosDeviationOsqp smoother;

  std::vector<GeometricPathPoint> raw_path;
  std::vector<CorridorSample> corridor;

  for (int i = 0; i < 10; ++i) {
    GeometricPathPoint pt;
    pt.pose.x = 1.0 * i;
    // Add small perturbation
    pt.pose.y = (i % 2 == 0) ? 0.2 : -0.2;
    pt.s = 1.0 * i;
    raw_path.push_back(pt);

    CorridorSample cs;
    cs.s = 1.0 * i;
    cs.minimum_lateral_offset = -1.0;
    cs.maximum_lateral_offset = 1.0;
    corridor.push_back(cs);
  }

  std::vector<GeometricPathPoint> smoothed_path;
  const bool success = smoother.Solve(raw_path, corridor, &smoothed_path);

  EXPECT_TRUE(success);
  EXPECT_EQ(smoothed_path.size(), raw_path.size());
  EXPECT_NEAR(smoothed_path.front().pose.x, raw_path.front().pose.x, 1e-3);
  EXPECT_NEAR(smoothed_path.front().pose.y, raw_path.front().pose.y, 1e-3);
  EXPECT_GT(smoothed_path.back().s, 0.0);
}

}  // namespace
}  // namespace open_space_planning
}  // namespace apollo
