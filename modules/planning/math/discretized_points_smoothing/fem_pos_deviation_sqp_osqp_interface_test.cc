/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/planning/math/discretized_points_smoothing/fem_pos_deviation_sqp_osqp_interface.h"

#include <cstdio>
#include <cstdlib>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "cyber/init.h"

namespace apollo {
namespace planning {

namespace {

std::vector<std::pair<double, double>> StraightLineRefPoints() {
  return {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}};
}

}  // namespace

TEST(FemPosDeviationSqpOsqpInterfaceTest, SolveSucceedsForStraightLineInput) {
  FemPosDeviationSqpOsqpInterface solver;
  const auto ref_points = StraightLineRefPoints();
  solver.set_ref_points(ref_points);
  solver.set_bounds_around_refs(std::vector<double>(ref_points.size(), 0.0));
  solver.set_curvature_constraint(1.0);
  solver.set_time_limit(1.0);
  solver.set_sqp_pen_max_iter(0);

  ASSERT_TRUE(solver.Solve());
  ASSERT_EQ(solver.opt_xy().size(), ref_points.size());

  for (size_t i = 0; i < ref_points.size(); ++i) {
    EXPECT_NEAR(solver.opt_xy()[i].first, ref_points[i].first, 1e-6);
    EXPECT_NEAR(solver.opt_xy()[i].second, ref_points[i].second, 1e-6);
  }
}

TEST(FemPosDeviationSqpOsqpInterfaceTest, SolveFailsWhenInputSizesMismatch) {
  FemPosDeviationSqpOsqpInterface solver;
  solver.set_ref_points(StraightLineRefPoints());
  solver.set_bounds_around_refs({0.0, 0.0, 0.0});

  EXPECT_FALSE(solver.Solve());
}

}  // namespace planning
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  apollo::cyber::Init(argv[0]);
  const int result = RUN_ALL_TESTS();
  apollo::cyber::Clear();
  std::fflush(nullptr);
  std::_Exit(result);
}
