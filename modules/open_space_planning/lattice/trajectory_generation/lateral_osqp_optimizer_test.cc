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

#include "modules/open_space_planning/lattice/trajectory_generation/lateral_osqp_optimizer.h"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "cyber/init.h"

namespace apollo {
namespace planning {

TEST(LateralOSQPOptimizerTest, OptimizeSucceedsForFeasibleBounds) {
  LateralOSQPOptimizer optimizer;
  const std::array<double, 3> d_state = {0.0, 0.0, 0.0};
  const std::vector<std::pair<double, double>> d_bounds = {
      {-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}};

  ASSERT_TRUE(optimizer.optimize(d_state, 1.0, d_bounds));

  const auto frenet_path = optimizer.GetFrenetFramePath();
  ASSERT_EQ(frenet_path.size(), d_bounds.size());
  EXPECT_NEAR(frenet_path.front().l(), d_state[0], 1e-6);
  EXPECT_NEAR(frenet_path.front().dl(), d_state[1], 1e-6);
  EXPECT_NEAR(frenet_path.front().ddl(), d_state[2], 1e-6);
  EXPECT_NEAR(frenet_path.back().dl(), 0.0, 1e-6);
  EXPECT_NEAR(frenet_path.back().ddl(), 0.0, 1e-6);

  const auto trajectory = optimizer.GetOptimalTrajectory();
  EXPECT_NEAR(trajectory.Evaluate(0, 0.0), d_state[0], 1e-6);
  EXPECT_NEAR(trajectory.Evaluate(1, 0.0), d_state[1], 1e-6);
  EXPECT_NEAR(trajectory.Evaluate(2, 0.0), d_state[2], 1e-6);
}

TEST(LateralOSQPOptimizerTest, OptimizeFailsForInfeasibleBounds) {
  LateralOSQPOptimizer optimizer;
  const std::array<double, 3> d_state = {0.0, 0.0, 0.0};
  const std::vector<std::pair<double, double>> d_bounds = {
      {1.0, 1.0}, {1.0, 1.0}, {1.0, 1.0}};

  EXPECT_FALSE(optimizer.optimize(d_state, 1.0, d_bounds));
}

TEST(LateralOSQPOptimizerTest, OptimizeClearsPreviousSolutionState) {
  LateralOSQPOptimizer optimizer;
  const std::array<double, 3> d_state = {0.0, 0.0, 0.0};
  const std::vector<std::pair<double, double>> first_bounds = {
      {-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}};
  const std::vector<std::pair<double, double>> second_bounds = {{-0.5, 0.5},
                                                                {-0.5, 0.5}};

  ASSERT_TRUE(optimizer.optimize(d_state, 1.0, first_bounds));
  ASSERT_TRUE(optimizer.optimize(d_state, 1.0, second_bounds));

  const auto frenet_path = optimizer.GetFrenetFramePath();
  EXPECT_EQ(frenet_path.size(), second_bounds.size());
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
