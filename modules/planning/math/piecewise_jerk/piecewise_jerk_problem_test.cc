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

#include "modules/planning/math/piecewise_jerk/piecewise_jerk_problem.h"

#include <array>
#include <cstdio>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "cyber/init.h"

namespace apollo {
namespace planning {

namespace {

class TestPiecewiseJerkProblem : public PiecewiseJerkProblem {
 public:
  using PiecewiseJerkProblem::PiecewiseJerkProblem;

  PiecewiseJerkProblemData BuildData() { return FormulateProblem(); }

  void ReleaseData(PiecewiseJerkProblemData* data) { FreeData(data); }

 protected:
  void CalculateKernel(std::vector<OSQPFloat>* P_data,
                       std::vector<OSQPInt>* P_indices,
                       std::vector<OSQPInt>* P_indptr) override {
    const OSQPInt num_variables = static_cast<OSQPInt>(3 * num_of_knots_);
    P_data->assign(static_cast<size_t>(num_variables), 1.0);
    P_indices->clear();
    P_indptr->clear();

    for (OSQPInt i = 0; i < num_variables; ++i) {
      P_indices->push_back(i);
      P_indptr->push_back(i);
    }
    P_indptr->push_back(num_variables);
  }

  void CalculateOffset(std::vector<OSQPFloat>* q) override {
    q->assign(3 * num_of_knots_, 0.0);
  }
};

}  // namespace

TEST(PiecewiseJerkProblemTest, FormulatedMatricesOwnTheirStorage) {
  TestPiecewiseJerkProblem problem(3, 1.0, std::array<double, 3>{0.0, 0.0, 0.0});

  auto data = problem.BuildData();
  ASSERT_NE(data.P, nullptr);
  ASSERT_NE(data.A, nullptr);
  EXPECT_EQ(data.P->owned, 1);
  EXPECT_EQ(data.A->owned, 1);

  problem.ReleaseData(&data);
}

TEST(PiecewiseJerkProblemTest, OptimizeSucceedsForFeasibleProblem) {
  TestPiecewiseJerkProblem problem(3, 1.0, std::array<double, 3>{0.0, 0.0, 0.0});
  problem.set_x_bounds(-1.0, 1.0);
  problem.set_dx_bounds(-1.0, 1.0);
  problem.set_ddx_bounds(-1.0, 1.0);

  ASSERT_TRUE(problem.Optimize(100));
  ASSERT_EQ(problem.opt_x().size(), 3U);
  ASSERT_EQ(problem.opt_dx().size(), 3U);
  ASSERT_EQ(problem.opt_ddx().size(), 3U);

  for (double value : problem.opt_x()) {
    EXPECT_NEAR(value, 0.0, 1e-6);
  }
  for (double value : problem.opt_dx()) {
    EXPECT_NEAR(value, 0.0, 1e-6);
  }
  for (double value : problem.opt_ddx()) {
    EXPECT_NEAR(value, 0.0, 1e-6);
  }
}

TEST(PiecewiseJerkProblemTest, OptimizeFailsForInfeasibleBounds) {
  TestPiecewiseJerkProblem problem(3, 1.0, std::array<double, 3>{0.0, 0.0, 0.0});
  problem.set_x_bounds(1.0, 1.0);

  EXPECT_FALSE(problem.Optimize(100));
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
