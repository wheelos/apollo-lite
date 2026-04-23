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

void ExpectUpperTriangularSorted(const std::vector<OSQPInt>& indptr,
                                 const std::vector<OSQPInt>& indices,
                                 const OSQPInt num_cols) {
  ASSERT_EQ(indptr.size(), static_cast<size_t>(num_cols + 1));
  for (OSQPInt col = 0; col < num_cols; ++col) {
    ASSERT_LE(indptr[col], indptr[col + 1]);
    for (OSQPInt idx = indptr[col]; idx < indptr[col + 1]; ++idx) {
      EXPECT_LE(indices[idx], col);
      if (idx + 1 < indptr[col + 1]) {
        EXPECT_LT(indices[idx], indices[idx + 1]);
      }
    }
  }
}

bool HasOffDiagonalEntry(const std::vector<OSQPInt>& indptr,
                         const std::vector<OSQPInt>& indices,
                         const OSQPInt num_cols) {
  for (OSQPInt col = 0; col < num_cols; ++col) {
    for (OSQPInt idx = indptr[col]; idx < indptr[col + 1]; ++idx) {
      if (indices[idx] < col) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

class FemPosDeviationSqpOsqpInterfaceTestPeer {
 public:
  static void PrepareKernelTest(FemPosDeviationSqpOsqpInterface* solver,
                                const int num_points) {
    solver->num_of_points_ = num_points;
    solver->num_of_pos_variables_ = num_points * 2;
    solver->num_of_slack_variables_ = num_points - 2;
    solver->num_of_variables_ = solver->num_of_pos_variables_ +
                                solver->num_of_slack_variables_;
  }

  static void PrepareAffineConstraintTest(
      FemPosDeviationSqpOsqpInterface* solver, const int num_points) {
    PrepareKernelTest(solver, num_points);
    solver->num_of_variable_constraints_ = solver->num_of_variables_;
    solver->num_of_curvature_constraints_ = num_points - 2;
    solver->num_of_constraints_ = solver->num_of_variable_constraints_ +
                                  solver->num_of_curvature_constraints_;
  }

  static void CalculateKernel(FemPosDeviationSqpOsqpInterface* solver,
                              std::vector<OSQPFloat>* P_data,
                              std::vector<OSQPInt>* P_indices,
                              std::vector<OSQPInt>* P_indptr) {
    solver->CalculateKernel(P_data, P_indices, P_indptr);
  }

  static void CalculateAffineConstraint(
      FemPosDeviationSqpOsqpInterface* solver,
      const std::vector<std::pair<double, double>>& points,
      std::vector<OSQPFloat>* A_data, std::vector<OSQPInt>* A_indices,
      std::vector<OSQPInt>* A_indptr,
      std::vector<OSQPFloat>* lower_bounds,
      std::vector<OSQPFloat>* upper_bounds) {
    solver->CalculateAffineConstraint(points, A_data, A_indices, A_indptr,
                                      lower_bounds, upper_bounds);
  }
};

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

TEST(FemPosDeviationSqpOsqpInterfaceTest, KernelUsesUpperTriangularSortedCsc) {
  FemPosDeviationSqpOsqpInterface solver;
  const auto ref_points = StraightLineRefPoints();
  solver.set_ref_points(ref_points);
  solver.set_bounds_around_refs(std::vector<double>(ref_points.size(), 0.0));
  FemPosDeviationSqpOsqpInterfaceTestPeer::PrepareKernelTest(
      &solver, static_cast<int>(ref_points.size()));

  std::vector<OSQPFloat> P_data;
  std::vector<OSQPInt> P_indices;
  std::vector<OSQPInt> P_indptr;
  FemPosDeviationSqpOsqpInterfaceTestPeer::CalculateKernel(
      &solver, &P_data, &P_indices, &P_indptr);

  const OSQPInt num_cols = static_cast<OSQPInt>(P_indptr.size() - 1);
  ExpectUpperTriangularSorted(P_indptr, P_indices, num_cols);
  EXPECT_TRUE(HasOffDiagonalEntry(P_indptr, P_indices, num_cols));
}

TEST(FemPosDeviationSqpOsqpInterfaceTest,
     AffineConstraintSparsityPatternRemainsStableAcrossIterations) {
  FemPosDeviationSqpOsqpInterface solver;
  const auto ref_points = StraightLineRefPoints();
  solver.set_ref_points(ref_points);
  solver.set_bounds_around_refs(std::vector<double>(ref_points.size(), 0.5));
  FemPosDeviationSqpOsqpInterfaceTestPeer::PrepareAffineConstraintTest(
      &solver, static_cast<int>(ref_points.size()));

  std::vector<OSQPFloat> first_a_data;
  std::vector<OSQPInt> first_a_indices;
  std::vector<OSQPInt> first_a_indptr;
  std::vector<OSQPFloat> first_lower_bounds;
  std::vector<OSQPFloat> first_upper_bounds;
  FemPosDeviationSqpOsqpInterfaceTestPeer::CalculateAffineConstraint(
      &solver, ref_points, &first_a_data, &first_a_indices, &first_a_indptr,
      &first_lower_bounds, &first_upper_bounds);

  auto perturbed_points = ref_points;
  perturbed_points[1].second = 0.2;
  perturbed_points[2].first = 2.1;
  perturbed_points[2].second = -0.1;

  std::vector<OSQPFloat> second_a_data;
  std::vector<OSQPInt> second_a_indices;
  std::vector<OSQPInt> second_a_indptr;
  std::vector<OSQPFloat> second_lower_bounds;
  std::vector<OSQPFloat> second_upper_bounds;
  FemPosDeviationSqpOsqpInterfaceTestPeer::CalculateAffineConstraint(
      &solver, perturbed_points, &second_a_data, &second_a_indices,
      &second_a_indptr, &second_lower_bounds, &second_upper_bounds);

  EXPECT_EQ(first_a_indptr, second_a_indptr);
  EXPECT_EQ(first_a_indices, second_a_indices);
  ASSERT_EQ(first_a_data.size(), second_a_data.size());

  bool values_changed = false;
  for (size_t i = 0; i < first_a_data.size(); ++i) {
    if (first_a_data[i] != second_a_data[i]) {
      values_changed = true;
      break;
    }
  }
  EXPECT_TRUE(values_changed);
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
