/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/math/math_utils.h"

#include <cstdio>
#include <cstdlib>

#include "gtest/gtest.h"

#include "cyber/init.h"
#include "osqp.h"

namespace apollo {
namespace common {
namespace math {

TEST(MathUtilsTest, CrossProd) {
  EXPECT_NEAR(CrossProd({0, 0}, {0, 1}, {1, 0}), -1.0, 1e-5);
  EXPECT_NEAR(CrossProd({0, 0}, {1, 0}, {0, 1}), 1.0, 1e-5);
  EXPECT_NEAR(CrossProd({0, 1}, {0, 0}, {1, 0}), 1.0, 1e-5);
  EXPECT_NEAR(CrossProd({1, 2}, {3, 4}, {5, 6}), 0.0, 1e-5);
  EXPECT_NEAR(CrossProd({1, 2}, {3, 4}, {6, 5}), -4.0, 1e-5);
  EXPECT_NEAR(CrossProd({2, 2}, {7, 5}, {3, 4}), 7.0, 1e-5);
}

TEST(MathUtilsTest, InnerProd) {
  EXPECT_NEAR(InnerProd({0, 0}, {0, 1}, {1, 0}), 0.0, 1e-5);
  EXPECT_NEAR(InnerProd({0, 0}, {1, 0}, {0, 1}), 0.0, 1e-5);
  EXPECT_NEAR(InnerProd({0, 1}, {0, 0}, {1, 0}), 1.0, 1e-5);
  EXPECT_NEAR(InnerProd({1, 2}, {3, 4}, {5, 6}), 16.0, 1e-5);
  EXPECT_NEAR(InnerProd({1, 2}, {3, 4}, {6, 5}), 16.0, 1e-5);
  EXPECT_NEAR(InnerProd({2, 2}, {7, 5}, {3, 4}), 11.0, 1e-5);
  EXPECT_NEAR(InnerProd({2, 2}, {0, 0}, {3, 4}), -6.0, 1e-5);
}

TEST(MathUtilsTest, WrapAngle) {
  EXPECT_NEAR(WrapAngle(-1.2), -1.2 + M_PI * 2.0, 1e-6);
  EXPECT_NEAR(WrapAngle(3.4), 3.4, 1e-6);
  EXPECT_NEAR(WrapAngle(5.6), 5.6, 1e-6);
  EXPECT_NEAR(WrapAngle(7.8), 7.8 - M_PI * 2.0, 1e-6);
  EXPECT_NEAR(WrapAngle(12.4), std::fmod(12.4, M_PI * 2.0), 1e-6);
  EXPECT_NEAR(WrapAngle(-12.4), std::fmod(-12.4, M_PI * 2.0) + M_PI * 2.0,
              1e-6);
}

TEST(MathUtilsTest, NormalizeAngle) {
  EXPECT_DOUBLE_EQ(1.5, NormalizeAngle(1.5));
  EXPECT_DOUBLE_EQ(1.5 - M_PI, NormalizeAngle(1.5 + M_PI));
  EXPECT_DOUBLE_EQ(1.5, NormalizeAngle(1.5 + M_PI * 2));
  EXPECT_DOUBLE_EQ(1.5, NormalizeAngle(1.5 - M_PI * 2));
  EXPECT_DOUBLE_EQ(-1.5, NormalizeAngle(-1.5));
  EXPECT_DOUBLE_EQ(-9.0 + M_PI * 2, NormalizeAngle(-9.0));
  EXPECT_DOUBLE_EQ(-M_PI, NormalizeAngle(-M_PI));
  EXPECT_DOUBLE_EQ(-M_PI, NormalizeAngle(M_PI));
  EXPECT_DOUBLE_EQ(-M_PI, NormalizeAngle(-M_PI * 3));
  EXPECT_DOUBLE_EQ(-M_PI, NormalizeAngle(M_PI * 3));
  EXPECT_DOUBLE_EQ(0.0, NormalizeAngle(M_PI * 4));
}

TEST(MathUtilsTest, Square) {
  EXPECT_DOUBLE_EQ(121.0, Square(11.0));
  EXPECT_FLOAT_EQ(144.0f, Square(-12.0f));
  EXPECT_EQ(169, Square(-13));
  EXPECT_EQ(2147395600, Square(46340));
  EXPECT_EQ(-2147479015, Square(46341));  // Overflow!
}

TEST(MathUtilsTest, Sqr) {
  EXPECT_DOUBLE_EQ(121.0, Sqr(11.0));
  EXPECT_DOUBLE_EQ(0.25, Sqr(0.5));
  EXPECT_DOUBLE_EQ(169.0, Sqr(-13.0));
}

TEST(MathUtilsTest, Sigmoid) { EXPECT_DOUBLE_EQ(0.5, Sigmoid(0.0)); }

TEST(MathUtilsTest, Clamp) {
  EXPECT_EQ(1, Clamp(1, 0, 6));
  EXPECT_EQ(6, Clamp(7, 0, 6));
  EXPECT_EQ(0, Clamp(-1, 0, 6));
  EXPECT_EQ(0, Clamp(0, 0, 6));  // test lower bound as input
  EXPECT_EQ(6, Clamp(6, 0, 6));  // test upper bound as input
}

TEST(MathUtilsTest, RotateVector2d) {
  double expected_x = 0.0;
  double expected_y = std::sqrt(2.0);

  Eigen::Vector2d result = RotateVector2d({1.0, 1.0}, M_PI / 4);

  auto tol = 1.0e-10;
  EXPECT_NEAR(expected_x, result.x(), tol);
  EXPECT_NEAR(expected_y, result.y(), tol);

  expected_x = -1.0;
  expected_y = 0.0;
  result = RotateVector2d({0.0, 1.0}, M_PI / 2);
  EXPECT_NEAR(expected_x, result.x(), tol);
  EXPECT_NEAR(expected_y, result.y(), tol);

  expected_x = -1.0;
  expected_y = 0.0;
  result = RotateVector2d({1.0, 0.0}, M_PI);
  EXPECT_NEAR(expected_x, result.x(), tol);
  EXPECT_NEAR(expected_y, result.y(), tol);
}

TEST(MathUtilsTest, AlmostEqualTest) {
  double x = 2.0;
  double x_root = std::sqrt(x);
  double y = Square(x_root);
  EXPECT_TRUE(almost_equal(x, y, 2));
  EXPECT_FALSE(almost_equal(y, 2.0001, 2));
}

TEST(MathUtilsTest, QPSTTest) {
  // Load problem data
  OSQPFloat P_x[3] = {
      4.00000000000000000000,
      1.00000000000000000000,
      2.00000000000000000000,
  };
  OSQPInt P_nnz = 3;
  OSQPInt P_i[3] = {
      0,
      0,
      1,
  };
  OSQPInt P_p[3] = {
      0,
      1,
      3,
  };
  OSQPFloat q[2] = {
      1.00000000000000000000,
      1.00000000000000000000,
  };
  OSQPFloat A_x[4] = {
      1.00000000000000000000,
      1.00000000000000000000,
      1.00000000000000000000,
      1.00000000000000000000,
  };
  OSQPInt A_nnz = 4;
  OSQPInt A_i[4] = {
      0,
      1,
      0,
      2,
  };
  OSQPInt A_p[3] = {
      0,
      2,
      4,
  };
  OSQPFloat l[3] = {
      1.00000000000000000000,
      0.00000000000000000000,
      0.00000000000000000000,
  };
  OSQPFloat u[3] = {
      1.00000000000000000000,
      0.69999999999999995559,
      0.69999999999999995559,
  };
  OSQPInt n = 2;
  OSQPInt m = 3;

  // Problem settings
  OSQPSettings *settings =
      reinterpret_cast<OSQPSettings *>(std::malloc(sizeof(OSQPSettings)));
  OSQPCscMatrix *P = OSQPCscMatrix_new(n, n, P_nnz, P_x, P_i, P_p);
  OSQPCscMatrix *A = OSQPCscMatrix_new(m, n, A_nnz, A_x, A_i, A_p);
  OSQPSolver *solver = nullptr;

  // Define Solver settings as default
  osqp_set_default_settings(settings);

  // Setup workspace
  ASSERT_EQ(osqp_setup(&solver, P, q, A, l, u, m, n, settings), 0);
  ASSERT_NE(solver, nullptr);

  // Solve Problem
  EXPECT_EQ(osqp_solve(solver), 0);

  // Clean workspace
  osqp_cleanup(solver);
  OSQPCscMatrix_free(A);
  OSQPCscMatrix_free(P);
  std::free(settings);
}

}  // namespace math
}  // namespace common
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  apollo::cyber::Init(argv[0]);
  const int result = RUN_ALL_TESTS();
  apollo::cyber::Clear();
  std::fflush(nullptr);
  std::_Exit(result);
}
