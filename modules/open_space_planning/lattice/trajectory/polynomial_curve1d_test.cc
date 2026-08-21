#include "modules/open_space_planning/lattice/trajectory/cubic_polynomial_curve1d.h"

#include "gtest/gtest.h"
#include "modules/open_space_planning/lattice/trajectory/quartic_polynomial_curve1d.h"
#include "modules/open_space_planning/lattice/trajectory/quintic_polynomial_curve1d.h"

namespace apollo {
namespace open_space_planning {
namespace lattice {
namespace {

TEST(PolynomialCurve1dTest, CubicMatchesBoundaryConditions) {
  CubicPolynomialCurve1d curve(1.0, 2.0, 0.5, 8.0, 2.0);

  EXPECT_DOUBLE_EQ(curve.Evaluate(0, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(curve.Evaluate(1, 0.0), 2.0);
  EXPECT_DOUBLE_EQ(curve.Evaluate(2, 0.0), 0.5);
  EXPECT_NEAR(curve.Evaluate(0, 2.0), 8.0, 1.0e-12);
}

TEST(PolynomialCurve1dTest, QuarticMatchesBoundaryConditions) {
  QuarticPolynomialCurve1d curve(0.0, 1.0, 0.2, 3.0, -0.1, 4.0);

  EXPECT_DOUBLE_EQ(curve.Evaluate(0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(curve.Evaluate(1, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(curve.Evaluate(2, 0.0), 0.2);
  EXPECT_NEAR(curve.Evaluate(1, 4.0), 3.0, 1.0e-12);
  EXPECT_NEAR(curve.Evaluate(2, 4.0), -0.1, 1.0e-12);
}

TEST(PolynomialCurve1dTest, QuinticMatchesBoundaryConditions) {
  QuinticPolynomialCurve1d curve(1.0, 0.5, 0.1, 7.0, 1.5, -0.2, 3.0);

  EXPECT_DOUBLE_EQ(curve.Evaluate(0, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(curve.Evaluate(1, 0.0), 0.5);
  EXPECT_DOUBLE_EQ(curve.Evaluate(2, 0.0), 0.1);
  EXPECT_NEAR(curve.Evaluate(0, 3.0), 7.0, 1.0e-12);
  EXPECT_NEAR(curve.Evaluate(1, 3.0), 1.5, 1.0e-12);
  EXPECT_NEAR(curve.Evaluate(2, 3.0), -0.2, 1.0e-12);
}

}  // namespace
}  // namespace lattice
}  // namespace open_space_planning
}  // namespace apollo

