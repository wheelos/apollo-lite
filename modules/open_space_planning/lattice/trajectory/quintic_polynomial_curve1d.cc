/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#include "modules/open_space_planning/lattice/trajectory/quintic_polynomial_curve1d.h"

#include <cassert>
#include <sstream>

namespace apollo {
namespace open_space_planning {
namespace lattice {

QuinticPolynomialCurve1d::QuinticPolynomialCurve1d(
    const std::array<double, 3>& start, const std::array<double, 3>& end,
    double parameter)
    : QuinticPolynomialCurve1d(start[0], start[1], start[2], end[0], end[1],
                               end[2], parameter) {}

QuinticPolynomialCurve1d::QuinticPolynomialCurve1d(
    double x0, double dx0, double ddx0, double x1, double dx1, double ddx1,
    double parameter) {
  assert(parameter > 0.0);
  ComputeCoefficients(x0, dx0, ddx0, x1, dx1, ddx1, parameter);
  parameter_length_ = parameter;
}

double QuinticPolynomialCurve1d::Evaluate(std::uint32_t order,
                                          double parameter) const {
  switch (order) {
    case 0:
      return ((((coefficients_[5] * parameter + coefficients_[4]) * parameter +
                coefficients_[3]) *
                   parameter +
               coefficients_[2]) *
                  parameter +
              coefficients_[1]) *
                 parameter +
             coefficients_[0];
    case 1:
      return (((5.0 * coefficients_[5] * parameter +
                4.0 * coefficients_[4]) *
                   parameter +
               3.0 * coefficients_[3]) *
                  parameter +
              2.0 * coefficients_[2]) *
                 parameter +
             coefficients_[1];
    case 2:
      return ((20.0 * coefficients_[5] * parameter +
               12.0 * coefficients_[4]) *
                  parameter +
              6.0 * coefficients_[3]) *
                 parameter +
             2.0 * coefficients_[2];
    case 3:
      return (60.0 * coefficients_[5] * parameter +
              24.0 * coefficients_[4]) *
                 parameter +
             6.0 * coefficients_[3];
    case 4:
      return 120.0 * coefficients_[5] * parameter +
             24.0 * coefficients_[4];
    case 5:
      return 120.0 * coefficients_[5];
    default:
      return 0.0;
  }
}

std::string QuinticPolynomialCurve1d::ToString() const {
  std::ostringstream stream;
  for (double coefficient : coefficients_) {
    stream << coefficient << '\t';
  }
  stream << parameter_length_ << '\n';
  return stream.str();
}

double QuinticPolynomialCurve1d::Coefficient(std::size_t order) const {
  assert(order < coefficients_.size());
  return coefficients_[order];
}

void QuinticPolynomialCurve1d::ComputeCoefficients(
    double x0, double dx0, double ddx0, double x1, double dx1, double ddx1,
    double parameter) {
  coefficients_[0] = x0;
  coefficients_[1] = dx0;
  coefficients_[2] = 0.5 * ddx0;

  const double parameter2 = parameter * parameter;
  const double parameter3 = parameter * parameter2;
  const double c0 =
      (x1 - 0.5 * parameter2 * ddx0 - dx0 * parameter - x0) / parameter3;
  const double c1 = (dx1 - ddx0 * parameter - dx0) / parameter2;
  const double c2 = (ddx1 - ddx0) / parameter;

  coefficients_[3] = 0.5 * (20.0 * c0 - 8.0 * c1 + c2);
  coefficients_[4] = (-15.0 * c0 + 7.0 * c1 - c2) / parameter;
  coefficients_[5] = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / parameter2;
}

}  // namespace lattice
}  // namespace open_space_planning
}  // namespace apollo

