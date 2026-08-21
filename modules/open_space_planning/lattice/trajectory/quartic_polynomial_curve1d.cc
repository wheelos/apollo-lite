/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#include "modules/open_space_planning/lattice/trajectory/quartic_polynomial_curve1d.h"

#include <cassert>
#include <sstream>

namespace apollo {
namespace open_space_planning {
namespace lattice {

QuarticPolynomialCurve1d::QuarticPolynomialCurve1d(
    const std::array<double, 3>& start, const std::array<double, 2>& end,
    double parameter)
    : QuarticPolynomialCurve1d(start[0], start[1], start[2], end[0], end[1],
                               parameter) {}

QuarticPolynomialCurve1d::QuarticPolynomialCurve1d(
    double x0, double dx0, double ddx0, double dx1, double ddx1,
    double parameter) {
  assert(parameter > 0.0);
  parameter_length_ = parameter;
  ComputeCoefficients(x0, dx0, ddx0, dx1, ddx1, parameter);
}

double QuarticPolynomialCurve1d::Evaluate(std::uint32_t order,
                                          double parameter) const {
  switch (order) {
    case 0:
      return (((coefficients_[4] * parameter + coefficients_[3]) * parameter +
               coefficients_[2]) *
                  parameter +
              coefficients_[1]) *
                 parameter +
             coefficients_[0];
    case 1:
      return ((4.0 * coefficients_[4] * parameter +
               3.0 * coefficients_[3]) *
                  parameter +
              2.0 * coefficients_[2]) *
                 parameter +
             coefficients_[1];
    case 2:
      return (12.0 * coefficients_[4] * parameter +
              6.0 * coefficients_[3]) *
                 parameter +
             2.0 * coefficients_[2];
    case 3:
      return 24.0 * coefficients_[4] * parameter + 6.0 * coefficients_[3];
    case 4:
      return 24.0 * coefficients_[4];
    default:
      return 0.0;
  }
}

std::string QuarticPolynomialCurve1d::ToString() const {
  std::ostringstream stream;
  for (double coefficient : coefficients_) {
    stream << coefficient << '\t';
  }
  stream << parameter_length_ << '\n';
  return stream.str();
}

double QuarticPolynomialCurve1d::Coefficient(std::size_t order) const {
  assert(order < coefficients_.size());
  return coefficients_[order];
}

void QuarticPolynomialCurve1d::ComputeCoefficients(
    double x0, double dx0, double ddx0, double dx1, double ddx1,
    double parameter) {
  coefficients_[0] = x0;
  coefficients_[1] = dx0;
  coefficients_[2] = 0.5 * ddx0;

  const double b0 = dx1 - ddx0 * parameter - dx0;
  const double b1 = ddx1 - ddx0;
  const double parameter2 = parameter * parameter;
  const double parameter3 = parameter2 * parameter;

  coefficients_[3] =
      (3.0 * b0 - b1 * parameter) / (3.0 * parameter2);
  coefficients_[4] =
      (-2.0 * b0 + b1 * parameter) / (4.0 * parameter3);
}

}  // namespace lattice
}  // namespace open_space_planning
}  // namespace apollo

