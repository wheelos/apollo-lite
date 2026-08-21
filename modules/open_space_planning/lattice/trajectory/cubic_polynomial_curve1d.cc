/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#include "modules/open_space_planning/lattice/trajectory/cubic_polynomial_curve1d.h"

#include <cassert>
#include <sstream>

namespace apollo {
namespace open_space_planning {
namespace lattice {

CubicPolynomialCurve1d::CubicPolynomialCurve1d(
    const std::array<double, 3>& start, double end, double parameter)
    : CubicPolynomialCurve1d(start[0], start[1], start[2], end, parameter) {}

CubicPolynomialCurve1d::CubicPolynomialCurve1d(
    double x0, double dx0, double ddx0, double x1, double parameter) {
  assert(parameter > 0.0);
  ComputeCoefficients(x0, dx0, ddx0, x1, parameter);
  parameter_length_ = parameter;
}

double CubicPolynomialCurve1d::Evaluate(std::uint32_t order,
                                        double parameter) const {
  switch (order) {
    case 0:
      return ((coefficients_[3] * parameter + coefficients_[2]) * parameter +
              coefficients_[1]) *
                 parameter +
             coefficients_[0];
    case 1:
      return (3.0 * coefficients_[3] * parameter +
              2.0 * coefficients_[2]) *
                 parameter +
             coefficients_[1];
    case 2:
      return 6.0 * coefficients_[3] * parameter + 2.0 * coefficients_[2];
    case 3:
      return 6.0 * coefficients_[3];
    default:
      return 0.0;
  }
}

std::string CubicPolynomialCurve1d::ToString() const {
  std::ostringstream stream;
  for (double coefficient : coefficients_) {
    stream << coefficient << '\t';
  }
  stream << parameter_length_ << '\n';
  return stream.str();
}

double CubicPolynomialCurve1d::Coefficient(std::size_t order) const {
  assert(order < coefficients_.size());
  return coefficients_[order];
}

void CubicPolynomialCurve1d::ComputeCoefficients(
    double x0, double dx0, double ddx0, double x1, double parameter) {
  const double parameter2 = parameter * parameter;
  const double parameter3 = parameter * parameter2;
  coefficients_[0] = x0;
  coefficients_[1] = dx0;
  coefficients_[2] = 0.5 * ddx0;
  coefficients_[3] =
      (x1 - x0 - dx0 * parameter - coefficients_[2] * parameter2) /
      parameter3;
}

}  // namespace lattice
}  // namespace open_space_planning
}  // namespace apollo

