/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "modules/open_space_planning/lattice/trajectory/polynomial_curve1d.h"

namespace apollo {
namespace open_space_planning {
namespace lattice {

class QuarticPolynomialCurve1d final : public PolynomialCurve1d {
 public:
  QuarticPolynomialCurve1d(const std::array<double, 3>& start,
                           const std::array<double, 2>& end,
                           double parameter);
  QuarticPolynomialCurve1d(double x0, double dx0, double ddx0, double dx1,
                           double ddx1, double parameter);

  double Evaluate(std::uint32_t order, double parameter) const override;
  double ParamLength() const override { return parameter_length_; }
  std::string ToString() const override;
  double Coefficient(std::size_t order) const override;
  std::size_t Order() const override { return 4; }

 private:
  void ComputeCoefficients(double x0, double dx0, double ddx0, double dx1,
                           double ddx1, double parameter);

  std::array<double, 5> coefficients_{{0.0, 0.0, 0.0, 0.0, 0.0}};
};

}  // namespace lattice
}  // namespace open_space_planning
}  // namespace apollo

