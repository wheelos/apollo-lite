/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#pragma once

#include <cstddef>

#include "modules/open_space_planning/lattice/trajectory/curve1d.h"

namespace apollo {
namespace open_space_planning {
namespace lattice {

class PolynomialCurve1d : public Curve1d {
 public:
  ~PolynomialCurve1d() override = default;

  virtual double Coefficient(std::size_t order) const = 0;
  virtual std::size_t Order() const = 0;

 protected:
  double parameter_length_ = 0.0;
};

}  // namespace lattice
}  // namespace open_space_planning
}  // namespace apollo

