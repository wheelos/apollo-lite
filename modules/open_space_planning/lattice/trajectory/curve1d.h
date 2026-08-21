/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <string>

namespace apollo {
namespace open_space_planning {
namespace lattice {

class Curve1d {
 public:
  virtual ~Curve1d() = default;

  virtual double Evaluate(std::uint32_t order, double parameter) const = 0;
  virtual double ParamLength() const = 0;
  virtual std::string ToString() const = 0;
};

}  // namespace lattice
}  // namespace open_space_planning
}  // namespace apollo

