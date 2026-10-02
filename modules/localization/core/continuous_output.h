// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include "modules/localization/core/types.h"

namespace apollo {
namespace localization {
namespace unified {

class ContinuousOutput {
 public:
  explicit ContinuousOutput(const LocalConfig& config) : config_(config) {}

  void Reset(const LocalState& internal);
  Result Advance(const LocalState& previous_internal,
                 const LocalState& internal,
                 const Matrix15d& error_transition, double dt);
  const LocalState& state() const { return state_; }
  bool active() const { return active_; }
  bool correction_budget_exceeded() const {
    return correction_budget_exceeded_;
  }

 private:
  // This 15D covariance supports finite propagation and degradation budgets.
  // It is not a qualified joint marginal with the corrected internal state or
  // reused local geometry, and must not authorize LocalMotionIncrement export.
  LocalConfig config_;
  LocalState state_;
  Eigen::Vector3d pose_correction_velocity_ = Eigen::Vector3d::Zero();
  bool active_ = false;
  bool correction_budget_exceeded_ = false;
};

}  // namespace unified
}  // namespace localization
}  // namespace apollo
