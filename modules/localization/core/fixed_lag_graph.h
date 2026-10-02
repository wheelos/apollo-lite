// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <memory>

#include "modules/localization/core/global_types.h"
#include "modules/localization/core/local_motion.h"

namespace apollo {
namespace localization {
namespace unified {

struct FixedLagConfig {
  double lag = 3.0;
  double solve_budget_ms = 50.0;
  double innovation_gate = 22.458;
  double robust_threshold = 2.5;
  uint32_t max_states = 1000;
  uint32_t max_observations = 2000;
  uint32_t max_iterations = 5;
  double max_linearization_angle = 0.2;
};

// All state here, including the nuisance error chain, is global-owned.
// There is intentionally no output carrying local state/bias corrections.
class FixedLagGraph {
 public:
  explicit FixedLagGraph(const FixedLagConfig& config);
  ~FixedLagGraph();
  Result ValidateConfig() const;
  // The caller must first verify an independent, full-pose hypothesis.
  Result Initialize(const LocalState& local, const GlobalObservation& anchor);
  Result AddLocal(const LocalState& local, const MotionIncrement& motion);
  // Only exact retained measurement times; never use a nearest keyframe.
  Result Observe(const GlobalObservation& observation);
  bool initialized() const;
  const GlobalState& state() const;

 private:
  struct Impl;
  FixedLagConfig config_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace unified
}  // namespace localization
}  // namespace apollo
