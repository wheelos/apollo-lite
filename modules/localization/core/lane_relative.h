// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <vector>

#include "modules/localization/core/types.h"

namespace apollo {
namespace localization {
namespace unified {

struct LanePolicy {
  double minimum_width = 2.0;
  double maximum_width = 5.0;
  double front = 0.0;
  double rear = 0.0;
  double half_width = 0.0;
  double clearance = 0.0;
  double boundary_std = 0.0;
  double heading_std = 0.0;
  double sigma_multiplier = 3.0;
  double reference_x = 0.0;
};

struct LaneGeometry {
  std::vector<Eigen::Vector3d> left;
  std::vector<Eigen::Vector3d> right;
};

struct LaneRelation {
  double lateral = 0.0;
  double heading = 0.0;
  double width = 0.0;
  double left_clearance = 0.0;
  double right_clearance = 0.0;
  double lateral_std = 0.0;
  double heading_std = 0.0;
  bool contained = false;
  Reason containment_reason = Reason::HISTORY_UNAVAILABLE;
};

// Computes measured body-to-lane relation; never changes ODOM or creates MAP.
Result EvaluateLane(const LaneGeometry& geometry, const LanePolicy& policy,
                    double motion_position_std, double motion_heading_std,
                    LaneRelation* relation);

}  // namespace unified
}  // namespace localization
}  // namespace apollo
