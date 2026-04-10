// Copyright 2025 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-01-03
//  Author: daohu527

#pragma once

#include "modules/planning/common/obstacle.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

// 1. Semantic Enhancement Layer Configuration
struct SemanticSafetyConfig {
  double lat_buffer = 0.2;  // Lateral bufferral buffer
  double lon_buffer = 0.5;  // Vertical bufferical buffer
};

class ObstacleDecider {
 public:
  static SemanticSafetyConfig GetSafetyMargin(const Obstacle& obs);

  static InteractionType ComputeInteractionType(const SLBoundary& obs_sl,
                                                const Obstacle& obs,
                                                double ego_width,
                                                const ReferenceLine& ref_line,
                                                const SLBoundary& adc_sl,
                                                bool is_change_lane_path);

 private:
  // Determine whether (position, reversal, divergent intent) can be directly
  // ignored.
  static bool IsIgnorable(const SLBoundary& obs_sl, const Obstacle& obs,
                          const ReferenceLine& ref_line,
                          const SLBoundary& adc_sl, bool is_change_lane_path);

  // Determine whether a static obstacle is blocking the road
  static bool IsStaticBlocking(const SLBoundary& obs_sl, const Obstacle& obs,
                               double ego_width, const ReferenceLine& ref_line);
};

}  // namespace planning
}  // namespace apollo
