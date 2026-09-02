// Copyright 2026 WheelOS. All Rights Reserved.
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

#pragma once

#include <memory>
#include <vector>

#include "modules/planning/proto/planner_open_space_config.pb.h"

#include "modules/common/math/vec2d.h"
#include "modules/open_space_planning/common/status.h"
#include "modules/open_space_planning/common/types.h"
#include "modules/open_space_planning/route/route_planner.h"
#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"

namespace apollo {
namespace open_space_planning {

class HybridAStarRoutePlanner : public RoutePlanner {
 public:
  HybridAStarRoutePlanner();
  explicit HybridAStarRoutePlanner(
      const planning::PlannerOpenSpaceConfig& config);
  virtual ~HybridAStarRoutePlanner() = default;

  Status Plan(const RoutePlanningRequest& request,
              std::vector<RouteCandidate>* candidates) override;

  static void ExtractXYBounds(const PlanningProblem& problem,
                              std::vector<double>* xy_bounds);

  static void ExtractObstacles(
      const PlanningProblem& problem,
      std::vector<std::vector<common::math::Vec2d>>* obstacles_vertices_vec);

 private:
  planning::PlannerOpenSpaceConfig config_;
  std::unique_ptr<planning::HybridAStar> hybrid_a_star_;
};

}  // namespace open_space_planning
}  // namespace apollo
