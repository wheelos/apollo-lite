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

#include <vector>

#include "modules/open_space_planning/common/status.h"
#include "modules/open_space_planning/common/types.h"

namespace apollo {
namespace open_space_planning {

struct RoutePlanningRequest {
  const PlanningProblem& problem;
  std::size_t maximum_candidate_count = 1;
  RouteSearchParadigm paradigm = RouteSearchParadigm::kAuto;
};

class RoutePlanner {
 public:
  virtual ~RoutePlanner() = default;

  // Successful results are ordered from lowest to highest route cost.
  virtual Status Plan(const RoutePlanningRequest& request,
                      std::vector<RouteCandidate>* candidates) = 0;
};

}  // namespace open_space_planning
}  // namespace apollo
