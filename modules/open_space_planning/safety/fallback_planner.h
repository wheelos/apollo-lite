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

#include "modules/open_space_planning/common/status.h"
#include "modules/open_space_planning/common/types.h"

namespace apollo {
namespace open_space_planning {

struct FallbackPlanningRequest {
  const PlanningProblem& problem;
};

class FallbackPlanner {
 public:
  virtual ~FallbackPlanner() = default;

  virtual Status Plan(const FallbackPlanningRequest& request,
                      PhysicalTrajectory* trajectory) = 0;
};

}  // namespace open_space_planning
}  // namespace apollo
