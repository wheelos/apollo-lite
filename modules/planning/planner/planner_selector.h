/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "modules/common/status/status.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/planner/planner.h"

namespace apollo {
namespace planning {

class PlannerSelector {
 public:
  static common::Status CreateStandardPlanner(
      const PlanningConfig& planning_config,
      const std::shared_ptr<DependencyInjector>& injector,
      std::unique_ptr<Planner>* planner);

  static common::Status CreateNavigationPlanner(
      const PlanningConfig& planning_config,
      const std::shared_ptr<DependencyInjector>& injector,
      std::unique_ptr<Planner>* planner);

  static PlannerType ResolveStandardPlannerType(
      const PlanningConfig& planning_config);
  static PlannerType ResolveNavigationPlannerType(
      const PlanningConfig& planning_config);

  static bool IsLegacyPlannerType(PlannerType planner_type);
  static std::string PlannerTypeName(PlannerType planner_type);
};

}  // namespace planning
}  // namespace apollo
