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
#include <vector>

#include "modules/planning/mode/mode_resolution.h"
#include "modules/planning/planning_base.h"
#include "modules/planning/planning_runtime_context.h"

namespace apollo {
namespace planning {

struct PlanningShellRegistration {
  PlanningMode mode = MODE_UNKNOWN;
  PlanningShellType shell = PLANNING_SHELL_UNKNOWN;
  PlanningOperatingDomain domain = DOMAIN_UNKNOWN;
  std::string name;
  std::unique_ptr<PlanningBase> planner;
};

// Registry owns long-lived shell executors so shell switching only changes the
// active execution family, not the command-level runtime contract.
class PlanningShellRegistry {
 public:
  void Clear();

  bool Register(PlanningMode mode, PlanningShellType shell,
                PlanningOperatingDomain domain, std::string name,
                std::unique_ptr<PlanningBase> planner);

  bool HasShell(PlanningMode mode) const;

  const PlanningShellRegistration* FindByMode(PlanningMode mode) const;

  PlanningBase* GetPlannerForMode(PlanningMode mode) const;

  ModeShellAvailability BuildAvailability() const;

 private:
  std::vector<PlanningShellRegistration> shells_;
};

}  // namespace planning
}  // namespace apollo
