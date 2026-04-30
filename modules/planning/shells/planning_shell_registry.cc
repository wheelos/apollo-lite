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

#include "modules/planning/shells/planning_shell_registry.h"

#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

namespace {

void MarkAvailability(PlanningMode mode, ModeShellAvailability* availability) {
  CHECK_NOTNULL(availability);
  switch (mode) {
    case MODE_LANE_GRAPH:
      availability->lane_graph_available = true;
      break;
    case MODE_CORRIDOR:
      availability->corridor_available = true;
      break;
    case MODE_OPEN_SPACE:
      availability->open_space_available = true;
      break;
    case MODE_FREE_SPACE:
      availability->free_space_available = true;
      break;
    case MODE_SAFETY_HOLD:
      availability->safety_hold_available = true;
      break;
    case MODE_UNKNOWN:
    default:
      break;
  }
}

}  // namespace

void PlanningShellRegistry::Clear() { shells_.clear(); }

bool PlanningShellRegistry::Register(PlanningMode mode, PlanningShellType shell,
                                     PlanningOperatingDomain domain,
                                     std::string name,
                                     std::unique_ptr<PlanningBase> planner) {
  if (mode == MODE_UNKNOWN || shell == PLANNING_SHELL_UNKNOWN ||
      planner == nullptr || planner->Mode() != mode || HasShell(mode)) {
    return false;
  }

  PlanningShellRegistration registration;
  registration.mode = mode;
  registration.shell = shell;
  registration.domain = domain;
  registration.name = std::move(name);
  registration.planner = std::move(planner);
  shells_.push_back(std::move(registration));
  return true;
}

bool PlanningShellRegistry::HasShell(PlanningMode mode) const {
  return FindByMode(mode) != nullptr;
}

const PlanningShellRegistration* PlanningShellRegistry::FindByMode(
    PlanningMode mode) const {
  for (const auto& shell : shells_) {
    if (shell.mode == mode) {
      return &shell;
    }
  }
  return nullptr;
}

PlanningBase* PlanningShellRegistry::GetPlannerForMode(PlanningMode mode) const {
  const auto* shell = FindByMode(mode);
  return shell == nullptr ? nullptr : shell->planner.get();
}

ModeShellAvailability PlanningShellRegistry::BuildAvailability() const {
  ModeShellAvailability availability;
  for (const auto& shell : shells_) {
    MarkAvailability(shell.mode, &availability);
  }
  return availability;
}

}  // namespace planning
}  // namespace apollo
