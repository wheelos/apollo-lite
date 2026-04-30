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

#include "modules/planning/planner/planner_selector.h"

#include "google/protobuf/repeated_field.h"

#include "cyber/common/log.h"
#include "modules/planning/planner/lattice/lattice_planner.h"
#include "modules/planning/planner/navi/navi_planner.h"
#include "modules/planning/planner/public_road/public_road_planner.h"
#include "modules/planning/planner/rtk/rtk_replay_planner.h"

namespace apollo {
namespace planning {

namespace {

std::unique_ptr<Planner> CreatePlanner(
    PlannerType planner_type,
    const std::shared_ptr<DependencyInjector>& injector) {
  switch (planner_type) {
    case PlannerType::PUBLIC_ROAD:
      return std::make_unique<PublicRoadPlanner>(injector);
    case PlannerType::NAVI:
      return std::make_unique<NaviPlanner>(injector);
    case PlannerType::LATTICE:
      return std::make_unique<LatticePlanner>(injector);
    case PlannerType::RTK:
      return std::make_unique<RTKReplayPlanner>(injector);
    default:
      return nullptr;
  }
}

void WarnIfMultipleCandidatesConfigured(
    const google::protobuf::RepeatedField<int>& planner_types,
    const char* shell_name) {
  if (planner_types.size() > 1) {
    AWARN << shell_name
          << " shell received multiple planner_type entries; only the first "
             "entry is honored for backward compatibility";
  }
}

void WarnIfLegacyPlannerSelected(PlannerType planner_type,
                                 const char* shell_name) {
  switch (planner_type) {
    case PlannerType::RTK:
      AWARN << shell_name
            << " selected RTKReplayPlanner, which is legacy tooling/replay "
               "logic and should not be used as a production online planner";
      break;
    case PlannerType::LATTICE:
      AWARN << shell_name
            << " selected LatticePlanner, which is a legacy reference-line "
               "planner for structured lane scenarios, not a production shell";
      break;
    case PlannerType::PUBLIC_ROAD:
    case PlannerType::NAVI:
    default:
      break;
  }
}

common::Status InvalidPlannerStatus(PlannerType planner_type,
                                    const char* shell_name) {
  return common::Status(
      common::ErrorCode::PLANNING_ERROR,
      std::string("planner type ") + PlannerSelector::PlannerTypeName(planner_type) +
          " is incompatible with " + shell_name + " shell");
}

}  // namespace

common::Status PlannerSelector::CreateStandardPlanner(
    const PlanningConfig& planning_config,
    const std::shared_ptr<DependencyInjector>& injector,
    std::unique_ptr<Planner>* planner) {
  if (planner == nullptr) {
    return common::Status(common::ErrorCode::PLANNING_ERROR,
                          "planner output is null");
  }
  const auto planner_type = ResolveStandardPlannerType(planning_config);
  switch (planner_type) {
    case PlannerType::PUBLIC_ROAD:
    case PlannerType::LATTICE:
    case PlannerType::RTK:
      WarnIfLegacyPlannerSelected(planner_type, "standard");
      *planner = CreatePlanner(planner_type, injector);
      return *planner != nullptr
                 ? common::Status::OK()
                 : common::Status(common::ErrorCode::PLANNING_ERROR,
                                  "failed to create standard planner");
    case PlannerType::NAVI:
      return InvalidPlannerStatus(planner_type, "standard");
    default:
      return common::Status(common::ErrorCode::PLANNING_ERROR,
                            "unknown standard planner type");
  }
}

common::Status PlannerSelector::CreateNavigationPlanner(
    const PlanningConfig& planning_config,
    const std::shared_ptr<DependencyInjector>& injector,
    std::unique_ptr<Planner>* planner) {
  if (planner == nullptr) {
    return common::Status(common::ErrorCode::PLANNING_ERROR,
                          "planner output is null");
  }
  const auto planner_type = ResolveNavigationPlannerType(planning_config);
  switch (planner_type) {
    case PlannerType::NAVI:
      *planner = CreatePlanner(planner_type, injector);
      return *planner != nullptr
                 ? common::Status::OK()
                 : common::Status(common::ErrorCode::PLANNING_ERROR,
                                  "failed to create navigation planner");
    case PlannerType::RTK:
      WarnIfLegacyPlannerSelected(planner_type, "navigation");
      *planner = CreatePlanner(planner_type, injector);
      return *planner != nullptr
                 ? common::Status::OK()
                 : common::Status(common::ErrorCode::PLANNING_ERROR,
                                  "failed to create navigation planner");
    case PlannerType::PUBLIC_ROAD:
    case PlannerType::LATTICE:
      return InvalidPlannerStatus(planner_type, "navigation");
    default:
      return common::Status(common::ErrorCode::PLANNING_ERROR,
                            "unknown navigation planner type");
  }
}

PlannerType PlannerSelector::ResolveStandardPlannerType(
    const PlanningConfig& planning_config) {
  if (!planning_config.has_standard_planning_config()) {
    return PlannerType::PUBLIC_ROAD;
  }
  const auto& config = planning_config.standard_planning_config();
  WarnIfMultipleCandidatesConfigured(config.planner_type(), "standard");
  if (config.planner_type_size() == 0) {
    return PlannerType::PUBLIC_ROAD;
  }
  return config.planner_type(0);
}

PlannerType PlannerSelector::ResolveNavigationPlannerType(
    const PlanningConfig& planning_config) {
  if (!planning_config.has_navigation_planning_config()) {
    return PlannerType::NAVI;
  }
  const auto& config = planning_config.navigation_planning_config();
  WarnIfMultipleCandidatesConfigured(config.planner_type(), "navigation");
  if (config.planner_type_size() == 0) {
    return PlannerType::NAVI;
  }
  return config.planner_type(0);
}

bool PlannerSelector::IsLegacyPlannerType(PlannerType planner_type) {
  return planner_type == PlannerType::RTK ||
         planner_type == PlannerType::LATTICE;
}

std::string PlannerSelector::PlannerTypeName(PlannerType planner_type) {
  return PlannerType_Name(planner_type);
}

}  // namespace planning
}  // namespace apollo
