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

#include "modules/common/status/status.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/local_view.h"
#include "modules/planning/mode/mode_resolution.h"
#include "modules/planning/mode/shell_transition_policy.h"
#include "modules/planning/open_space_planning.h"
#include "modules/planning/planning_base.h"
#include "modules/planning/planning_runtime_context.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/shells/planning_shell_registry.h"

namespace apollo {
namespace planning {

class PlanningCoordinator {
 public:
  explicit PlanningCoordinator(
      const std::shared_ptr<DependencyInjector>& injector);

  common::Status Init(const PlanningConfig& config, bool use_navigation_mode);

  PlanningCoordinatorState PreviewState(const LocalView& local_view) const;

  void RunOnce(const LocalView& local_view,
               ADCTrajectory* const adc_trajectory);

  const PlanningCoordinatorState& state() const { return state_; }

 private:
  PlanningCoordinatorState BuildState(const LocalView& local_view) const;
  ModeShellAvailability BuildModeShellAvailability() const;
  void GenerateSafetyHoldTrajectory(const LocalView& local_view,
                                    ADCTrajectory* adc_trajectory) const;
  PlanningMode ResolveLegacyMode() const;
  PlanningBase* GetPlannerForMode(PlanningMode mode) const;

  bool use_navigation_mode_ = false;
  PlanningConfig config_;
  PlanningCoordinatorState state_;
  mutable ShellTransitionPolicyState shell_transition_state_;
  std::shared_ptr<DependencyInjector> injector_;
  PlanningShellRegistry shell_registry_;
};

}  // namespace planning
}  // namespace apollo
