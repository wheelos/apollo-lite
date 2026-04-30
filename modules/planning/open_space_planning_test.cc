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

#include "modules/planning/open_space_planning.h"

#include "gtest/gtest.h"

#include "modules/planning/common/dependency_injector.h"

namespace apollo {
namespace planning {

TEST(OpenSpacePlanningTest, SupportsParkInCommand) {
  LocalView local_view;
  local_view.planning_command = std::make_shared<PlanningCommand>();
  local_view.planning_command->set_requested_scene(SCENE_PARK_IN);

  EXPECT_TRUE(OpenSpacePlanning::SupportsOpenSpaceCommand(local_view));
}

TEST(OpenSpacePlanningTest, SupportsPullOutCommand) {
  LocalView local_view;
  local_view.planning_command = std::make_shared<PlanningCommand>();
  local_view.planning_command->set_requested_scene(SCENE_PULL_OUT);

  EXPECT_TRUE(OpenSpacePlanning::SupportsOpenSpaceCommand(local_view));
}

TEST(OpenSpacePlanningTest, SupportsParkingGoalFallbackWithoutScene) {
  LocalView local_view;
  local_view.planning_command = std::make_shared<PlanningCommand>();
  local_view.planning_command->mutable_goal()
      ->mutable_parking_goal()
      ->set_parking_space_id("parking-space-42");

  EXPECT_TRUE(OpenSpacePlanning::SupportsOpenSpaceCommand(local_view));
}

TEST(OpenSpacePlanningTest, ResetRecipeStateClearsCommandScopedState) {
  auto injector = std::make_shared<DependencyInjector>();
  OpenSpacePlanning planning(injector);

  planning.runtime_state_.active_recipe_kind =
      OpenSpacePlanning::RecipeKind::kPullOut;
  planning.runtime_state_.recipe_progress =
      OpenSpacePlanning::RecipeProgress::kHandoffReady;
  planning.runtime_state_.active_command_id = "pull-out-command";
  planning.runtime_state_.active_stage_type = PARK_AND_GO_PRE_CRUISE;
  planning.runtime_state_.last_gear = canbus::Chassis::GEAR_REVERSE;

  planning.ResetRecipeState();

  EXPECT_EQ(planning.runtime_state_.active_recipe_kind,
            OpenSpacePlanning::RecipeKind::kUnknown);
  EXPECT_EQ(planning.runtime_state_.recipe_progress,
            OpenSpacePlanning::RecipeProgress::kActive);
  EXPECT_TRUE(planning.runtime_state_.active_command_id.empty());
  EXPECT_EQ(planning.runtime_state_.active_stage_type, StageType::NO_STAGE);
  EXPECT_EQ(planning.runtime_state_.last_gear, canbus::Chassis::GEAR_DRIVE);
}

}  // namespace planning
}  // namespace apollo
