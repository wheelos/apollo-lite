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

#include "gtest/gtest_prod.h"

#include "modules/common/status/status.h"
#include "modules/planning/planning_base.h"
#include "modules/planning/scenarios/scenario.h"
#include "modules/planning/scenarios/stage.h"

namespace apollo {
namespace planning {

class OpenSpacePlanning : public PlanningBase {
 public:
  explicit OpenSpacePlanning(
      const std::shared_ptr<DependencyInjector>& injector)
      : PlanningBase(injector) {}
  ~OpenSpacePlanning() override = default;

  std::string Name() const override;
  PlanningMode Mode() const override { return MODE_OPEN_SPACE; }

  static bool SupportsOpenSpaceCommand(const LocalView& local_view);

  common::Status Init(const PlanningConfig& config) override;

  void RunOnce(const LocalView& local_view,
               ADCTrajectory* const ptr_trajectory_pb) override;

  common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* const trajectory) override;

 private:
  enum class RecipeKind {
    kUnknown = 0,
    kParkIn = 1,
    kPullOver = 2,
    kPullOut = 3,
  };

  enum class RecipeProgress {
    kActive = 0,
    kHandoffReady = 1,
    kCompleted = 2,
  };

  struct Recipe {
    std::string name;
    ScenarioType scenario_type = ScenarioType::LANE_FOLLOW;
    StageType entry_stage_type = StageType::NO_STAGE;
    StageType terminal_handoff_stage = StageType::NO_STAGE;
    ScenarioConfig scenario_config;
    std::unique_ptr<scenario::Scenario> stage_factory;
  };

  // OpenSpaceRuntimeState stores command-scoped execution state that may
  // persist across cycles within one open-space command, but must be reset when
  // the active command/recipe changes.
  struct OpenSpaceRuntimeState {
    RecipeKind active_recipe_kind = RecipeKind::kUnknown;
    RecipeProgress recipe_progress = RecipeProgress::kActive;
    std::string active_command_id;
    StageType active_stage_type = StageType::NO_STAGE;
    std::unique_ptr<scenario::Stage> open_space_stage;
    canbus::Chassis::GearPosition last_gear = canbus::Chassis::GEAR_DRIVE;

    void Reset();
  };

  common::Status LoadRecipes();
  common::Status InitFrame(const uint32_t sequence_num,
                           const common::TrajectoryPoint& planning_start_point,
                           const common::VehicleState& vehicle_state);
  common::Status EnsureStage(RecipeKind recipe_kind);
  common::Status CreateStage(const Recipe& recipe, StageType stage_type);

  RecipeKind ResolveRecipeKind(const LocalView& local_view) const;
  const Recipe* FindRecipe(RecipeKind recipe_kind) const;
  void ResetRecipeState();
  canbus::Chassis::GearPosition ResolveOpenSpacePublishedGear(
      canbus::Chassis::GearPosition segment_gear =
          canbus::Chassis::GEAR_NONE) const;
  void ApplyOpenSpaceTrajectoryMetadata(ADCTrajectory* ptr_trajectory_pb,
                                        bool handoff_continuation) const;
  void ApplyTerminalCompletionDecision(ADCTrajectory* ptr_trajectory_pb) const;
  void FillLastTrajectory(ADCTrajectory* ptr_trajectory_pb) const;

  common::VehicleState AlignTimeStamp(const common::VehicleState& vehicle_state,
                                      double curr_timestamp) const;
  void GenerateStopTrajectory(ADCTrajectory* ptr_trajectory_pb) const;
  void FillOpenSpaceTrajectory(ADCTrajectory* ptr_trajectory_pb) const;

  Recipe park_in_recipe_;
  Recipe pull_over_recipe_;
  Recipe pull_out_recipe_;
  OpenSpaceRuntimeState runtime_state_;

  FRIEND_TEST(OpenSpacePlanningTest, ResetRecipeStateClearsCommandScopedState);
};

}  // namespace planning
}  // namespace apollo
