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

#include "modules/planning/planning_runtime_status_builder.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(PlanningRuntimeStatusBuilderTest, BuildsExplicitRuntimeSnapshot) {
  PlanningRuntimeStatusBuilder builder;
  PlanningSemanticSummary semantics;
  semantics.runtime_state = RUNTIME_HOLDING;
  semantics.command_completed = true;
  semantics.near_terminal = true;
  semantics.completion_reason = "goal reached";

  HybridManeuverSummary hybrid;
  ValidationResult validation;
  validation.trajectory_valid = false;
  validation.command_admissible = true;
  validation.fallback_active = true;
  validation.reason = "validation fallback";

  PlanningCoordinatorState coordinator;
  coordinator.mission_id = "mission-1";
  coordinator.command_id = "command-1";
  coordinator.mission_identity.set_revision(3);
  coordinator.resolved_mode = MODE_LANE_GRAPH;
  coordinator.desired_mode = MODE_OPEN_SPACE;
  coordinator.active_shell = PLANNING_SHELL_ON_LANE;
  coordinator.desired_shell = PLANNING_SHELL_OPEN_SPACE;
  coordinator.transition_pending = true;

  PlanningExecutionContext execution;
  execution.set_mission_id("mission-2");
  execution.set_command_id("command-2");
  execution.set_active_mode(MODE_CORRIDOR);
  execution.add_blockers("execution blocker");

  MissionCommandIdentity accepted_directive_identity;
  accepted_directive_identity.set_revision(4);
  CapabilitySet capability_set;
  capability_set.has_lane_graph = true;
  capability_set.can_run_on_lane_shell = true;

  const auto status = builder.Build(
      "planning_test", semantics, hybrid, validation, coordinator, execution,
      accepted_directive_identity, &capability_set, "cycle hold");

  EXPECT_EQ(status.state(), RUNTIME_HOLDING);
  EXPECT_EQ(status.mission_id(), "mission-2");
  EXPECT_EQ(status.command_id(), "command-2");
  EXPECT_EQ(status.active_mode(), MODE_CORRIDOR);
  ASSERT_TRUE(status.has_transition());
  EXPECT_EQ(status.transition().trigger(), "cycle hold");
  ASSERT_EQ(status.blockers_size(), 2);
  EXPECT_EQ(status.blockers(0), "execution blocker");
  EXPECT_EQ(status.blockers(1), "cycle hold");
  ASSERT_TRUE(status.has_mission_identity());
  EXPECT_EQ(status.mission_identity().revision(), 3);
  ASSERT_TRUE(status.has_accepted_directive_identity());
  EXPECT_EQ(status.accepted_directive_identity().revision(), 4);
  ASSERT_TRUE(status.has_capability());
  EXPECT_TRUE(status.capability().has_lane_graph());
  EXPECT_TRUE(status.capability().can_run_on_lane_shell());
  EXPECT_TRUE(status.has_validation());
  EXPECT_FALSE(status.validation().trajectory_valid());
  EXPECT_TRUE(status.validation().command_admissible());
  EXPECT_TRUE(status.validation().fallback_active());
  EXPECT_EQ(status.validation().validation_reason(), "validation fallback");
  EXPECT_TRUE(status.completion().command_completed());
  EXPECT_EQ(status.completion().completion_reason(), "goal reached");
}

}  // namespace planning
}  // namespace apollo
