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

#include "modules/planning/common/hybrid_maneuver_supervisor.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(HybridManeuverSupervisorTest, MarksParkInEntryAsPendingApproval) {
  HybridManeuverSupervisor supervisor;
  PlanningCoordinatorState state;
  state.active_scene = SCENE_PARK_IN;
  state.active_shell = PLANNING_SHELL_ON_LANE;
  state.desired_shell = PLANNING_SHELL_OPEN_SPACE;
  state.transition_pending = true;
  state.reason = "holding on_lane shell while transition to open_space stabilizes";

  const auto summary = supervisor.Evaluate(state, nullptr, RUNTIME_RUNNING);

  EXPECT_EQ(summary.active_maneuver, HYBRID_MANEUVER_PARK_IN);
  EXPECT_EQ(summary.active_segment, MANEUVER_SEGMENT_PREPARE_ENTRY);
  EXPECT_EQ(summary.handoff_state, HANDOFF_STATE_PENDING_APPROVAL);
  EXPECT_FALSE(summary.handoff_reason.empty());
}

TEST(HybridManeuverSupervisorTest, MarksPullOutStageCruiseAsHandoffReady) {
  HybridManeuverSupervisor supervisor;
  PlanningCoordinatorState state;
  state.active_scene = SCENE_PULL_OUT;
  state.active_shell = PLANNING_SHELL_OPEN_SPACE;
  state.desired_shell = PLANNING_SHELL_OPEN_SPACE;

  PlanningStatus planning_status;
  planning_status.mutable_scenario()->set_stage_type(PARK_AND_GO_CRUISE);

  const auto summary = supervisor.Evaluate(state, &planning_status, RUNTIME_RUNNING);

  EXPECT_EQ(summary.active_maneuver, HYBRID_MANEUVER_PULL_OUT);
  EXPECT_EQ(summary.active_segment, MANEUVER_SEGMENT_HANDOFF_EXIT);
  EXPECT_EQ(summary.handoff_state, HANDOFF_STATE_READY);
}

TEST(HybridManeuverSupervisorTest, MarksPullOutResumeAfterOpenSpaceCommit) {
  HybridManeuverSupervisor supervisor;
  PlanningCoordinatorState state;
  state.active_scene = SCENE_PULL_OUT;
  state.previous_shell = PLANNING_SHELL_OPEN_SPACE;
  state.active_shell = PLANNING_SHELL_ON_LANE;
  state.desired_shell = PLANNING_SHELL_ON_LANE;

  const auto summary = supervisor.Evaluate(state, nullptr, RUNTIME_RUNNING);

  EXPECT_EQ(summary.active_maneuver, HYBRID_MANEUVER_PULL_OUT);
  EXPECT_EQ(summary.active_segment, MANEUVER_SEGMENT_GUIDED_RESUME);
  EXPECT_EQ(summary.handoff_state, HANDOFF_STATE_COMMITTED);
}

TEST(HybridManeuverSupervisorTest, IgnoresLaneCruiseAsHybridManeuver) {
  HybridManeuverSupervisor supervisor;
  PlanningCoordinatorState state;
  state.active_scene = SCENE_LANE_CRUISE;
  state.active_shell = PLANNING_SHELL_ON_LANE;

  const auto summary = supervisor.Evaluate(state, nullptr, RUNTIME_RUNNING);

  EXPECT_EQ(summary.active_maneuver, HYBRID_MANEUVER_NONE);
  EXPECT_EQ(summary.active_segment, MANEUVER_SEGMENT_NONE);
  EXPECT_EQ(summary.handoff_state, HANDOFF_STATE_NONE);
}

}  // namespace planning
}  // namespace apollo
