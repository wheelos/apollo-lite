#include "modules/planning/mode/shell_transition_policy.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

namespace {

ModeShellAvailability FullAvailability() {
  ModeShellAvailability availability;
  availability.lane_graph_available = true;
  availability.corridor_available = true;
  availability.open_space_available = true;
  availability.free_space_available = true;
  availability.safety_hold_available = true;
  return availability;
}

CapabilitySet FullCapability() {
  CapabilitySet capability;
  capability.has_lane_graph = true;
  capability.has_local_corridor = true;
  capability.has_parking_roi = true;
  capability.has_drivable_area = true;
  capability.has_goal_pose = true;
  capability.has_stop_target = true;
  capability.can_run_on_lane_shell = true;
  capability.can_run_corridor_shell = true;
  capability.can_run_safety_hold_shell = true;
  capability.can_run_structured_mapless_shell = true;
  capability.can_run_open_space_shell = true;
  return capability;
}

}  // namespace

TEST(ShellTransitionPolicyTest, HoldsCrossShellTransitionUntilStable) {
  PlanningCoordinatorState previous_state;
  previous_state.resolved_mode = MODE_LANE_GRAPH;
  previous_state.active_shell = PLANNING_SHELL_ON_LANE;

  ModeResolutionResult resolution;
  resolution.requested_mode = MODE_FREE_SPACE;
  resolution.resolved_mode = MODE_FREE_SPACE;

  auto capability = FullCapability();
  auto availability = FullAvailability();
  ShellTransitionPolicyState policy_state;

  const auto decision = ShellTransitionPolicy::Apply(
      resolution, previous_state, previous_state.command_id,
      previous_state.active_scene, &capability, availability, &policy_state);

  EXPECT_EQ(decision.active_mode, MODE_LANE_GRAPH);
  EXPECT_EQ(decision.active_shell, PLANNING_SHELL_ON_LANE);
  EXPECT_EQ(decision.desired_mode, MODE_FREE_SPACE);
  EXPECT_EQ(decision.desired_shell, PLANNING_SHELL_STRUCTURED_MAPLESS);
  EXPECT_TRUE(decision.transition_pending);
  EXPECT_TRUE(decision.continuity_hold);
  EXPECT_EQ(policy_state.stable_cycle_count, 1U);
}

TEST(ShellTransitionPolicyTest, ApprovesStableCrossShellTransition) {
  PlanningCoordinatorState previous_state;
  previous_state.resolved_mode = MODE_LANE_GRAPH;
  previous_state.active_shell = PLANNING_SHELL_ON_LANE;

  ModeResolutionResult resolution;
  resolution.requested_mode = MODE_FREE_SPACE;
  resolution.resolved_mode = MODE_FREE_SPACE;

  auto capability = FullCapability();
  auto availability = FullAvailability();
  ShellTransitionPolicyState policy_state;

  EXPECT_TRUE(ShellTransitionPolicy::Apply(
                  resolution, previous_state, previous_state.command_id,
                  previous_state.active_scene, &capability, availability,
                  &policy_state)
                  .transition_pending);

  const auto decision = ShellTransitionPolicy::Apply(
      resolution, previous_state, previous_state.command_id,
      previous_state.active_scene, &capability, availability, &policy_state);

  EXPECT_EQ(decision.active_mode, MODE_FREE_SPACE);
  EXPECT_EQ(decision.active_shell, PLANNING_SHELL_STRUCTURED_MAPLESS);
  EXPECT_FALSE(decision.transition_pending);
  EXPECT_FALSE(decision.continuity_hold);
  EXPECT_EQ(policy_state.stable_cycle_count, 0U);
}

TEST(ShellTransitionPolicyTest, SwitchesImmediatelyWhenCurrentShellBreaks) {
  PlanningCoordinatorState previous_state;
  previous_state.resolved_mode = MODE_LANE_GRAPH;
  previous_state.active_shell = PLANNING_SHELL_ON_LANE;

  ModeResolutionResult resolution;
  resolution.requested_mode = MODE_CORRIDOR;
  resolution.resolved_mode = MODE_CORRIDOR;

  auto capability = FullCapability();
  capability.has_lane_graph = false;
  capability.can_run_on_lane_shell = false;
  auto availability = FullAvailability();
  ShellTransitionPolicyState policy_state;

  const auto decision = ShellTransitionPolicy::Apply(
      resolution, previous_state, previous_state.command_id,
      previous_state.active_scene, &capability, availability, &policy_state);

  EXPECT_EQ(decision.active_mode, MODE_CORRIDOR);
  EXPECT_EQ(decision.active_shell, PLANNING_SHELL_CORRIDOR);
  EXPECT_FALSE(decision.transition_pending);
}

TEST(ShellTransitionPolicyTest, UsesOpenSpaceShellForOpenSpaceMode) {
  PlanningCoordinatorState previous_state;
  previous_state.resolved_mode = MODE_LANE_GRAPH;
  previous_state.active_shell = PLANNING_SHELL_ON_LANE;

  ModeResolutionResult resolution;
  resolution.requested_mode = MODE_OPEN_SPACE;
  resolution.resolved_mode = MODE_OPEN_SPACE;

  auto capability = FullCapability();
  auto availability = FullAvailability();
  ShellTransitionPolicyState policy_state;

  const auto pending = ShellTransitionPolicy::Apply(
      resolution, previous_state, previous_state.command_id,
      previous_state.active_scene, &capability, availability, &policy_state);
  EXPECT_EQ(pending.desired_shell, PLANNING_SHELL_OPEN_SPACE);
  EXPECT_TRUE(pending.transition_pending);

  const auto decision = ShellTransitionPolicy::Apply(
      resolution, previous_state, previous_state.command_id,
      previous_state.active_scene, &capability, availability, &policy_state);
  EXPECT_EQ(decision.active_shell, PLANNING_SHELL_OPEN_SPACE);
  EXPECT_EQ(decision.active_mode, MODE_OPEN_SPACE);
}

TEST(ShellTransitionPolicyTest, SwitchesImmediatelyForNewCommandContext) {
  PlanningCoordinatorState previous_state;
  previous_state.command_id = "old-command";
  previous_state.active_scene = SCENE_LANE_CRUISE;
  previous_state.resolved_mode = MODE_LANE_GRAPH;
  previous_state.active_shell = PLANNING_SHELL_ON_LANE;

  ModeResolutionResult resolution;
  resolution.requested_mode = MODE_OPEN_SPACE;
  resolution.resolved_mode = MODE_OPEN_SPACE;

  auto capability = FullCapability();
  auto availability = FullAvailability();
  ShellTransitionPolicyState policy_state;

  const auto decision = ShellTransitionPolicy::Apply(
      resolution, previous_state, "new-command", SCENE_PARK_IN, &capability,
      availability, &policy_state);

  EXPECT_EQ(decision.active_mode, MODE_OPEN_SPACE);
  EXPECT_EQ(decision.active_shell, PLANNING_SHELL_OPEN_SPACE);
  EXPECT_FALSE(decision.transition_pending);
  EXPECT_FALSE(decision.continuity_hold);
  EXPECT_EQ(policy_state.stable_cycle_count, 0U);
}

}  // namespace planning
}  // namespace apollo
