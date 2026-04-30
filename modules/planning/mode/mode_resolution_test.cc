#include "modules/planning/mode/mode_resolution.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(ModeResolutionTest, InfersDockAsFreeSpaceMode) {
  PlanningCommand command;
  command.set_requested_scene(SCENE_DOCK);

  EXPECT_EQ(ModeResolution::InferRequestedMode(&command, MODE_LANE_GRAPH),
            MODE_FREE_SPACE);
}

TEST(ModeResolutionTest, ResolvesFreeSpaceWhenShellAndCapabilityReady) {
  PlanningCommand command;
  command.set_requested_scene(SCENE_DOCK);

  CapabilitySet capability;
  capability.has_drivable_area = true;
  capability.has_goal_pose = true;
  capability.has_structured_mapless_context = true;
  capability.can_run_structured_mapless_shell = true;

  ModeShellAvailability availability;
  availability.free_space_available = true;
  availability.safety_hold_available = true;

  const auto result =
      ModeResolution::Resolve(&command, &capability, availability,
                              MODE_LANE_GRAPH);

  EXPECT_EQ(result.requested_mode, MODE_FREE_SPACE);
  EXPECT_EQ(result.resolved_mode, MODE_FREE_SPACE);
  EXPECT_TRUE(result.reason.empty());
  EXPECT_TRUE(result.blockers.empty());
}

TEST(ModeResolutionTest, FallsBackToSafetyHoldWhenFreeSpaceUnavailable) {
  PlanningCommand command;
  command.set_requested_scene(SCENE_DOCK);

  CapabilitySet capability;
  capability.has_drivable_area = true;
  capability.has_goal_pose = true;
  capability.has_stop_target = true;
  capability.can_run_safety_hold_shell = true;

  ModeShellAvailability availability;
  availability.free_space_available = true;
  availability.safety_hold_available = true;

  const auto result =
      ModeResolution::Resolve(&command, &capability, availability,
                              MODE_LANE_GRAPH);

  EXPECT_EQ(result.requested_mode, MODE_FREE_SPACE);
  EXPECT_EQ(result.resolved_mode, MODE_SAFETY_HOLD);
  EXPECT_NE(result.reason.find("requested free_space mode degraded to "
                               "safety_hold"),
            std::string::npos);
}

TEST(ModeResolutionTest, HonorsAllowedDegradedModes) {
  PlanningCommand command;
  command.set_requested_scene(SCENE_DOCK);
  command.mutable_fallback()->add_allowed_degraded_modes(MODE_SAFETY_HOLD);

  CapabilitySet capability;
  capability.has_drivable_area = true;
  capability.has_goal_pose = true;
  capability.has_stop_target = true;
  capability.can_run_safety_hold_shell = true;

  ModeShellAvailability availability;
  availability.lane_graph_available = true;
  availability.safety_hold_available = true;

  const auto result =
      ModeResolution::Resolve(&command, &capability, availability,
                              MODE_LANE_GRAPH);

  EXPECT_EQ(result.resolved_mode, MODE_SAFETY_HOLD);
}

TEST(ModeResolutionTest, ReturnsUnknownWhenNoExecutableFallbackExists) {
  PlanningCommand command;
  command.set_requested_scene(SCENE_DOCK);

  CapabilitySet capability;
  capability.has_drivable_area = true;
  capability.has_goal_pose = true;

  ModeShellAvailability availability;

  const auto result =
      ModeResolution::Resolve(&command, &capability, availability,
                              MODE_LANE_GRAPH);

  EXPECT_EQ(result.requested_mode, MODE_FREE_SPACE);
  EXPECT_EQ(result.resolved_mode, MODE_UNKNOWN);
  EXPECT_FALSE(result.reason.empty());
  EXPECT_FALSE(result.blockers.empty());
}

TEST(ModeResolutionTest, TreatsMissingCorridorShellContractAsCapabilityGap) {
  PlanningCommand command;
  command.set_requested_scene(SCENE_DOCK);

  CapabilitySet capability;
  capability.has_drivable_area = true;
  capability.has_goal_pose = true;
  capability.has_structured_mapless_context = true;
  capability.has_stop_target = true;
  capability.can_run_safety_hold_shell = true;

  ModeShellAvailability availability;
  availability.free_space_available = true;
  availability.safety_hold_available = true;

  const auto result =
      ModeResolution::Resolve(&command, &capability, availability,
                              MODE_LANE_GRAPH);

  EXPECT_EQ(result.resolved_mode, MODE_SAFETY_HOLD);
  EXPECT_NE(result.reason.find("structured-mapless shell input contract"),
            std::string::npos);
}

TEST(ModeResolutionTest, KeepsParkInOnLegacyShellWhenOnlyLocalParkingIsNeeded) {
  PlanningCommand command;
  command.set_requested_scene(SCENE_PARK_IN);

  EXPECT_EQ(ModeResolution::InferRequestedMode(&command, MODE_LANE_GRAPH),
            MODE_LANE_GRAPH);
}

TEST(ModeResolutionTest, ResolvesOpenSpaceWhenGlobalGuidedParkInIsRequested) {
  PlanningCommand command;
  command.set_requested_scene(SCENE_PARK_IN);
  command.mutable_open_space()->set_navigation_type(
      OPEN_SPACE_NAV_GLOBAL_GUIDED);

  CapabilitySet capability;
  capability.has_goal_pose = true;
  capability.has_known_open_space_environment = true;
  capability.can_run_open_space_shell = true;

  ModeShellAvailability availability;
  availability.open_space_available = true;
  availability.safety_hold_available = true;

  const auto result =
      ModeResolution::Resolve(&command, &capability, availability,
                              MODE_LANE_GRAPH);

  EXPECT_EQ(result.requested_mode, MODE_OPEN_SPACE);
  EXPECT_EQ(result.resolved_mode, MODE_OPEN_SPACE);
  EXPECT_TRUE(result.reason.empty());
}

TEST(ModeResolutionTest, ExplainsUnknownEnvironmentOpenSpaceGap) {
  PlanningCommand command;
  command.set_requested_scene(SCENE_PARK_IN);
  command.mutable_open_space()->set_navigation_type(
      OPEN_SPACE_NAV_EXPLORATION);
  command.mutable_open_space()->set_allow_exploration(true);

  CapabilitySet capability;
  capability.has_goal_pose = true;
  capability.supports_open_space_exploration = true;
  capability.has_stop_target = true;
  capability.can_run_safety_hold_shell = true;

  ModeShellAvailability availability;
  availability.open_space_available = true;
  availability.safety_hold_available = true;

  const auto result =
      ModeResolution::Resolve(&command, &capability, availability,
                              MODE_LANE_GRAPH);

  EXPECT_EQ(result.resolved_mode, MODE_SAFETY_HOLD);
  EXPECT_NE(result.reason.find("exploration requested"), std::string::npos);
}

}  // namespace planning
}  // namespace apollo
