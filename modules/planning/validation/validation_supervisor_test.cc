#include "modules/planning/validation/validation_supervisor.h"

#include <memory>

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

namespace {

ADCTrajectory BuildTrajectory() {
  ADCTrajectory trajectory;
  auto* point = trajectory.add_trajectory_point();
  point->set_relative_time(0.0);
  point->set_v(0.0);
  point->set_a(0.0);
  point->mutable_path_point()->set_x(1.0);
  point->mutable_path_point()->set_y(2.0);
  point->mutable_path_point()->set_theta(0.3);
  point->mutable_path_point()->set_s(0.0);
  return trajectory;
}

}  // namespace

TEST(ValidationSupervisorTest, RejectsUnknownModeWhenHoldDisallowed) {
  ValidationSupervisor supervisor;
  LocalView local_view;
  local_view.planning_command = std::make_shared<PlanningCommand>();
  local_view.planning_command->mutable_fallback()->set_allow_hold(false);

  PlanningCoordinatorState planning_state;
  planning_state.requested_mode = MODE_FREE_SPACE;
  planning_state.resolved_mode = MODE_UNKNOWN;
  planning_state.reason = "free-space shell unavailable";

  auto trajectory = BuildTrajectory();
  const auto result =
      supervisor.Validate({&local_view, &planning_state, &trajectory});

  EXPECT_FALSE(result.command_admissible);
  EXPECT_TRUE(result.should_hold);
  EXPECT_NE(result.reason.find("hold disallowed"), std::string::npos);
}

TEST(ValidationSupervisorTest, RejectsUnlistedDegradedMode) {
  ValidationSupervisor supervisor;
  LocalView local_view;
  local_view.planning_command = std::make_shared<PlanningCommand>();
  local_view.planning_command->mutable_fallback()->add_allowed_degraded_modes(
      MODE_SAFETY_HOLD);

  PlanningCoordinatorState planning_state;
  planning_state.requested_mode = MODE_FREE_SPACE;
  planning_state.resolved_mode = MODE_LANE_GRAPH;

  auto trajectory = BuildTrajectory();
  const auto result =
      supervisor.Validate({&local_view, &planning_state, &trajectory});

  EXPECT_FALSE(result.command_admissible);
  EXPECT_TRUE(result.should_hold);
  EXPECT_NE(result.reason.find("allowed degraded modes"), std::string::npos);
}

TEST(ValidationSupervisorTest, AcceptsSafetyHoldFallbackWhenAllowed) {
  ValidationSupervisor supervisor;
  LocalView local_view;
  local_view.planning_command = std::make_shared<PlanningCommand>();
  local_view.planning_command->mutable_fallback()->add_allowed_degraded_modes(
      MODE_SAFETY_HOLD);
  local_view.capability_set = std::make_shared<CapabilitySet>();
  local_view.capability_set->has_stop_target = true;
  local_view.capability_set->can_run_safety_hold_shell = true;

  PlanningCoordinatorState planning_state;
  planning_state.requested_mode = MODE_FREE_SPACE;
  planning_state.resolved_mode = MODE_SAFETY_HOLD;

  auto trajectory = BuildTrajectory();
  const auto result =
      supervisor.Validate({&local_view, &planning_state, &trajectory});

  EXPECT_TRUE(result.command_admissible);
  EXPECT_FALSE(result.should_hold);
  EXPECT_TRUE(result.reason.empty());
}

}  // namespace planning
}  // namespace apollo
