// Copyright 2025 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/mission/common/mission_command_supervisor.h"

#include "gtest/gtest.h"

namespace apollo {
namespace mission {
namespace {

planning::PlanningCommand BuildCommand(const std::string& mission_id,
                                       const std::string& command_id,
                                       uint32_t priority,
                                       bool preemptible) {
  planning::PlanningCommand command;
  command.set_mission_id(mission_id);
  command.set_command_id(command_id);
  command.set_action(planning::COMMAND_ACTIVATE);
  command.set_priority(priority);
  command.set_preemptible(preemptible);
  command.set_requested_scene(planning::SCENE_LANE_CRUISE);
  command.set_preferred_mode(planning::MODE_LANE_GRAPH);
  return command;
}

planning::PlanningRuntimeStatus BuildPlanningStatus(
    const std::string& mission_id, const std::string& command_id,
    planning::RuntimeState state, const std::string& reason) {
  planning::PlanningRuntimeStatus status;
  status.set_mission_id(mission_id);
  status.set_command_id(command_id);
  status.set_state(state);
  status.set_reason(reason);
  return status;
}

}  // namespace

TEST(MissionCommandSupervisorTest, FailureKeepsQueueBlockedForRecovery) {
  MissionCommandSupervisor supervisor;
  supervisor.SetCurrentMissionId("mission-A");

  std::vector<planning::PlanningCommand> commands_to_publish;
  supervisor.EvaluatePlanningCommand(
      BuildCommand("mission-A", "cmd-A", 1, false), &commands_to_publish);
  ASSERT_EQ(commands_to_publish.size(), 1u);

  commands_to_publish.clear();
  supervisor.EvaluatePlanningCommand(
      BuildCommand("mission-A", "cmd-B", 1, false), &commands_to_publish);
  EXPECT_TRUE(commands_to_publish.empty());

  commands_to_publish.clear();
  supervisor.UpdatePlanningRuntimeStatus(
      BuildPlanningStatus("mission-A", "cmd-A", planning::RUNTIME_FAILED,
                          "planning failed"),
      &commands_to_publish);
  EXPECT_TRUE(commands_to_publish.empty());

  const auto snapshot = supervisor.GetSnapshot();
  EXPECT_TRUE(snapshot.active_command_id.empty());
  ASSERT_EQ(snapshot.queued_command_ids.size(), 1u);
  EXPECT_EQ(snapshot.queued_command_ids.front(), "cmd-B");
  EXPECT_TRUE(snapshot.operator_recovery_required);
  EXPECT_EQ(snapshot.last_terminal_command_status.command_id, "cmd-A");
  EXPECT_EQ(snapshot.last_terminal_command_status.state,
            CommandLifecycleState::kFailed);
  EXPECT_EQ(snapshot.last_terminal_command_status.reason, "planning failed");
  EXPECT_FALSE(snapshot.recovery_state.allowed_actions.empty());
}

TEST(MissionCommandSupervisorTest, PriorityDoesNotImplicitlyPreempt) {
  MissionCommandSupervisor supervisor;
  supervisor.SetCurrentMissionId("mission-A");

  std::vector<planning::PlanningCommand> commands_to_publish;
  supervisor.EvaluatePlanningCommand(
      BuildCommand("mission-A", "cmd-A", 1, true), &commands_to_publish);
  ASSERT_EQ(commands_to_publish.size(), 1u);

  commands_to_publish.clear();
  supervisor.EvaluatePlanningCommand(
      BuildCommand("mission-A", "cmd-B", 2, true), &commands_to_publish);
  EXPECT_TRUE(commands_to_publish.empty());

  commands_to_publish.clear();
  supervisor.UpdatePlanningRuntimeStatus(
      BuildPlanningStatus("mission-A", "cmd-A", planning::RUNTIME_COMPLETED,
                          "completed"),
      &commands_to_publish);
  ASSERT_EQ(commands_to_publish.size(), 1u);
  EXPECT_EQ(commands_to_publish.front().action(), planning::COMMAND_ACTIVATE);
  EXPECT_EQ(commands_to_publish.front().command_id(), "cmd-B");

  const auto snapshot = supervisor.GetSnapshot();
  EXPECT_EQ(snapshot.active_command_id, "cmd-B");
  EXPECT_FALSE(snapshot.operator_recovery_required);
  EXPECT_EQ(snapshot.active_command_status.dispatch_count, 1u);
}

TEST(MissionCommandSupervisorTest, CompletedCommandDispatchesNextQueuedCommand) {
  MissionCommandSupervisor supervisor;
  supervisor.SetCurrentMissionId("mission-A");

  std::vector<planning::PlanningCommand> commands_to_publish;
  supervisor.EvaluatePlanningCommand(
      BuildCommand("mission-A", "cmd-A", 1, false), &commands_to_publish);
  ASSERT_EQ(commands_to_publish.size(), 1u);

  commands_to_publish.clear();
  supervisor.EvaluatePlanningCommand(
      BuildCommand("mission-A", "cmd-B", 1, false), &commands_to_publish);
  EXPECT_TRUE(commands_to_publish.empty());

  commands_to_publish.clear();
  supervisor.UpdatePlanningRuntimeStatus(
      BuildPlanningStatus("mission-A", "cmd-A", planning::RUNTIME_COMPLETED,
                          "done"),
      &commands_to_publish);
  ASSERT_EQ(commands_to_publish.size(), 1u);
  EXPECT_EQ(commands_to_publish.front().command_id(), "cmd-B");

  const auto snapshot = supervisor.GetSnapshot();
  EXPECT_EQ(snapshot.active_command_id, "cmd-B");
  EXPECT_EQ(snapshot.last_terminal_command_status.command_id, "cmd-A");
  EXPECT_EQ(snapshot.last_terminal_command_status.state,
            CommandLifecycleState::kCompleted);
}

TEST(MissionCommandSupervisorTest, RecoveryAckAndRetryRedispatchesFailedCommand) {
  MissionCommandSupervisor supervisor;
  supervisor.SetCurrentMissionId("mission-A");

  auto command = BuildCommand("mission-A", "cmd-A", 1, false);
  command.mutable_recovery()->set_allow_retry(true);
  command.mutable_recovery()->set_retry_budget(1);
  command.mutable_recovery()->set_recovery_timeout_sec(5.0);

  std::vector<planning::PlanningCommand> commands_to_publish;
  supervisor.EvaluatePlanningCommand(command, &commands_to_publish);
  ASSERT_EQ(commands_to_publish.size(), 1u);

  commands_to_publish.clear();
  supervisor.UpdatePlanningRuntimeStatus(
      BuildPlanningStatus("mission-A", "cmd-A", planning::RUNTIME_FAILED,
                          "planning failed"),
      &commands_to_publish);
  EXPECT_TRUE(commands_to_publish.empty());

  auto snapshot = supervisor.GetSnapshot();
  EXPECT_TRUE(snapshot.operator_recovery_required);
  EXPECT_EQ(snapshot.recovery_state.retry_budget_remaining, 1u);
  EXPECT_FALSE(snapshot.recovery_state.operator_acknowledged);

  EXPECT_TRUE(supervisor.AcknowledgeRecovery());
  commands_to_publish.clear();
  EXPECT_TRUE(supervisor.RetryRecovery(&commands_to_publish));
  ASSERT_EQ(commands_to_publish.size(), 1u);
  EXPECT_EQ(commands_to_publish.front().command_id(), "cmd-A");

  snapshot = supervisor.GetSnapshot();
  EXPECT_FALSE(snapshot.operator_recovery_required);
  EXPECT_EQ(snapshot.active_command_id, "cmd-A");
  EXPECT_EQ(snapshot.active_command_status.dispatch_count, 2u);
}

TEST(MissionCommandSupervisorTest, RecoveryAckAndResumeDispatchesQueuedCommand) {
  MissionCommandSupervisor supervisor;
  supervisor.SetCurrentMissionId("mission-A");

  auto first = BuildCommand("mission-A", "cmd-A", 1, false);
  first.mutable_recovery()->set_allow_resume(true);
  auto second = BuildCommand("mission-A", "cmd-B", 1, false);

  std::vector<planning::PlanningCommand> commands_to_publish;
  supervisor.EvaluatePlanningCommand(first, &commands_to_publish);
  ASSERT_EQ(commands_to_publish.size(), 1u);
  commands_to_publish.clear();
  supervisor.EvaluatePlanningCommand(second, &commands_to_publish);
  EXPECT_TRUE(commands_to_publish.empty());

  supervisor.UpdatePlanningRuntimeStatus(
      BuildPlanningStatus("mission-A", "cmd-A", planning::RUNTIME_FAILED,
                          "planning failed"),
      &commands_to_publish);
  EXPECT_TRUE(commands_to_publish.empty());

  EXPECT_TRUE(supervisor.AcknowledgeRecovery());
  commands_to_publish.clear();
  EXPECT_TRUE(supervisor.ResumeRecovery(&commands_to_publish));
  ASSERT_EQ(commands_to_publish.size(), 1u);
  EXPECT_EQ(commands_to_publish.front().command_id(), "cmd-B");

  const auto snapshot = supervisor.GetSnapshot();
  EXPECT_EQ(snapshot.active_command_id, "cmd-B");
  EXPECT_FALSE(snapshot.operator_recovery_required);
}

TEST(MissionCommandSupervisorTest, ControlStatusIsDiagnosticOnly) {
  MissionCommandSupervisor supervisor;
  supervisor.SetCurrentMissionId("mission-A");

  std::vector<planning::PlanningCommand> commands_to_publish;
  supervisor.EvaluatePlanningCommand(
      BuildCommand("mission-A", "cmd-A", 1, false), &commands_to_publish);
  ASSERT_EQ(commands_to_publish.size(), 1u);
  commands_to_publish.clear();

  control::ControlRuntimeStatus control_status;
  control_status.set_mission_id("mission-A");
  control_status.set_command_id("cmd-A");
  control_status.set_state(control::CONTROL_RUNTIME_FAULTED);
  control_status.set_reason("diagnostic control fault");
  supervisor.UpdateControlRuntimeStatus(control_status, &commands_to_publish);

  EXPECT_TRUE(commands_to_publish.empty());
  const auto snapshot = supervisor.GetSnapshot();
  EXPECT_EQ(snapshot.active_command_id, "cmd-A");
  EXPECT_EQ(snapshot.active_command_status.state,
            CommandLifecycleState::kDispatched);
  EXPECT_EQ(snapshot.active_command_status.control_state,
            control::CONTROL_RUNTIME_FAULTED);
}

}  // namespace mission
}  // namespace apollo
