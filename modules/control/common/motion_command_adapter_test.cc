#include "modules/control/common/motion_command_adapter.h"

#include "gtest/gtest.h"

namespace apollo {
namespace control {

TEST(MotionCommandAdapterTest, ConvertsHoldWithoutInferringFromEmptyPath) {
  planning::MotionExecutionCommand command;
  command.mutable_header()->set_timestamp_sec(1.0);
  command.mutable_start_condition()->mutable_expected_position()->set_x(1.0);
  command.mutable_start_condition()->mutable_expected_position()->set_y(2.0);
  command.mutable_start_condition()->set_expected_heading(0.5);
  command.mutable_start_condition()->set_expected_gear(
      canbus::Chassis::GEAR_DRIVE);
  command.mutable_primitive()->set_type(
      planning::MOTION_PRIMITIVE_STANDSTILL_HOLD);
  command.mutable_primitive()->mutable_standstill_hold();

  MotionCommandAdapter adapter;
  planning::ADCTrajectory trajectory;
  std::string reason;
  ASSERT_TRUE(
      adapter.ToLegacyControllerInput(command, &trajectory, &reason));
  EXPECT_EQ(trajectory.trajectory_point_size(), 2);
  EXPECT_EQ(trajectory.control_intent().execution_channel(),
            planning::EXECUTION_CHANNEL_PRIMITIVE);
  EXPECT_EQ(trajectory.control_intent().primitive_type(),
            planning::CONTROL_PRIMITIVE_STANDSTILL_HOLD);
}

TEST(MotionCommandAdapterTest, PreservesExplicitSemanticContext) {
  planning::MotionExecutionCommand command;
  command.mutable_trajectory()->set_gear(canbus::Chassis::GEAR_DRIVE);
  auto* intent = command.mutable_control_intent();
  intent->set_longitudinal_intent(planning::LON_INTENT_PRECISE_STOP);
  intent->mutable_target_stop_point()->set_x(5.0);
  intent->set_terminal_position_tolerance_m(0.02);
  command.mutable_execution()->set_active_scene(planning::SCENE_PARK_IN);
  MotionCommandAdapter adapter;
  planning::ADCTrajectory trajectory;
  std::string reason;
  ASSERT_TRUE(adapter.ToLegacyControllerInput(command, &trajectory, &reason));
  EXPECT_EQ(trajectory.execution().active_scene(), planning::SCENE_PARK_IN);
  EXPECT_EQ(trajectory.control_intent().longitudinal_intent(),
            planning::LON_INTENT_PRECISE_STOP);
  EXPECT_DOUBLE_EQ(trajectory.control_intent().target_stop_point().x(), 5.0);
  EXPECT_DOUBLE_EQ(
      trajectory.control_intent().terminal_position_tolerance_m(), 0.02);
}

}  // namespace control
}  // namespace apollo
