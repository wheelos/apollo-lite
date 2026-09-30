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

}  // namespace control
}  // namespace apollo
