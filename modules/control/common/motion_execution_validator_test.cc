#include "modules/control/common/motion_execution_validator.h"

#include <limits>

#include "gtest/gtest.h"

namespace apollo {
namespace control {
namespace {

MotionExecutionCapabilities AllCapabilities() {
  MotionExecutionCapabilities capabilities;
  capabilities.supported = {
      planning::MOTION_CAPABILITY_TRAJECTORY_TRACKING,
      planning::MOTION_CAPABILITY_STANDSTILL_HOLD,
      planning::MOTION_CAPABILITY_POSE_SERVO,
      planning::MOTION_CAPABILITY_CORRIDOR_SERVO,
      planning::MOTION_CAPABILITY_ROTATE_IN_PLACE,
  };
  return capabilities;
}

planning::MotionExecutionCommand BaseCommand() {
  planning::MotionExecutionCommand command;
  command.mutable_header()->set_timestamp_sec(10.0);
  command.mutable_header()->set_module_name("planning");
  command.mutable_header()->set_frame_id("map");
  command.mutable_identity()->set_producer_epoch("planning-boot-1");
  command.mutable_identity()->set_aggregate_id("motion-stream");
  command.mutable_identity()->set_command_id("command");
  command.mutable_identity()->set_revision(1);
  command.set_reference_frame_id("map");
  command.set_effective_time_sec(10.0);
  command.set_expiry_time_sec(20.0);
  command.set_priority(1);
  command.set_preemptible(true);

  auto* start = command.mutable_start_condition();
  start->mutable_expected_position()->set_x(0.0);
  start->mutable_expected_position()->set_y(0.0);
  start->mutable_expected_position()->set_z(0.0);
  start->set_expected_heading(0.0);
  start->set_expected_gear(canbus::Chassis::GEAR_DRIVE);
  start->set_max_position_error_m(0.2);
  start->set_max_heading_error_rad(0.1);
  start->set_max_abs_speed_mps(0.5);
  start->set_snapshot_time_sec(9.5);
  start->set_reference_frame_id("map");

  auto* constraints = command.mutable_constraints();
  constraints->set_max_speed_mps(1.0);
  constraints->set_max_acceleration_mps2(1.0);
  constraints->set_max_deceleration_mps2(1.0);
  constraints->set_max_jerk_mps3(1.0);
  constraints->set_max_distance_m(5.0);
  constraints->set_max_yaw_rate_radps(0.5);
  constraints->set_max_yaw_acceleration_radps2(0.5);
  constraints->set_max_abs_curvature_per_m(0.5);
  constraints->set_max_abs_curvature_derivative_per_m2(0.5);

  auto* completion = command.mutable_completion();
  completion->set_position_tolerance_m(0.1);
  completion->set_heading_tolerance_rad(0.1);
  completion->set_speed_tolerance_mps(0.05);
  completion->set_settle_time_sec(0.2);
  completion->set_execution_timeout_sec(5.0);

  auto* envelope = command.mutable_spatial_envelope();
  envelope->set_max_lateral_deviation_m(0.5);
  auto* corner = envelope->add_boundary();
  corner->set_x(-1.0);
  corner->set_y(-1.0);
  corner = envelope->add_boundary();
  corner->set_x(2.0);
  corner->set_y(-1.0);
  corner = envelope->add_boundary();
  corner->set_x(2.0);
  corner->set_y(2.0);
  corner = envelope->add_boundary();
  corner->set_x(-1.0);
  corner->set_y(2.0);
  return command;
}

planning::MotionExecutionCommand ValidTrajectoryCommand() {
  auto command = BaseCommand();
  command.add_required_capability(
      planning::MOTION_CAPABILITY_TRAJECTORY_TRACKING);
  auto* trajectory = command.mutable_trajectory();
  trajectory->set_trajectory_id("trajectory-1");
  trajectory->set_gear(canbus::Chassis::GEAR_DRIVE);
  auto* point = trajectory->add_point();
  point->mutable_path_point()->set_x(0.0);
  point->mutable_path_point()->set_y(0.0);
  point->mutable_path_point()->set_z(0.0);
  point->mutable_path_point()->set_theta(0.0);
  point->mutable_path_point()->set_s(0.0);
  point->mutable_path_point()->set_kappa(0.0);
  point->mutable_path_point()->set_dkappa(0.0);
  point->set_speed_mps(0.0);
  point->set_acceleration_mps2(0.0);
  point->set_relative_time_sec(0.0);
  auto* next_point = trajectory->add_point();
  next_point->CopyFrom(*point);
  next_point->mutable_path_point()->set_x(1.0);
  next_point->mutable_path_point()->set_s(1.0);
  next_point->set_relative_time_sec(1.0);
  return command;
}

planning::MotionExecutionCommand ValidHoldCommand() {
  auto command = BaseCommand();
  command.mutable_constraints()->set_max_speed_mps(0.0);
  command.add_required_capability(
      planning::MOTION_CAPABILITY_STANDSTILL_HOLD);
  auto* primitive = command.mutable_primitive();
  primitive->set_type(planning::MOTION_PRIMITIVE_STANDSTILL_HOLD);
  primitive->mutable_standstill_hold()->set_reauthorization_period_sec(1.0);
  return command;
}

TEST(MotionExecutionValidatorTest, AcceptsValidTrajectory) {
  MotionExecutionValidator validator(AllCapabilities());
  const auto result = validator.Validate(ValidTrajectoryCommand(), 11.0);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.execution_type,
            planning::MOTION_EXECUTION_TYPE_TRAJECTORY);
}

TEST(MotionExecutionValidatorTest, RejectsTrajectoryWithoutHardEnvelope) {
  MotionExecutionValidator validator(AllCapabilities());
  auto command = ValidTrajectoryCommand();
  command.clear_spatial_envelope();

  const auto result = validator.Validate(command, 11.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reject_reason,
            planning::MOTION_REJECT_INVALID_COMPLETION);
}

TEST(MotionExecutionValidatorTest, AcceptsBoundedHoldPrimitive) {
  MotionExecutionValidator validator(AllCapabilities());
  const auto result = validator.Validate(ValidHoldCommand(), 11.0);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.execution_type,
            planning::MOTION_EXECUTION_TYPE_PRIMITIVE);
}

TEST(MotionExecutionValidatorTest, RejectsExpiredCommand) {
  MotionExecutionValidator validator(AllCapabilities());
  const auto result = validator.Validate(ValidTrajectoryCommand(), 21.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reject_reason, planning::MOTION_REJECT_STALE);
}

TEST(MotionExecutionValidatorTest, RejectsConflictingHeaderFrame) {
  MotionExecutionValidator validator(AllCapabilities());
  auto command = ValidTrajectoryCommand();
  command.mutable_header()->set_frame_id("odom");
  const auto result = validator.Validate(command, 11.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reject_reason,
            planning::MOTION_REJECT_INVALID_FRAME);
}

TEST(MotionExecutionValidatorTest, RejectsMissingStartCondition) {
  MotionExecutionValidator validator(AllCapabilities());
  auto command = ValidTrajectoryCommand();
  command.clear_start_condition();

  const auto result = validator.Validate(command, 11.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reject_reason,
            planning::MOTION_REJECT_INVALID_START_CONDITION);
}

TEST(MotionExecutionValidatorTest, RejectsUnsupportedRotateInPlace) {
  MotionExecutionCapabilities capabilities;
  capabilities.supported = {
      planning::MOTION_CAPABILITY_STANDSTILL_HOLD,
  };
  MotionExecutionValidator validator(capabilities);
  auto command = BaseCommand();
  command.add_required_capability(
      planning::MOTION_CAPABILITY_ROTATE_IN_PLACE);
  auto* primitive = command.mutable_primitive();
  primitive->set_type(planning::MOTION_PRIMITIVE_ROTATE_IN_PLACE);
  primitive->mutable_rotate_in_place()->set_target_heading(1.0);

  const auto result = validator.Validate(command, 11.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reject_reason,
            planning::MOTION_REJECT_UNSUPPORTED_CAPABILITY);
}

TEST(MotionExecutionValidatorTest, RejectsMovingPrimitiveWithoutEnvelope) {
  MotionExecutionValidator validator(AllCapabilities());
  auto command = BaseCommand();
  command.clear_spatial_envelope();
  command.add_required_capability(
      planning::MOTION_CAPABILITY_CORRIDOR_SERVO);
  auto* primitive = command.mutable_primitive();
  primitive->set_type(planning::MOTION_PRIMITIVE_CORRIDOR_SERVO);
  auto* corridor = primitive->mutable_corridor_servo();
  corridor->mutable_target_position()->set_x(1.0);
  corridor->mutable_target_position()->set_y(0.0);
  corridor->set_target_heading(0.0);
  corridor->set_direction(planning::MOTION_DIRECTION_FORWARD);

  const auto result = validator.Validate(command, 11.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reject_reason,
            planning::MOTION_REJECT_INVALID_CONSTRAINTS);
}

TEST(MotionExecutionValidatorTest, AcceptsExplicitStraightCorridorServo) {
  MotionExecutionValidator validator(AllCapabilities());
  auto command = BaseCommand();
  command.add_required_capability(
      planning::MOTION_CAPABILITY_CORRIDOR_SERVO);
  auto* centerline =
      command.mutable_spatial_envelope()->add_reference_centerline();
  centerline->set_x(0.0);
  centerline->set_y(0.0);
  centerline =
      command.mutable_spatial_envelope()->add_reference_centerline();
  centerline->set_x(1.0);
  centerline->set_y(0.0);
  auto* primitive = command.mutable_primitive();
  primitive->set_type(planning::MOTION_PRIMITIVE_CORRIDOR_SERVO);
  auto* corridor = primitive->mutable_corridor_servo();
  corridor->mutable_target_position()->set_x(1.0);
  corridor->mutable_target_position()->set_y(0.0);
  corridor->set_target_heading(0.0);
  corridor->set_direction(planning::MOTION_DIRECTION_FORWARD);

  const auto result = validator.Validate(command, 11.0);
  EXPECT_TRUE(result.accepted);
}

TEST(MotionExecutionValidatorTest, RejectsCorridorWithoutCenterline) {
  MotionExecutionValidator validator(AllCapabilities());
  auto command = BaseCommand();
  auto* primitive = command.mutable_primitive();
  primitive->set_type(planning::MOTION_PRIMITIVE_CORRIDOR_SERVO);
  auto* corridor = primitive->mutable_corridor_servo();
  corridor->mutable_target_position()->set_x(1.0);
  corridor->mutable_target_position()->set_y(0.0);
  corridor->set_target_heading(0.0);
  corridor->set_direction(planning::MOTION_DIRECTION_FORWARD);

  const auto result = validator.Validate(command, 11.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reject_reason, planning::MOTION_REJECT_INVALID_PAYLOAD);
}

TEST(MotionExecutionValidatorTest, RejectsCorridorWithoutSettledCompletion) {
  MotionExecutionValidator validator(AllCapabilities());
  auto command = BaseCommand();
  command.mutable_completion()->clear_speed_tolerance_mps();
  auto* centerline =
      command.mutable_spatial_envelope()->add_reference_centerline();
  centerline->set_x(0.0);
  centerline->set_y(0.0);
  centerline =
      command.mutable_spatial_envelope()->add_reference_centerline();
  centerline->set_x(1.0);
  centerline->set_y(0.0);
  auto* primitive = command.mutable_primitive();
  primitive->set_type(planning::MOTION_PRIMITIVE_CORRIDOR_SERVO);
  auto* corridor = primitive->mutable_corridor_servo();
  corridor->mutable_target_position()->set_x(1.0);
  corridor->mutable_target_position()->set_y(0.0);
  corridor->set_target_heading(0.0);
  corridor->set_direction(planning::MOTION_DIRECTION_FORWARD);

  const auto result = validator.Validate(command, 11.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reject_reason, planning::MOTION_REJECT_INVALID_PAYLOAD);
}

TEST(MotionExecutionValidatorTest, RejectsMovingPrimitiveWithoutHardBoundary) {
  MotionExecutionValidator validator(AllCapabilities());
  auto command = BaseCommand();
  command.mutable_spatial_envelope()->clear_boundary();
  auto* centerline =
      command.mutable_spatial_envelope()->add_reference_centerline();
  centerline->set_x(0.0);
  centerline->set_y(0.0);
  centerline =
      command.mutable_spatial_envelope()->add_reference_centerline();
  centerline->set_x(1.0);
  centerline->set_y(0.0);
  auto* primitive = command.mutable_primitive();
  primitive->set_type(planning::MOTION_PRIMITIVE_CORRIDOR_SERVO);
  auto* corridor = primitive->mutable_corridor_servo();
  corridor->mutable_target_position()->set_x(1.0);
  corridor->mutable_target_position()->set_y(0.0);
  corridor->set_target_heading(0.0);
  corridor->set_direction(planning::MOTION_DIRECTION_FORWARD);

  const auto result = validator.Validate(command, 11.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reject_reason,
            planning::MOTION_REJECT_INVALID_CONSTRAINTS);
}

TEST(MotionExecutionValidatorTest, AcceptsExplicitRotateInPlace) {
  MotionExecutionValidator validator(AllCapabilities());
  auto command = BaseCommand();
  command.add_required_capability(
      planning::MOTION_CAPABILITY_ROTATE_IN_PLACE);
  auto* primitive = command.mutable_primitive();
  primitive->set_type(planning::MOTION_PRIMITIVE_ROTATE_IN_PLACE);
  auto* rotate = primitive->mutable_rotate_in_place();
  rotate->set_target_heading(1.0);
  rotate->mutable_pivot_position()->set_x(0.0);
  rotate->mutable_pivot_position()->set_y(0.0);
  rotate->set_max_position_deviation_m(0.1);
  rotate->set_direction(planning::ROTATION_DIRECTION_COUNTERCLOCKWISE);

  const auto result = validator.Validate(command, 11.0);
  EXPECT_TRUE(result.accepted);
}

TEST(MotionExecutionValidatorTest, RejectsNonFiniteTrajectoryGeometry) {
  MotionExecutionValidator validator(AllCapabilities());
  auto command = ValidTrajectoryCommand();
  command.mutable_trajectory()
      ->mutable_point(0)
      ->mutable_path_point()
      ->set_kappa(std::numeric_limits<double>::quiet_NaN());

  const auto result = validator.Validate(command, 11.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reject_reason, planning::MOTION_REJECT_INVALID_PAYLOAD);
}

TEST(MotionExecutionValidatorTest, RejectsNegativeCompletionTolerance) {
  MotionExecutionValidator validator(AllCapabilities());
  auto command = ValidTrajectoryCommand();
  command.mutable_completion()->set_speed_tolerance_mps(-0.1);

  const auto result = validator.Validate(command, 11.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reject_reason,
            planning::MOTION_REJECT_INVALID_COMPLETION);
}

TEST(MotionExecutionValidatorTest, RejectsImpliedJerkViolation) {
  MotionExecutionValidator validator(AllCapabilities());
  auto command = ValidTrajectoryCommand();
  auto* point = command.mutable_trajectory()->mutable_point(1);
  point->set_relative_time_sec(0.1);
  point->set_acceleration_mps2(0.5);

  const auto result = validator.Validate(command, 11.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reject_reason, planning::MOTION_REJECT_INVALID_PAYLOAD);
}

TEST(MotionExecutionValidatorTest, RejectsMissingProducerEpoch) {
  MotionExecutionValidator validator(AllCapabilities());
  auto command = ValidTrajectoryCommand();
  command.mutable_identity()->clear_producer_epoch();

  const auto result = validator.Validate(command, 11.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reject_reason, planning::MOTION_REJECT_MISSING_CONTEXT);
}

TEST(MotionExecutionValidatorTest, AcceptsReverseStationOrdering) {
  MotionExecutionValidator validator(AllCapabilities());
  auto command = ValidTrajectoryCommand();
  command.mutable_trajectory()->set_gear(canbus::Chassis::GEAR_REVERSE);
  command.mutable_trajectory()
      ->mutable_point(1)
      ->mutable_path_point()
      ->set_s(-1.0);

  const auto result = validator.Validate(command, 11.0);
  EXPECT_TRUE(result.accepted);
}

TEST(MotionExecutionValidatorTest, RejectsMissingDirectionalGear) {
  MotionExecutionValidator validator(AllCapabilities());
  auto command = ValidTrajectoryCommand();
  command.mutable_trajectory()->clear_gear();

  const auto result = validator.Validate(command, 11.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reject_reason, planning::MOTION_REJECT_INVALID_PAYLOAD);
}

}  // namespace
}  // namespace control
}  // namespace apollo
