#include "modules/control/common/motion_primitive_executor.h"

#include <cmath>
#include <limits>
#include <utility>

#include "gtest/gtest.h"
#include "modules/control/common/strategy_orchestrator.h"
#include "modules/control/common/motion_execution_monitor.h"

namespace apollo {
namespace control {
namespace {

MotionPrimitiveModel Model() {
  MotionPrimitiveModel model;
  model.max_speed_mps = 2.0;
  model.max_acceleration_mps2 = 1.0;
  model.max_deceleration_mps2 = 1.0;
  model.max_jerk_mps3 = 2.0;
  model.max_curvature_per_m = 0.4;
  model.max_curvature_derivative_per_m2 = 0.5;
  model.max_lateral_acceleration_mps2 = 0.5;
  model.footprint_radius_m = 0.5;
  model.sample_period_sec = 0.05;
  model.spatial_step_m = 0.05;
  model.max_state_age_sec = 0.5;
  model.max_tracking_heading_error_rad = 0.2;
  return model;
}

common::VehicleState State(double time = 10.0) {
  common::VehicleState state;
  state.set_x(0.0);
  state.set_y(0.0);
  state.set_z(0.0);
  state.set_heading(0.0);
  state.set_timestamp(time);
  state.set_linear_velocity(0.0);
  state.set_linear_acceleration(0.0);
  state.set_kappa(0.0);
  state.set_angular_velocity(0.0);
  return state;
}

planning::MotionExecutionCommand Command(bool reverse = false) {
  planning::MotionExecutionCommand command;
  command.mutable_header()->set_timestamp_sec(10.0);
  command.mutable_header()->set_module_name("planning");
  command.mutable_header()->set_frame_id("map");
  command.mutable_identity()->set_producer_epoch("epoch");
  command.mutable_identity()->set_aggregate_id("mission");
  command.mutable_identity()->set_command_id("pose");
  command.mutable_identity()->set_revision(1);
  command.set_authority_generation(1);
  command.mutable_authorized_mission_identity()->set_producer_epoch(
      "mission-boot-1");
  command.mutable_authorized_mission_identity()->set_aggregate_id("mission");
  command.mutable_authorized_mission_identity()->set_command_id("mission");
  command.mutable_authorized_mission_identity()->set_revision(1);
  command.set_reference_frame_id("map");
  command.set_effective_time_sec(10.0);
  command.set_expiry_time_sec(40.0);
  auto* start = command.mutable_start_condition();
  start->mutable_expected_position()->set_x(0.0);
  start->mutable_expected_position()->set_y(0.0);
  start->set_expected_heading(0.0);
  start->set_expected_gear(reverse ? canbus::Chassis::GEAR_REVERSE
                                 : canbus::Chassis::GEAR_DRIVE);
  start->set_max_position_error_m(0.1);
  start->set_max_heading_error_rad(0.1);
  start->set_max_abs_speed_mps(0.5);
  start->set_snapshot_time_sec(10.0);
  start->set_reference_frame_id("map");
  auto* constraints = command.mutable_constraints();
  constraints->set_max_speed_mps(1.5);
  constraints->set_max_acceleration_mps2(0.8);
  constraints->set_max_deceleration_mps2(0.8);
  constraints->set_max_jerk_mps3(1.5);
  constraints->set_max_distance_m(10.0);
  constraints->set_max_abs_curvature_per_m(0.3);
  constraints->set_max_abs_curvature_derivative_per_m2(0.4);
  auto* completion = command.mutable_completion();
  completion->set_position_tolerance_m(0.05);
  completion->set_heading_tolerance_rad(0.05);
  completion->set_speed_tolerance_mps(0.02);
  completion->set_settle_time_sec(0.2);
  completion->set_execution_timeout_sec(25.0);
  auto* envelope = command.mutable_spatial_envelope();
  envelope->set_max_lateral_deviation_m(0.2);
  for (const auto& xy : {std::pair<double, double>{-13.0, -3.0},
                         {13.0, -3.0}, {13.0, 3.0}, {-13.0, 3.0}}) {
    auto* corner = envelope->add_boundary();
    corner->set_x(xy.first);
    corner->set_y(xy.second);
  }
  auto* primitive = command.mutable_primitive();
  primitive->set_type(planning::MOTION_PRIMITIVE_POSE_SERVO);
  auto* pose = primitive->mutable_pose_servo();
  pose->mutable_target_position()->set_x(reverse ? -4.0 : 4.0);
  pose->mutable_target_position()->set_y(0.0);
  pose->set_target_heading(0.0);
  pose->set_direction(reverse ? planning::MOTION_DIRECTION_REVERSE
                             : planning::MOTION_DIRECTION_FORWARD);
  return command;
}

planning::MotionExecutionCommand Curve(bool reverse = false) {
  auto command = Command(reverse);
  const double sign = reverse ? -1.0 : 1.0;
  auto* pose = command.mutable_primitive()->mutable_pose_servo();
  pose->mutable_target_position()->set_x(5.0 * std::sin(sign * 0.4));
  pose->mutable_target_position()->set_y(5.0 * (1.0 - std::cos(0.4)));
  pose->set_target_heading(sign * 0.4);
  auto* first = command.mutable_spatial_envelope()->add_reference_path();
  first->set_x(0.0);
  first->set_y(0.0);
  first->set_s(0.0);
  first->set_theta(0.0);
  first->set_kappa(0.2);
  first->set_dkappa(0.0);
  auto* last = command.mutable_spatial_envelope()->add_reference_path();
  last->CopyFrom(command.spatial_envelope().reference_path(0));
  last->set_x(pose->target_position().x());
  last->set_y(pose->target_position().y());
  last->set_theta(pose->target_heading());
  last->set_s(sign * 2.0);
  return command;
}

TEST(MotionPrimitiveExecutorTest, AlignedPoseProducesBoundedReference) {
  MotionPrimitiveExecutor executor(Model());
  const auto command = Command();
  const auto result = executor.Update(command, State(), "map", 10.0);
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_FALSE(result.completed);
  EXPECT_FALSE(result.hold_confirmed);
  ASSERT_GT(result.reference.trajectory_point_size(), 2);
  EXPECT_EQ(result.reference.control_intent().tracking_mode(),
            planning::TRACKING_MODE_PATH_SPEED);
  EXPECT_EQ(result.reference.control_intent().execution_channel(),
            planning::EXECUTION_CHANNEL_PRIMITIVE);
  const auto& constraints = command.constraints();
  double previous_time = -1.0;
  double previous_s = -1.0;
  for (const auto& point : result.reference.trajectory_point()) {
    EXPECT_GT(point.relative_time(), previous_time);
    EXPECT_GE(point.path_point().s() + 1e-8, previous_s);
    EXPECT_NEAR(point.path_point().y(), 0.0, 1e-8);
    EXPECT_GE(point.v(), 0.0);
    EXPECT_LE(point.v(), constraints.max_speed_mps() + 1e-8);
    EXPECT_LE(point.a(), constraints.max_acceleration_mps2() + 1e-8);
    EXPECT_GE(point.a(), -constraints.max_deceleration_mps2() - 1e-8);
    EXPECT_LE(std::abs(point.da()), constraints.max_jerk_mps3() + 1e-8);
    previous_time = point.relative_time();
    previous_s = point.path_point().s();
  }
  const auto& last = result.reference.trajectory_point(
      result.reference.trajectory_point_size() - 1);
  EXPECT_NEAR(last.path_point().x(), 4.0, 1e-8);
  EXPECT_NEAR(last.v(), 0.0, 1e-8);
  EXPECT_NEAR(last.a(), 0.0, 1e-8);
}

TEST(MotionPrimitiveExecutorTest, SpatialIntentSurvivesLegacyStrategy) {
  MotionPrimitiveExecutor executor(Model());
  auto command = Command();
  command.mutable_execution()->set_active_scene(planning::SCENE_PARK_IN);
  command.mutable_control_intent()->set_require_full_stop(true);
  auto result = executor.Update(command, State(), "map", 10.0);
  ASSERT_TRUE(result.accepted) << result.reason;
  StrategyOrchestrator strategy;
  const auto goal = BuildControlCommandGoal(result.reference);
  strategy.Apply(strategy.Resolve(goal), &result.reference);
  EXPECT_EQ(result.reference.control_intent().tracking_mode(),
            planning::TRACKING_MODE_PATH_SPEED);
  EXPECT_EQ(result.reference.control_intent().lateral_intent(),
            planning::LAT_INTENT_TRACK_PATH);
  EXPECT_EQ(result.reference.control_intent().primitive_type(),
            planning::CONTROL_PRIMITIVE_NONE);
  EXPECT_EQ(result.reference.control_intent().execution_channel(),
            planning::EXECUTION_CHANNEL_PRIMITIVE);
  EXPECT_DOUBLE_EQ(result.reference.control_intent().target_stop_point().x(), 4.0);
  EXPECT_FALSE(result.reference.control_intent().require_full_stop());
  EXPECT_EQ(result.reference.execution().active_scene(), planning::SCENE_PARK_IN);
}

TEST(MotionPrimitiveExecutorTest, ReverseReferenceHasSignedStationAndVelocity) {
  MotionPrimitiveExecutor executor(Model());
  const auto result = executor.Update(Command(true), State(), "map", 10.0);
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(result.reference.gear(), canbus::Chassis::GEAR_REVERSE);
  for (const auto& point : result.reference.trajectory_point()) {
    EXPECT_LE(point.path_point().s(), 1e-8);
    EXPECT_LE(point.v(), 1e-8);
    EXPECT_NEAR(point.path_point().theta(), 0.0, 1e-8);
  }
}

TEST(MotionPrimitiveExecutorTest, ExplicitCircularGuidanceSupportsBothDirections) {
  for (bool reverse : {false, true}) {
    MotionPrimitiveExecutor executor(Model());
    const auto result = executor.Update(Curve(reverse), State(), "map", 10.0);
    ASSERT_TRUE(result.accepted) << result.reason;
    for (const auto& point : result.reference.trajectory_point()) {
      EXPECT_NEAR(point.path_point().kappa(), 0.2, 1e-8);
      EXPECT_NEAR(std::hypot(point.path_point().x(),
                             point.path_point().y() - 5.0), 5.0, 1e-5);
      EXPECT_LE(point.v() * point.v() * 0.2,
                Model().max_lateral_acceleration_mps2 + 1e-8);
    }
  }
}

TEST(MotionPrimitiveExecutorTest, CorridorUsesExplicitSpatialGuidance) {
  auto command = Curve();
  const auto pose = command.primitive().pose_servo();
  auto* primitive = command.mutable_primitive();
  primitive->set_type(planning::MOTION_PRIMITIVE_CORRIDOR_SERVO);
  auto* corridor = primitive->mutable_corridor_servo();
  corridor->mutable_target_position()->CopyFrom(pose.target_position());
  corridor->set_target_heading(pose.target_heading());
  corridor->set_direction(pose.direction());
  MotionPrimitiveExecutor executor(Model());
  const auto result = executor.Update(command, State(), "map", 10.0);
  ASSERT_TRUE(result.accepted) << result.reason;
}

TEST(MotionPrimitiveExecutorTest, StraightLegacyCorridorIsNotCornerCutting) {
  auto command = Command();
  const auto pose = command.primitive().pose_servo();
  auto* primitive = command.mutable_primitive();
  primitive->set_type(planning::MOTION_PRIMITIVE_CORRIDOR_SERVO);
  auto* corridor = primitive->mutable_corridor_servo();
  corridor->mutable_target_position()->CopyFrom(pose.target_position());
  corridor->set_target_heading(pose.target_heading());
  corridor->set_direction(pose.direction());
  for (double x : {0.0, 2.0, 4.0}) {
    auto* point = command.mutable_spatial_envelope()->add_reference_centerline();
    point->set_x(x);
    point->set_y(0.0);
  }
  MotionPrimitiveExecutor executor(Model());
  const auto straight = executor.Update(command, State(), "map", 10.0);
  ASSERT_TRUE(straight.accepted) << straight.reason;
  executor.Reset();
  command.mutable_spatial_envelope()->mutable_reference_centerline(1)->set_y(1.0);
  EXPECT_FALSE(executor.Update(command, State(), "map", 10.0).accepted);
}

TEST(MotionPrimitiveExecutorTest, RejectsArbitraryUnguidedPoseAndRotation) {
  auto command = Command();
  command.mutable_primitive()->mutable_pose_servo()->set_target_heading(0.5);
  MotionPrimitiveExecutor executor(Model());
  EXPECT_FALSE(executor.Update(command, State(), "map", 10.0).accepted);
  command.mutable_primitive()->set_type(planning::MOTION_PRIMITIVE_ROTATE_IN_PLACE);
  EXPECT_EQ(executor.Update(command, State(), "map", 10.0).reject_reason,
            planning::MOTION_REJECT_UNSUPPORTED_CAPABILITY);
}

TEST(MotionPrimitiveExecutorTest, RejectsInconsistentAndExcessCurvatureGuidance) {
  auto command = Curve();
  command.mutable_spatial_envelope()->mutable_reference_path(1)->set_theta(0.5);
  MotionPrimitiveExecutor executor(Model());
  EXPECT_FALSE(executor.Update(command, State(), "map", 10.0).accepted);
  command = Curve();
  command.mutable_constraints()->set_max_abs_curvature_per_m(0.1);
  EXPECT_FALSE(executor.Update(command, State(), "map", 10.0).accepted);
}

TEST(MotionPrimitiveExecutorTest, RejectsMissingModelAndCurvatureConstraints) {
  MotionPrimitiveExecutor missing_model(MotionPrimitiveModel{});
  EXPECT_FALSE(missing_model.Update(Command(), State(), "map", 10.0).accepted);
  auto command = Command();
  command.mutable_constraints()->clear_max_abs_curvature_per_m();
  MotionPrimitiveExecutor executor(Model());
  EXPECT_FALSE(executor.Update(command, State(), "map", 10.0).accepted);
}

TEST(MotionPrimitiveExecutorTest, RejectsDirectionGearMismatch) {
  auto command = Command(true);
  command.mutable_start_condition()->set_expected_gear(canbus::Chassis::GEAR_DRIVE);
  MotionPrimitiveExecutor executor(Model());
  EXPECT_FALSE(executor.Update(command, State(), "map", 10.0).accepted);
}

TEST(MotionPrimitiveExecutorTest, RejectsFrameStaleMissingAndNonfiniteState) {
  const auto command = Command();
  for (int scenario = 0; scenario < 5; ++scenario) {
    MotionPrimitiveExecutor executor(Model());
    auto state = State();
    std::string frame = "map";
    if (scenario == 0) {
      frame = "odom";
    } else if (scenario == 1) {
      state.set_timestamp(9.0);
    } else if (scenario == 2) {
      state.clear_heading();
    } else if (scenario == 3) {
      state.set_x(std::numeric_limits<double>::quiet_NaN());
    } else {
      state.set_timestamp(11.0);
    }
    const auto result = executor.Update(command, state, frame, 10.0);
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.reference.trajectory_point_size(), 0);
    EXPECT_FALSE(result.completed);
  }
}

TEST(MotionPrimitiveExecutorTest, RejectsStartMismatch) {
  MotionPrimitiveExecutor executor(Model());
  auto state = State();
  state.set_y(0.15);
  EXPECT_EQ(executor.Update(Command(), state, "map", 10.0).reject_reason,
            planning::MOTION_REJECT_INVALID_START_CONDITION);
}

TEST(MotionPrimitiveExecutorTest, RequiresWholeFootprintInsideEnvelope) {
  auto command = Command();
  for (auto& point : *command.mutable_spatial_envelope()->mutable_boundary()) {
    point.set_y(point.y() > 0.0 ? 0.6 : -0.6);
  }
  MotionPrimitiveExecutor executor(Model());
  EXPECT_FALSE(executor.Update(command, State(), "map", 10.0).accepted);
}

TEST(MotionPrimitiveExecutorTest, LiveCorridorDepartureLatchesFailure) {
  MotionPrimitiveExecutor executor(Model());
  const auto command = Command();
  ASSERT_TRUE(executor.Update(command, State(), "map", 10.0).accepted);
  auto state = State(10.1);
  state.set_y(0.3);
  const auto failure = executor.Update(command, state, "map", 10.1);
  EXPECT_FALSE(failure.accepted);
  EXPECT_EQ(failure.reference.trajectory_point_size(), 0);
  EXPECT_FALSE(executor.Update(command, State(10.2), "map", 10.2).accepted);
  executor.Reset();
  EXPECT_TRUE(executor.Update(command, State(10.3), "map", 10.3).accepted);
}

TEST(MotionPrimitiveExecutorTest, ChangedPayloadWithSameIdentityIsRejected) {
  MotionPrimitiveExecutor executor(Model());
  auto command = Command();
  ASSERT_TRUE(executor.Update(command, State(), "map", 10.0).accepted);
  command.mutable_primitive()->mutable_pose_servo()->mutable_target_position()->
      set_x(5.0);
  EXPECT_EQ(executor.Update(command, State(10.1), "map", 10.1).reject_reason,
            planning::MOTION_REJECT_INVALID_TRANSITION);
}

TEST(MotionPrimitiveExecutorTest, RepeatedStateCannotAccumulateSettleEvidence) {
  auto command = Command();
  command.mutable_primitive()->mutable_pose_servo()->mutable_target_position()->
      set_x(0.0);
  MotionPrimitiveExecutor executor(Model());
  const auto first = executor.Update(command, State(), "map", 10.0);
  ASSERT_TRUE(first.accepted) << first.reason;
  EXPECT_FALSE(first.completed);
  const auto repeated = executor.Update(command, State(), "map", 10.3);
  EXPECT_TRUE(repeated.accepted);
  EXPECT_FALSE(repeated.completed);
  const auto stale = executor.Update(command, State(), "map", 10.6);
  EXPECT_FALSE(stale.accepted);
  EXPECT_FALSE(stale.completed);
}

TEST(MotionPrimitiveExecutorTest, CompletionRequiresMeasuredPoseSpeedAndSettle) {
  auto command = Command();
  command.mutable_primitive()->mutable_pose_servo()->mutable_target_position()->
      set_x(0.0);
  MotionPrimitiveExecutor executor(Model());
  const auto first = executor.Update(command, State(), "map", 10.0);
  ASSERT_TRUE(first.accepted) << first.reason;
  EXPECT_FALSE(first.completed);
  EXPECT_FALSE(executor.Update(command, State(10.1), "map", 10.1).completed);
  const auto settled = executor.Update(command, State(10.3), "map", 10.3);
  ASSERT_TRUE(settled.accepted) << settled.reason;
  EXPECT_TRUE(settled.completed);
  EXPECT_TRUE(settled.hold_confirmed);
  EXPECT_TRUE(settled.reference.control_intent().require_full_stop());
}

TEST(MotionPrimitiveExecutorTest, MovingAtTargetIsNotCompletion) {
  auto command = Command();
  command.mutable_primitive()->mutable_pose_servo()->mutable_target_position()->
      set_x(0.0);
  auto state = State();
  state.set_linear_velocity(0.1);
  MotionPrimitiveExecutor executor(Model());
  const auto result = executor.Update(command, state, "map", 10.0);
  EXPECT_FALSE(result.completed);
  EXPECT_FALSE(result.hold_confirmed);
  EXPECT_FALSE(result.accepted);
}

TEST(MotionPrimitiveExecutorTest, ImpossibleStoppingBudgetIsRejected) {
  auto command = Command();
  command.mutable_completion()->set_execution_timeout_sec(0.3);
  MotionPrimitiveExecutor executor(Model());
  const auto result = executor.Update(command, State(), "map", 10.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reference.trajectory_point_size(), 0);
}

TEST(MotionPrimitiveExecutorTest, LiveDynamicsAndHeadingStayWithinBounds) {
  for (int scenario = 0; scenario < 3; ++scenario) {
    MotionPrimitiveExecutor executor(Model());
    const auto command = Command();
    ASSERT_TRUE(executor.Update(command, State(), "map", 10.0).accepted);
    auto state = State(10.1);
    if (scenario == 0) {
      state.set_linear_velocity(2.0);
    } else if (scenario == 1) {
      state.set_heading(0.3);
    } else {
      state.set_linear_velocity(-0.3);
    }
    const auto result = executor.Update(command, state, "map", 10.1);
    EXPECT_FALSE(result.accepted);
    EXPECT_FALSE(result.completed);
    EXPECT_EQ(result.reference.trajectory_point_size(), 0);
  }
}

TEST(MotionPrimitiveExecutorTest, ReferenceEndpointIsNotTerminalEvidence) {
  MotionPrimitiveExecutor executor(Model());
  const auto command = Command();
  ASSERT_TRUE(executor.Update(command, State(), "map", 10.0).accepted);
  for (int cycle = 1; cycle < 20; ++cycle) {
    const double now = 10.0 + cycle * 0.1;
    const auto result = executor.Update(command, State(now), "map", now);
    ASSERT_TRUE(result.accepted) << result.reason;
    EXPECT_FALSE(result.completed);
    EXPECT_FALSE(result.hold_confirmed);
  }
}

TEST(MotionPrimitiveExecutorTest, NewIdentityRestartsSettleEvidence) {
  auto command = Command();
  command.mutable_primitive()->mutable_pose_servo()->mutable_target_position()->
      set_x(0.0);
  MotionPrimitiveExecutor executor(Model());
  ASSERT_TRUE(executor.Update(command, State(), "map", 10.0).accepted);
  command.mutable_identity()->set_revision(2);
  const auto next = executor.Update(command, State(10.3), "map", 10.3);
  ASSERT_TRUE(next.accepted) << next.reason;
  EXPECT_FALSE(next.completed);
}

TEST(MotionPrimitiveExecutorTest, ClosedLoopReferenceFollowingSettlesAtGoal) {
  const auto command = Command();
  MotionPrimitiveExecutor executor(Model());
  auto state = State();
  double now = 10.0;
  bool completed = false;
  for (int cycle = 0; cycle < 600 && !completed; ++cycle) {
    const auto result = executor.Update(command, state, "map", now);
    ASSERT_TRUE(result.accepted) << "cycle=" << cycle << " " << result.reason;
    completed = result.completed;
    ASSERT_GE(result.reference.trajectory_point_size(), 2);
    const auto& next = result.reference.trajectory_point(1);
    now += next.relative_time();
    state.set_timestamp(now);
    state.set_x(next.path_point().x());
    state.set_y(next.path_point().y());
    state.set_heading(next.path_point().theta());
    state.set_kappa(next.path_point().kappa());
    state.set_linear_velocity(next.v());
    state.set_linear_acceleration(next.a());
  }
  EXPECT_TRUE(completed);
  EXPECT_NEAR(state.x(), 4.0, command.completion().position_tolerance_m());
  EXPECT_LE(std::abs(state.linear_velocity()),
            command.completion().speed_tolerance_mps());
}

TEST(MotionExecutionMonitorTest, HoldEvidenceKeepsAuthorizationAndDetectsDrift) {
  auto command = Command();
  command.mutable_primitive()->set_type(planning::MOTION_PRIMITIVE_STANDSTILL_HOLD);
  command.mutable_primitive()->mutable_standstill_hold()->
      set_reauthorization_period_sec(1.0);
  MotionExecutionMonitor monitor(0.5, 0.1, 0.5);
  EXPECT_FALSE(monitor.Update(command, State(), "map", 10.0).hold_confirmed);
  const auto settled = monitor.Update(command, State(10.3), "map", 10.3);
  EXPECT_TRUE(settled.accepted);
  EXPECT_TRUE(settled.hold_confirmed);
  EXPECT_FALSE(settled.completed);
  auto drifted = State(10.4);
  drifted.set_x(0.1);
  EXPECT_FALSE(monitor.Update(command, drifted, "map", 10.4).accepted);
}

TEST(MotionExecutionMonitorTest, TrajectoryCompletionRequiresMeasuredRest) {
  auto command = Command();
  command.clear_primitive();
  auto* first = command.mutable_trajectory()->add_point();
  first->mutable_path_point()->set_x(0.0);
  first->mutable_path_point()->set_y(0.0);
  first->mutable_path_point()->set_theta(0.0);
  first->set_relative_time_sec(0.0);
  auto* last = command.mutable_trajectory()->add_point();
  last->CopyFrom(command.trajectory().point(0));
  last->mutable_path_point()->set_x(1.0);
  last->set_relative_time_sec(1.0);
  MotionExecutionMonitor monitor(0.5, 0.1, 0.5);
  auto state = State();
  state.set_x(1.0);
  EXPECT_FALSE(monitor.Update(command, state, "map", 10.0).completed);
  state.set_timestamp(10.4);
  EXPECT_FALSE(monitor.Update(command, state, "map", 10.4).completed);
  state.set_timestamp(10.8);
  EXPECT_FALSE(monitor.Update(command, state, "map", 10.8).completed);
  state.set_timestamp(11.0);
  state.set_linear_velocity(0.1);
  EXPECT_FALSE(monitor.Update(command, state, "map", 11.0).completed);
  state.set_timestamp(11.1);
  state.set_linear_velocity(0.0);
  EXPECT_FALSE(monitor.Update(command, state, "map", 11.1).completed);
  state.set_timestamp(11.4);
  const auto settled = monitor.Update(command, state, "map", 11.4);
  EXPECT_TRUE(settled.accepted);
  EXPECT_TRUE(settled.completed);
  EXPECT_FALSE(settled.hold_confirmed);
}

TEST(MotionExecutionMonitorTest, MissingFrameOrRepeatedStateCannotComplete) {
  auto command = Command();
  command.mutable_primitive()->set_type(planning::MOTION_PRIMITIVE_STANDSTILL_HOLD);
  MotionExecutionMonitor monitor(0.5, 0.1, 0.5);
  EXPECT_FALSE(monitor.Update(command, State(), "", 10.0).accepted);
  EXPECT_TRUE(monitor.Update(command, State(), "map", 10.0).accepted);
  EXPECT_FALSE(monitor.Update(command, State(), "map", 10.3).hold_confirmed);
  EXPECT_FALSE(monitor.Update(command, State(), "map", 10.6).accepted);
}

TEST(MotionExecutionMonitorTest, LiveTrajectoryBoundsAreNotJustTerminalChecks) {
  auto command = Command();
  command.clear_primitive();
  command.mutable_trajectory()->set_gear(canbus::Chassis::GEAR_DRIVE);
  for (double x : {0.0, 4.0}) {
    auto* point = command.mutable_trajectory()->add_point();
    point->mutable_path_point()->set_x(x);
    point->mutable_path_point()->set_y(0.0);
    point->mutable_path_point()->set_theta(0.0);
    point->set_relative_time_sec(x);
  }
  for (int scenario = 0; scenario < 4; ++scenario) {
    MotionExecutionMonitor monitor(0.5, 0.1, 0.5);
    auto state = State();
    state.set_x(2.0);
    if (scenario == 0) {
      state.set_y(0.3);
    } else if (scenario == 1) {
      state.set_heading(0.2);
    } else if (scenario == 2) {
      state.set_linear_velocity(2.0);
    } else {
      state.set_linear_velocity(-0.1);
    }
    EXPECT_FALSE(monitor.Update(command, state, "map", 10.0).accepted);
  }
}

}  // namespace
}  // namespace control
}  // namespace apollo
